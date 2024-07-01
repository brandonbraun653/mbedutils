/******************************************************************************
 *  File Name:
 *    pico_serial.cpp
 *
 *  Description:
 *    RPI Pico SDK based implementation of the serial driver. This provides a
 *    concrete way for a project to configure UARTs on the RP2040 and bind them
 *    to the mbedutils serial interface.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/array.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/hardware/pico/pico_serial.hpp>
#include <mbedutils/interfaces/irq_intf.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <mbedutils/interfaces/serial_intf.hpp>
#include <mbedutils/interfaces/util_intf.hpp>
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

namespace mb::hw::serial
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr uint32_t UARTx_FIFO_THR_4B  = 0u;
  static constexpr uint32_t UARTx_FIFO_THR_8B  = 1u;
  static constexpr uint32_t UARTx_FIFO_THR_16B = 2u;
  static constexpr uint32_t UARTx_FIFO_THR_24B = 3u;
  static constexpr uint32_t UARTx_FIFO_THR_28B = 4u;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct ControlBlock
  {
    uart_inst_t             *hw;                       /**< Pico SDK UART peripheral instance */
    mb::osal::mb_mutex_t     mutex;                    /**< Mutex for locking device access */
    size_t                   usr_channel;              /**< User logical channel, not instance index */
    size_t                   tx_bytes;                 /**< Number of bytes left to transmit */
    size_t                   rx_bytes;                 /**< Number of bytes left to receive */
    int                      dma_rx_channel;           /**< System dma channel for RX-ing */
    int                      dma_tx_channel;           /**< System dma channel for TX-ing */
    intf::RXCompleteCallback usr_rx_complete_callback; /**< RX completion callback */
    intf::TXCompleteCallback usr_tx_complete_callback; /**< TX completion callback */
    volatile bool            tx_in_progress;           /**< Flag to indicate if a TX operation is in progress */
    volatile bool            rx_in_progress;           /**< Flag to indicate if an RX operation is in progress */

    void reset()
    {
      hw                       = nullptr;
      mutex                    = nullptr;
      usr_channel              = 0;
      tx_bytes                 = 0;
      rx_bytes                 = 0;
      dma_rx_channel           = -1;
      dma_tx_channel           = -1;
      usr_rx_complete_callback = {};
      usr_tx_complete_callback = {};
      tx_in_progress           = false;
      rx_in_progress           = false;
    }
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  /**
   * @brief Hardware state information for each UART peripheral
   */
  static etl::array<ControlBlock, NUM_UARTS> s_cb;
  static size_t                              s_driver_initialized;

  /*---------------------------------------------------------------------------
  Private Function Declarations
  ---------------------------------------------------------------------------*/

  static ControlBlock *get_cb( const size_t channel );
  static ControlBlock *get_cb( uart_inst_t *uart );
  static int           get_uart_irq( uart_inst_t *uart );
  static void          irq_handler_uart_0();
  static void          irq_handler_uart_1();
  static void          irq_handler_uart_x( uart_inst_t *uart );
  static void          dma_tx_complete();
  static void          dma_rx_complete();

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void pico::initialize()
  {
    if( s_driver_initialized == mb::DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    for( auto &cb : s_cb )
    {
      cb.reset();
    }

    s_driver_initialized = mb::DRIVER_INITIALIZED_KEY;
  }


  void pico::configure( const pico::UartConfig &config )
  {
    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    mbed_dbg_assert( s_driver_initialized == mb::DRIVER_INITIALIZED_KEY );
    mbed_dbg_assert( config.uart );

    /* Make sure the hardware registers exist */
    auto hw_idx = uart_get_index( config.uart );
    mbed_dbg_assert( hw_idx < s_cb.size() );

    /*-------------------------------------------------------------------------
    Acquire exclusive access to the UART peripheral for configuration
    -------------------------------------------------------------------------*/
    if( s_cb[ hw_idx ].mutex == nullptr )
    {
      mbed_assert( mb::osal::buildMutexStrategy( s_cb[ hw_idx ].mutex ) );
    }

    if( !mb::osal::tryLockMutex( s_cb[ hw_idx ].mutex ) )
    {
      mbed_assert_continue_msg( false, "UART %d locked", config.usr_channel );
      return;
    }

    /*-------------------------------------------------------------------------
    Enable system level interrupts
    -------------------------------------------------------------------------*/
    irq_handler_t irq_handler = ( hw_idx == 0 ) ? irq_handler_uart_0 : irq_handler_uart_1;

    irq_set_exclusive_handler( get_uart_irq( s_cb[ hw_idx ].hw ), irq_handler );
    irq_set_enabled( get_uart_irq( s_cb[ hw_idx ].hw ), true );

    /*-------------------------------------------------------------------------
    Configure DMA handlers
    -------------------------------------------------------------------------*/
    irq_add_shared_handler( DMA_IRQ_0, dma_tx_complete, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY );
    irq_set_enabled( DMA_IRQ_0, true );

    irq_add_shared_handler( DMA_IRQ_1, dma_rx_complete, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY );
    irq_set_enabled( DMA_IRQ_1, true );

    /*-------------------------------------------------------------------------
    Configure the UART peripheral
    -------------------------------------------------------------------------*/
    const uint actual_baud = uart_init( config.uart, config.baudrate );
    const float pct_error = ( ( float )actual_baud - ( float )config.baudrate ) / ( float )config.baudrate;
    mbed_assert_continue_msg( pct_error < 0.03f, "UART %d baud rate error > 3%. Exp: %d, Act: %d",
                              config.usr_channel, config.baudrate, actual_baud );

    /* Enable the FIFO so we can catch when the transfer is complete */
    uart_set_fifo_enabled( config.uart, true );

    /* Simple serial. No flow control signals. */
    uart_set_hw_flow( config.uart, false, false );

    /* Configure the user's data formatting */
    uart_set_format( config.uart, config.data_bits, config.stop_bits, config.parity );

    /* Disable FIFO threshold IRQs initially */
    uart_set_irq_enables( config.uart, false, false );

    /* Set fifo trigger levels to 1/4 full/empty */
    uart_get_hw( config.uart )->ifls =
        ( UARTx_FIFO_THR_8B << UART_UARTIFLS_RXIFLSEL_LSB ) | ( UARTx_FIFO_THR_8B << UART_UARTIFLS_TXIFLSEL_LSB );

    /* Enable DMA requests: RX, TX */
    uart_get_hw( config.uart )->dmacr = UART_UARTDMACR_RXDMAE_BITS | UART_UARTDMACR_TXDMAE_BITS;

    /*-------------------------------------------------------------------------
    Configure the TX and RX pins
    -------------------------------------------------------------------------*/
    gpio_set_function( config.tx_pin, GPIO_FUNC_UART );
    gpio_set_function( config.rx_pin, GPIO_FUNC_UART );

    /*-------------------------------------------------------------------------
    Configure the rest of the control block
    -------------------------------------------------------------------------*/
    s_cb[ hw_idx ].hw          = config.uart;
    s_cb[ hw_idx ].usr_channel = config.usr_channel;

    mb::osal::unlockMutex( s_cb[ hw_idx ].mutex );
  }


  bool intf::lock( const size_t channel, const size_t timeout )
  {
    if( s_driver_initialized != mb::DRIVER_INITIALIZED_KEY )
    {
      return false;
    }

    return mb::osal::tryLockMutex( get_cb( channel )->mutex, timeout );
  }


  void intf::unlock( const size_t channel )
  {
    if( s_driver_initialized != mb::DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::osal::unlockMutex( get_cb( channel )->mutex );
  }


  int intf::write_async( const size_t channel, const void *data, const size_t length )
  {
    auto cb = get_cb( channel );

    /*-------------------------------------------------------------------------
    Hopefully we have some DMA channels left. Only acquire right now since we
    just now learned we need it.
    -------------------------------------------------------------------------*/
    if( cb->dma_tx_channel < 0 )
    {
      cb->dma_tx_channel = dma_claim_unused_channel( true );
      if( !mbed_assert_continue_msg( cb->dma_tx_channel >= 0, "UART %d DMA TX channel claim failed", cb->usr_channel ) )
      {
        return -1;
      }

      // Unsure what previous state this channel was in, so clean it up
      dma_channel_cleanup( cb->dma_tx_channel );
    }

    /*-------------------------------------------------------------------------
    Ensure the driver is ready for immediate TX. When calling from the user TX
    complete callback, this should always be true. If not, we have no way of
    notifying the driver and have to wait until the next write attempt.
    -------------------------------------------------------------------------*/
    if( cb->tx_in_progress || dma_channel_is_busy( cb->dma_tx_channel ) || !uart_is_writable( cb->hw ) )
    {
      mbed_assert_continue_msg( !mb::irq::in_isr(), "Failed to send %d bytes on channel %d", cb->usr_channel, length );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Configure the DMA channel for TX
    -------------------------------------------------------------------------*/
    auto dma_cfg = dma_channel_get_default_config( cb->dma_tx_channel );

    channel_config_set_transfer_data_size( &dma_cfg, DMA_SIZE_8 );
    channel_config_set_read_increment( &dma_cfg, true );
    channel_config_set_write_increment( &dma_cfg, false );
    channel_config_set_dreq( &dma_cfg, uart_get_dreq( cb->hw, true ) );
    dma_channel_configure( cb->dma_tx_channel, &dma_cfg, &uart_get_hw( cb->hw )->dr, data, length, false );

    dma_channel_set_irq0_enabled( cb->dma_tx_channel, true );

    /*-------------------------------------------------------------------------
    Update the control block with the new transfer information
    -------------------------------------------------------------------------*/
    cb->tx_bytes       = length;
    cb->tx_in_progress = true;

    /*-------------------------------------------------------------------------
    Start the transfer
    -------------------------------------------------------------------------*/
    dma_start_channel_mask( 1u << cb->dma_tx_channel );

    return length;
  }


  void intf::on_tx_complete( const size_t channel, intf::TXCompleteCallback callback )
  {
    get_cb( channel )->usr_tx_complete_callback = callback;
  }


  void intf::write_abort( const size_t channel )
  {
    /*-------------------------------------------------------------------------
    We're not using DMA TX interrupts, so we can just abort the channel without
    worrying about the spurrious behavior described in the SDK documentation.
    -------------------------------------------------------------------------*/
    auto cb = get_cb( channel );
    if( !cb->tx_in_progress )
    {
      return;
    }

    cb->tx_in_progress = false;
    if( cb->dma_tx_channel >= 0 )
    {
      dma_channel_abort( cb->dma_tx_channel );
    }
  }


  int intf::read_async( const size_t channel, void *data, const size_t length, const size_t timeout )
  {
    auto cb = get_cb( channel );

    /*-------------------------------------------------------------------------
    Hopefully we have some DMA channels left. Only acquire right now since we
    just now learned we need it.
    -------------------------------------------------------------------------*/
    if( cb->dma_rx_channel < 0 )
    {
      cb->dma_rx_channel = dma_claim_unused_channel( true );
      if( !mbed_assert_continue_msg( cb->dma_rx_channel >= 0, "UART %d DMA RX channel claim failed", cb->usr_channel ) )
      {
        return -1;
      }

      // Unsure what previous state this channel was in, so clean it up
      dma_channel_cleanup( cb->dma_rx_channel );
    }


    // Several ways to exit a "read" transfer:
    // 1. The DMA channel completes the transfer b/c all bytes received. Need to mask idle event. Idle timeout comes AFTER dma
    // complete.
    // 2. Idle timeout isr fires and we have not received all bytes. Need to abort the DMA transfer. Need to invoke user
    // callback with what we got.
    // 3. User calls read_abort() and we need to stop the DMA channel and clean up. Callback invoked with what we got.
    // 4. System timeout expires and we have not received all bytes. Need to invoke user callback with what we got.
    //      - This can be weird cause we may have received 0 bytes, in-progress of receiving, or in the middle of timing out.
    // 5. DMA RX buffer is smaller than the number of bytes being sent...can't really handle that at this level. Have to pray that
    //    the user can empty that buffer and start a new RX transfer before the UART RX fifo fills up.



    // Hmmm. Is there a way I can register a callback for just the channel?
    //

    // dma_channel_set_irq0_enabled( cb->dma_tx_channel, true );
    // irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    // irq_set_enabled(DMA_IRQ_0, true);

    return -1;
  }


  void intf::on_rx_complete( const size_t channel, intf::RXCompleteCallback callback )
  {
  }


  void intf::read_abort( const size_t channel )
  {
    // Be wary about the spurrious behavior here.
  }


  /*-------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------*/

  /**
   * @brief Gets the control block mapped to the given channel.
   *
   * @param channel  User facing channel number
   * @return ControlBlock*
   */
  static ControlBlock *get_cb( const size_t channel )
  {
    for( auto &cb : s_cb )
    {
      if( cb.usr_channel == channel )
      {
        return &cb;
      }
    }

    mbed_assert_always();
    return nullptr;
  }


  /**
   * @brief Gets the control block for the given UART peripheral.
   *
   * @param uart  UART peripheral instance
   * @return ControlBlock*
   */
  static ControlBlock *get_cb( uart_inst_t *uart )
  {
    for( auto &cb : s_cb )
    {
      if( cb.hw == uart )
      {
        return &cb;
      }
    }

    mbed_assert_always();
    return nullptr;
  }


  /**
   * @brief Get IRQ number for the given UART peripheral.
   *
   * @param uart  UART peripheral instance
   * @return int  IRQ number
   */
  int get_uart_irq( uart_inst_t *uart )
  {
    int uart_index = uart_get_index( uart );
    return uart_index == 0 ? UART0_IRQ : UART1_IRQ;
  }


  /**
   * @brief Handle the UART 0 interrupt event
   */
  static void irq_handler_uart_0()
  {
    irq_handler_uart_x( uart0 );
  }


  /**
   * @brief Handle the UART 1 interrupt event
   */
  static void irq_handler_uart_1()
  {
    irq_handler_uart_x( uart1 );
  }


  /**
   * @brief UARTx interrupt handler for all events
   */
  void irq_handler_uart_x( uart_inst_t *uart )
  {
    /*-------------------------------------------------------------------------
    Cache the internal state that got us here
    -------------------------------------------------------------------------*/
    const uint32_t enabled = uart_get_hw( uart )->mis;
    const uint32_t status  = uart_get_hw( uart )->ris;
    ControlBlock  *cb      = get_cb( uart );

    /*-------------------------------------------------------------------------
    At a minimum ACK all events, then process them if we've actually got a
    transfer in progress.
    -------------------------------------------------------------------------*/
    uart_get_hw( uart )->icr = status;

    if( !cb->tx_in_progress && !cb->rx_in_progress )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Handle Transmit Complete: Occurs when the FIFO falls below the threshold.
    -------------------------------------------------------------------------*/
    if( ( enabled & UART_UARTIMSC_TXIM_BITS ) && ( status & UART_UARTMIS_TXMIS_BITS ) )
    {
      /*-------------------------------------------------------------------------
      Disable the TX interrupt to prevent re-entry. This will be re-enabled
      when the next DMA transfer completes.
      -------------------------------------------------------------------------*/
      uart_get_hw( uart )->imsc &= ~( 1u << UART_UARTIMSC_TXIM_LSB );

      cb->tx_in_progress = false;
      if( cb->usr_tx_complete_callback )
      {
        cb->usr_tx_complete_callback( cb->usr_channel, cb->tx_bytes );
      }
    }

    /*-------------------------------------------------------------------------
    Handle Receive Events
    -------------------------------------------------------------------------*/
    if( 1 /* Mask of receive events */ )
    {
      // We know here that there was an idle timeout or the FIFO is full. Neither
      // of those indicate a complete transfer. We need to wait for the DMA to
      // finish the transfer before we can call the user's callback.

      // DMA may have already completed in case of idle timeout, so set RX
      // in progress to false here.

      // Can I trigger the abort/complete DMA transfer from here? Would certainly
      // simplify chaining the events together...DMA always needs to clean up. Only
      // valid in the case that the DMA is not already complete.
    }

    /*-------------------------------------------------------------------------
    Handle Errors
    -------------------------------------------------------------------------*/
    if( 1 /* Mask of error events */ ) {}
  }


  /**
   * @brief Handle the DMA TX completion event
   */
  static void dma_tx_complete()
  {
    /*-------------------------------------------------------------------------
    Check all UART peripherals for a TX completion event rather than each DMA
    channel. There are far fewer UARTs.
    -------------------------------------------------------------------------*/
    for( auto &cb : s_cb )
    {
      /*-----------------------------------------------------------------------
      Guard against interrupts we clearly did not generate.
      -----------------------------------------------------------------------*/
      if( !cb.tx_in_progress || ( cb.dma_tx_channel < 0 ) || ( cb.dma_tx_channel >= ( int )NUM_DMA_CHANNELS ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Check if the DMA channel is the one that completed. Assuming so, enable
      the UART TX interrupt to catch the FIFO threshold event. This will allow
      the UART time to empty the FIFO without blocking and then call the user's
      TX completion callback.
      -----------------------------------------------------------------------*/
      if( dma_channel_get_irq0_status( cb.dma_tx_channel ) )
      {
        dma_channel_acknowledge_irq0( cb.dma_tx_channel );
        uart_get_hw( cb.hw )->imsc |= 1u << UART_UARTIMSC_TXIM_LSB;
      }
    }
  }


  static void dma_rx_complete()
  {
  }

}    // namespace mb::hw::serial
