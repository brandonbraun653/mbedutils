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
#include <mbedutils/interfaces/time_intf.hpp>
#include <mbedutils/interfaces/util_intf.hpp>
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "pico/time.h"

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
    size_t                   tx_bytes;                 /**< Number of bytes expected to transmit */
    size_t                   rx_expected;                 /**< Number of bytes expected to receive */
    size_t                   rx_actual;
    size_t rx_fifo_threshold; /**< RX FIFO threshold */
    uint8_t *     rx_buffer;               /**< User buffer for RX-ing */
    int                      dma_rx_channel;           /**< System dma channel for RX-ing */
    int                      dma_tx_channel;           /**< System dma channel for TX-ing */
    intf::RXCompleteCallback usr_rx_complete_callback; /**< RX completion callback */
    intf::TXCompleteCallback usr_tx_complete_callback; /**< TX completion callback */
    volatile bool            tx_in_progress;           /**< Flag to indicate if a TX operation is in progress */
    volatile bool            rx_in_progress;           /**< Flag to indicate if an RX operation is in progress */
    volatile alarm_id_t      rx_alarm;                 /**< RX timeout alarm ID */
    size_t                   rx_active_tick; /**< RX timeout in milliseconds */
    size_t rx_inactive_timeout; /**< RX timeout in milliseconds */

    void reset()
    {
      hw                       = nullptr;
      mutex                    = nullptr;
      usr_channel              = 0;
      tx_bytes                 = 0;
      rx_expected                 = 0;
      rx_fifo_threshold        = 0;
      rx_actual                = 0;
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
  static void          isr_dma_tx_complete();

  static int64_t rx_alarm_cb( alarm_id_t id, void *user_data );

  static bool flush_rx_fifo_to_user_buffer( ControlBlock *cb, bool use_dma );
  static void terminate_rx_operation( ControlBlock *cb, bool invoke_callback );

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
    Configure DMA handlers. We'll map TX to IRQ 0 and RX to IRQ 1.
    -------------------------------------------------------------------------*/
    irq_add_shared_handler( DMA_IRQ_0, isr_dma_tx_complete, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY );
    irq_set_enabled( DMA_IRQ_0, true );

    /*-------------------------------------------------------------------------
    Configure the UART peripheral
    -------------------------------------------------------------------------*/
    const uint actual_baud = uart_init( config.uart, config.baudrate );
    const float pct_error = ( ( float )actual_baud - ( float )config.baudrate ) / ( float )config.baudrate;
    mbed_assert_continue_msg( pct_error <= 0.03f, "UART %d baud rate error > 3%. Exp: %d, Act: %d",
                              config.usr_channel, config.baudrate, actual_baud );

    /* Enable the FIFO so we can catch when the transfer is complete */
    uart_set_fifo_enabled( config.uart, true );

    /* Simple serial. No flow control signals. */
    uart_set_hw_flow( config.uart, false, false );

    /* Configure the user's data formatting */
    uart_set_format( config.uart, config.data_bits, config.stop_bits, config.parity );

    /* Disable FIFO threshold IRQs initially */
    uart_set_irq_enables( config.uart, false, false );

    /* Set tx fifo trigger levels */
    uart_get_hw( config.uart )->ifls = UARTx_FIFO_THR_16B << UART_UARTIFLS_TXIFLSEL_LSB;
    uart_get_hw( config.uart )->ifls |= UARTx_FIFO_THR_16B << UART_UARTIFLS_RXIFLSEL_LSB;

    s_cb[ hw_idx ].rx_fifo_threshold = 16;

    /* Enable DMA requests: RX, TX */
    uart_get_hw( config.uart )->dmacr = UART_UARTDMACR_RXDMAE_BITS | UART_UARTDMACR_TXDMAE_BITS;

    /* Flush the RX fifo */
    while( uart_is_readable( config.uart ) )
    {
      uart_getc( config.uart );
    }

    /* Clear all possible interrupt states from configuration */
    uart_get_hw( config.uart )->icr = 0xFFFF;

    /*-------------------------------------------------------------------------
    Configure the TX and RX pins
    -------------------------------------------------------------------------*/
    gpio_set_function( config.tx_pin, GPIO_FUNC_UART );
    gpio_set_function( config.rx_pin, GPIO_FUNC_UART );

    /*-------------------------------------------------------------------------
    Configure the rest of the control block
    -------------------------------------------------------------------------*/
    s_cb[ hw_idx ].hw                       = config.uart;
    s_cb[ hw_idx ].usr_channel              = config.usr_channel;
    s_cb[ hw_idx ].tx_bytes                 = 0;
    s_cb[ hw_idx ].rx_expected                 = 0;
    s_cb[ hw_idx ].tx_in_progress           = false;
    s_cb[ hw_idx ].rx_in_progress           = false;
    s_cb[ hw_idx ].usr_rx_complete_callback = {};
    s_cb[ hw_idx ].usr_tx_complete_callback = {};

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
    /*-------------------------------------------------------------------------
    Input Sanitization
    -------------------------------------------------------------------------*/
    if( !data || !length || ( s_driver_initialized != mb::DRIVER_INITIALIZED_KEY ) )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Hopefully we have some DMA channels left. Only acquire right now since we
    just now learned we need it.
    -------------------------------------------------------------------------*/
    auto cb = get_cb( channel );
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
      mbed_assert_continue_msg( !mb::irq::in_isr(), "Failed to send %d bytes on channel %d", length, cb->usr_channel );
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
    auto cb = get_cb( channel );
    if( !cb->tx_in_progress )
    {
      return;
    }

    cb->tx_in_progress = false;
    if( cb->dma_tx_channel >= 0 )
    {
      dma_channel_set_irq0_enabled( cb->dma_tx_channel, false );
      dma_channel_abort( cb->dma_tx_channel );
      dma_channel_acknowledge_irq0( cb->dma_tx_channel );
      dma_channel_set_irq0_enabled( cb->dma_tx_channel, true );
    }
  }


  int intf::read_async( const size_t channel, void *data, const size_t length, const size_t timeout )
  {
    /*-------------------------------------------------------------------------
    Input Sanitization
    -------------------------------------------------------------------------*/
    if( !data || !length || !timeout || ( s_driver_initialized != mb::DRIVER_INITIALIZED_KEY ) )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Hopefully we have some DMA channels left. Only acquire right now since we
    just now learned we need it.
    -------------------------------------------------------------------------*/
    auto cb = get_cb( channel );
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

    /*-------------------------------------------------------------------------
    Ensure the driver is ready for immediate RX
    -------------------------------------------------------------------------*/
    if( cb->rx_in_progress || dma_channel_is_busy( cb->dma_rx_channel ) )
    {
      mbed_assert_continue_msg( !mb::irq::in_isr(), "Failed to start reading channel %d", cb->usr_channel );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Set up a timer to expire if we don't receive all the bytes in time
    -------------------------------------------------------------------------*/
    cb->rx_alarm = add_alarm_in_ms( timeout, rx_alarm_cb, cb, true );
    if( cb->rx_alarm < 0 )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Update the control block with the new transfer information
    -------------------------------------------------------------------------*/
    cb->rx_buffer           = reinterpret_cast<uint8_t *>( data );
    cb->rx_expected         = length;
    cb->rx_actual           = 0;
    cb->rx_in_progress      = true;
    cb->rx_active_tick      = mb::time::millis();
    cb->rx_inactive_timeout = timeout;

    /*-------------------------------------------------------------------------
    Read as many bytes from the FIFO that we can before listening for more
    -------------------------------------------------------------------------*/
    if( flush_rx_fifo_to_user_buffer( cb, false ) )
    {
      terminate_rx_operation( cb, false );
      return length;
    }

    /*-------------------------------------------------------------------------
    Start listening for data. This will fire an interrupt when the RX FIFO
    hits the threshold or the line idle timeout occurs with data in the FIFO.
    -------------------------------------------------------------------------*/
    uart_get_hw( cb->hw )->imsc = ( 1u << UART_UARTIMSC_RXIM_LSB ) | ( 1u << UART_UARTIMSC_RTIM_LSB );

    return 0;
  }


  void intf::on_rx_complete( const size_t channel, intf::RXCompleteCallback callback )
  {
    get_cb( channel )->usr_rx_complete_callback = callback;
  }


  void intf::read_abort( const size_t channel )
  {
    auto cb = get_cb( channel );
    if( !cb->rx_in_progress )
    {
      return;
    }

    dma_channel_wait_for_finish_blocking( cb->dma_rx_channel );
    terminate_rx_operation( cb, true );
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

    const bool tx_event         = ( enabled & UART_UARTIMSC_TXIM_BITS ) && ( status & UART_UARTRIS_TXRIS_BITS );
    const bool rx_event         = ( enabled & UART_UARTIMSC_RXIM_BITS ) && ( status & UART_UARTRIS_RXRIS_BITS );
    const bool rx_timeout_event = ( enabled & UART_UARTIMSC_RTIM_BITS ) && ( status & UART_UARTRIS_RTRIS_BITS );

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
    if( cb->tx_in_progress && tx_event )
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

      // TODO: Handle TX errors
    }

    /*-------------------------------------------------------------------------
    Handle receive events. No data moves unless the RX fifo hits its threshold
    or the RX timeout occurs while data is present in the FIFO.
    -------------------------------------------------------------------------*/
    if( cb->rx_in_progress && ( rx_event || rx_timeout_event ) && uart_is_readable( cb->hw ) )
    {
      cb->rx_active_tick = mb::time::millis();

      /*-----------------------------------------------------------------------
      Filled the fifo. Flush it with DMA. If the flush resulted in filling
      the buffer, wait for DMA to finish, then terminate the operation.
      -----------------------------------------------------------------------*/
      if( rx_event && flush_rx_fifo_to_user_buffer( cb, true ) )
      {
        dma_channel_wait_for_finish_blocking( cb->dma_rx_channel );
        terminate_rx_operation( cb, true );
      }

      /*-----------------------------------------------------------------------
      Line idle timeout. This is the termination point for the RX operation.
      -----------------------------------------------------------------------*/
      if( rx_timeout_event )
      {
        flush_rx_fifo_to_user_buffer( cb, false );
        terminate_rx_operation( cb, true );
      }

      // TODO: Handle RX errors
    }
  }


  /**
   * @brief Handle the DMA TX completion event
   */
  static void isr_dma_tx_complete()
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


  /**
   * @brief Timeout alarm for when the RX transfer takes too long.
   *
   * @param id Alarm identifier from registration
   * @param user_data Control block for the UART peripheral
   * @return int64_t
   */
  static int64_t rx_alarm_cb( alarm_id_t id, void *user_data )
  {
    /*-------------------------------------------------------------------------
    Don't reschedule the alarm if the RX operation has been terminated.
    -------------------------------------------------------------------------*/
    ControlBlock *cb = static_cast<ControlBlock *>( user_data );
    if( !cb->rx_in_progress || ( id != cb->rx_alarm ) )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    There's been some RX activity? Reschedule a new timeout window.
    -------------------------------------------------------------------------*/
    if( ( mb::time::millis() - cb->rx_active_tick ) < cb->rx_inactive_timeout )
    {
      return cb->rx_inactive_timeout;
    }

    /*-------------------------------------------------------------------------
    Otherwise, we've timed out, likely without receiving any data.
    -------------------------------------------------------------------------*/
    flush_rx_fifo_to_user_buffer( cb, false );
    terminate_rx_operation( cb, true );
    return 0;
  }


  /**
   * @brief Flush data to the user buffer from the RX FIFO
   *
   * @param cb      Control block for the UART peripheral
   * @param use_dma Should we use DMA to read the RX FIFO?
   * @return bool   True if the buffer was filled, false otherwise
   */
  static bool flush_rx_fifo_to_user_buffer( ControlBlock *cb, bool use_dma )
  {
    /*-------------------------------------------------------------------------
    Wait for DMA to finish. It's possible we got here because the RX FIFO
    hit the threshold and we're trying to flush it, THEN a timeout occurred in
    the middle of the flush. We need to allow the DMA to empty the FIFO to the
    appropriate level before we start pulling bytes out manually.
    -------------------------------------------------------------------------*/
    dma_channel_wait_for_finish_blocking( cb->dma_rx_channel );

    if( use_dma )
    {
      /*-----------------------------------------------------------------------
      DMA is the most efficient way to read the RX FIFO. Read as much as we can
      into the user buffer, but don't exceed the threshold. We need to leave at
      least one byte in the FIFO to accurately detect a line idle event.
      -----------------------------------------------------------------------*/
      uint8_t *write_address = cb->rx_buffer + cb->rx_actual;
      size_t   write_length  = cb->rx_expected - cb->rx_actual;

      if( write_length > cb->rx_fifo_threshold )
      {
        write_length = cb->rx_fifo_threshold - 1;
      }

      mbed_dbg_assert( ( write_address + write_length ) <= ( cb->rx_buffer + cb->rx_expected ) );
      cb->rx_actual += write_length;

      /*-----------------------------------------------------------------------
      Configure the DMA channel to read the RX FIFO
      -----------------------------------------------------------------------*/
      auto dma_cfg = dma_channel_get_default_config( cb->dma_rx_channel );
      channel_config_set_transfer_data_size( &dma_cfg, DMA_SIZE_8 );
      channel_config_set_read_increment( &dma_cfg, false );
      channel_config_set_write_increment( &dma_cfg, true );
      channel_config_set_dreq( &dma_cfg, uart_get_dreq( cb->hw, false ) );
      channel_config_set_irq_quiet( &dma_cfg, true );

      dma_channel_configure( cb->dma_rx_channel, &dma_cfg, write_address, &uart_get_hw( cb->hw )->dr, write_length, true );
    }
    else
    {
      /*-----------------------------------------------------------------------
      Manually read the RX FIFO. This is less efficient than DMA, but is
      sufficient when a timeout occurs and we don't know the state of the FIFO.
      -----------------------------------------------------------------------*/
      while( uart_is_readable( cb->hw ) && ( cb->rx_actual < cb->rx_expected ) )
      {
        cb->rx_buffer[ cb->rx_actual++ ] = ( uint8_t )uart_get_hw( cb->hw )->dr;
      }
    }

    mbed_dbg_assert( cb->rx_actual <= cb->rx_expected );
    return cb->rx_actual == cb->rx_expected;
  }

  /**
   * @brief Terminate the RX operation and invoke the user's RX completion callback
   *
   * @param cb             Control block for the UART peripheral
   * @param invoke_callback If true, the user's RX completion callback will be invoked
   */
  void terminate_rx_operation( ControlBlock *cb, bool invoke_callback )
  {
    cancel_alarm( cb->rx_alarm );
    cb->rx_in_progress = false;
    uart_get_hw( cb->hw )->imsc &= ~( ( 1u << UART_UARTIMSC_RXIM_LSB ) | ( 1u << UART_UARTIMSC_RTIM_LSB ) );

    if( invoke_callback && cb->usr_rx_complete_callback )
    {
      cb->usr_rx_complete_callback( cb->usr_channel, cb->rx_actual );
    }
  }
}    // namespace mb::hw::serial
