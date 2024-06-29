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
#include <mbedutils/interfaces/serial_intf.hpp>
#include <mbedutils/drivers/hardware/pico/pico_serial.hpp>

namespace mb::hw::serial
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct ControlBlock
  {
    uart_inst_t *uart;        /**< Pico SDK UART peripheral instance */
    size_t       usr_channel; /**< User logical channel, not instance index */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  /**
   * @brief Hardware state information for each UART peripheral
   */
  static etl::array<ControlBlock, NUM_UARTS> s_cb;

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Get's the UART peripheral instance for the given channel.
   *
   * @param channel  User facing channel number
   * @return uart_inst_t*
   */
  static uart_inst_t *get_uart( const size_t channel )
  {
    for( auto &cb : s_cb )
    {
      if( cb.usr_channel == channel )
      {
        return cb.uart;
      }
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void pico::initialize()
  {
  }


  void pico::configure( const pico::UartConfig &config )
  {
  }


  bool intf::lock( const size_t channel, const size_t timeout )
  {
    return false;
  }


  void intf::unlock( const size_t channel )
  {
  }


  int intf::async_write( const size_t channel, const void *data, const size_t length )
  {
    return -1;
  }


  void intf::on_tx_complete( const size_t channel, intf::TXCompleteCallback callback )
  {
  }


  void intf::abort_write( const size_t channel )
  {
  }


  int intf::async_read( const size_t channel, void *data, const size_t length, const size_t timeout )
  {
    return -1;
  }


  void intf::on_rx_complete( const size_t channel, intf::RXCompleteCallback callback )
  {
  }


  void intf::abort_read( const size_t channel )
  {
  }

}    // namespace mb::hw::serial
