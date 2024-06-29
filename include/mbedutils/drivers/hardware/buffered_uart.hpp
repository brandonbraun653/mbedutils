/******************************************************************************
 *  File Name:
 *    buffered_uart.hpp
 *
 *  Description:
 *    Interface for a buffered serial driver that can be used to send and
 *    receive data from a UART perihperal without having to worry about buffer
 *    management.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_HARDWARE_BUFFERED_UART_HPP
#define MBEDUTILS_HARDWARE_BUFFERED_UART_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/hardware/serial_intf.hpp>

namespace mb::hw::uart
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class BufferedUART : public serial::AbstractHWInterface
  {
  public:
    BufferedUART();
    ~BufferedUART() = default;
    int write( const void *const buffer, const size_t length, const size_t timeout ) final override;
    int read( void *const buffer, const size_t length, const size_t timeout ) final override;

    /**
     * @brief Prepares the serial interface for operation.
     *
     * This does not configure the real HW (that's left up to the user), but
     * rather prepares the internal buffers and other resources for operation.
     *
     * @param config Desired operating parameters
     * @return int Zero on success, negative on error
     */
    int configure( const ::mb::hw::serial::Config &config );

  private:
    ::mb::hw::serial::Config mConfig;
  };
}  // namespace mb::hw::uart

#endif  /* !MBEDUTILS_HARDWARE_BUFFERED_UART_HPP */
