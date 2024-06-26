/******************************************************************************
 *  File Name:
 *    uart_intf.hpp
 *
 *  Description:
 *    Minimalist UART interface for sending and receiving data.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_UART_INTF_HPP
#define MBEDUTILS_UART_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/config.hpp>
#include <cstddef>
#include <cstdint>

namespace mbedutils::intf::uart
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Writes data to the specified UART channel.
   *
   * Data must be contiguous in memory as the backing implementation may
   * be based on DMA or other hardware accelerated methods. If buffered, the
   * entire data frame must be able to be written in a single operation.
   *
   * The function should only return once the data has been successfully
   * written to the wire, or fully buffered for later transmission.
   *
   * @param channel Channel to write data to
   * @param data Data to write. Must be at least "length" bytes long
   * @param length Number of bytes to write
   * @return int Number of bytes actually written. Negative on error.
   */
  int write( const size_t channel, const uint8_t *data, const size_t length );

  /**
   * @brief Checks to see if there is space available to write data.
   *
   * @param channel Which channel to check for space
   * @return size_t Number of bytes available to write
   */
  size_t tx_available( const size_t channel );

  /**
   * @brief Reads data from the specified UART channel.
   *
   * Data must be contiguous in memory as the backing implementation may
   * be based on DMA or other hardware accelerated methods. If all the
   * requested data is not available, the function should read what it
   * can and return that amount immediately.
   *
   * @param channel Channel to read data from
   * @param data Where to read data into
   * @param length Number of bytes to read
   * @return int Number of bytes actually read. Negative on error.
   */
  int read( const size_t channel, uint8_t *data, const size_t length );

  /**
   * @brief Checks to see if there is data available to read.
   *
   * @param channel Channel to check for data
   * @return size_t Number of bytes available to read
   */
  size_t rx_available( const size_t channel );

}    // namespace mbedutils::intf::uart

#endif /* !MBEDUTILS_UART_INTF_HPP */
