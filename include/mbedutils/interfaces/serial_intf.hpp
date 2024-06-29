/******************************************************************************
 *  File Name:
 *    serial_intf.hpp
 *
 *  Description:
 *    Minimalist serial interface for sending and receiving data. This really
 *    doesn't have a dependency on a particular HW interface like USART, UART,
 *    RS485, etc. It's just a way to send and receive a byte stream over a
 *    generic serial link.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_SERIAL_INTF_HPP
#define MBEDUTILS_SERIAL_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/delegate.h>
#include <mbedutils/config.hpp>

namespace mb::hw::serial::intf
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Callback function for when a TX operation completes.
   *
   * @param channel   Which UART channel the TX completed on
   * @param num_bytes Number of bytes that were transmitted
   */
  using TXCompleteCallback = etl::delegate<void( const size_t channel, const size_t num_bytes )>;

  /**
   * @brief Callback function for when an RX operation completes.
   *
   * @param channel   Which UART channel the RX completed on
   * @param num_bytes Number of bytes that were received
   */
  using RXCompleteCallback = etl::delegate<void( const size_t channel, const size_t num_bytes )>;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Acquires a lock on the specified UART channel.
   *
   * Intended use case is to allow drivers above this interface layer to
   * maintain ownership of the channel until the RX or TX operation is
   * completed. This is useful for ensuring that the channel is not accessed
   * by multiple threads simultaneously.
   *
   * @param channel Channel to lock
   * @param timeout How long to wait for the lock (ms)
   * @return bool True if the lock was acquired, false if timed out
   */
  bool lock( const size_t channel, const size_t timeout );

  /**
   * @brief Releases a lock on the specified UART channel.
   *
   * @param channel Channel to unlock
   */
  void unlock( const size_t channel );

  /**
   * @brief Asynchronously writes data to the specified UART channel.
   * @warning Implementations can be called from an ISR context.
   *
   * Implementation Notes:
   *  - The write should only proceed if the hardware is immediately ready to
   *    begin transmition. If the hardware is not ready, the function should
   *    return immediately with a negative value.
   *
   *  - Data must be contiguous in memory up to the length specified,
   *    as the backing implementation may be based on DMA or other
   *    hardware accelerated methods.
   *
   *  - Once the transmission is complete, the callback registered with
   *    on_tx_complete should be invoked so that any more data can be sent.
   *
   * @param channel Channel to write data to
   * @param data    Data to write. Must be at least "length" bytes long.
   * @param length  Number of bytes to write
   * @return int    Number of bytes actually written. Negative on error.
   */
  int async_write( const size_t channel, const void *data, const size_t length );

  /**
   * @brief Register a function to be invoked when the last TX completes.
   * @warning This may be called from an ISR context.
   *
   * This is useful for things like flow control or other mechanisms that
   * require knowing when the last byte has been sent.
   *
   * @param channel  The channel to register the callback for
   * @param callback The function to call when the TX completes
   */
  void on_tx_complete( const size_t channel, TXCompleteCallback callback );

  /**
   * @brief Reset mechanism to clear out any pending TX operations.
   *
   * This is useful for aborting a TX operation that is in progress if for some reason
   * the consuming driver gets stuck and needs to be reset.
   *
   * @param channel Which channel to reset
   */
  void abort_write( const size_t channel );

  /**
   * @brief Asynchronously reads data from the specified UART channel.
   *
   * Implementation Notes:
   *  - If the hardware is not immediately ready to begin receiving data, the
   *    function should return a negative value.
   *
   *  - Data must be contiguous in memory up to the length specified,
   *    as the backing implementation may be based on DMA or other
   *    hardware accelerated methods.
   *
   *  - Once the reception is complete (or timed out), the callback registered with
   *    on_rx_complete should be invoked so that more data can be received.
   *
   * @param channel Channel to read data from
   * @param data    Where to read data into. Must be at least "length" bytes long
   * @param length  Number of bytes to read.
   * @param timeout How long to wait for the read to complete (ms)
   * @return int    Number of bytes actually read. Negative on error.
   */
  int async_read( const size_t channel, void *data, const size_t length, const size_t timeout );

  /**
   * @brief Register a function to be invoked when the last RX completes.
   * @warning This may be called from an ISR context.
   *
   * This is useful for things like flow control or other mechanisms that
   * require knowing when the last byte has been received.
   *
   * @param channel  The channel to register the callback for
   * @param callback The function to call when the RX completes
   */
  void on_rx_complete( const size_t channel, RXCompleteCallback callback );

  /**
   * @brief Reset mechanism to clear out any pending RX operations.
   *
   * This is useful for aborting an RX operation that is in progress if for some reason
   * the consuming driver gets stuck and needs to be reset.
   *
   * @param channel Which channel to reset
   */
  void abort_read( const size_t channel );

}    // namespace mb::intf::uart

#endif /* !MBEDUTILS_SERIAL_INTF_HPP */
