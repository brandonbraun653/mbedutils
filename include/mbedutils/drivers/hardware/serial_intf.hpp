/******************************************************************************
 *  File Name:
 *    serial_intf.hpp
 *
 *  Description:
 *    Virtual interface describing serial communication capabilities
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
#include <etl/bip_buffer_spsc_atomic.h>

namespace mb::hw::serial
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Bipartite buffer, suitable for DMA hardware transactions
   */
  using BipBuffer = etl::ibip_buffer_spsc_atomic<uint8_t>;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct Config
  {
    size_t      channel;  /**< Hardware channel to interface with */
    BipBuffer * rxBuffer; /**< Receive buffer for the HW */
    BipBuffer * txBuffer; /**< Transmit buffer for the HW */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Very basic interface that all serial data links use
   */
  class AbstractHWInterface
  {
  public:
    virtual ~AbstractHWInterface() = default;

    /**
     * @brief Writes data onto the wire
     *
     * @param buffer  Buffer to write
     * @param length  Number of bytes to write from the buffer
     * @param timeout Total time the transaction may take to occur in milliseconds
     * @return int    Number of bytes actually written, negative on error
     */
    virtual int write( const void *const buffer, const size_t length, const size_t timeout ) = 0;

    /**
     * @brief Read a number of bytes from the wire
     *
     * This will read from internal IO buffers.
     *
     * @param buffer  Buffer to read into
     * @param length  Number of bytes to read
     * @param timeout Total time the transaction may take to occur in milliseconds
     * @return int    Number of bytes actually read, negative on error
     */
    virtual int read( void *const buffer, const size_t length, const size_t timeout ) = 0;
  };

}  // namespace mb::hw::serial

#endif  /* !MBEDUTILS_SERIAL_INTF_HPP */
