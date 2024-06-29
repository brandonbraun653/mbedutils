/******************************************************************************
 *  File Name:
 *    buffered_uart.hpp
 *
 *  Description:
 *    Interface for a buffered serial driver that can be used to send and
 *    receive data from a serial perihperal without worrying about buffer
 *    management.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_HARDWARE_SERIAL_HPP
#define MBEDUTILS_HARDWARE_SERIAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>
#include <etl/span.h>
#include <etl/bip_buffer_spsc_atomic.h>
#include <mbedutils/drivers/threading/extensions.hpp>

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

  /**
   * @brief Default configuration data for a serial driver
   */
  struct Config
  {
    size_t     channel;  /**< Hardware channel to interface with */
    BipBuffer *rxBuffer; /**< Receive buffer for the HW */
    BipBuffer *txBuffer; /**< Transmit buffer for the HW */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Very basic interface that all serial data links use
   */
  class ISerial : public virtual mb::osal::LockableInterface, public virtual mb::osal::AsyncIOInterface
  {
  public:
    virtual ~ISerial() = default;

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


  /**
   * @brief A buffered serial driver that can be used to send and receive data.
   * @note This consumes the given serial channel and never releases it until the destructor is called.
   *
   * This driver couples with the mb::hw::serial::intf functions to provide an asynchronous
   * buffered interface for sending and receiving data over a serial link. The driver
   * assumes that the underlying hardware is configured and ready to go.
   *
   * If you need thread safety, be sure to take advantage of the Lockable interface as
   * the driver is not thread safe by default.
   */
  class SerialDriver : public ISerial, public mb::osal::Lockable<SerialDriver>, public mb::osal::AsyncIO<SerialDriver>
  {
  public:
    SerialDriver();
    ~SerialDriver();
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
    friend class ::mb::osal::Lockable<SerialDriver>;
    friend class ::mb::osal::AsyncIO<SerialDriver>;

    /**
     * @brief State machine states for the UART operation
     */
    enum class State : size_t
    {
      READY,
      IN_PROGRESS,
      ABORTED,
      COMPLETE
    };

    /**
     * @brief Control block for managing the transfer state
     *
     * This is used to keep track of the current state of the transfer operation
     * and how many bytes are left to transmit/recieve. The definitions change
     * slightly depending on if the operation is a read or write.
     */
    struct TransferControl
    {
      State              state;     /**< Current transfer state */
      size_t             remaining; /**< Number of bytes left to transfer */
      size_t             expected;  /**< Number of bytes expected to transmit/recieve */
      etl::span<uint8_t> buffer;    /**< Data buffer to transfer into/outof */
    };

    bool                     mIsConfigured;
    ::mb::hw::serial::Config mConfig;
    TransferControl          mTXControl;
    TransferControl          mRXControl;
  };
}    // namespace mb::hw::serial

#endif /* !MBEDUTILS_HARDWARE_SERIAL_HPP */
