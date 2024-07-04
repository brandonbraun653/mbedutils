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
#include <etl/delegate.h>
#include <mbedutils/drivers/threading/lock.hpp>
#include <mbedutils/drivers/threading/asyncio.hpp>

namespace mb::hw::serial
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Bipartite buffer, suitable for DMA hardware transactions
   */
  using BipBuffer = etl::ibip_buffer_spsc_atomic<uint8_t>;

  /**
   * @brief Generic callback for when a serial operation completes
   */
  using CompletionCallback = etl::delegate<void()>;

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
  Interfaces
  ---------------------------------------------------------------------------*/

  /**
   * @brief Very basic interface that all serial data links use
   */
  class ISerial : public virtual mb::thread::LockableInterface, public virtual mb::thread::AsyncIOInterface
  {
  public:
    virtual ~ISerial() = default;

    /**
     * @brief Writes data onto the wire
     *
     * @param buffer  Buffer to write
     * @param length  Number of bytes to write from the buffer
     * @return int    Number of bytes actually written, negative on error
     */
    virtual int write( const void *const buffer, const size_t length ) = 0;

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

    /**
     * @brief Determines how many bytes are available to read
     *
     * @return size_t  Number of bytes available
     */
    virtual size_t readable() = 0;

    /**
     * @brief Registers a callback to be invoked when a write operation completes
     * @warning This may be invoked from an ISR.
     *
     * @param callback  Callback to invoke
     */
    virtual void onWriteComplete( CompletionCallback callback ) = 0;

    /**
     * @brief Registers a callback to be invoked when a read operation completes
     * @warning This may be invoked from an ISR.
     *
     * @param callback  Callback to invoke
     */
    virtual void onReadComplete( CompletionCallback callback ) = 0;
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief A buffered serial driver that can be used to send and receive data.
   *
   * This driver couples with the serial::intf functions to provide an
   * asynchronous buffered interface for sending and receiving data over a
   * serial link. All hardware configuration must be handled by user code.
   *
   * This class is thread and ISR safe, assuming the underlying hardware driver
   * is written properly.
   */
  class SerialDriver : public ISerial, public mb::thread::Lockable<SerialDriver>, public mb::thread::AsyncIO<SerialDriver, 2>
  {
  public:
    SerialDriver();
    ~SerialDriver();
    int    write( const void *const buffer, const size_t length ) final override;
    int    read( void *const buffer, const size_t length, const size_t timeout ) final override;
    size_t readable() final override;
    void   onWriteComplete( CompletionCallback callback ) final override;
    void   onReadComplete( CompletionCallback callback ) final override;

    /**
     * @brief Prepares the serial interface for operation.
     * @warning Assumes exclusive ownership of the serial channel until close() is called.
     *
     * This does not configure the real HW (that's left up to the user), but
     * rather prepares the internal buffers and other resources for operation.
     *
     * @param config Desired operating parameters
     * @return int Zero on success, negative on error
     */
    int open( const ::mb::hw::serial::Config &config );

    /**
     * @brief Closes the serial interface, preventing further operation.
     *
     * Must call open() again before using.
     */
    void close();

  private:
    friend class ::mb::thread::Lockable<SerialDriver>;
    friend class ::mb::thread::AsyncIO<SerialDriver>;

    /**
     * @brief Control block for managing the transfer state
     */
    struct TransferControl
    {
      bool               in_progress; /**< Current transfer state */
      etl::span<uint8_t> buffer;      /**< Data buffer to transfer into/outof */
      CompletionCallback callback;    /**< Callback to invoke when the transfer completes */
    };

    bool                     mIsConfigured;
    ::mb::hw::serial::Config mConfig;
    TransferControl          mTXControl;
    TransferControl          mRXControl;

    void on_tx_complete_callback( const size_t channel, const size_t num_bytes );
    void on_rx_complete_callback( const size_t channel, const size_t num_bytes );
  };
}    // namespace mb::hw::serial

#endif /* !MBEDUTILS_HARDWARE_SERIAL_HPP */
