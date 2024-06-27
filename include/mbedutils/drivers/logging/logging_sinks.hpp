/******************************************************************************
 *  File Name:
 *    logging_sinks.hpp
 *
 *  Description:
 *    Available sinks that may be used for logging
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_LOGGING_SINK_INTERFACE_HPP
#define MBEDUTILS_LOGGING_SINK_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/string.h>
#include <mbedutils/drivers/logging/logging_types.hpp>
#include <mbedutils/drivers/threading/mutex_extensions.hpp>

namespace mbedutils::logging
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Defines the interface expected for all logging sinks
   *
   * Each sink is capable of being thread safe due to inheriting from the
   * Lockable class. The logging driver will handle all locking/unlocking
   * operations for registered sinks, but any direct consumers of a sink
   * will need to manually acquire exclusive access.
   */
  class SinkInterface : public osal::Lockable<SinkInterface>
  {
  public:
    bool  enabled;  /**< Is this sink enabled?  */
    Level logLevel; /**< Current minimum log level the sink will accept */

    SinkInterface() : enabled( false ), logLevel( Level::LVL_MAX )
    {
    }

    virtual ~SinkInterface() = default;

    /**
     * @brief Opens the sink for use
     *
     * @return ErrCode
     */
    virtual ErrCode open() = 0;

    /**
     * @brief Closes the sink and releases any resources
     *
     * @return ErrCode
     */
    virtual ErrCode close() = 0;

    /**
     * @brief Flushes any pending data to the sink
     *
     * @return ErrCode
     */
    virtual ErrCode flush() = 0;

    /**
     * @brief Provides the core functionality of the sink by logging messages.
     *
     * @note   Assume the memory can be modified/destroyed after return
     *
     * @param level     The log level the message was sent at
     * @param message   The message to be logged. Can be any kind of data
     * @param length    How large the message is in bytes
     * @return ErrCode  Whether or not the logging action succeeded
     */
    virtual ErrCode insert( const Level level, const void *const message, const size_t length ) = 0;

  protected:
    friend osal::Lockable<SinkInterface>;
  };

  /**
   * @brief Logs messages to the standard output via std::cout
   * @warning This should not be used in a bare-metal environment.
   */
  class ConsoleSink : public SinkInterface
  {
  public:
    ConsoleSink() = default;
    ~ConsoleSink() = default;
    ErrCode open() final override;
    ErrCode close() final override;
    ErrCode flush() final override;
    ErrCode insert( const Level level, const void *const message, const size_t length ) final override;
  };

  /**
   * @brief Logs messages to a file using the standard C++ file I/O layer.
   * @warning This should not be used in a bare-metal environment.
   */
  class STLFileSink : public SinkInterface
  {
  public:
    STLFileSink();
    ~STLFileSink();
    ErrCode open() final override;
    ErrCode close() final override;
    ErrCode flush() final override;
    ErrCode insert( const Level level, const void *const message, const size_t length ) final override;

    /**
     * @brief Assigns the file to write logs to
     *
     * @param file Path to the file to create or append to
     */
    void setFile( const etl::string_view &file );

  private:
    etl::string<128> mFile;
    int mFileHandle;
  };

  /**
   * @brief Log messages to a serial channel using the mbedutils UART driver.
   *
   * This is intended for use in a bare-metal environment where the typical
   * logging mechanism is some kind of serial interface.
   */
  class UARTSink : public SinkInterface
  {
  public:
    UARTSink();
    ~UARTSink() = default;
    ErrCode open() final override;
    ErrCode close() final override;
    ErrCode flush() final override;
    ErrCode insert( const Level level, const void *const message, const size_t length ) final override;

    /**
     * @brief Assigns the serial channel to use for logging
     *
     * @param channel Which channel to hook into
     */
    void assignChannel( const size_t channel );

  private:
    size_t mChannel;
  };
}  // namespace mbedutils::logging

#endif  /* !MBEDUTILS_LOGGING_SINK_INTERFACE_HPP */
