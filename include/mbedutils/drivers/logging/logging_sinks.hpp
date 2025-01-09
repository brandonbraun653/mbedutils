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
#include <etl/string.h>
#include <flashdb.h>
#include <mbedutils/drivers/hardware/serial.hpp>
#include <mbedutils/drivers/logging/logging_types.hpp>
#include <mbedutils/drivers/threading/lock.hpp>

namespace mb::logging
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Callback for reading a log entry from the logger.
   * @see https://github.com/armink/FlashDB/blob/master/samples/tsdb_sample.c
   *
   * Use the examples in the link above to understand context of this callback.
   *
   * @param log     Pointer to the log entry storage
   * @param length  Length of the log entry, zero if no more logs
   * @return bool   True to terminate the read, false to read the next log
   */
  using LogReader = etl::delegate<bool( const void *const message, const size_t length )>;

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
  class SinkInterface : public thread::Lockable<SinkInterface>
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
     * @brief Erases all data from the sink
     *
     * @return ErrCode
     */
    virtual ErrCode erase() = 0;

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
    virtual ErrCode write( const Level level, const void *const message, const size_t length ) = 0;

    /**
     * @brief Extremely simple reader to iterate over log entries.
     *
     * This will iterate the entire database and call the visitor function for each log entry.
     * It is up to the visitor to determine if the log entry is of interest or if the iteration
     * should be terminated early. See the LogReader delegate for more information.
     *
     * @param visitor Function to call for each log entry
     * @param direction Direction to read the logs, true = oldest to newest, false = newest to oldest
     */
    virtual void read( LogReader visitor, const bool direction = false ) = 0;

  protected:
    friend thread::Lockable<SinkInterface>;
  };

  /**
   * @brief Logs messages to the standard output via std::cout
   * @warning This should not be used in a bare-metal environment.
   */
  class ConsoleSink : public SinkInterface
  {
  public:
    ConsoleSink()  = default;
    ~ConsoleSink() = default;
    ErrCode open() final override;
    ErrCode close() final override;
    ErrCode flush() final override;
    ErrCode write( const Level level, const void *const message, const size_t length ) final override;

    ErrCode erase() final override
    {
      return ErrCode::ERR_OK;
    };

    void read( LogReader visitor, const bool direction = false ) final override
    {
      ( void )visitor;
      ( void )direction;
    };
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
    ErrCode write( const Level level, const void *const message, const size_t length ) final override;
    ErrCode erase() final override;
    void    read( LogReader visitor, const bool direction = false ) final override;

    /**
     * @brief Assigns the file to write logs to
     *
     * @param file Path to the file to create or append to
     */
    void setFile( const etl::string_view &file );

  private:
    etl::string<128> mFile;
    int              mFileHandle;
  };

  /**
   * @brief Log raw messages to a serial channel
   *
   * This is intended for use in a bare-metal environment where the typical
   * logging mechanism is some kind of serial interface.
   */
  class SerialSink : public SinkInterface
  {
  public:
    SerialSink();
    ~SerialSink() = default;
    ErrCode open() final override;
    ErrCode close() final override;
    ErrCode flush() final override;
    ErrCode write( const Level level, const void *const message, const size_t length ) final override;

    ErrCode erase() final override
    {
      return ErrCode::ERR_OK;
    };

    void read( LogReader visitor, const bool direction = false ) final override
    {
      ( void )visitor;
      ( void )direction;
    };

    /**
     * @brief Assigns the serial driver to use for logging
     *
     * @param serial Driver to attach for logging
     */
    void assignDriver( ::mb::hw::serial::ISerial &serial );

  private:
    ::mb::hw::serial::ISerial *mSerial; /**< Driver for logging messages */
  };

  /**
   * @brief Time Series Database Sink, built on top of FlashDB.
   *
   * Stores logs in a persistent database that can be queried at a later time. This sink
   * may be used in either sim/realtime environments with a non-volatile backend.
   */
  class TSDBSink : public SinkInterface
  {
  public:
    using FALString = etl::string<FAL_DEV_NAME_MAX>;

    /**
     * @brief Configuration data for the NVM based KVDB
     *
     * This should be built via binding to the Storage struct members.
     */
    struct Config
    {
      FALString dev_name;      /**< Which device is being accessed */
      FALString part_name;     /**< Sub-partition being accessed */
      uint32_t  max_log_size;  /**< Maximum size of a single log entry */
      uint8_t  *reader_buffer; /**< (Optional) Buffer for reading log entries, sized at max_log_size */
    };

    TSDBSink();
    ~TSDBSink() = default;
    ErrCode open() final override;
    ErrCode close() final override;
    ErrCode flush() final override;
    ErrCode write( const Level level, const void *const message, const size_t length ) final override;
    ErrCode erase() final override;
    void    read( LogReader visitor, const bool direction = false ) final override;

    /**
     * @brief Configures the resources used for the sink
     *
     * @param config Configuration structure for the sink
     */
    void configure( const Config &config );

  private:
    bool     mInitialized;      /**< Has the sink been initialized? */
    fdb_tsdb mDB;               /**< Timeseries DB control structure */
    alignas( 4 ) uint32_t _pad; /**< See db_kv_mgr.hpp for explanation on why this is here */
    uint8_t *mReaderBuffer;     /**< Buffer for reading log entries into */
  };
}    // namespace mb::logging

#endif /* !MBEDUTILS_LOGGING_SINK_INTERFACE_HPP */
