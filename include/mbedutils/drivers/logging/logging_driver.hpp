/******************************************************************************
 *  File Name:
 *    logging_driver.hpp
 *
 *  Description:
 *    Logging utilities interface. This presents a system wide logging driver
 *    capable of registering multiple sinks for output. The driver is thread
 *    safe as long as the underlying sinks are thread safe.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_LOGGING_DRIVER_HPP
#define MBEDUTILS_LOGGING_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/logging/logging_types.hpp>

namespace mb::logging
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the logging driver
   *
   * Must be called once at powerup to ensure system memory is
   * properly initialized and the driver is ready to use.
   */
  void initialize();

  /**
   * @brief Sets a minimum log threshold needed to emit messages to registered sinks
   *
   * @param level The global log level to be set
   * @return ErrCode
   */
  mb::logging::ErrCode setGlobalLogLevel( const mb::logging::Level level );

  /**
   * @brief Registers a sink with the back end driver
   *
   * @param sink The sink to be registered
   * @return ErrCode
   */
  mb::logging::ErrCode registerSink( mb::logging::SinkHandle_rPtr &sink );

  /**
   * @brief Removes the sink from the back end driver
   *
   * If nullptr is passed in, all sinks are removed.
   *
   * @param sink The sink that should be removed
   * @return ErrCode
   */
  mb::logging::ErrCode removeSink( mb::logging::SinkHandle_rPtr &sink );

  /**
   * @brief Sets the default global logger instance.
   *
   * This can be useful for systems that want to have a single output
   * sink for all logging messages, like dedicated console loggers or
   * remote logging servers.
   *
   * @param sink The sink to become the root
   * @return ErrCode
   */
  mb::logging::ErrCode setRootSink( mb::logging::SinkHandle_rPtr &sink );

  /**
   * @brief Gets the default global root logger instance.
   *
   * The actual sink can change over time depending on how many times
   * setRootSink() is called with a different sink handle.
   *
   * @return SinkHandle_rPtr
   */
  mb::logging::SinkHandle_rPtr getRootSink();

  /**
   * @brief Emits a log to every registered sink.
   *
   * @param lvl    The severity level of the message to be logged
   * @param msg    Raw byte message to be logged
   * @param length Length of the log message
   * @return ErrCode
   */
  mb::logging::ErrCode log( const mb::logging::Level lvl, const void *const msg, const size_t length );

  /**
   * @brief Emits a formatted string to every registered sink
   *
   * @param lvl  The severity level of the message to be logged
   * @param file Which file this is being logged from
   * @param line Line this is being logged from
   * @param fmt  Format string
   * @return ErrCode
   */
  mb::logging::ErrCode flog( const mb::logging::Level lvl, const char *const file, const size_t line, const char *fmt, ... );

}  // namespace mb::Logging

#endif  /* !MBEDUTILS_LOGGING_DRIVER_HPP */
