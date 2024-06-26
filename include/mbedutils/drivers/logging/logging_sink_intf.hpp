/******************************************************************************
 *  File Name:
 *    logging_sink_intf.hpp
 *
 *  Description:
 *    Interface describing what a sink should provide
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_LOGGING_SINK_INTERFACE_HPP
#define MBEDUTILS_LOGGING_SINK_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/string.h>
#include <mbedutils/drivers/logging/logging_types.hpp>
#include <mbedutils/drivers/threading/mutex_extensions.hpp>

namespace mbedutils::logging
{
  class SinkInterface : public osal::Lockable<SinkInterface>
  {
  public:
    bool enabled;
    Level logLevel;
    etl::string<32> name;

    SinkInterface() : enabled( false ), logLevel( Level::LVL_MAX ), name( "" )
    {
    }

    virtual ~SinkInterface() = default;

    virtual ErrCode open() = 0;
    virtual ErrCode close() = 0;
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
    virtual ErrCode log( const Level level, const void *const message, const size_t length ) = 0;

  private:
    friend osal::Lockable<SinkInterface>;
  };
}  // namespace mbedutils::logging

#endif  /* !MBEDUTILS_LOGGING_SINK_INTERFACE_HPP */
