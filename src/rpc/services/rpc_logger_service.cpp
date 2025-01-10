/******************************************************************************
 *  File Name:
 *    rpc_logger_service.cpp
 *
 *  Description:
 *    RPC service for handling logger operations
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "mbed_rpc.pb.h"
#include "mbedutils/assert.hpp"
#include "mbedutils/drivers/logging/logging_types.hpp"
#include "mbedutils/drivers/threading/thread.hpp"
#include "mbedutils/interfaces/util_intf.hpp"
#include <etl/unordered_map.h>
#include <mbedutils/drivers/rpc/rpc_logger_service.hpp>
#include <mbedutils/config.hpp>

/*-----------------------------------------------------------------------------
Configuration
-----------------------------------------------------------------------------*/

#if !defined( MBEDUTILS_RPC_LOGGER_SERVICE_MAX_LOGGERS )
/**
 * @brief Number of loggers that may be bound to the service
 */
#define MBEDUTILS_RPC_LOGGER_SERVICE_MAX_LOGGERS ( 1 )
#endif

namespace mb::rpc::service::logger
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using LoggerMap = etl::unordered_map<uint8_t, logging::SinkHandle_rPtr, MBEDUTILS_RPC_LOGGER_SERVICE_MAX_LOGGERS>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static size_t    s_initialized;
  static LoggerMap s_loggers;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    s_loggers.clear();
    s_initialized = mb::DRIVER_INITIALIZED_KEY;
  }


  bool bind( const uint8_t id, logging::SinkHandle_rPtr log )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( ( s_initialized != mb::DRIVER_INITIALIZED_KEY ) || s_loggers.full() )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Bind the logger
    -------------------------------------------------------------------------*/
    s_loggers.insert( { id, log } );
    return true;
  }


  /*---------------------------------------------------------------------------
  Erase Service
  ---------------------------------------------------------------------------*/

  ErrId EraseService::processRequest()
  {
    /*-------------------------------------------------------------------------
    Find a logger maching the request ID, then erase it.
    -------------------------------------------------------------------------*/
    response.success = false;

    auto logger = s_loggers.find( request.which );
    if( logger != s_loggers.end() )
    {
      response.success = ( logger->second->erase() == logging::ErrCode::ERR_OK );
    }

    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }


  /*---------------------------------------------------------------------------
  Write Service
  ---------------------------------------------------------------------------*/

  ErrId WriteService::processRequest()
  {
    /*---------------------------------------------------------------------------
    Convert the request level to the appropriate enum
    ---------------------------------------------------------------------------*/
    logging::Level level;
    switch( request.level )
    {
      case mbed_rpc_LoggerWriteRequest_Level_LEVEL_TRACE:
        level = logging::Level::LVL_TRACE;
        break;

      case mbed_rpc_LoggerWriteRequest_Level_LEVEL_DEBUG:
        level = logging::Level::LVL_DEBUG;
        break;

      case mbed_rpc_LoggerWriteRequest_Level_LEVEL_INFO:
        level = logging::Level::LVL_INFO;
        break;

      case mbed_rpc_LoggerWriteRequest_Level_LEVEL_WARN:
        level = logging::Level::LVL_WARN;
        break;

      case mbed_rpc_LoggerWriteRequest_Level_LEVEL_ERROR:
        level = logging::Level::LVL_ERROR;
        break;

      case mbed_rpc_LoggerWriteRequest_Level_LEVEL_FATAL:
        level = logging::Level::LVL_FATAL;
        break;

      default:
        return mbed_rpc_ErrorCode_ERR_SVC_INVALID_ARG;
    }

    /*-------------------------------------------------------------------------
    Find a logger maching the request ID, then write to it.
    -------------------------------------------------------------------------*/
    response.success = false;

    auto logger = s_loggers.find( request.which );
    if( logger != s_loggers.end() )
    {
      response.success = ( logger->second->write( level, request.data.bytes, request.data.size ) == logging::ErrCode::ERR_OK );
    }

    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }


  /*---------------------------------------------------------------------------
  Read Service
  ---------------------------------------------------------------------------*/

  ErrId ReadService::processRequest()
  {
    /*-------------------------------------------------------------------------
    Request already in progress?
    -------------------------------------------------------------------------*/
    if( async )
    {
      return mbed_rpc_ErrorCode_ERR_SVC_BUSY;
    }

    /*-------------------------------------------------------------------------
    Look up the logger
    -------------------------------------------------------------------------*/
    auto logger = s_loggers.find( mId );
    if( logger == s_loggers.end() )
    {
      return mbed_rpc_ErrorCode_ERR_SVC_INVALID_ARG;
    }

    /*-------------------------------------------------------------------------
    Enable the async behavior
    -------------------------------------------------------------------------*/
    async      = true;
    mReadIdx   = 0;
    mReadCount = request.count;
    mDirection = request.direction;
    mId        = request.which;

    return mbed_rpc_ErrorCode_ERR_SVC_ASYNC;
  }


  void ReadService::runAsyncProcess()
  {
    using namespace logging;

    /*-------------------------------------------------------------------------
    Look up the logger, which should exist
    -------------------------------------------------------------------------*/
    auto logger = s_loggers.find( mId );
    if( logger == s_loggers.end() )
    {
      async = false;
      mbed_assert_continue_msg( false, "Logger not found: %d", mId );
      return;
    }

    /*-------------------------------------------------------------------------
    Invoke the read, then disable the async behavior once it completes
    -------------------------------------------------------------------------*/
    auto cb = LogReader::create<ReadService, &ReadService::reader_callback>( *this );
    logger->second->read( cb, mDirection );
    async = false;
  }


  bool ReadService::reader_callback( const void *const message, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Prepare the response
    -------------------------------------------------------------------------*/
    response.header    = request.header;
    response.index     = mReadIdx++;
    response.data.size = length;

    size_t copy_size = std::min( length, sizeof( response.data.bytes ) );
    memcpy( response.data.bytes, message, copy_size );

    /*-------------------------------------------------------------------------
    Try to send the response
    -------------------------------------------------------------------------*/
    size_t attempts   = 0;
    size_t sleep_time = 5;

    while( !server->publishMessage( rspId, &response ) )
    {
      mb::thread::this_thread::sleep_for( sleep_time );
      if( ++attempts >= 3 )
      {
        mbed_assert_continue_msg( false, "Failed to send response" );
        return true;    // Give up, stop the read
      }

      sleep_time *= 2;    // Exponential backoff
    }

    return false;    // Request the next message in the log
  }
}    // namespace mb::rpc::service::logger
