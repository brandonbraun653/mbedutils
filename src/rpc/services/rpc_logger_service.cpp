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
#include "mbedutils/drivers/logging/logging_types.hpp"
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
    async = true;
    return mbed_rpc_ErrorCode_ERR_SVC_ASYNC_WITH_RSP;
  }


  void ReadService::runAsyncProcess()
  {
    async = false;
  }

}    // namespace mb::rpc::service::logger
