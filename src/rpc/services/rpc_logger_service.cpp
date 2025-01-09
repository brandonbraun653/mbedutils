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
    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }


  void EraseService::runAsyncProcess()
  {
  }


  /*---------------------------------------------------------------------------
  Write Service
  ---------------------------------------------------------------------------*/

  ErrId WriteService::processRequest()
  {
    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }


  void WriteService::runAsyncProcess()
  {
  }


  /*---------------------------------------------------------------------------
  Read Service
  ---------------------------------------------------------------------------*/

  ErrId ReadService::processRequest()
  {
    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }


  void ReadService::runAsyncProcess()
  {
  }

}    // namespace mb::rpc::service::logger
