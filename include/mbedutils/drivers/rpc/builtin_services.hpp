/******************************************************************************
 *  File Name:
 *    builtin_services.hpp
 *
 *  Description:
 *    RPC services that come pre-built with the system
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_RPC_PING_SERVICE_HPP
#define MBEDUTILS_RPC_PING_SERVICE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbed_rpc.pb.h>
#include <mbedutils/drivers/rpc/rpc_message.hpp>
#include <mbedutils/drivers/rpc/rpc_service.hpp>

namespace mb::rpc::service
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Service for handling ping requests
   */
  class PingService : public mb::rpc::BaseService<mbed_rpc_PingMessage, mbed_rpc_PingMessage>
  {
  public:
    PingService() :
        BaseService<mbed_rpc_PingMessage, mbed_rpc_PingMessage>( "PingService", mbed_rpc_BuiltinService_SVC_PING,
                                                                 mbed_rpc_BuiltinMessage_MSG_PING,
                                                                 mbed_rpc_BuiltinMessage_MSG_PING ){};
    ~PingService() = default;

    /**
     * @copydoc IService::processRequest
     */
    ErrId processRequest() final override
    {
      memcpy( &response, &request, sizeof( response ) );
      return mbed_rpc_ErrorCode_ERR_NO_ERROR;
    }
  };

  /**
   * @brief Service for testing error handling flow
   */
  class TestErrorService : public mb::rpc::BaseService<mbed_rpc_NullMessage, mbed_rpc_NullMessage>
  {
  public:
    TestErrorService() :
        BaseService<mbed_rpc_NullMessage, mbed_rpc_NullMessage>( "TestErrorService", mbed_rpc_BuiltinService_SVC_TEST_ERROR,
                                                                 mbed_rpc_BuiltinMessage_MSG_NULL,
                                                                 mbed_rpc_BuiltinMessage_MSG_NULL ){};
    ~TestErrorService() = default;

    /**
     * @copydoc IService::processRequest
     */
    ErrId processRequest() final override
    {
      return mbed_rpc_ErrorCode_ERR_SVC_FAILED;
    }
  };
}    // namespace mb::rpc::service

#endif /* !MBEDUTILS_RPC_PING_SERVICE_HPP */
