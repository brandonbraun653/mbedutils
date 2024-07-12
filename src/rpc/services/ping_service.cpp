/******************************************************************************
 *  File Name:
 *    ping_service.cpp
 *
 *  Description:
 *    Insert Description
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/rpc/builtin_services.hpp>

namespace mb::rpc::services
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  PingService::PingService() :
      IRPCService( mbed_rpc_BuiltinServices_SVC_PING, mbed_rpc_BuiltinMessages_MSG_PING, mbed_rpc_BuiltinMessages_MSG_PING,
                   "PingService" ),
      mPingResponse( {} )
  {
  }

  void PingService::processRequest( const mb::rpc::server::Request &request, mb::rpc::server::Response &response )
  {
    // Extract the ping message
    mPingResponse = *(mbed_rpc_Ping*)request.data;

    response.data = &mPingResponse;
  }

}  // namespace
