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
      IRPCService( mbed_rpc_BuiltinService_SVC_PING, mbed_rpc_BuiltinMessage_MSG_PING, mbed_rpc_BuiltinMessage_MSG_PING,
                   "PingService" )
  {
  }

  ErrId PingService::processRequest( server::Server &server, const IRPCMessage &req, IRPCMessage &rsp )
  {
    if( req.id != rsp.id )
    {
      return mbed_rpc_ErrorCode_ERR_SVC_FAILED;
    }

    memcpy( rsp.data_impl, req.data_impl, req.max_data_size );
    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }

}  // namespace
