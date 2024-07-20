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
#include <mbedutils/drivers/rpc/rpc_server.hpp>

namespace mb::rpc::services
{
  /*---------------------------------------------------------------------------
  Ping Service
  ---------------------------------------------------------------------------*/

  ErrId PingService::processRequest( const void *req, void *rsp )
  {
    // if( req.id != rsp.id )
    // {
    //   return mbed_rpc_ErrorCode_ERR_SVC_FAILED;
    // }

    // memcpy( rsp.data_impl, req.data_impl, req.max_data_size );
    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }

}  // namespace
