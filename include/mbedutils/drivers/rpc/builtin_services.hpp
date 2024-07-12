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
#include <mbedutils/drivers/rpc/rpc_server.hpp>
#include <mbed_rpc.pb.h>

namespace mb::rpc::services
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Service for handling ping requests
   */
  class PingService : public mb::rpc::server::IRPCService
  {
  public:
    PingService();
    ~PingService() = default;

    /**
     * @copydoc IRPCService::processRequest
     */
    void processRequest( const mb::rpc::server::Request &request, mb::rpc::server::Response &response ) final override;

  private:
    mbed_rpc_Ping mPingResponse;
  };

}  // namespace mb::rpc::services

#endif  /* !MBEDUTILS_RPC_PING_SERVICE_HPP */
