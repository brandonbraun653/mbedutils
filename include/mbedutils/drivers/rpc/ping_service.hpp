/******************************************************************************
 *  File Name:
 *    ping_service.hpp
 *
 *  Description:
 *    RPC service for handling ping requests
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
    ~PingService();

    /**
     * @copydoc IRPCService::initialize
     */
    void initialize() final override;

    /**
     * @copydoc IRPCService::shutdown
     */
    void shutdown() final override;

    /**
     * @copydoc IRPCService::processRequest
     */
    void processRequest( const mb::rpc::server::Request &request, mb::rpc::server::Response &response ) final override;

    /**
     * @copydoc IRPCService::getServiceId
     */
    SvcId getServiceId() const final override;

    /**
     * @copydoc IRPCService::getServiceName
     */
    etl::string_view getServiceName() const final override;

    /**
     * @copydoc IRPCService::getRequestMessageId
     */
    MsgId getRequestMessageId() const final override;

    /**
     * @copydoc IRPCService::getResponseMessageId
     */
    MsgId getResponseMessageId() const final override;

  private:
    mbed_rpc_Ping mPingResponse;
  };
}  // namespace mb::rpc::services

#endif  /* !MBEDUTILS_RPC_PING_SERVICE_HPP */
