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
#include <mbedutils/drivers/rpc/rpc_service.hpp>
#include <mbedutils/drivers/rpc/rpc_message.hpp>
#include <mbed_rpc.pb.h>

namespace mb::rpc::services
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using PingStorage = ServiceStorage<mbed_rpc_PingMessage, message::PingMessage::msg_id, mbed_rpc_PingMessage, message::PingMessage::msg_id>;

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Service for handling ping requests
   */
  class PingService : public mb::rpc::IService
  {
  public:
    PingService() = default;
    ~PingService() = default;

    /**
     * @copydoc IService::processRequest
     */
    ErrId processRequest( const void *req, void *rsp ) final override;
  };

}  // namespace mb::rpc::services

#endif  /* !MBEDUTILS_RPC_PING_SERVICE_HPP */
