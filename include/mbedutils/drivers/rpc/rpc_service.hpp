/******************************************************************************
 *  File Name:
 *    rpc_service.hpp
 *
 *  Description:
 *    Service descriptor for a remote procedure call
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_RPC_SERVICE_HPP
#define MBEDUTILS_RPC_SERVICE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <mbedutils/drivers/rpc/rpc_common.hpp>
#include <mbedutils/drivers/rpc/rpc_message.hpp>
#include <etl/array.h>
#include <etl/span.h>

namespace mb::rpc
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Helper template to declare the proper memory for an RPC service.
   *
   * @tparam *Name
   * @tparam Request
   * @tparam ReqNPBSize
   * @tparam Response
   * @tparam RspNPBSize
   */
  template<typename Request, const size_t ReqNPBSize, typename Response, const size_t RspNPBSize>
  struct ServiceStorage
  {
    //const etl::string<strlen( Name ) + 1> name{ Name };
    Request req;
    Response rsp;
    etl::array<uint8_t, message::MaxWireSize<ReqNPBSize>()> reqDecodeBuffer;
    etl::array<uint8_t, message::MaxWireSize<RspNPBSize>()> rspEncodeBuffer;
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Core interface for describing a service to the RPC server.
   *
   * Where possible, the number of virtual interfaces was reduced to help with
   * code size and performance. Embedded platforms don't really have much
   * memory to spare, so it's important to keep things as lean as possible while
   * still abstracting the necessary functionality.
   */
  class IService
  {
  public:
    virtual ~IService() = default;

    const char        *name;
    SvcId              svcId;
    void              *reqData;
    void              *rspData;
    MsgId              reqId;
    MsgId              rspId;
    etl::span<uint8_t> reqDecodeBuffer;
    etl::span<uint8_t> rspEncodeBuffer;

    /**
     * @brief A highly generic RPC service stub
     *
     * This function is called by the server to process a request. The server caches the message
     * data in the request and response objects, so the service can access the data as needed.
     *
     * @param server  Server instance that is processing the request
     * @param req     Request input data
     * @param rsp     Response output data
     * @return mbed_rpc_ErrorCode_ERR_NO_ERROR   If the service executed successfully
     * @return mbed_rpc_ErrorCode_ERR_SVC_ASYNC  If the service is async and will manually send a message later
     * @return mbed_rpc_ErrorCode_<any>          Any other error code that may have occurred
     */
    virtual ErrId processRequest( const void *req, void *rsp ) = 0;
  };

}  // namespace mb::rpc

#endif  /* !MBEDUTILS_RPC_SERVICE_HPP */
