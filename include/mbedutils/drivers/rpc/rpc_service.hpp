/******************************************************************************
 *  File Name:
 *    rpc_service.hpp
 *
 *  Description:
 *    Interface to describe a remote procedure call service
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
#include <etl/array.h>
#include <etl/span.h>
#include <mbedutils/drivers/rpc/rpc_common.hpp>
#include <mbedutils/drivers/rpc/rpc_message.hpp>

namespace mb::rpc
{
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

    const char *name;
    const SvcId svcId;
    const MsgId reqId;
    const MsgId rspId;

    /**
     * @brief Initializes the service with the necessary information
     *
     * @param name  Name of the service
     * @param svcId Expected service identifier
     * @param reqId Expected request identifier
     * @param rspId Expected response identifier
     */
    IService( const char *name, const SvcId svcId, const MsgId reqId, const MsgId rspId ) :
        name( name ), svcId( svcId ), reqId( reqId ), rspId( rspId )
    {
    }

    /**
     * @brief Retrieves a pointer to the request data and its size
     *
     * @param data Pointer to the data
     * @param size Max size of the structure storing the data
     */
    virtual void getRequestData( void *&data, size_t &size ) = 0;

    /**
     * @brief Retrieves a pointer to the response data and its size
     *
     * @param data Pointer to the data
     * @param size Max size of the structure storing the data
     */
    virtual void getResponseData( void *&data, size_t &size ) = 0;

    /**
     * @brief A highly generic RPC service stub
     *
     * This function is called by the server to process a request. All
     * input/output is cached in the final service object itself.
     *
     * @return mbed_rpc_ErrorCode_ERR_NO_ERROR   If the service executed successfully
     * @return mbed_rpc_ErrorCode_ERR_SVC_ASYNC  If the service is async and will manually send a message later
     * @return mbed_rpc_ErrorCode_<any>          Any other error code that may have occurred
     */
    virtual ErrId processRequest() = 0;
  };


  /**
   * @brief Derived service type providing storage for request and response data.
   *
   * @tparam Request  NanoPB message type for the request
   * @tparam Response NanoPB message type for the response
   */
  template<typename Request, typename Response>
  class BaseService : public IService
  {
  public:
    /**
     * @brief Initializes the service with the necessary information
     *
     * @param name  Name of the service
     * @param svcId Expected service identifier
     * @param reqId Expected request identifier
     * @param rspId Expected response identifier
     */
    BaseService( const char *name, const SvcId svcId, const MsgId reqId, const MsgId rspId ) :
        IService( name, svcId, reqId, rspId )
    {
    }

    virtual ~BaseService() = default;

    void getRequestData( void *&data, size_t &size ) final override
    {
      data = &request;
      size = sizeof( Request );
    }

    void getResponseData( void *&data, size_t &size ) final override
    {
      data = &response;
      size = sizeof( Response );
    }

  protected:
    Request  request;  /**< Stores the request data */
    Response response; /**< Stores the response data */
  };
}    // namespace mb::rpc

#endif /* !MBEDUTILS_RPC_SERVICE_HPP */
