/******************************************************************************
 *  File Name:
 *    rpc_server.hpp
 *
 *  Description:
 *    Interface for a remote procedure call server
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_RPC_SERVER_HPP
#define MBEDUTILS_RPC_SERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/array.h>
#include <etl/delegate.h>
#include <etl/span.h>
#include <etl/string.h>
#include <etl/unordered_map.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/hardware/serial.hpp>
#include <mbedutils/drivers/rpc/rpc_common.hpp>
#include <mbedutils/drivers/rpc/rpc_message.hpp>
#include <mbedutils/threading.hpp>


namespace mb::rpc::server
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/

  class Server;
}    // namespace mb::rpc::server

namespace mb::rpc::service
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

    const char     *name;   /**< Name of the service */
    const SvcId     svcId;  /**< Expected service identifier */
    const MsgId     reqId;  /**< Expected request identifier */
    const MsgId     rspId;  /**< Expected response identifier */
    bool            async;  /**< Is the service processing asynchronously? */
    server::Server *server; /**< Server instance that owns this service */


    /**
     * @brief Initializes the service with the necessary information
     *
     * @param name  Name of the service
     * @param svcId Expected service identifier
     * @param reqId Expected request identifier
     * @param rspId Expected response identifier
     */
    IService( const char *name, const SvcId svcId, const MsgId reqId, const MsgId rspId ) :
        name( name ), svcId( svcId ), reqId( reqId ), rspId( rspId ), async( false ), server( nullptr )
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
     * @return mbed_rpc_ErrorCode_ERR_NO_ERROR            If the service executed successfully. Sends a response.
     * @return mbed_rpc_ErrorCode_ERR_SVC_ASYNC           If the service is async and will manually send a message later
     * @return mbed_rpc_ErrorCode_ERR_SVC_ASYNC_WITH_RSP  If the service is async but still needs to send a response immediately
     * @return mbed_rpc_ErrorCode_<any>                   Any other error code that may have occurred
     */
    virtual ErrId processRequest() = 0;

    /**
     * @brief Run asynchronous processes needed to support the service
     */
    virtual void runAsyncProcess() = 0;
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
     * @param async Is the service asynchronous?
     */
    BaseService( const char *name, const SvcId svcId, const MsgId reqId, const MsgId rspId ) :
        IService( name, svcId, reqId, rspId ), request(), response()
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

    void runAsyncProcess() override
    {
      mbed_dbg_assert_msg( false, "%s async enabled but no async process defined", name );
      async = false;
    }

  protected:
    Request  request;  /**< Stores the request data */
    Response response; /**< Stores the response data */
  };


  /**
   * @brief Service for handling ping requests
   */
  class PingService : public BaseService<mbed_rpc_PingMessage, mbed_rpc_PingMessage>
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
    ErrId processRequest() final override;
  };


  /**
   * @brief Service for testing error handling flow
   */
  class TestErrorService : public BaseService<mbed_rpc_NullMessage, mbed_rpc_NullMessage>
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
    ErrId processRequest() final override;
  };


  /**
   * @brief Service for notifying a client when a certain amount of time has elapsed
   */
  class NotifyTimeElapsedService : public BaseService<mbed_rpc_NotifyTimeElapsedRequest, mbed_rpc_NotifyTimeElapsedResponse>
  {
  public:
    NotifyTimeElapsedService() :
        BaseService<mbed_rpc_NotifyTimeElapsedRequest, mbed_rpc_NotifyTimeElapsedResponse>(
            "NotifyTimeElapsedService", mbed_rpc_BuiltinService_SVC_NOTIFY_TIME_ELAPSED,
            mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_REQ, mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_RSP ){};
    ~NotifyTimeElapsedService() = default;

    /**
     * @copydoc IService::processRequest
     */
    mb::rpc::ErrId processRequest() final override;

    /**
     * @copydoc IService::runAsyncProcess
     */
    void runAsyncProcess() final override;

  private:
    size_t start_time; /**< Time the request was received */
    size_t alarm_time; /**< Time in the future the message should be sent */
  };
}    // namespace mb::rpc::service

namespace mb::rpc::server
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Hashmap data-structure for storing Service descriptors with a fixed size
   *
   * @tparam N  Number of elements to store
   */
  template<const size_t N>
  using ServiceStorage = etl::unordered_map<SvcId, ::mb::rpc::service::IService *, N>;

  /**
   * @brief A reference to a service descriptor storage location
   */
  using ServiceRegistry = etl::iunordered_map<SvcId, ::mb::rpc::service::IService *>;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Specifies the system resources consumed by the RPC server.
   *
   * These resources are required for the server to operate and must be
   * provided by the user of the server. This could be static or dynamically
   * allocated memory, but it must be available for the server to exclusively
   * use.
   */
  struct Config
  {
    mb::hw::serial::ISerial *iostream;     /**< Serial driver to use for communication */
    StreamBuffer            *streamBuffer; /**< Buffer for incoming data stream */
    ServiceRegistry         *registry;     /**< Service descriptor storage */
    etl::span<uint8_t>       encodeBuffer; /**< Scratch buffer for encoding messages */
    etl::span<uint8_t>       decodeBuffer; /**< Scratch buffer for decoding messages */
  };

  /**
   * @brief Statically allocate necessary storage for the server to operate.
   *
   * The buffers should be sized according to the largest possible message
   * sent or received by one of the services. In particular, the stream buffer
   * should hold several multiples of that message size to prevent overflow.
   *
   * @tparam NumServices  How many services may be registered at once
   * @tparam RXStream     Size of the receive buffer for incoming data
   * @tparam TXTranscode  Size of the scratch buffer for encoding ANY message
   * @tparam RXTranscode  Size of the scratch buffer for decoding ANY message
   */
  template<size_t NumServices, size_t RXStream, size_t TXTranscode, size_t RXTranscode>
  struct Storage
  {
    ServiceStorage<NumServices> registry;     /**< Service instance memory */
    StreamStorage<RXStream>     streamBuffer; /**< Stream data buffer for accumulating frames */
    ScratchStorage<TXTranscode> encodeBuffer; /**< Workspace for a single TX frame */
    ScratchStorage<RXTranscode> decodeBuffer; /**< Workspace for a single RX frame */

    /**
     * @brief Helper method to create a configuration object for the server.
     *
     * @param iostream  Serial driver to use for communication
     * @return Config
     */
    inline Config make_config( mb::hw::serial::ISerial &iostream )
    {
      return Config{ &iostream, &streamBuffer, &registry, encodeBuffer, decodeBuffer };
    }
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Core RPC server implementation.
   *
   * This is a stateless server, meaning that it does not maintain any connection history
   * or state between calls. Once it's open, it's ready to process requests and will do
   * so until it's closed by the host application.
   *
   * It is partially cooperative with the given IOStream object, meaning it will share
   * the TX line with the rest of the system, however it requires exclusive access
   * to the RX line. The user must ensure that the RX line is not emptied by other parts
   * of the system while the server is running, else it may miss incoming requests.
   */
  class Server : mb::thread::Lockable<Server>
  {
  public:
    Server();
    ~Server();

    /**
     * @brief Prepares the server for operation.
     *
     * Once this function returns, the server is ready to process incoming
     * requests. The user must call close() to release resources when done.
     *
     * @param config  Configuration data for the server
     * @return true   The server was opened successfully.
     * @return false  Failed to open for some reason.
     */
    bool open( const Config &config );

    /**
     * @brief Tear down the server and release all resources.
     */
    void close();

    /**
     * @brief Process all pending RPC service requests.
     *
     * This is a blocking call that will process all pending requests in the
     * queue. The total execution time is variable depending on the number of
     * requests in the queue and the time it takes to process each one.
     */
    void runServices();

    /**
     * @brief Registers a service to be handled by this server instance.
     *
     * @param svc     The service to register
     * @return true   The service was registered successfully.
     * @return false  Failed to register for some reason.
     */
    bool addService( ::mb::rpc::service::IService *const svc );

    /**
     * @brief Remove a service from this RPC server.
     *
     * @param id Id of the service to remove.
     */
    void removeService( const SvcId id );

    /**
     * @brief Throws an error response back to the client for a given transaction.
     *
     * @param txn_id      Transaction ID to respond to
     * @param error_code  Error code to send
     * @param msg         Optional message to send along with the error
     */
    void throwError( const size_t txn_id, const mbed_rpc_ErrorCode error_code, const etl::string_view msg );

    /**
     * @brief Publishes a message to the client.
     *
     * @param id     Message Id to publish
     * @param data   Contents of the message to publish
     * @return true  If the message was published successfully
     * @return false If the message could not be published
     */
    bool publishMessage( const MsgId id, void *const data );

  private:
    friend class ::mb::thread::Lockable<Server>;

    bool   mIsOpen;
    Config mCfg;

    /* Builtin Services */
    service::PingService              mPingService;
    service::TestErrorService         mTestErrorService;
    service::NotifyTimeElapsedService mNotifyTimeElapsedService;

    bool process_next_request();
    bool read_cobs_frame();
    bool write_cobs_frame();
    void isr_on_io_read_complete();
  };
}    // namespace mb::rpc::server

#endif /* !MBEDUTILS_RPC_SERVER_HPP */
