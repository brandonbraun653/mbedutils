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
#include <etl/delegate.h>
#include <etl/string.h>
#include <etl/unordered_map.h>
#include <mbedutils/drivers/hardware/serial.hpp>
#include <mbedutils/drivers/rpc/rpc_common.hpp>
#include <mbedutils/thread.hpp>
#include <mbedutils/assert.hpp>

namespace mb::rpc::server
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  struct Request;
  struct Response;
  class IRPCService;
  class Server;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Hashmap data-structure for storing Service descriptors with a fixed size
   *
   * @tparam N  Number of elements to store
   */
  template<const size_t N>
  using ServiceStorage = etl::unordered_map<SvcId, IRPCService*, N>;

  /**
   * @brief A reference to a service descriptor storage location
   */
  using ServiceRegistry = etl::iunordered_map<SvcId, IRPCService*>;


  /**
   * @brief Hashmap data-structure for storing Message descriptors with a fixed size
   *
   * @tparam N  Number of elements to store
   */
  template<const size_t N>
  using MessageStorage = etl::unordered_map<MsgId, IRPCMessage*, N>;

  /**
   * @brief Hashmap data-structure for storing Message descriptors
   */
  using MessageRegistry = etl::iunordered_map<MsgId, IRPCMessage*>;


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Specifies the system resources consumed by the RPC server.
   */
  struct Config
  {
    mb::hw::serial::ISerial *iostream;      /**< Serial driver to use for communication */
    StreamBuffer            *rxBuffer;      /**< Buffer for incoming data stream */
    StreamBuffer            *txBuffer;      /**< Buffer for outgoing data stream */
    ServiceRegistry         *svcReg;        /**< Service descriptor storage */
    MessageRegistry         *msgReg;        /**< Message descriptor storage */
    uint8_t                 *txScratch;     /**< Scratch buffer for encoding messages */
    size_t                   txScratchSize; /**< Size of the scratch buffer */
    uint8_t                 *rxScratch;     /**< Scratch buffer for decoding messages */
    size_t                   rxScratchSize; /**< Size of the scratch buffer */
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
  class IRPCService
  {
  public:
    constexpr IRPCService( const SvcId id, const MsgId req, const MsgId rsp, const etl::string_view name ) :
        mServiceId( id ), mRequestType( req ), mResponseType( rsp ), mName( name ){};

    virtual ~IRPCService() = default;

    /**
     * @brief Initializes the service and allocates any resources it needs.
     */
    virtual void initialize(){};

    /**
     * @brief Tears down the service and releases all resources.
     */
    virtual void shutdown(){};

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
    virtual ErrId processRequest( Server& server, const IRPCMessage &req, IRPCMessage &rsp )
    {
      mbed_dbg_assert_always();
      return mbed_rpc_ErrorCode_ERR_MAX_ERROR;
    };

    /**
     * @brief Get's the name of the service for debugging purposes.
     *
     * @return etl::string_view
     */
    constexpr inline etl::string_view getServiceName() const
    {
      return mName;
    }

    /**
     * @brief Get's the ID of the service for internal use.
     *
     * @return SvcId
     */
    constexpr inline SvcId getServiceId() const
    {
      return mServiceId;
    }

    /**
     * @brief Get the message ID that this service expects to receive.
     *
     * @return MsgId
     */
    constexpr inline MsgId getRequestMessageId() const
    {
      return mRequestType;
    }

    /**
     * @brief Get the message ID that this service will respond with.
     *
     * @return MsgId
     */
    constexpr inline MsgId getResponseMessageId() const
    {
      return mResponseType;
    }

  protected:
    const SvcId            mServiceId;    /**< Unique ID for this service */
    const MsgId            mRequestType;  /**< What message type this service expects */
    const MsgId            mResponseType; /**< What message type this service responds with */
    const etl::string_view mName;         /**< Name of the service */
  };


  /**
   * @brief Core RPC server implementation.
   *
   * This is a stateless server, meaning that it does not maintain any connection history
   * or state between calls. Once it's open, it's ready to process requests and will do
   * so until it's closed by the host application.
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
    bool addService( IRPCService *const svc );

    /**
     * @brief Remove a service from this RPC server.
     *
     * @param id Id of the service to remove.
     */
    void removeService( const SvcId id );

    /**
     * @brief Register a message this server should know how to decode.
     *
     * @param msg     The message to register
     * @return true   The message was registered successfully.
     * @return false  Failed to register for some reason.
     */
    bool addMessage( IRPCMessage *const msg );

    /**
     * @brief Removes a message from this RPC server.
     *
     * @param id Id of the message to remove.
     */
    void removeMessage( const MsgId id );

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
     * @param msg    The message to publish
     * @return true  If the message was published successfully
     * @return false If the message could not be published
     */
    bool publishMessage( IRPCMessage &dsc );

  private:
    friend class ::mb::thread::Lockable<Server>;

    bool   mIsOpen;
    Config mCfg;

    bool process_next_request();
    bool read_cobs_frame();
    bool write_cobs_frame();
    void isr_on_io_write_complete();
    void isr_on_io_read_complete();
  };
}    // namespace mb::rpc::server

#endif /* !MBEDUTILS_RPC_SERVER_HPP */
