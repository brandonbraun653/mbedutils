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
#include "pb.h"
#include <cstdint>
#include <cstddef>
#include <etl/array.h>
#include <etl/circular_buffer.h>
#include <etl/delegate.h>
#include <etl/string.h>
#include <etl/unordered_map.h>
#include <etl/vector.h>
#include <mbedutils/drivers/hardware/serial.hpp>
#include <mbedutils/thread.hpp>

namespace mb::rpc::server
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  struct Request;
  struct Response;
  class IRPCService;
  class IRPCMessage;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Identifier for a unique message type
   */
  using MsgId = uint16_t;

  /**
   * @brief Identifier for a unique service
   */
  using SvcId = uint16_t;

  /**
   * @brief Size independent circular buffer reference for holding stream data
   */
  using StreamBuffer = etl::icircular_buffer<uint8_t>;

  /**
   * @brief
   *
   */
  using COBSFrame = etl::ivector<uint8_t>;

  using NanoPBFrame = etl::ivector<uint8_t>;

  /**
   * @brief Hashmap data-structure for storing Service descriptors
   */
  using ServiceRegistry = etl::iunordered_map<SvcId, IRPCService>;

  /**
   * @brief Hashmap data-structure for storing Message descriptors
   */
  using MessageRegistry = etl::iunordered_map<MsgId, IRPCMessage>;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Abstracts a single request message.
   *
   * This provides enough information to resolve the message decoder at runtime.
   */
  struct Request
  {
    MsgId       type; /**< What message type this request contains */
    uint16_t    size; /**< Byte size of the payload, if given */
    const void *data; /**< Optional data payload of the request */
  };


  /**
   * @brief Abstracts a single response message.
   *
   * This provides enough information to resolve the message encoder at runtime.
   */
  struct Response
  {
    MsgId    type; /**< What message type this response contains */
    uint16_t size; /**< Size of the data payload, if given */
    void    *data; /**< Optional data payload of the response */
  };


  /**
   * @brief Specifies the system resources consumed by the RPC server.
   */
  struct Config
  {
    mb::hw::serial::ISerial &iostream; /**< Serial driver to use for communication */
    StreamBuffer            &rxBuffer; /**< Buffer for incoming data stream */
    StreamBuffer            &txBuffer; /**< Buffer for outgoing data stream */
    ServiceRegistry         &svcReg;   /**< Service descriptor storage */
    MessageRegistry         &msgReg;   /**< Message descriptor storage */

    /**
     * @brief Configure how the server should handle incoming requests.
     *
     * If enabled, this will process requests immediately as they come in. If
     * disabled, requests will be queued up and processed in a batch, but this
     * places a requirement on the user to call runServices() periodically.
     */
    bool handleImmediate;
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Core interface for describing a service to the RPC server.
   */
  class IRPCService
  {
  public:
    SvcId svcId; /**< Which service this is */
    MsgId reqId; /**< Input request data message id */
    MsgId rspId; /**< Output response data message id */

    virtual ~IRPCService() = default;

  /**
   * @brief A highly generic RPC service stub
   *
   * @param req     Request input data
   * @param rsp     Response output data
   * @return true   If the service executed successfully
   * @return false  Something went wrong
   */
    virtual bool stub( const Request &req, Response &rsp ) = 0;
  };

  /**
   * @brief Descriptor for registering a message type with the RPC server.
   */
  class IRPCMessage
  {
  public:
    MsgId               type;         /**< What message type this descriptor references */
    const pb_msgdesc_t *descriptor;   /**< NanoPB generated descriptor for encoding/decoding */
    size_t              max_enc_size; /**< Max encoded size of the message in bytes */
    size_t              type_size;    /**< Size of the raw message structure */

    bool encode( /* input/output? */ );

    bool decode( /* output/input? */);
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
     * @brief Registers a service to be handled by this server instance.
     *
     * @param svc     The service to register
     * @return true   The service was registered successfully.
     * @return false  Failed to register for some reason.
     */
    bool addService( const IRPCService &svc );

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
    bool addMessage( const IRPCMessage &msg );

    /**
     * @brief Removes a message from this RPC server.
     *
     * @param id Id of the message to remove.
     */
    void removeMessage( const MsgId id );

    /**
     * @brief Sends a response for an RPC service that is asynchronous.
     *
     * This is intended to be used by a service that may take a long time to
     * complete and is not immediately available to send a response.
     *
     * @param rsp     Response to send
     * @return true   Message was queued successfully.
     * @return false  Something went wrong during an attempt to send.
     */
    bool sendAsyncResponse( const Response &rsp );

    /**
     * @brief Process all pending RPC service requests.
     *
     * This is a blocking call that will process all pending requests in the
     * queue. The total execution time is variable depending on the number of
     * requests in the queue and the time it takes to process each one.
     */
    void runServices();

  protected:

    bool rpc_invoke( const SvcId svc, const Request &req, Response &rsp );

    void rpc_throw_error( const size_t txn_id, uint32_t error_code, etl::string_view &msg );

    bool rpc_validate_request( const SvcId svc, const Request &req, uint32_t &error_code );

    bool rpc_pack_response( const SvcId svc, void *const msg_data, const size_t msg_size, const Request &req, Response &rsp );

    bool rpc_publish_message( const pb_msgdesc_t *dsc, const void * data );

  private:
    friend class ::mb::thread::Lockable<Server>;

    Config mConfig;

    size_t stream_read( COBSFrame &buffer );
    void stream_write( const COBSFrame &buffer );
  };
}  // namespace mb::rpc::server

#endif  /* !MBEDUTILS_RPC_SERVER_HPP */
