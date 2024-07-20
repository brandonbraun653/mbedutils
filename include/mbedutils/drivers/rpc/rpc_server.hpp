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
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/hardware/serial.hpp>
#include <mbedutils/drivers/rpc/rpc_common.hpp>
#include <mbedutils/drivers/rpc/rpc_message.hpp>
#include <mbedutils/thread.hpp>

namespace mb::rpc::server
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class IService;
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
  using ServiceStorage = etl::unordered_map<SvcId, ::mb::rpc::IService *, N>;

  /**
   * @brief A reference to a service descriptor storage location
   */
  using ServiceRegistry = etl::iunordered_map<SvcId, ::mb::rpc::IService *>;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Specifies the system resources consumed by the RPC server.
   */
  struct Config
  {
    mb::hw::serial::ISerial *iostream;  /**< Serial driver to use for communication */
    StreamBuffer            *rxBuffer;  /**< Buffer for incoming data stream */
    ServiceRegistry         *svcReg;    /**< Service descriptor storage */
    etl::span<uint8_t>       txScratch; /**< Scratch buffer for encoding messages */
    etl::span<uint8_t>       rxScratch; /**< Scratch buffer for decoding messages */
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
    bool addService( ::mb::rpc::IService *const svc );

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

    bool                 mIsOpen;
    Config               mCfg;
    service::PingService mPingService;

    bool process_next_request();
    bool read_cobs_frame();
    bool write_cobs_frame();
    void isr_on_io_read_complete();
  };
}    // namespace mb::rpc::server

#endif /* !MBEDUTILS_RPC_SERVER_HPP */
