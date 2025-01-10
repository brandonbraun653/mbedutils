/******************************************************************************
 *  File Name:
 *    rpc_logger_service.hpp
 *
 *  Description:
 *    Common logging service built on the RPC system. This is a networking
 *    interface layer that allows remote systems to interact with the logging
 *    sinks on the local system.
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#include "mbed_rpc.pb.h"
#include <cstdint>
#ifndef MBEDUTILS_RPC_LOGGER_SERVICE_HPP
#define MBEDUTILS_RPC_LOGGER_SERVICE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/logging/logging_sinks.hpp>
#include <mbedutils/drivers/rpc/builtin_messages.hpp>
#include <mbedutils/drivers/rpc/rpc_server.hpp>

namespace mb::rpc::service::logger
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the logger service module
   */
  void initialize();

  /**
   * @brief Map a logger to an ID the service can use to route messages
   *
   * @param id ID assigned to the logger
   * @param log Instance of the logger to interact with
   * @return true  Logger was successfully mapped
   * @return false Logger could not be mapped
   */
  bool bind( const uint8_t id, logging::SinkHandle_rPtr log );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class EraseService : public BaseService<mbed_rpc_LoggerEraseRequest, mbed_rpc_LoggerEraseResponse>
  {
  public:
    EraseService() :
        BaseService<mbed_rpc_LoggerEraseRequest, mbed_rpc_LoggerEraseResponse>(
            "LoggerEraseService", mbed_rpc_BuiltinService_SVC_LOGGER_ERASE, mbed_rpc_BuiltinMessage_MSG_LOGGER_ERASE_REQ,
            mbed_rpc_BuiltinMessage_MSG_LOGGER_ERASE_RSP ){};
    ~EraseService() = default;

    /**
     * @copydoc IService::processRequest
     */
    mb::rpc::ErrId processRequest() final override;
  };


  class WriteService : public BaseService<mbed_rpc_LoggerWriteRequest, mbed_rpc_LoggerWriteResponse>
  {
  public:
    WriteService() :
        BaseService<mbed_rpc_LoggerWriteRequest, mbed_rpc_LoggerWriteResponse>(
            "LoggerWriteService", mbed_rpc_BuiltinService_SVC_LOGGER_WRITE, mbed_rpc_BuiltinMessage_MSG_LOGGER_WRITE_REQ,
            mbed_rpc_BuiltinMessage_MSG_LOGGER_WRITE_RSP ){};
    ~WriteService() = default;

    /**
     * @copydoc IService::processRequest
     */
    mb::rpc::ErrId processRequest() final override;
  };


  class ReadService : public BaseService<mbed_rpc_LoggerReadRequest, mbed_rpc_LoggerReadStreamResponse>
  {
  public:
    ReadService() :
        BaseService<mbed_rpc_LoggerReadRequest, mbed_rpc_LoggerReadStreamResponse>(
            "LoggerReadService", mbed_rpc_BuiltinService_SVC_LOGGER_READ, mbed_rpc_BuiltinMessage_MSG_LOGGER_READ_REQ,
            mbed_rpc_BuiltinMessage_MSG_LOGGER_READ_STREAM_RSP ){};
    ~ReadService() = default;

    /**
     * @copydoc IService::processRequest
     */
    mb::rpc::ErrId processRequest() final override;

    /**
     * @copydoc IService::runAsyncProcess
     */
    void runAsyncProcess() final override;

  protected:
    /**
     * @copydoc mb::logging::LogReader
     */
    bool reader_callback( const void *const message, const size_t length );

  private:
    uint16_t mReadIdx;   /**< Index of the current read operation */
    uint16_t mReadCount; /**< Number of messages to read */
    bool     mDirection; /**< Direction of the read operation */
    uint8_t  mId;        /**< ID of the logger to read from */
  };

}    // namespace mb::rpc::service::logger

#endif /* !MBEDUTILS_RPC_LOGGER_SERVICE_HPP */
