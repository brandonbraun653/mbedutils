/******************************************************************************
 *  File Name:
 *    builtin_messages.hpp
 *
 *  Description:
 *    Default message types that are built into the RPC system
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_RPC_BUILTIN_MESSAGES_HPP
#define MBEDUTILS_RPC_BUILTIN_MESSAGES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/rpc/rpc_message.hpp>

namespace mb::rpc::message
{
  /*---------------------------------------------------------------------------
  Builtin Messages
  ---------------------------------------------------------------------------*/

  static constexpr Descriptor NullMessage{ mbed_rpc_BuiltinMessage_MSG_NULL, mbed_rpc_BuiltinMessageVersion_MSG_VER_NULL,
                                           mbed_rpc_NullMessage_fields, mbed_rpc_NullMessage_size };

  static constexpr Descriptor PingMessage{ mbed_rpc_BuiltinMessage_MSG_PING, mbed_rpc_BuiltinMessageVersion_MSG_VER_PING,
                                           mbed_rpc_PingMessage_fields, mbed_rpc_PingMessage_size };

  static constexpr Descriptor ErrorMessage{ mbed_rpc_BuiltinMessage_MSG_ERROR, mbed_rpc_BuiltinMessageVersion_MSG_VER_ERROR,
                                            mbed_rpc_ErrorMessage_fields, mbed_rpc_ErrorMessage_size };

  static constexpr Descriptor ConsoleMessage{ mbed_rpc_BuiltinMessage_MSG_CONSOLE,
                                              mbed_rpc_BuiltinMessageVersion_MSG_VER_CONSOLE, mbed_rpc_ConsoleMessage_fields,
                                              mbed_rpc_ConsoleMessage_size };

  static constexpr Descriptor NotifyTimeElapsedRequest{ mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_REQ,
                                                        mbed_rpc_BuiltinMessageVersion_MSG_VER_NOTIFY_TIME_ELAPSED_REQ,
                                                        mbed_rpc_NotifyTimeElapsedRequest_fields,
                                                        mbed_rpc_NotifyTimeElapsedRequest_size };

  static constexpr Descriptor NotifyTimeElapsedResponse{ mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_RSP,
                                                         mbed_rpc_BuiltinMessageVersion_MSG_VER_NOTIFY_TIME_ELAPSED_RSP,
                                                         mbed_rpc_NotifyTimeElapsedResponse_fields,
                                                         mbed_rpc_NotifyTimeElapsedResponse_size };

  static constexpr Descriptor LoggerEraseRequest{ mbed_rpc_BuiltinMessage_MSG_LOGGER_ERASE_REQ,
                                                  mbed_rpc_BuiltinMessageVersion_MSG_VER_LOGGER_ERASE_REQ,
                                                  mbed_rpc_LoggerEraseRequest_fields, mbed_rpc_LoggerEraseRequest_size };

  static constexpr Descriptor LoggerEraseResponse{ mbed_rpc_BuiltinMessage_MSG_LOGGER_ERASE_RSP,
                                                   mbed_rpc_BuiltinMessageVersion_MSG_VER_LOGGER_ERASE_RSP,
                                                   mbed_rpc_LoggerEraseResponse_fields, mbed_rpc_LoggerEraseResponse_size };

  static constexpr Descriptor LoggerReadRequest{ mbed_rpc_BuiltinMessage_MSG_LOGGER_READ_REQ,
                                                 mbed_rpc_BuiltinMessageVersion_MSG_VER_LOGGER_READ_REQ,
                                                 mbed_rpc_LoggerReadRequest_fields, mbed_rpc_LoggerReadRequest_size };

  static constexpr Descriptor LoggerReadStreamResponse{ mbed_rpc_BuiltinMessage_MSG_LOGGER_READ_STREAM_RSP,
                                                        mbed_rpc_BuiltinMessageVersion_MSG_VER_LOGGER_READ_STREAM_RSP,
                                                        mbed_rpc_LoggerReadStreamResponse_fields,
                                                        mbed_rpc_LoggerReadStreamResponse_size };

  static constexpr Descriptor LoggerWriteRequest{ mbed_rpc_BuiltinMessage_MSG_LOGGER_WRITE_REQ,
                                                  mbed_rpc_BuiltinMessageVersion_MSG_VER_LOGGER_WRITE_REQ,
                                                  mbed_rpc_LoggerWriteRequest_fields, mbed_rpc_LoggerWriteRequest_size };

  static constexpr Descriptor LoggerWriteResponse{ mbed_rpc_BuiltinMessage_MSG_LOGGER_WRITE_RSP,
                                                   mbed_rpc_BuiltinMessageVersion_MSG_VER_LOGGER_WRITE_RSP,
                                                   mbed_rpc_LoggerWriteResponse_fields, mbed_rpc_LoggerWriteResponse_size };
}    // namespace mb::rpc::message

#endif /* !MBEDUTILS_RPC_BUILTIN_MESSAGES_HPP */
