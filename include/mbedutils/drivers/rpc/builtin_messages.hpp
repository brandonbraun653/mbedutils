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

  /**
   * @brief Null message descriptor
   */
  static constexpr Descriptor NullMessage{ mbed_rpc_BuiltinMessage_MSG_NULL, mbed_rpc_BuiltinMessageVersion_MSG_VER_NULL,
                                           nullptr, 0 };

  /**
   * @brief Ping message descriptor
   */
  static constexpr Descriptor PingMessage{ mbed_rpc_BuiltinMessage_MSG_PING, mbed_rpc_BuiltinMessageVersion_MSG_VER_PING,
                                           mbed_rpc_PingMessage_fields, mbed_rpc_PingMessage_size };

  /**
   * @brief Error message descriptor
   */
  static constexpr Descriptor ErrorMessage{ mbed_rpc_BuiltinMessage_MSG_ERROR, mbed_rpc_BuiltinMessageVersion_MSG_VER_ERROR,
                                            mbed_rpc_ErrorMessage_fields, mbed_rpc_ErrorMessage_size };

}    // namespace mb::rpc::message

#endif /* !MBEDUTILS_RPC_BUILTIN_MESSAGES_HPP */
