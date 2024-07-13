/******************************************************************************
 *  File Name:
 *    builtin_messages.hpp
 *
 *  Description:
 *    Built in RPC message definitions
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_RPC_BUILTIN_MESSAGES_HPP
#define MBEDUTILS_RPC_BUILTIN_MESSAGES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <etl/message.h>
#include <mbed_rpc.pb.h>
#include <mbedutils/drivers/rpc/rpc_common.hpp>
#include <pb.h>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

#define DECLARE_RPC_MESSAGE( msgName, msgId, npbFields, npbSize, msgType )                       \
  class msgName : public IRPCMessage                                                             \
  {                                                                                              \
  public:                                                                                        \
    constexpr msgName() : IRPCMessage( msgId, npbFields, npbSize, sizeof( msgType ) ), data{ 0 } \
    {                                                                                            \
    }                                                                                            \
                                                                                                 \
    ~msgName() = default;                                                                        \
                                                                                                 \
    msgType data;                                                                                \
  }

namespace mb::rpc::messages
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  DECLARE_RPC_MESSAGE( PingMessage, mbed_rpc_BuiltinMessages_MSG_PING, mbed_rpc_Ping_fields, mbed_rpc_Ping_size, mbed_rpc_Ping );

}  // namespace mb::rpc::messages

#endif  /* !MBEDUTILS_RPC_BUILTIN_MESSAGES_HPP */
