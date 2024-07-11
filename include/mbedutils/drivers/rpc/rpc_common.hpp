/******************************************************************************
 *  File Name:
 *    rpc_common.hpp
 *
 *  Description:
 *    Common declarations and definitions for the RPC system
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_RPC_COMMON_HPP
#define MBEDUTILS_RPC_COMMON_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/circular_buffer.h>
#include <etl/vector.h>
#include <mbed_rpc.pb.h>

namespace mb::rpc
{
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
   * @brief Error code type for RPC operations
   */
  using ErrId = mbed_rpc_ErrorCode;

  /**
   * @brief Size independent circular buffer reference for holding stream data
   */
  using StreamBuffer = etl::icircular_buffer<uint8_t>;

  /**
   * @brief Size independent COBS encoded frame for RPC communication
   */
  using COBSFrame = etl::ivector<uint8_t>;

  /**
   * @brief Size independent NanoPB encoded frame for RPC communication
   */
  using NanoPBFrame = etl::ivector<uint8_t>;
}  // namespace mb::rpc

#endif  /* !MBEDUTILS_RPC_COMMON_HPP */
