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
#include <cobs.h>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <etl/circular_buffer.h>
#include <etl/crc.h>
#include <etl/vector.h>
#include <mbed_rpc.pb.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <mbedutils/assert.hpp>

namespace mb::rpc
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Identifier for a unique service
   */
  using SvcId = uint8_t;

  /**
   * @brief Error code type for RPC operations
   */
  using ErrId = mbed_rpc_ErrorCode;

  /**
   * @brief Fixed size circular buffer for holding stream data
   *
   * @tparam N Number of elements to store
   */
  template<const size_t N>
  using StreamStorage = etl::circular_buffer<uint8_t, N>;

  /**
   * @brief Size independent circular buffer reference for holding stream data
   */
  using StreamBuffer = etl::icircular_buffer<uint8_t>;

  /**
   * @brief Fixed size array for storing scratch memory when encoding/decoding messages
   *
   * @tparam N  Number of elements to store
   */
  template<const size_t N>
  using ScratchStorage = etl::array<uint8_t, N>;

}    // namespace mb::rpc::messages

#endif /* !MBEDUTILS_RPC_COMMON_HPP */
