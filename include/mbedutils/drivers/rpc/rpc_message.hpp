/******************************************************************************
 *  File Name:
 *    rpc_message.hpp
 *
 *  Description:
 *    RPC message descriptor for a remote procedure call
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_RPC_MESSAGE_HPP
#define MBEDUTILS_RPC_MESSAGE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cobs.h>
#include <cstddef>
#include <mbed_rpc.pb.h>
#include <pb.h>
#include <etl/unordered_map.h>

namespace mb::rpc
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Identifier for a unique message type
   */
  using MsgId = uint8_t;

  /**
   * @brief Storage for the size of a message.
   */
  using MsgSize = uint16_t;

  /**
   * @brief Storage for the version of a message.
   */
  using MsgVer = uint8_t;

  /**
   * @brief Alias for the NanoPB message descriptor
   */
  using MsgFields = const pb_msgdesc_t *;


  /*-----------------------------------------------------------------------------
  Structures
  -----------------------------------------------------------------------------*/

  /**
   * @brief Descriptor for how to encode/decode a message
   */
  struct MsgDsc
  {
    MsgVer    version;
    MsgFields fields;
    pb_size_t max_buf_size;
  };


  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Hashmap data-structure for storing Message descriptors with a fixed size
   *
   * @tparam N  Number of elements to store
   */
  template<const size_t N>
  using DescriptorStorage = etl::unordered_map<MsgId, MsgDsc, N>;

  /**
   * @brief Hashmap data-structure for storing Message descriptors
   */
  using DescriptorRegistry = etl::iunordered_map<MsgId, MsgDsc>;

}

namespace mb::rpc::message
{

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief Storage size of the null terminator that COBS ends every frame with
   */
  static constexpr size_t COBS_TERM_SIZE = 1u;

  /**
   * @brief Storage for the CRC16 value that gets prefixed to every frame
   */
  static constexpr size_t COBS_CRC_SIZE = 2u;


  /*---------------------------------------------------------------------------
  Constexpr Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Computes the maximum buffer size needed for a message.
   *
   * This accounts for NanoPB serialization, COBS framing for both encoding
   * and decoding, and a CRC for validation.
   *
   * @param npbSize Auto-generated size reported by NanoPB for a message type
   * @return constexpr size_t Buffer size needed to send/receive the message
   */
  template<const pb_size_t npbSize>
  static constexpr size_t MaxWireSize()
  {
    constexpr pb_size_t encode_size = COBS_ENCODE_DST_BUF_LEN_MAX( npbSize + COBS_CRC_SIZE );
    constexpr pb_size_t decode_size = COBS_DECODE_DST_BUF_LEN_MAX( npbSize + COBS_CRC_SIZE );
    constexpr pb_size_t coding_max_size = std::max<pb_size_t>( encode_size, decode_size );

    return COBS_CRC_SIZE + coding_max_size + COBS_TERM_SIZE;
  }

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Define constexpr attributes of a message
   *
   * @tparam id       Message identifier
   * @tparam ver      Current version of the message
   * @tparam length   NanoPB size of the message
   */
  template<MsgId id, MsgVer ver, pb_size_t length>
  struct MsgAttr
  {
    static inline constexpr MsgId     msg_id              = id;
    static inline constexpr MsgVer    msg_ver             = ver;
    static inline constexpr pb_size_t fields_array_length = length;
    static inline constexpr pb_size_t wire_buffer_length  = MaxWireSize<length>();
  };

  /*---------------------------------------------------------------------------
  Builtin Messages
  ---------------------------------------------------------------------------*/

  /* clang-format off */
  using NullMessage  = MsgAttr<mbed_rpc_BuiltinMessage_MSG_NULL,  mbed_rpc_BuiltinMessageVersion_MSG_VER_NULL,  0>;
  using ErrorMessage = MsgAttr<mbed_rpc_BuiltinMessage_MSG_ERROR, mbed_rpc_BuiltinMessageVersion_MSG_VER_ERROR, mbed_rpc_ErrorMessage_size>;
  using PingMessage  = MsgAttr<mbed_rpc_BuiltinMessage_MSG_PING,  mbed_rpc_BuiltinMessageVersion_MSG_VER_PING,  mbed_rpc_PingMessage_size>;
  /* clang-format on */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the message module
   *
   * @param reg Registry to use for storing message descriptors
   */
  void initialize( DescriptorRegistry *const reg );

  /**
   * @brief Register a message this module should know how to decode.
   *
   * @param id      Identifier to register under
   * @param msg     The message to register
   * @return true   The message was registered successfully.
   * @return false  Failed to register for some reason.
   */
  bool addDescriptor( const MsgId id, const MsgDsc &dsc );

  /**
   * @brief Removes a message from this module.
   *
   * @param id Id of the message to remove.
   */
  void removeDescriptor( const MsgId id );

  /**
   * @brief Get a message descriptor registered with this module
   *
   * @param id  Id of the message to lookup
   * @return const IRPCMessage*
   */
  const MsgDsc *getDescriptor( const MsgId id );

  /**
   * @brief Encodes a message for transmission over the wire.
   *
   * @param msg_id        Which message this is
   * @param npb_struct    Input data to be encoded from
   * @param cobs_out_buf  Output buffer to encode the data into
   * @param cobs_out_size Maximum size of the output buffer
   * @return size_t       Total number of encoded bytes, including null terminator
   */
  size_t encode_to_wire( const MsgId msg_id, void *const npb_struct, void *cobs_out_buf, const size_t cobs_out_size );

  /**
   * @brief Decode a message received from the wire.
   *
   * If the message is successfully decoded, the message ID will be returned
   * and the message data will be stored in the provided buffer. The message
   * data will be in the form of a NanoPB struct.
   *
   * @param msg_id        Which message this is (output)
   * @param cobs_in_buf   Input buffer containing the complete COBS frame
   * @param cobs_in_size  Size of the COBS frame, including null terminator
   * @return bool         True if the message was decoded successfully
   */
  bool decode_from_wire( MsgId &msg_id, void *const cobs_in_buf, const size_t cobs_in_size );

}    // namespace mb::rpc

#endif /* !MBEDUTILS_RPC_MESSAGE_HPP */
