/******************************************************************************
 *  File Name:
 *    rpc_message.hpp
 *
 *  Description:
 *    Message interface for RPC
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
#include <etl/unordered_map.h>
#include <mbed_rpc.pb.h>
#include <pb.h>

namespace mb::rpc
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Sequence ID for a message
   */
  using SeqId = uint8_t;

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
}    // namespace mb::rpc

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
  static constexpr size_t TranscodeSize( pb_size_t npbSize )
  {
    // Implements: COBS_ENCODE_DST_BUF_LEN_MAX
    constexpr auto encode_size = []( pb_size_t size ) { return size + ( ( size + 253u ) / 254u ); };

    // Implements: COBS_DECODE_DST_BUF_LEN_MAX
    constexpr auto decode_size = []( pb_size_t size ) { return ( size == 0 ) ? 0u : ( size - 1u ); };

    pb_size_t encodeSize    = encode_size( npbSize + COBS_CRC_SIZE );
    pb_size_t decodeSize    = decode_size( npbSize + COBS_CRC_SIZE );
    pb_size_t codingMaxSize = std::max<pb_size_t>( encodeSize, decodeSize );

    return codingMaxSize + COBS_TERM_SIZE;
  }

  /*-----------------------------------------------------------------------------
  Structures
  -----------------------------------------------------------------------------*/

  /**
   * @brief Descriptor for how to encode/decode a message
   */
  class Descriptor
  {
  public:
    const MsgId     id;             /**< Identifier of the message */
    const MsgVer    version;        /**< Version of the message */
    const MsgFields fields;         /**< Descriptor for encoding/decoding */
    const pb_size_t transcode_size; /**< Minimum buffer size for encoding/decoding */

    /**
     * @brief Construct a new Descriptor object
     *
     * @param ver   Version of the message
     * @param flds  NanoPB message descriptor
     * @param size  NanoPB max encoded size
     */
    constexpr Descriptor( const MsgId id, const MsgVer ver, const MsgFields flds, const pb_size_t size ) :
        id( id ), version( ver ), fields( flds ), transcode_size( TranscodeSize( size ) )
    {
    }
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
  using DescriptorStorage = etl::unordered_map<MsgId, Descriptor, N>;

  /**
   * @brief Hashmap data-structure for storing Message descriptors
   */
  using DescriptorRegistry = etl::iunordered_map<MsgId, Descriptor>;

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
   * @brief Get the next sequence ID for the system
   *
   * @return SeqId
   */
  SeqId next_seq_id();

  /**
   * @brief Register a message descriptor this module should know how to decode.
   *
   * @param dsc     The descriptor to register
   * @return true   The descriptor was registered successfully.
   * @return false  Failed to register for some reason.
   */
  bool addDescriptor( const Descriptor &dsc );

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
   * @return const Descriptor*
   */
  const Descriptor *getDescriptor( const MsgId id );

  /**
   * @brief In-place encodes a message for transmission over the wire.
   *
   * Output Framing:
   * [COBS Start][CRC16][NanoPB Data][COBS Term]
   *
   * @param msg_id        Which message this is
   * @param npb_struct    Input data to be encoded from
   * @param cobs_out_buf  Output buffer to encode the data into
   * @param cobs_out_size Maximum size of the output buffer
   * @return size_t       Total number of encoded bytes, including null terminator
   */
  size_t encode_to_wire( const MsgId msg_id, void *const npb_struct, void *cobs_out_buf, const size_t cobs_out_size );

  /**
   * @brief In-place decode a message received from the wire.
   *
   * If the message is successfully decoded, the original data NanoPB
   * data structure will be stored in the same buffer.
   *
   * Input Framing:
   * [COBS Start][CRC16][NanoPB Data][COBS Term]
   *
   * Output Framing:
   * [NanoPB Data]
   *
   * @param cobs_in_buf   Input buffer containing the complete COBS frame
   * @param cobs_in_size  Size of the COBS frame, including null terminator
   * @return bool         True if the message was decoded successfully
   */
  bool decode_from_wire( void *const cobs_in_buf, const size_t cobs_in_size );

}    // namespace mb::rpc::message

#endif /* !MBEDUTILS_RPC_MESSAGE_HPP */
