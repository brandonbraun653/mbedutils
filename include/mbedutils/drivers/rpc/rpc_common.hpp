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

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

#define DECLARE_RPC_MESSAGE( msgName, msgId, msgVer, npbFields, npbSize, msgType )                                    \
  class msgName : public IRPCMessage                                                                                  \
  {                                                                                                                   \
  public:                                                                                                             \
    msgType message;                                                                                                  \
    constexpr msgName() : IRPCMessage( msgId, msgVer, npbFields, npbSize, sizeof( msgType ), &message ), message{ 0 } \
    {                                                                                                                 \
    }                                                                                                                 \
                                                                                                                      \
    ~msgName() = default;                                                                                             \
  }

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
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Describes a single RPC message type with key information for encoding/decoding.
   */
  class IRPCMessage
  {
  public:
    const MsgId     id;            /**< Unique identifier for the message */
    const MsgVer    version;       /**< Version of the message */
    const MsgFields fields;        /**< NanoPB message descriptor */
    const MsgSize   max_wire_size; /**< Max size of the fully encoded message */
    const MsgSize   max_data_size; /**< Max size of the un-encoded data storage */
    void *const     data_impl;     /**< Pointer to the data structure */
    size_t          data_size;     /**< Actual size of the data structure being sent */

    /**
     * @brief Constructs the message descriptor
     *
     * @param id         Unique identifier for the message
     * @param npb_fields NanoPB message descriptor
     * @param npb_size   Size of the NanoPB message
     * @param struct_size  Size of the un-encoded data storage structure
     */
    constexpr IRPCMessage( const MsgId id, const MsgVer ver, MsgFields npb_fields, const MsgSize npb_size,
                           const MsgSize struct_size, void *const data_impl ) :
        id( id ),
        version( ver ), fields( npb_fields ),
        max_wire_size( std::max<MsgSize>( COBS_ENCODE_DST_BUF_LEN_MAX( npb_size ), COBS_DECODE_DST_BUF_LEN_MAX( npb_size ) ) + COBS_TERM_SIZE  + COBS_CRC_SIZE ),
        max_data_size( struct_size ), data_impl( data_impl ), data_size( struct_size )
    {
    }

    /**
     * @brief Default initialize the message data
     */
    void init()
    {
      if( data_impl )
      {
        memset( data_impl, 0, max_data_size );
        mbed_rpc_Header *header = reinterpret_cast<mbed_rpc_Header *>( data_impl );

        header->version = version;
        header->msgId   = id;
        data_size       = max_data_size;
      }
    }

    /**
     * @brief Encode a NanoPB message into a COBS encoded frame, ready for transmission.
     *
     * @param frame       Frame buffer to encode the data into
     * @param frame_size  Max bytes the frame can hold
     * @return true       Data was successfully encoded
     * @return false      Data could not be encoded
     */
    size_t encode_to_wire( uint8_t *const frame, const size_t frame_size ) const
    {
      /*-----------------------------------------------------------------------
      Input Validation
      -----------------------------------------------------------------------*/
      if( !data_impl || !frame || ( frame_size < max_wire_size) || ( data_size > max_data_size ) )
      {
        mbed_dbg_assert_continue_always();
        return 0;
      }

      /*-----------------------------------------------------------------------
      Make sure the header is up to date for this message
      -----------------------------------------------------------------------*/
      auto header     = reinterpret_cast<mbed_rpc_Header *>( data_impl );
      header->version = version;
      header->msgId   = id;

      /*-----------------------------------------------------------------------
      Prepend the CRC16 value to the frame this is computed over the entire
      message data structure.
      -----------------------------------------------------------------------*/
      etl::crc16_xmodem crc_calculator;

      uint8_t *data_ptr = reinterpret_cast<uint8_t *>( data_impl );
      std::copy( data_ptr, data_ptr + data_size, crc_calculator.input() );

      uint16_t *crc_ptr = reinterpret_cast<uint16_t *>( frame );
      *crc_ptr          = crc_calculator.value();

      /*-----------------------------------------------------------------------
      Encode the NanoPB message
      -----------------------------------------------------------------------*/
      pb_ostream_t stream = pb_ostream_from_buffer( frame + COBS_CRC_SIZE, frame_size - COBS_CRC_SIZE );
      if( !pb_encode( &stream, fields, data_impl ) )
      {
        mbed_dbg_assert_continue_always();
        return 0;
      }

      /*-----------------------------------------------------------------------
      Frame the data using COBS encoding
      -----------------------------------------------------------------------*/
      cobs_encode_result result = cobs_encode( frame, frame_size, frame, stream.bytes_written );
      if( result.status != COBS_ENCODE_OK )
      {
        mbed_dbg_assert_continue_always();
        return 0;
      }

      /*-----------------------------------------------------------------------
      Enforce null termination of the COBS frame
      -----------------------------------------------------------------------*/
      mbed_dbg_assert( result.out_len < frame_size );
      frame[ result.out_len ] = '\0';

      return result.out_len + COBS_TERM_SIZE;
    }

    /**
     * @brief Decode a NanoPB encoded frame into the original data structure.
     *
     * @param frame   Frame buffer to decode from
     * @param size    Number of bytes to decode
     * @return true   Data was successfully decoded
     * @return false  Data could not be decoded
     */
    bool decode_nanopb_frame( uint8_t *const frame, const size_t size ) const
    {
      /*-----------------------------------------------------------------------
      Input Validation
      -----------------------------------------------------------------------*/
      if( !data_impl || !frame || size > max_wire_size )
      {
        mbed_dbg_assert_continue_always();
        return false;
      }

      /*-----------------------------------------------------------------------
      Decode the NanoPB message
      -----------------------------------------------------------------------*/
      pb_istream_t stream = pb_istream_from_buffer( frame, size );
      if( !pb_decode( &stream, fields, data_impl ) )
      {
        mbed_assert_continue_msg( false, stream.errmsg );
        return false;
      }

      return true;
    }
  };

}    // namespace mb::rpc

namespace mb::rpc::messages
{
  /* MSG_NULL */
  class NullMessage : public IRPCMessage
  {
  public:
    constexpr NullMessage() : IRPCMessage( mbed_rpc_BuiltinMessage_MSG_NULL, 0, nullptr, 0, 0, nullptr )
    {
    }

    ~NullMessage() = default;
  };


  /* MSG_ERROR */
  DECLARE_RPC_MESSAGE( ErrorMessage, mbed_rpc_BuiltinMessage_MSG_ERROR, mbed_rpc_BuiltinMessageVersion_MSG_VER_ERROR,
                       mbed_rpc_ErrorMessage_fields, mbed_rpc_ErrorMessage_size, mbed_rpc_ErrorMessage );

  /* MSG_PING */
  DECLARE_RPC_MESSAGE( PingMessage, mbed_rpc_BuiltinMessage_MSG_PING, mbed_rpc_BuiltinMessageVersion_MSG_VER_PING,
                       mbed_rpc_PingMessage_fields, mbed_rpc_PingMessage_size, mbed_rpc_PingMessage );
}    // namespace mb::rpc::messages

#endif /* !MBEDUTILS_RPC_COMMON_HPP */
