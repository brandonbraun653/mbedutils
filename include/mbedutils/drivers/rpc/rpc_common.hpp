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
#include <etl/circular_buffer.h>
#include <etl/crc16.h>
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
   * @brief Identifier for a unique message type
   */
  using MsgId = uint16_t;

  /**
   * @brief Storage for the size of a message.
   */
  using MsgSize = uint16_t;

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
   * @brief Alias for the NanoPB message descriptor
   */
  using MsgFields = const pb_msgdesc_t *;


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Describes a single RPC message type with key information for encoding/decoding.
   */
  class IRPCMessage
  {
  public:
    const MsgId     id;        /**< Unique identifier for the message */
    const MsgFields fields;    /**< NanoPB message descriptor */
    const MsgSize   data_size; /**< Size of the un-encoded data storage */
    const MsgSize   wire_size; /**< Size of the wire encoded message */

    /**
     * @brief Constructs the message descriptor
     *
     * @param id         Unique identifier for the message
     * @param npb_fields NanoPB message descriptor
     * @param npb_size   Size of the NanoPB message
     * @param data_size  Size of the un-encoded data storage
     */
    constexpr IRPCMessage( const MsgId id, MsgFields npb_fields, const MsgSize npb_size, const MsgSize data_size ) :
      id( id ),
      fields( npb_fields ),
      data_size( data_size ),
      wire_size( std::max<MsgSize>( COBS_ENCODE_DST_BUF_LEN_MAX( npb_size ), COBS_DECODE_DST_BUF_LEN_MAX( npb_size ) ) + 1u )
    {
    }

    /**
     * @brief Encode a NanoPB message into a COBS encoded frame, ready for transmission.
     *
     * @param data    Data structure to encode
     * @param frame   Frame buffer to encode the data into
     * @param size    Size of the frame buffer
     * @return true   Data was successfully encoded
     * @return false  Data could not be encoded
     */
    size_t encode_to_wire( void * data, uint8_t *const frame, const size_t size ) const
    {
      /*-----------------------------------------------------------------------
      Input Validation
      -----------------------------------------------------------------------*/
      if( !data || !frame || size < wire_size )
      {
        mbed_dbg_assert_continue_always();
        return 0;
      }

      /*-----------------------------------------------------------------------
      Update the CRC field in the header
      -----------------------------------------------------------------------*/
      mbed_rpc_Header* header = reinterpret_cast<mbed_rpc_Header*>( data );
      header->crc = 0xFFFF;

      auto data_ptr = reinterpret_cast<const uint8_t *const>( data );
      etl::crc16 crc_calculator;
      std::copy( data_ptr, data_ptr + data_size, crc_calculator.input() );

      header->crc = crc_calculator.value();

      /*-----------------------------------------------------------------------
      Encode the NanoPB message
      -----------------------------------------------------------------------*/
      pb_ostream_t stream = pb_ostream_from_buffer( frame, size );
      if( !pb_encode( &stream, fields, data ) )
      {
        mbed_dbg_assert_continue_always();
        return 0;
      }

      /*-----------------------------------------------------------------------
      Frame the data using COBS encoding
      -----------------------------------------------------------------------*/
      cobs_encode_result result = cobs_encode( frame, size, frame, stream.bytes_written );
      if( result.status != COBS_ENCODE_OK )
      {
        mbed_dbg_assert_continue_always();
        return 0;
      }

      return result.out_len;
    }

    /**
     * @brief Decode a COBS + NanoPB encoded frame into the original data structure.
     *
     * @param data    Data structure to decode into
     * @param frame   Frame buffer to decode from
     * @param size    Number of bytes to decode
     * @return true   Data was successfully decoded
     * @return false  Data could not be decoded
     */
    bool decode_from_wire( void * data, uint8_t *const frame, const size_t size ) const
    {
      /*-----------------------------------------------------------------------
      Input Validation
      -----------------------------------------------------------------------*/
      if( !data || !frame || size < wire_size )
      {
        mbed_dbg_assert_continue_always();
        return false;
      }

      /*-----------------------------------------------------------------------
      Decode the COBS frame
      -----------------------------------------------------------------------*/
      cobs_decode_result result = cobs_decode( frame, size, frame, size );
      if( result.status != COBS_DECODE_OK )
      {
        mbed_dbg_assert_continue_msg( false, "COBS decode failed: %d", result.status );
        return false;
      }

      /*-----------------------------------------------------------------------
      Decode the NanoPB message
      -----------------------------------------------------------------------*/
      pb_istream_t stream = pb_istream_from_buffer( frame, result.out_len );
      if( !pb_decode( &stream, fields, data ) )
      {
        mbed_dbg_assert_continue_always();
        return false;
      }

      /*-----------------------------------------------------------------------
      Validate the CRC field in the header
      -----------------------------------------------------------------------*/
      auto data_ptr = reinterpret_cast<const uint8_t *const>( data );

      etl::crc16 crc_calculator;
      std::copy( data_ptr, data_ptr + data_size, crc_calculator.input() );

      if( crc_calculator.value() != 0 )
      {
        mbed_dbg_assert_continue_always();
        return false;
      }

      return true;
    }
  };

}  // namespace mb::rpc

#endif  /* !MBEDUTILS_RPC_COMMON_HPP */
