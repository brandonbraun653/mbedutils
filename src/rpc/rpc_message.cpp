/******************************************************************************
 *  File Name:
 *    rpc_message.cpp
 *
 *  Description:
 *    Message encode/decode implementations
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/crc.h>
#include <mbedutils/drivers/rpc/rpc_message.hpp>
#include <mbedutils/assert.hpp>
#include <pb_decode.h>
#include <pb_encode.h>

namespace mb::rpc::message
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static DescriptorRegistry * s_msg_reg;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize( DescriptorRegistry *const msgReg )
  {
    s_msg_reg = msgReg;
  }


  bool addDescriptor( const MsgId id, MsgDsc *const msg )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !s_msg_reg || !msg )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Inject the message
    -------------------------------------------------------------------------*/
    if( s_msg_reg->full() )
    {
      mbed_assert_continue_msg( false, "Unable to add message %d, registry is full", id );
      return false;
    }

    s_msg_reg->insert( { id, *msg } );
    return true;
  }


  void removeDescriptor( const MsgId id )
  {
    s_msg_reg->erase( id );
  }


  size_t encode_to_wire( const MsgId msg_id, void *const npb_struct, void *cobs_out_buf, const size_t cobs_out_size )
  {
    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    if( !npb_struct || !cobs_out_buf || !cobs_out_size )
    {
      mbed_dbg_assert_continue_always();
      return 0;
    }

    auto msg_iter = s_msg_reg->find( msg_id );
    if( msg_iter == s_msg_reg->end() )
    {
      mbed_assert_continue_msg( false, "Message type %d not found", msg_id );
      return 0;
    }

    const MsgDsc &msg_dsc = msg_iter->second;

    /*-------------------------------------------------------------------------
    Make sure the header is up to date for this message
    -------------------------------------------------------------------------*/
    auto header     = reinterpret_cast<mbed_rpc_Header *>( npb_struct );
    header->version = msg_dsc.version;
    header->msgId   = msg_id;

    /*-------------------------------------------------------------------------
    Compute the buffer offsets for the following steps. COBS encoding requires
    a special positioning of the CRC and NanoPB data. This is the layout:

    [COBS Start][CRC16][NanoPB Data][COBS Term]
    [  1 byte  ][  2  ][ Variable  ][ 1 byte  ]
    -------------------------------------------------------------------------*/
    static constexpr size_t CRC_START_OFFSET = 1u;
    static constexpr size_t NPB_START_OFFSET = CRC_START_OFFSET + COBS_CRC_SIZE;

    uint8_t  *cob_start = reinterpret_cast<uint8_t *>( cobs_out_buf );
    uint16_t *crc_start = reinterpret_cast<uint16_t *>( cob_start + CRC_START_OFFSET );
    uint8_t  *npb_start = cob_start + NPB_START_OFFSET;
    size_t    npb_size  = cobs_out_size - NPB_START_OFFSET - COBS_TERM_SIZE;

    /*-------------------------------------------------------------------------
    Encode the NanoPB message
    -------------------------------------------------------------------------*/
    pb_ostream_t stream = pb_ostream_from_buffer( npb_start, npb_size );
    if( !pb_encode( &stream, msg_dsc.fields, npb_struct ) )
    {
      mbed_dbg_assert_continue_always();
      return 0;
    }

    /*-------------------------------------------------------------------------
    Compute the frame's CRC16 value
    -------------------------------------------------------------------------*/
    etl::crc16_xmodem crc_calculator;
    crc_calculator.reset();
    std::copy( npb_start, npb_start + stream.bytes_written, crc_calculator.input() );
    *crc_start = crc_calculator.value();

    /*-----------------------------------------------------------------------
    Frame the data using COBS encoding
    -----------------------------------------------------------------------*/
    cobs_encode_result result = cobs_encode( cobs_out_buf, cobs_out_size, crc_start, stream.bytes_written + COBS_CRC_SIZE );
    if( result.status != COBS_ENCODE_OK )
    {
      mbed_dbg_assert_continue_always();
      return 0;
    }

    /*-----------------------------------------------------------------------
    Enforce null termination of the COBS frame
    -----------------------------------------------------------------------*/
    if( ( result.out_len + COBS_TERM_SIZE ) > cobs_out_size )
    {
      mbed_assert_continue_always();
      return 0;
    }

    cob_start[ result.out_len ] = '\0';
    return result.out_len + COBS_TERM_SIZE;
  }


  bool decode_from_wire( MsgId &msg_id, void *const cobs_in_buf, const size_t cobs_in_size )
  {
    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    if( !cobs_in_buf || !cobs_in_size )
    {
      mbed_dbg_assert_continue_always();
      return false;
    }

    /*-------------------------------------------------------------------------
    Force null termination of the COBS frame
    -------------------------------------------------------------------------*/
    uint8_t *data_frame = reinterpret_cast<uint8_t *>( cobs_in_buf );
    data_frame[ cobs_in_size - 1 ] = '\0';

    /*-------------------------------------------------------------------------
    Decode the COBS framed message. Output size is always less than or equal
    to input size. This will transform the buffer from COBS to raw NanoPB data.

    Input Framing:
    [COBS Start][CRC16][NanoPB Data][COBS Term]
    [  1 byte  ][  2  ][ Variable  ][ 1 byte  ]

    Output Framing:
    [CRC16][NanoPB Data]
    [  2  ][ Variable  ]
    -------------------------------------------------------------------------*/
    size_t decode_size = strlen( reinterpret_cast<char *>( data_frame ) );
    auto   cobs_result = cobs_decode( cobs_in_buf, cobs_in_size, cobs_in_buf, decode_size );

    if( cobs_result.status != COBS_DECODE_OK )
    {
      mbed_assert_continue_msg( false, "Failed to decode COBS frame" );
      return false;
    }

    /*-------------------------------------------------------------------------
    Validate the CRC field of the message. This is the first 2 bytes of the
    frame, prepended by the sender.
    -------------------------------------------------------------------------*/
    etl::crc16_xmodem crc_calculator;

    uint8_t *npb_start         = data_frame + COBS_CRC_SIZE;
    uint8_t *nanopb_data_end   = data_frame + cobs_result.out_len;
    size_t   nanopb_data_size  = nanopb_data_end - npb_start;
    std::copy( npb_start, nanopb_data_end, crc_calculator.input() );

    const uint16_t expected_crc = *reinterpret_cast<uint16_t *>( data_frame );
    const uint16_t actual_crc   = crc_calculator.value();

    if( expected_crc != actual_crc )
    {
      mbed_assert_continue_msg( false, "CRC mismatch. Expected: %d, Actual: %d", expected_crc, actual_crc );
      return false;
    }

    /*-------------------------------------------------------------------------
    Peek at the base message to determine the RPC type and ID
    -------------------------------------------------------------------------*/
    mbed_rpc_BaseMessage msg;
    pb_istream_t         stream = pb_istream_from_buffer( npb_start, nanopb_data_size );

    if( !pb_decode( &stream, mbed_rpc_BaseMessage_fields, &msg ) )
    {
      mbed_assert_continue_msg( false, "Failed to decode BaseMessage: %s", stream.errmsg );
      return false;
    }

    /*-------------------------------------------------------------------------
    Find the service and message in the registry, then validate processability.
    -------------------------------------------------------------------------*/
    auto msg_iter = s_msg_reg->find( msg.header.msgId );
    if( msg_iter == s_msg_reg->end() )
    {
      mbed_assert_continue_msg( false, "Message %d not found", msg.header.msgId );
      return false;
    }

    /*-------------------------------------------------------------------------
    Decode the full NanoPB message
    -------------------------------------------------------------------------*/
    stream = pb_istream_from_buffer( npb_start, nanopb_data_size );
    if( !pb_decode( &stream, msg_iter->second.fields, npb_start ) )
    {
      mbed_assert_continue_msg( false, stream.errmsg );
      return false;
    }

    return true;
  }

}    // namespace mb::rpc
