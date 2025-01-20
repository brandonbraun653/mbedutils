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
#include <etl/algorithm.h>
#include <etl/atomic.h>
#include <etl/crc.h>
#include <inttypes.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/rpc/rpc_common.hpp>
#include <mbedutils/drivers/rpc/rpc_message.hpp>
#include <mbedutils/drivers/threading/lock.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <mbedutils/osal.hpp>
#include <pb_decode.h>
#include <pb_encode.h>

namespace mb::rpc::message
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief Absolute smallest size a message can be transcoded to
   */
  static constexpr size_t MIN_TRANSCODE_SIZE = TranscodeSize( mbed_rpc_BaseMessage_size );

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static DescriptorRegistry            *s_msg_reg;
  static volatile SeqId                 s_msg_uuid;
  static mb::osal::mb_mutex_t           s_seq_mutex;
  static mb::osal::mb_recursive_mutex_t s_msg_reg_rmutex;
  static size_t                         s_largest_known_wire_message;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize( DescriptorRegistry *const msgReg )
  {
    s_msg_reg                    = msgReg;
    s_msg_uuid                   = 0;
    s_largest_known_wire_message = 0;

    mbed_assert( mb::osal::buildMutexStrategy( s_seq_mutex ) );
    mbed_assert( mb::osal::buildRecursiveMutexStrategy( s_msg_reg_rmutex ) );
  }


  SeqId next_seq_id()
  {
    mb::osal::enterCritical( s_seq_mutex );

    SeqId next_id = s_msg_uuid;
    s_msg_uuid    = ( s_msg_uuid + 1 ) % std::numeric_limits<SeqId>::max();

    mb::osal::exitCritical( s_seq_mutex );

    return next_id;
  }


  bool addDescriptor( const Descriptor &dsc )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !s_msg_reg )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( s_msg_reg_rmutex );
    if( s_msg_reg->full() )
    {
      mbed_dbg_assert_continue_msg( false, "Unable to add message %d, registry is full", dsc.id );
      return false;
    }

    /*-------------------------------------------------------------------------
    Check for duplicate messages
    -------------------------------------------------------------------------*/
    auto msg_iter = s_msg_reg->find( dsc.id );
    if( msg_iter != s_msg_reg->end() )
    {
      bool same_message = ( msg_iter->second.fields == dsc.fields ) && ( msg_iter->second.version == dsc.version );
      mbed_dbg_assert_continue_msg( same_message, "Message %d already exists in the registry", dsc.id );
      return same_message;
    }

    /*-------------------------------------------------------------------------
    Validate the descriptor
    -------------------------------------------------------------------------*/
    if( ( dsc.fields == nullptr ) || ( dsc.transcode_size < MIN_TRANSCODE_SIZE ) )
    {
      mbed_dbg_assert_continue_always();
      return false;
    }

    /*-------------------------------------------------------------------------
    Update the largest known message size
    -------------------------------------------------------------------------*/
    if( dsc.transcode_size > s_largest_known_wire_message )
    {
      s_largest_known_wire_message = dsc.transcode_size;
    }

    s_msg_reg->insert( { dsc.id, dsc } );
    return true;
  }


  void removeDescriptor( const MsgId id )
  {
    /*-------------------------------------------------------------------------
    Find the message being erased
    -------------------------------------------------------------------------*/
    if( !s_msg_reg )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _lock( s_msg_reg_rmutex );
    auto                           msg_iter = s_msg_reg->find( id );
    if( msg_iter == s_msg_reg->end() )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Erase the message
    -------------------------------------------------------------------------*/
    size_t txcode_size_of_erased_msg = msg_iter->second.transcode_size;
    s_msg_reg->erase( id );

    /*-------------------------------------------------------------------------
    Search for the next largest message size
    -------------------------------------------------------------------------*/
    if( txcode_size_of_erased_msg == s_largest_known_wire_message )
    {
      s_largest_known_wire_message = 0;
      for( auto &msg : *s_msg_reg )
      {
        if( msg.second.transcode_size > s_largest_known_wire_message )
        {
          s_largest_known_wire_message = msg.second.transcode_size;
        }
      }
    }
  }


  const Descriptor *getDescriptor( const MsgId id )
  {
    if( !s_msg_reg )
    {
      return nullptr;
    }

    auto msg_iter = s_msg_reg->find( id );
    if( msg_iter == s_msg_reg->end() )
    {
      return nullptr;
    }

    return &msg_iter->second;
  }


  size_t encode_to_wire( const MsgId msg_id, void *const npb_struct, void *cobs_out_buf, const size_t cobs_out_size )
  {
    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    if( !s_msg_reg || !npb_struct || !cobs_out_buf || ( cobs_out_size < MIN_TRANSCODE_SIZE ) )
    {
      mbed_dbg_assert_continue_always();
      return 0;
    }

    /* Find the message in the descriptor registry */
    auto msg_iter = s_msg_reg->find( msg_id );
    if( msg_iter == s_msg_reg->end() )
    {
      mbed_assert_continue_msg( false, "Message type %d not found", msg_id );
      return 0;
    }

    /* Check buffer sizing requirements */
    const Descriptor &msg_dsc = msg_iter->second;
    if( msg_dsc.transcode_size > cobs_out_size )
    {
      uintptr_t address_of_buf = reinterpret_cast<uintptr_t>( cobs_out_buf );
      mbed_assert_continue_msg( false, "Buf 0x%0" PRIXPTR " too small for msg %d: %u > %u", address_of_buf, msg_id,
                                msg_dsc.transcode_size, cobs_out_size );
      return 0;
    }

    /*-------------------------------------------------------------------------
    Make sure the header is up to date for this message
    -------------------------------------------------------------------------*/
    auto header     = static_cast<mbed_rpc_Header *>( npb_struct );
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

    uint8_t *cob_start = static_cast<uint8_t *>( cobs_out_buf );
    uint8_t *npb_start = cob_start + NPB_START_OFFSET;
    size_t   npb_size  = cobs_out_size - NPB_START_OFFSET - COBS_TERM_SIZE;

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
    uint16_t crc = crc_calculator.value();

    cob_start[ 2 ] = ( crc >> 8 ) & 0xFF;
    cob_start[ 1 ] = crc & 0xFF;

    /*-----------------------------------------------------------------------
    Frame the data using COBS encoding
    -----------------------------------------------------------------------*/
    cobs_encode_result result =
        cobs_encode( cobs_out_buf, cobs_out_size, cob_start + CRC_START_OFFSET, stream.bytes_written + COBS_CRC_SIZE );
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


  size_t decode_from_wire( void *const cobs_in_buf, const size_t cobs_in_size )
  {
    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    if( !s_msg_reg || !cobs_in_buf )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Check null termination of the COBS frame
    -------------------------------------------------------------------------*/
    char  *data_frame    = static_cast<char *>( cobs_in_buf );
    size_t null_term_pos = cobs_in_size - 1;
    if( !mbed_assert_continue( data_frame[ null_term_pos ] == '\0' ) )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Decode the COBS framed message.

    Input Framing:
    [COBS Start][CRC16][NanoPB Data][COBS Term]
    [  1 byte  ][  2  ][ Variable  ][ 1 byte  ]

    Output Framing:
    [CRC16][NanoPB Data]
    [  2  ][ Variable  ]
    -------------------------------------------------------------------------*/
    auto cobs_result = cobs_decode( cobs_in_buf, cobs_in_size, cobs_in_buf, null_term_pos );
    if( cobs_result.status != COBS_DECODE_OK )
    {
      mbed_assert_continue_msg( false, "COBS decode fail: %d", cobs_result.status );
      return 0;
    }

    /*-------------------------------------------------------------------------
    Validate the CRC field of the message. This is the first 2 bytes of the
    frame, prepended by the sender.
    -------------------------------------------------------------------------*/
    etl::crc16_xmodem crc_calculator;
    crc_calculator.reset();

    char  *npb_start        = data_frame + COBS_CRC_SIZE;
    char  *nanopb_data_end  = data_frame + cobs_result.out_len;
    size_t nanopb_data_size = nanopb_data_end - npb_start;
    etl::copy( npb_start, nanopb_data_end, crc_calculator.input() );

    const uint16_t expected_crc = *reinterpret_cast<uint16_t *>( cobs_in_buf );
    const uint16_t actual_crc   = crc_calculator.value();

    if( expected_crc != actual_crc )
    {
      mbed_assert_continue_msg( false, "CRC mismatch. Exp: %d, Act: %d", expected_crc, actual_crc );
      return 0;
    }

    /*-------------------------------------------------------------------------
    Peek at the base message to determine the RPC type and ID
    -------------------------------------------------------------------------*/
    mbed_rpc_BaseMessage base_msg;
    pb_istream_t         stream = pb_istream_from_buffer( reinterpret_cast<pb_byte_t *>( npb_start ), nanopb_data_size );

    if( !pb_decode( &stream, mbed_rpc_BaseMessage_fields, &base_msg ) )
    {
      mbed_assert_continue_msg( false, "Failed to decode BaseMessage: %s", stream.errmsg );
      return 0;
    }

    /*-------------------------------------------------------------------------
    Find the message in the registry and validate it.
    -------------------------------------------------------------------------*/
    auto base_msg_iter = s_msg_reg->find( base_msg.header.msgId );
    if( base_msg_iter == s_msg_reg->end() )
    {
      mbed_assert_continue_msg( false, "Message %d not found", base_msg.header.msgId );
      return 0;
    }

    auto base_msg_dsc = base_msg_iter->second;

    /* Validate the message version */
    if( base_msg.header.version != base_msg_dsc.version )
    {
      mbed_assert_continue_msg( false, "Version mismatch for msg %d. Exp: %d, Act: %d", base_msg.header.msgId,
                                base_msg_dsc.version, base_msg.header.version );
      return 0;
    }

    /* Can't decode if the actual data is bigger than the message buffer */
    if( cobs_in_size > base_msg_dsc.transcode_size )
    {
      uintptr_t address_of_buf = reinterpret_cast<uintptr_t>( cobs_in_buf );
      mbed_assert_continue_msg( false, "Buf 0x%0" PRIXPTR " too small for decoding msg %d: %zu > %zu", address_of_buf,
                                base_msg.header.msgId, base_msg_dsc.transcode_size, cobs_in_size );
      return 0;
    }

    /*-------------------------------------------------------------------------
    Decode the full NanoPB message

    Input Framing:
    [CRC16][NanoPB Data]
    [  2  ][ Variable  ]

    Output Framing:
    [NanoPB Data]
    [ Variable  ]
    -------------------------------------------------------------------------*/
    stream = pb_istream_from_buffer( reinterpret_cast<pb_byte_t *>( npb_start ), nanopb_data_size );
    if( !pb_decode_ex( &stream, base_msg_iter->second.fields, cobs_in_buf, PB_DECODE_NOINIT ) )
    {
      mbed_assert_continue_msg( false, stream.errmsg );
      return 0;
    }

    return nanopb_data_size - stream.bytes_left;
  }


  size_t largest_known_wire_message()
  {
    return s_largest_known_wire_message;
  }

}    // namespace mb::rpc::message
