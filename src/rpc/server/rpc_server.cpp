/******************************************************************************
 *  File Name:
 *    rpc_server.cpp
 *
 *  Description:
 *    Implementation of the RPC server interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cobs.h>
#include <mbedutils/rpc.hpp>
#include <nanoprintf.h>

namespace mb::rpc::server
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  Server::Server() : mIsOpen( false ), mCfg( {} )
  {
  }


  Server::~Server()
  {
  }


  bool Server::open( const Config &config )
  {
    using namespace mb::hw::serial;

    /*-------------------------------------------------------------------------
    Validate the configuration
    -------------------------------------------------------------------------*/
    /* clang-format off */
    if( mIsOpen                  || !config.iostream
     || !config.txBuffer         || !config.rxBuffer
     || !config.svcReg           || !config.msgReg
     || !config.txBuffer->size() || !config.rxBuffer->size()
     || !config.txScratch        || !config.rxScratch
     || !config.txScratchSize    || !config.rxScratchSize )
    { /* clang-format on */
      mbed_dbg_assert_continue_always();
      return false;
    }

    /*-------------------------------------------------------------------------
    Initialize inherited interfaces
    -------------------------------------------------------------------------*/
    this->initLockable();

    /*-------------------------------------------------------------------------
    Bind the IO stream to the server
    -------------------------------------------------------------------------*/
    if( !config.iostream->try_lock_for( 100 ) )
    {
      mbed_assert_continue_msg( false, "Failed to acquire the IO stream" );
      return false;
    }

    auto write_cb = CompletionCallback::create<Server, &Server::isr_on_io_write_complete>( *this );
    config.iostream->onWriteComplete( write_cb );

    auto read_cb = CompletionCallback::create<Server, &Server::isr_on_io_read_complete>( *this );
    config.iostream->onReadComplete( read_cb );

    /*-------------------------------------------------------------------------
    Initialize the configuration memory
    -------------------------------------------------------------------------*/
    config.svcReg->clear();
    config.msgReg->clear();
    config.txBuffer->clear();
    config.rxBuffer->clear();
    memset( config.txScratch, 0, config.txScratchSize );
    memset( config.rxScratch, 0, config.rxScratchSize );

    mCfg = config;
    mIsOpen = true;

    /*-------------------------------------------------------------------------
    Bind builtin services and messages to the server
    -------------------------------------------------------------------------*/
    mbed_assert_continue( this->addMessage( messages::PingMessage() ) );
    mbed_assert_continue( this->addService( services::PingService() ) );

    return true;
  }


  void Server::close()
  {
    if( mIsOpen )
    {
      mCfg.iostream->unlock();
      mIsOpen = false;
    }
  }


  void Server::runServices()
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen || !this->try_lock() )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Drain the RX buffer and process any incoming requests. Be sure to pump
    the TX buffer as well to ensure we don't accidentally fill it when a large
    number of requests are backlogged.
    -------------------------------------------------------------------------*/
    while( this->process_next_request() )
    {
      this->write_cobs_frame();
    }

    /*-------------------------------------------------------------------------
    Kick start the TX pump should it have gotten stalled. This will have no
    effect if already pumping. This is a good way to ensure that the TX buffer
    is always being emptied even if the RPC service implementer didn't follow
    the proper async response pattern.
    -------------------------------------------------------------------------*/
    this->write_cobs_frame();
    this->unlock();
  }


  bool Server::addService( const IRPCService &service )
  {
    return this->addService( std::move( service ) );
  }


  bool Server::addService( const IRPCService &&service )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Inject the service
    -------------------------------------------------------------------------*/
    if( mCfg.svcReg->full() )
    {
      mbed_assert_continue_msg( false, "Unable to add service %d, registry is full", service.getServiceId() );
      return false;
    }

    mCfg.svcReg->insert( { service.getServiceId(), service } );
    return true;
  }


  bool Server::addMessage( const IRPCMessage &message )
  {
    return this->addMessage( std::move( message ) );
  }


  bool Server::addMessage( const IRPCMessage &&message )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Inject the message
    -------------------------------------------------------------------------*/
    if( mCfg.msgReg->full() )
    {
      mbed_assert_continue_msg( false, "Unable to add message %d, registry is full", message.id );
      return false;
    }

    mCfg.msgReg->insert( { message.id, message } );
    return true;
  }


  void Server::removeService( const SvcId id )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Remove the service
    -------------------------------------------------------------------------*/
    mCfg.svcReg->erase( id );
  }


  void Server::removeMessage( const MsgId id )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Remove the message
    -------------------------------------------------------------------------*/
    mCfg.msgReg->erase( id );
  }


  void Server::throwError( const size_t txn_id, const mbed_rpc_ErrorCode error_code, const etl::string_view msg )
  {
    /*-------------------------------------------------------------------------
    Prepare the error message
    -------------------------------------------------------------------------*/
    messages::ErrorMessage err_msg;

    err_msg.init();
    err_msg.message.error = error_code;

    if( msg.size() > 0 )
    {
      mbed_assert_continue_msg( false, "RPC error -- Seq: %d, Err: %d, Msg: %s", txn_id, error_code, msg.data() );

      err_msg.message.detail.size = etl::min<pb_size_t>( msg.size(), sizeof( err_msg.message.detail.bytes ) );
      memcpy( err_msg.message.detail.bytes, msg.data(), err_msg.message.detail.size );
    }
    else
    {
      mbed_assert_continue_msg( false, "RPC error -- Seq: %d, Err: %d", txn_id, error_code );
    }

    /*-------------------------------------------------------------------------
    Publish the error message
    -------------------------------------------------------------------------*/
    publishMessage( err_msg );
  }


  bool Server::publishMessage( IRPCMessage &dsc )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Encode the message into the scratch tx buffer
    -------------------------------------------------------------------------*/
    if( !dsc.encode_to_wire( mCfg.txScratch, mCfg.txScratchSize ) )
    {
      mbed_assert_continue_msg( false, "Failed to encode message %d", dsc.id );
      return false;
    }

    return write_cobs_frame();
  }


  /**
   * @brief Pulls a single request from the RX buffer and processes it.
   *
   * This function is **not** thread safe and should only be called from a
   * protected context. Message request and response data is stored in the
   * registered message descriptors, meaning multiple requests cannot be
   * processed simultaneously, because they may share the same message type.
   *
   * @return true  A request was processed
   * @return false No requests available to process
   */
  bool Server::process_next_request()
  {
    char err_msg[ 128 ] = { 0 };

    /*-------------------------------------------------------------------------
    Try to pull a COBS frame out of the RX buffer and into the scratch buffer
    -------------------------------------------------------------------------*/
    if( !this->read_cobs_frame() )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Decode the message. Output size is always less than or equal to input size.
    This will transform the buffer from COBS encoded to raw NanoPB data.
    -------------------------------------------------------------------------*/
    size_t decode_size = strlen( reinterpret_cast<char *>( mCfg.rxScratch ) );
    auto   cobs_result = cobs_decode( mCfg.rxScratch, mCfg.rxScratchSize, mCfg.rxScratch, decode_size );

    if( cobs_result.status != COBS_DECODE_OK )
    {
      mbed_assert_continue_msg( false, "Failed to decode COBS frame" );
      return false;
    }

    /*-------------------------------------------------------------------------
    Peek at the base message to determine the RPC type and ID
    -------------------------------------------------------------------------*/
    mbed_rpc_BaseMessage msg;
    pb_istream_t stream = pb_istream_from_buffer( mCfg.rxScratch, cobs_result.out_len );

    if( !pb_decode( &stream, mbed_rpc_BaseMessage_fields, &msg ) )
    {
      mbed_assert_continue_msg( false, "Failed to decode base message" );
      return false;
    }

    /*-------------------------------------------------------------------------
    Find the service and message in the registry, then validate processability.
    -------------------------------------------------------------------------*/
    auto svc_iter = mCfg.svcReg->find( msg.header.svcId );
    auto msg_req_iter = mCfg.msgReg->find( msg.header.msgId );

    if( svc_iter == mCfg.svcReg->end() )
    {
      npf_snprintf( err_msg, sizeof( err_msg ), "Service %d not found", msg.header.svcId );
      throwError( msg.header.seqId, mbed_rpc_ErrorCode_ERR_SVC_NOT_FOUND, err_msg );
      return false;
    }

    if( msg_req_iter == mCfg.msgReg->end() )
    {
      npf_snprintf( err_msg, sizeof( err_msg ), "Message %d not found", msg.header.msgId );
      throwError( msg.header.seqId, mbed_rpc_ErrorCode_ERR_MSG_NOT_FOUND, err_msg );
      return false;
    }

    if( svc_iter->second.getRequestMessageId() != msg.header.msgId )
    {
      npf_snprintf( err_msg, sizeof( err_msg ), "Service %d does not accept message %d", msg.header.svcId,
                    msg.header.msgId );
      throwError( msg.header.seqId, mbed_rpc_ErrorCode_ERR_SVC_MSG, err_msg );
      return false;
    }

    /*-------------------------------------------------------------------------
    Find the response message type
    -------------------------------------------------------------------------*/
    auto msg_rsp_iter = mCfg.msgReg->find( svc_iter->second.getResponseMessageId() );
    if( msg_rsp_iter == mCfg.msgReg->end() )
    {
      mbed_assert_continue_msg( false, "RPC response msg %d not found", svc_iter->second.getResponseMessageId() );
      return false;
    }

    /*-------------------------------------------------------------------------
    Fully decode the message. Internally this will also validate the CRC field.
    -------------------------------------------------------------------------*/
    if( !msg_req_iter->second.decode_nanopb_frame( mCfg.rxScratch, cobs_result.out_len ) )
    {
      npf_snprintf( err_msg, sizeof( err_msg ), "Failed to decode NanoPB message %d", msg.header.msgId );
      throwError( msg.header.seqId, mbed_rpc_ErrorCode_ERR_MSG_DECODE, err_msg );
      return false;
    }

    /*-------------------------------------------------------------------------
    Invoke the service.
    -------------------------------------------------------------------------*/
    auto status = svc_iter->second.processRequest( *this, msg_req_iter->second, msg_rsp_iter->second );

    switch( status )
    {
      /*-----------------------------------------------------------------------
      Nominal path. Service completed and wants to send a response.
      -----------------------------------------------------------------------*/
      case mbed_rpc_ErrorCode_ERR_NO_ERROR:
        publishMessage( msg_rsp_iter->second );
        break;

      /*-----------------------------------------------------------------------
      Do nothing, the service will send the response asynchronously
      -----------------------------------------------------------------------*/
      case mbed_rpc_ErrorCode_ERR_SVC_ASYNC:
        break;

      /*-----------------------------------------------------------------------
      Fall through for all other error cases generated by the service.
      -----------------------------------------------------------------------*/
      default:
        throwError( msg.header.seqId, status, nullptr );
        break;
    }

    return true;
  }


  /**
   * @brief Reads the next full COBS frame from the RX buffer
   *
   * @return true  A full frame was read
   * @return false No full frame was read
   */
  bool Server::read_cobs_frame()
  {
    /*-------------------------------------------------------------------------
    Scan the RX buffer for a full frame
    -------------------------------------------------------------------------*/
    size_t scratch_idx = 0;

    for( auto iter = mCfg.rxBuffer->begin(); ( iter != mCfg.rxBuffer->end() ) && ( scratch_idx < mCfg.rxScratchSize );
         iter++ )
    {
      mCfg.rxScratch[ scratch_idx ] = *iter;

      if( *iter == 0 )
      {
        break;
      }

      scratch_idx++;
    }

    /*-------------------------------------------------------------------------
    Remove the frame data if a full COBS frame was found.
    -------------------------------------------------------------------------*/
    if( mCfg.rxScratch[ scratch_idx ] == 0 )
    {
      for( size_t i = 0; i < scratch_idx; i++ )
      {
        mCfg.rxBuffer->pop();
      }

      return true;
    }
    /*-------------------------------------------------------------------------
    If the buffer is full and no frame was found, then we have to flush it.
    This effectively means it's not possible to find a full frame anymore.
    -------------------------------------------------------------------------*/
    else if( scratch_idx == mCfg.rxScratchSize )
    {
      mbed_assert_continue_msg( false, "RX buffer full with no COBS frame. Discarding %d bytes.", scratch_idx );
      for( size_t i = 0; i < scratch_idx; i++ )
      {
        mCfg.rxBuffer->pop();
      }
    }

    return false;
  }


  /**
   * @brief Publishes a COBS frame to the wire.
   *
   * If the iostream is busy, it will be bufferred until the next write.
   *
   * @return true  The frame was successfully written
   * @return false The frame could not be written
   */
  bool Server::write_cobs_frame()
  {
    /*-------------------------------------------------------------------------
    Ensure null termination of the buffer, then get the length of the frame.
    COBS is null terminated, so we can use the scratch buffer directly.
    -------------------------------------------------------------------------*/
    mCfg.txScratch[ mCfg.txScratchSize - 1 ] = 0;

    size_t frame_size = strlen( reinterpret_cast<char *>( mCfg.txScratch ) );

    /*-------------------------------------------------------------------------
    Write the frame to the wire directly if we can.
    -------------------------------------------------------------------------*/
    size_t actual_size = 0;
    if( mCfg.txBuffer->empty() && ( mCfg.iostream->writeable() >= frame_size ) )
    {
      actual_size = mCfg.iostream->write( mCfg.txScratch, frame_size );
    }

    /*-------------------------------------------------------------------------
    Buffer any remaining bytes if the write failed for some reason or if the
    buffer contained data (indicating another write was already in progress).
    -------------------------------------------------------------------------*/
    size_t remaining = frame_size - actual_size;

    if( !remaining )
    {
      return true;
    }
    else if( mCfg.txBuffer->available() >= remaining )
    {
      for( size_t i = actual_size; i < frame_size; i++ )
      {
        mCfg.txBuffer->push( mCfg.txScratch[ i ] );
      }

      return true;
    }
    else
    {
      mbed_assert_continue_msg( false, "Lost %d bytes of COBS frame data", remaining );
      return false;
    }
  }


  /**
   * @brief Callback when the iostream has completed a write operation.
   * @warning This is an ISR context function.
   *
   * This is used to pump any pending bytes from the TX buffer out on the wire.
   */
  void Server::isr_on_io_write_complete()
  {
    if( !mIsOpen || mCfg.txBuffer->empty() )
    {
      return;
    }

    size_t write_size = etl::min<size_t>( mCfg.iostream->writeable(), mCfg.txBuffer->size() );
    if( write_size )
    {
      size_t actual_size = mCfg.iostream->write( *mCfg.txBuffer, write_size );
      mbed_dbg_assert_continue_msg( write_size == actual_size, "Failed to write all bytes to the wire" );
    }
  }


  /**
   * @brief Callback when the io stream has completed a read operation.
   * @warning This is an ISR context function.
   *
   * This is used to pump received bytes into the RX buffer to be processed.
   */
  void Server::isr_on_io_read_complete()
  {
    if( !mIsOpen || mCfg.rxBuffer->full() )
    {
      return;
    }

    size_t read_size = etl::min<size_t>( mCfg.iostream->readable(), mCfg.rxBuffer->available() );
    if( read_size )
    {
      size_t actual_size = mCfg.iostream->read( *mCfg.rxBuffer, read_size, 0 );
      mbed_dbg_assert_continue_msg( read_size == actual_size, "Failed to read all bytes from the wire" );
    }
  }

}    // namespace mb::rpc::server
