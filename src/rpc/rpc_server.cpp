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
    if( mIsOpen || !config.iostream || !config.registry
     || !config.streamBuffer        || !config.streamBuffer->available()
     || !config.encodeBuffer.size() || !config.decodeBuffer.size() )
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
    if( !config.iostream->try_lock_for( mb::thread::TIMEOUT_100MS ) )
    {
      mbed_assert_continue_msg( false, "Failed to acquire the IO stream" );
      return false;
    }

    auto read_cb = CompletionCallback::create<Server, &Server::isr_on_io_read_complete>( *this );
    config.iostream->onReadComplete( read_cb );
    config.iostream->unlock();

    /*-------------------------------------------------------------------------
    Initialize the configuration memory
    -------------------------------------------------------------------------*/
    config.registry->clear();
    config.streamBuffer->clear();
    memset( config.encodeBuffer.data(), 0, config.encodeBuffer.max_size() );
    memset( config.decodeBuffer.data(), 0, config.decodeBuffer.max_size() );

    mCfg    = config;
    mIsOpen = true;

    /*-------------------------------------------------------------------------
    Bind default services and messages to the server
    -------------------------------------------------------------------------*/
    /* Services */
    mbed_assert_continue( addService( &mPingService ) );
    mbed_assert_continue( addService( &mTestErrorService ) );

    /* Messages */
    mbed_assert_continue( message::addDescriptor( message::NullMessage ) );
    mbed_assert_continue( message::addDescriptor( message::PingMessage ) );
    mbed_assert_continue( message::addDescriptor( message::ErrorMessage ) );
    mbed_assert_continue( message::addDescriptor( message::ConsoleMessage ) );

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


  bool Server::addService( ::mb::rpc::IService *const service )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen || !service )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Inject the service
    -------------------------------------------------------------------------*/
    if( mCfg.registry->full() )
    {
      mbed_assert_continue_msg( false, "Registry is full: %s", service->name );
      return false;
    }

    mCfg.registry->insert( { service->svcId, service } );
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
    mCfg.registry->erase( id );
  }


  void Server::throwError( const size_t txn_id, const mbed_rpc_ErrorCode error_code, const etl::string_view msg )
  {
    /*-------------------------------------------------------------------------
    Prepare the error message
    -------------------------------------------------------------------------*/
    mbed_rpc_ErrorMessage err_msg;

    err_msg.error = error_code;

    if( msg.size() > 0 )
    {
      mbed_assert_continue_msg( false, "RPC error -- Seq: %d, Err: %d, Msg: %s", txn_id, error_code, msg.data() );

      err_msg.detail.size = etl::min<pb_size_t>( msg.size(), sizeof( err_msg.detail.bytes ) );
      memcpy( err_msg.detail.bytes, msg.data(), err_msg.detail.size );
    }
    else
    {
      mbed_assert_continue_msg( false, "RPC error -- Seq: %d, Err: %d", txn_id, error_code );
    }

    /*-------------------------------------------------------------------------
    Publish the error message
    -------------------------------------------------------------------------*/
    publishMessage( message::ErrorMessage.id, &err_msg );
  }


  bool Server::publishMessage( const MsgId id, void *const data )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    if( mCfg.encodeBuffer[ 0 ] != 0 )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Encode the message into the scratch tx buffer
    -------------------------------------------------------------------------*/
    if( !message::encode_to_wire( id, data, mCfg.encodeBuffer.data(), mCfg.encodeBuffer.size() ) )
    {
      mbed_assert_continue_msg( false, "Failed to encode message %d", id );
      mCfg.encodeBuffer[ 0 ] = 0;
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
    mb::rpc::IService                  *service      = nullptr;
    mbed_rpc_Header                    *req_header   = nullptr;
    const mb::rpc::message::Descriptor *msg_req_iter = nullptr;
    const mb::rpc::message::Descriptor *msg_rsp_iter = nullptr;

    /*-------------------------------------------------------------------------
    Critical section to guard the RX scratch buffer access
    -------------------------------------------------------------------------*/
    {
      mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

      /*-----------------------------------------------------------------------
      Try to pull a COBS frame out of the RX buffer and into the scratch buffer
      -----------------------------------------------------------------------*/
      if( !this->read_cobs_frame() )
      {
        return false;
      }

      /*-----------------------------------------------------------------------
      Decode the message
      -----------------------------------------------------------------------*/
      if( !message::decode_from_wire( mCfg.decodeBuffer.data(), mCfg.decodeBuffer.size() ) )
      {
        mbed_assert_continue_msg( false, "Failed to decode message" );
        return false;
      }

      /*-----------------------------------------------------------------------
      Find the service associated with the message
      -----------------------------------------------------------------------*/
      req_header    = reinterpret_cast<mbed_rpc_Header *>( mCfg.decodeBuffer.data() );
      auto svc_iter = mCfg.registry->find( req_header->svcId );

      if( ( svc_iter == mCfg.registry->end() ) || !svc_iter->second )
      {
        mbed_assert_continue_msg( false, "Service not found: %d", req_header->svcId );
        return false;
      }

      service = svc_iter->second;
      if( service->reqId != req_header->msgId )
      {
        mbed_assert_continue_msg( false, "RPC %s does not handle message %d", service->name, req_header->msgId );
        return false;
      }

      /*-----------------------------------------------------------------------
      Copy the data from the scratch buffer into the service's memory
      -----------------------------------------------------------------------*/
      msg_req_iter = message::getDescriptor( req_header->msgId );
      if( !msg_req_iter )
      {
        mbed_assert_continue_msg( false, "Request descriptor not found: %d", req_header->msgId );
        return false;
      }

      msg_rsp_iter = message::getDescriptor( service->rspId );
      if( !msg_rsp_iter )
      {
        mbed_assert_continue_msg( false, "Response descriptor not found: %d", service->rspId );
        return false;
      }

      void  *request_data = nullptr;
      size_t request_size = 0;
      service->getRequestData( request_data, request_size );

      memcpy( request_data, mCfg.decodeBuffer.data(), request_size );
    }    // End of critical section

    /*-------------------------------------------------------------------------
    Invoke the service and send the response (if any).
    -------------------------------------------------------------------------*/
    LOG_DEBUG( "RPC: %s", service->name );
    auto   status        = service->processRequest();
    void  *response_data = nullptr;
    size_t response_size = 0;

    switch( status )
    {
      /*-----------------------------------------------------------------------
      Nominal path. Service completed and wants to send a response.
      -----------------------------------------------------------------------*/
      case mbed_rpc_ErrorCode_ERR_NO_ERROR: {
        service->getResponseData( response_data, response_size );

        /*---------------------------------------------------------------------
        Update the header for the user. This is a static part of the protocol
        and should not be modified by the service.
        ---------------------------------------------------------------------*/
        auto rsp_header = reinterpret_cast<mbed_rpc_Header *>( response_data );

        rsp_header->seqId   = req_header->seqId;
        rsp_header->msgId   = msg_rsp_iter->id;
        rsp_header->svcId   = req_header->svcId;
        rsp_header->version = msg_rsp_iter->version;

        publishMessage( service->rspId, response_data );
      }
      break;

      /*-----------------------------------------------------------------------
      Do nothing, the service will send the response asynchronously, if at all.
      -----------------------------------------------------------------------*/
      case mbed_rpc_ErrorCode_ERR_SVC_ASYNC:
      case mbed_rpc_ErrorCode_ERR_SVC_NO_RSP:
        break;

      /*-----------------------------------------------------------------------
      Fall through for all other error cases generated by the service.
      -----------------------------------------------------------------------*/
      default:
        throwError( req_header->seqId, status, nullptr );
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
    Empty the RX buffer of any null bytes that may have been left over from
    a previous frame. New frames start on the first non-null byte.
    -------------------------------------------------------------------------*/
    while( !mCfg.streamBuffer->empty() && ( mCfg.streamBuffer->front() == 0 ) )
    {
      mCfg.streamBuffer->pop();
    }

    /*-------------------------------------------------------------------------
    Scan the RX buffer for a full frame
    -------------------------------------------------------------------------*/
    size_t scratch_idx = 0;

    for( auto iter = mCfg.streamBuffer->begin();
         ( iter != mCfg.streamBuffer->end() ) && ( scratch_idx < mCfg.decodeBuffer.max_size() ); iter++ )
    {
      mCfg.decodeBuffer[ scratch_idx ] = *iter;

      if( *iter == 0 )
      {
        break;
      }

      scratch_idx++;
    }

    /*-------------------------------------------------------------------------
    Remove the frame data if a full COBS frame was found.
    -------------------------------------------------------------------------*/
    if( ( scratch_idx != 0 ) && ( mCfg.decodeBuffer[ scratch_idx ] == 0 ) )
    {
      for( size_t i = 0; i < scratch_idx; i++ )
      {
        mCfg.streamBuffer->pop();
      }

      /*-----------------------------------------------------------------------
      If the frame was larger than all known frame sizes, it's invalid.
      -----------------------------------------------------------------------*/
      if( scratch_idx > message::largest_known_wire_message() )
      {
        return false;
      }

      return true;
    }

    /*-------------------------------------------------------------------------
    If the buffer is full and no frame was found, then we have to flush it.
    This effectively means it's not possible to find a full frame anymore.
    -------------------------------------------------------------------------*/
    else if( scratch_idx == mCfg.decodeBuffer.max_size() )
    {
      mbed_assert_continue_msg( false, "RX buffer full with no COBS frame. Discarding %d bytes.", scratch_idx );
      for( size_t i = 0; i < scratch_idx; i++ )
      {
        mCfg.streamBuffer->pop();
      }
    }

    return false;
  }


  /**
   * @brief Publishes a pending COBS frame to the wire.
   */
  bool Server::write_cobs_frame()
  {
    /*-------------------------------------------------------------------------
    Ensure null termination of the buffer, then get the length of the frame.
    -------------------------------------------------------------------------*/
    *mCfg.encodeBuffer.end() = 0;

    size_t data_frame_size = strlen( reinterpret_cast<char *>( mCfg.encodeBuffer.data() ) );
    size_t cobs_frame_size = data_frame_size + 1u;
    size_t iostream_size   = 0;

    /*-------------------------------------------------------------------------
    Acquire the lock on the iostream. We may not be the only one transmitting.
    -------------------------------------------------------------------------*/
    if( !mCfg.iostream->try_lock_for( mb::thread::TIMEOUT_10MS ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Write the frame to the wire directly if we can.
    -------------------------------------------------------------------------*/
    bool write_success = false;

    if( data_frame_size && ( mCfg.iostream->writeable() >= cobs_frame_size ) )
    {
      iostream_size          = mCfg.iostream->write( mCfg.encodeBuffer.data(), cobs_frame_size );
      mCfg.encodeBuffer[ 0 ] = 0;

      write_success = iostream_size == cobs_frame_size;
    }

    mCfg.iostream->unlock();
    return write_success;
  }


  /**
   * @brief Callback when the io stream has completed a read operation.
   * @warning This is an ISR context function.
   *
   * This is used to pump received bytes into the RX buffer to be processed.
   */
  void Server::isr_on_io_read_complete()
  {
    if( !mIsOpen || mCfg.streamBuffer->full() )
    {
      return;
    }

    size_t read_size = etl::min<size_t>( mCfg.iostream->readable(), mCfg.streamBuffer->available() );
    if( read_size )
    {
      size_t actual_size = mCfg.iostream->read( *mCfg.streamBuffer, read_size, 0 );
      mbed_dbg_assert_continue_msg( read_size == actual_size, "Failed to read all bytes from the wire" );
    }
  }

}    // namespace mb::rpc::server
