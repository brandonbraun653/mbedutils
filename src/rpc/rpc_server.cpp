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

namespace mb::rpc::service
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  ErrId PingService::processRequest()
  {
    memcpy( &response, &request, sizeof( response ) );
    return mbed_rpc_ErrorCode_ERR_NO_ERROR;
  }


  ErrId TestErrorService::processRequest()
  {
    return mbed_rpc_ErrorCode_ERR_SVC_FAILED;
  }


  mb::rpc::ErrId NotifyTimeElapsedService::processRequest()
  {
    start_time = mb::time::millis();
    alarm_time = request.delay_time + start_time;
    async      = true;

    return mbed_rpc_ErrorCode_ERR_SVC_ASYNC;
  }


  void NotifyTimeElapsedService::runAsyncProcess()
  {
    size_t current_time = mb::time::millis();
    if( current_time >= alarm_time )
    {
      response.header       = request.header;
      response.elapsed_time = current_time - start_time;

      if( this->server->publishMessage( rspId, &response ) )
      {
        alarm_time = std::numeric_limits<size_t>::max();
        start_time = 0;
        async      = false;
      }
    }
  }
}    // namespace mb::rpc::service

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
    if( mIsOpen
     || !config.iostream
     || !config.registry
     || !config.streamBuffer
     || !config.streamBuffer->available()
     || !config.encodeBuffer.size()
     || ( config.encodeBuffer.size() % sizeof( uint32_t ) != 0 )
     || !config.decodeBuffer.size()
     || ( config.decodeBuffer.size() % sizeof( uint32_t ) != 0 ) )
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
    Bind builtin services and messages to the server
    -------------------------------------------------------------------------*/
    /* Services */
    mbed_assert_continue( addService( &mPingService ) );
    mbed_assert_continue( addService( &mTestErrorService ) );
    mbed_assert_continue( addService( &mNotifyTimeElapsedService ) );

    /* Messages */
    mbed_assert_continue( message::addDescriptor( message::NullMessage ) );
    mbed_assert_continue( message::addDescriptor( message::PingMessage ) );
    mbed_assert_continue( message::addDescriptor( message::ErrorMessage ) );
    mbed_assert_continue( message::addDescriptor( message::ConsoleMessage ) );
    mbed_assert_continue( message::addDescriptor( message::NotifyTimeElapsedRequest ) );
    mbed_assert_continue( message::addDescriptor( message::NotifyTimeElapsedResponse ) );

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

    /*-------------------------------------------------------------------------
    Process any asynchronous services that need to run
    -------------------------------------------------------------------------*/
    for( auto &service : *mCfg.registry )
    {
      if( service.second->async )
      {
        service.second->runAsyncProcess();
      }
    }

    this->unlock();
  }


  bool Server::addService( ::mb::rpc::service::IService *const service )
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

    service->server = this;
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
    mb::rpc::service::IService         *service      = nullptr;
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
      size_t frame_size = read_cobs_frame();
      if( frame_size == 0 )
      {
        return false;
      }

      /*-----------------------------------------------------------------------
      Decode the message
      -----------------------------------------------------------------------*/
      if( !message::decode_from_wire( mCfg.decodeBuffer.data(), frame_size ) )
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
        mbed_assert_continue_msg( false, "RPC %s invalid request message %d ", service->name, req_header->msgId );
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

      /*-----------------------------------------------------------------------
      Copy the full request size. Must copy the full structure type out even
      though less data may have been received due to NPB unpacking semantics.
      -----------------------------------------------------------------------*/
      memcpy( request_data, mCfg.decodeBuffer.data(), request_size );
    }    // End of critical section

    /*-------------------------------------------------------------------------
    Invoke the service and send the response (if any).
    -------------------------------------------------------------------------*/
    LOG_TRACE( "RPC: %s", service->name );
    auto   status        = service->processRequest();
    void  *response_data = nullptr;
    size_t response_size = 0;

    switch( status )
    {
      /*-----------------------------------------------------------------------
      Nominal path. Service completed and wants to send a response.
      -----------------------------------------------------------------------*/
      case mbed_rpc_ErrorCode_ERR_NO_ERROR:
      case mbed_rpc_ErrorCode_ERR_SVC_ASYNC_WITH_RSP: {
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
   * @return Frame size including null terminator if found, else 0
   */
  size_t Server::read_cobs_frame()
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
    size_t frame_size = 0;

    for( auto iter = mCfg.streamBuffer->begin();
         ( iter != mCfg.streamBuffer->end() ) && ( frame_size < mCfg.decodeBuffer.max_size() ); iter++ )
    {
      mCfg.decodeBuffer[ frame_size++ ] = *iter;
      if( *iter == 0 )
      {
        break;
      }
    }

    /*-------------------------------------------------------------------------
    Remove data from the stream buffer
    -------------------------------------------------------------------------*/
    if( ( frame_size > 1 ) && ( mCfg.decodeBuffer[ frame_size - 1 ] == 0 ) )
    {
      /*-----------------------------------------------------------------------
      Clear the remainder of the buffer to zero. This helps construct clean NPB
      structures when the decode process is invoked.
      -----------------------------------------------------------------------*/
      etl::fill( mCfg.decodeBuffer.begin() + frame_size, mCfg.decodeBuffer.end(), 0 );

      /*-----------------------------------------------------------------------
      Remove the full frame data
      -----------------------------------------------------------------------*/
      for( size_t i = 0; i < frame_size; i++ )
      {
        mCfg.streamBuffer->pop();
      }

      /*-----------------------------------------------------------------------
      The found frame must be less than the largest known message size, else
      we've received a new message type or the data is bad.
      -----------------------------------------------------------------------*/
      if( frame_size <= message::largest_known_wire_message() )
      {
        return frame_size;
      }
    }
    else if( frame_size == mCfg.decodeBuffer.max_size() )
    {
      /*-----------------------------------------------------------------------
      If the buffer is full and no frame was found, then we have to flush it.
      This effectively means it's not possible to find a full frame anymore.
      -----------------------------------------------------------------------*/
      mbed_assert_continue_msg( false, "RX buffer full with no COBS frame. Discarding %d bytes.", frame_size );
      mCfg.streamBuffer->clear();
    }

    return 0;
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
