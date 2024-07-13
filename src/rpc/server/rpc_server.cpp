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
#include <mbedutils/rpc.hpp>

namespace mb::rpc::server
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  Server::Server() : mIsOpen( false ), mConfig( {} )
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
    { /* clang-format on*/
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

    mConfig = config;
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
      mConfig.iostream->unlock();
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
    if( mConfig.svcReg->full() )
    {
      mbed_assert_continue_msg( false, "Unable to add service %d, registry is full", service.getServiceId() );
      return false;
    }

    mConfig.svcReg->insert( { service.getServiceId(), service } ) ;
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
    if( mConfig.msgReg->full() )
    {
      mbed_assert_continue_msg( false, "Unable to add message %d, registry is full", message.id );
      return false;
    }

    mConfig.msgReg->insert( { message.id, message } );
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
    mConfig.svcReg->erase( id );
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
    mConfig.msgReg->erase( id );
  }


  bool Server::sendAsyncResponse( const Response &rsp )
  {
    /*-------------------------------------------------------------------------
    Entrancy Protection
    -------------------------------------------------------------------------*/
    if( !mIsOpen )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( this->mLockableMutex );

    // TODO: Careful, competing with the ISR here for access (pulls data from the tx buffer).
    // May not be an issue. Check the documentation for the buffer type.

    return false;
  }


  bool Server::rpc_invoke( const SvcId svc, const Request &req, Response &rsp )
  {
    // TODO: Pass around the actual service object instead of just the ID
    return false;
  }


  void Server::rpc_throw_error( const size_t txn_id, uint32_t error_code, etl::string_view &msg )
  {
  }


  bool Server::rpc_validate_request( const SvcId svc, const Request &req, uint32_t &error_code )
  {
    // TODO: Pass around the actual service object instead of just the ID
    return false;
  }


  bool Server::rpc_pack_response( const SvcId svc, void *const msg_data, const size_t msg_size, const Request &req, Response &rsp )
  {
    // TODO: Pass around the actual service object instead of just the ID
    return false;
  }


  bool Server::rpc_publish_message( const pb_msgdesc_t *dsc, const void *data )
  {
    return false;
  }


  /**
   * @brief Reads the next full COBS frame from the RX buffer
   *
   * @return true  A full frame was read
   * @return false No full frame was read
   */
  bool Server::read_cobs_frame()
  {
    // TODO: read into the the config rx scratch buffer
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
    // TODO: write from the config tx scratch buffer
    return false;
  }


  /**
   * @brief Callback when the iostream has completed a write operation.
   * @warning This is an ISR context function.
   *
   * This is used to pump any pending bytes from the TX buffer out on the wire.
   */
  void Server::isr_on_io_write_complete()
  {
    if( !mIsOpen )
    {
      return;
    }

    // TODO: Can I do better than byte by byte copy here? Ideally I would like to write the entire buffer at once.
    // Maybe I can do some kind of contiguous buffer copy...or expand the serial interface to accept
    // a circular buffer? That would be nice.

    // size_t write( const etl::icircular_buffer<uint8_t> &buffer, const size_t size );
  }


  /**
   * @brief Callback when the io stream has completed a read operation.
   * @warning This is an ISR context function.
   *
   * This is used to pump received bytes into the RX buffer to be processed.
   */
  void Server::isr_on_io_read_complete()
  {
    if( !mIsOpen )
    {
      return;
    }

    // TODO: Consider expanding the serial interface to accept circular buffers. This would allow for
    // more efficient data handling.
    // size_t read( etl::icircular_buffer<uint8_t> &buffer, const size_t size );
  }


  bool Server::process_next_request()
  {
    if( !this->read_cobs_frame() )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Decode the message
    -------------------------------------------------------------------------*/
    // TODO: Write a common "header peek" function that utilizes the scratch buffer and returns a pointer to the header.
    return false;
  }
}    // namespace mb::rpc::server
