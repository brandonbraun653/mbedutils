/******************************************************************************
 *  File Name:
 *    message.cpp
 *
 *  Description:
 *    Task message implementation for mbedutils
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "mbedutils/drivers/threading/lock.hpp"
#include <mbedutils/threading.hpp>
#include <mbedutils/util.hpp>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Message Implementation
  ---------------------------------------------------------------------------*/

  Message::Message() : sender( TASK_ID_INVALID ), priority( MSG_PRIORITY_LOWEST ), data( nullptr ), size( 0 )
  {
  }


  Message::Message( const Message &msg )
  {
    sender   = msg.sender;
    priority = msg.priority;
    data     = msg.data;
    size     = msg.size;
  }


  Message::~Message()
  {
  }

  bool Message::is_valid() const
  {
    return ( data != nullptr ) && ( size > 0 );
  }


  bool Message::copy_from( const Message &msg )
  {
    if( !msg.is_valid() || !( msg.size == size ) )
    {
      return false;
    }

    sender   = msg.sender;
    priority = msg.priority;
    size     = msg.size;

    memcpy( data, msg.data, size );

    return true;
  }


  /*---------------------------------------------------------------------------
  MessageQueue Implementation
  ---------------------------------------------------------------------------*/

  MessageQueue::MessageQueue() : Lockable<MessageQueue>(), mConfigured( ~DRIVER_INITIALIZED_KEY ), mPool( nullptr ), mQueue( nullptr )
  {
  }


  MessageQueue::~MessageQueue()
  {
  }


  void MessageQueue::configure( const Config &cfg )
  {
    /*-------------------------------------------------------------------------
    Validate entrance conditions
    -------------------------------------------------------------------------*/
    if( ( mConfigured == DRIVER_INITIALIZED_KEY ) || ( cfg.pool == nullptr ) || ( cfg.queue == nullptr ) )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Accept the configuration
    -------------------------------------------------------------------------*/
    mPool       = cfg.pool;
    mQueue      = cfg.queue;
    mConfigured = DRIVER_INITIALIZED_KEY;

    /*-------------------------------------------------------------------------
    Clear out any existing messages
    -------------------------------------------------------------------------*/
    mPool->clear_successor_chain();
    mQueue->clear();
  }


  bool MessageQueue::push( Message &msg )
  {
    /*-------------------------------------------------------------------------
    Validate the input
    -------------------------------------------------------------------------*/
    if( mConfigured != DRIVER_INITIALIZED_KEY || !msg.is_valid() )
    {
      return false;
    }

    RecursiveLockGuard _lock( this->mLockableMutex );
    if( mQueue->full() )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Allocate a memory block for the message and copy the data in
    -------------------------------------------------------------------------*/
    Message tmp;
    tmp.size = msg.size;
    tmp.data = mPool->allocate( msg.size, 1 );

    if( !tmp.copy_from( msg ) )
    {
      mPool->release( tmp.data );
      return false;
    }

    /*-------------------------------------------------------------------------
    Push the message onto the queue and sort by priority
    -------------------------------------------------------------------------*/
    mQueue->push_back( tmp );
    etl::sort( mQueue->begin(), mQueue->end(), Internal::TskMsgCompare() );

    return true;
  }


  bool MessageQueue::pop( Message &msg )
  {
    /*-------------------------------------------------------------------------
    Validate the input
    -------------------------------------------------------------------------*/
    if( mConfigured != DRIVER_INITIALIZED_KEY )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Peek is very nearly the same as pop, so just call it and then remove the
    message from the queue if it was successful.
    -------------------------------------------------------------------------*/
    RecursiveLockGuard _lock( this->mLockableMutex );
    if( peek( msg ) )
    {
      mPool->release( mQueue->front().data );
      mQueue->pop_front();
      return true;
    }
    else
    {
      return false;
    }
  }


  bool MessageQueue::peek( Message &msg )
  {
    /*-------------------------------------------------------------------------
    Validate the input
    -------------------------------------------------------------------------*/
    if( ( mConfigured != DRIVER_INITIALIZED_KEY ) || !msg.is_valid() )
    {
      return false;
    }

    RecursiveLockGuard _lock( this->mLockableMutex );
    if( mQueue->empty() )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Make a copy of the top message in the queue
    -------------------------------------------------------------------------*/
    auto top = mQueue->front();
    if( !msg.copy_from( top ) )
    {
      return false;
    }

    return true;
  }


  bool MessageQueue::empty()
  {
    if( mConfigured != DRIVER_INITIALIZED_KEY )
    {
      return true;
    }

    RecursiveLockGuard _lock( this->mLockableMutex );
    return mQueue->empty();
  }


  size_t MessageQueue::size()
  {
    if( mConfigured != DRIVER_INITIALIZED_KEY )
    {
      return 0;
    }

    RecursiveLockGuard _lock( this->mLockableMutex );
    return mQueue->size();
  }

}  // namespace mb::thread
