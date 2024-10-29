/******************************************************************************
 *  File Name:
 *    thread.cpp
 *
 *  Description:
 *    System level threading management functions
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "mbedutils/drivers/threading/thread.hpp"
#include "mbedutils/drivers/threading/lock.hpp"
#include "mbedutils/drivers/threading/message.hpp"
#include "mbedutils/interfaces/util_intf.hpp"
#include <mbedutils/threading.hpp>
#include <mbedutils/interfaces/thread_intf.hpp>
#include <mbedutils/util.hpp>
#include <etl/flat_map.h>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static Internal::ControlBlockMap s_tsk_control_blocks;
  static mb::osal::mb_mutex_t      s_module_mutex;
  static size_t                    s_module_ready;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void driver_setup( const Internal::ModuleConfig &cfg )
  {
    /*-------------------------------------------------------------------------
    Initialize static memory
    -------------------------------------------------------------------------*/
    /* clang-format off */
    if( mbed_assert( mb::osal::buildMutexStrategy( s_module_mutex ) ) &&
        mbed_assert( cfg.tsk_control_blocks ) &&
        ( s_module_ready != DRIVER_INITIALIZED_KEY ) )
    {
      /* clang-format on */
      s_tsk_control_blocks = cfg.tsk_control_blocks;
      s_tsk_control_blocks->clear();

      /*-----------------------------------------------------------------------
      Initialize the underlying threading implementation
      -----------------------------------------------------------------------*/
      mb::thread::intf::driver_setup();
      s_module_ready = DRIVER_INITIALIZED_KEY;
    }
  }


  void driver_teardown()
  {
    if( s_module_ready != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::thread::intf::driver_teardown();
    mb::osal::destroyMutex( s_module_mutex );
    s_module_ready = ~DRIVER_INITIALIZED_KEY;
  }


  Task &&create( const Task::Config &cfg )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( s_module_ready != DRIVER_INITIALIZED_KEY )
    {
      Task empty_task;
      return etl::move( empty_task );
    }

    /*-------------------------------------------------------------------------
    Construct a basic invalid task to return if the creation fails
    -------------------------------------------------------------------------*/
    Task new_task;
    new_task.mId     = TASK_ID_INVALID;
    new_task.mHandle = -1;

    /*-------------------------------------------------------------------------
    Configure the queue if one was provided
    -------------------------------------------------------------------------*/
    if( cfg.msg_queue_inst )
    {
      /*-----------------------------------------------------------------------
      Ensure the message queue configuration is valid
      -----------------------------------------------------------------------*/
      if( cfg.msg_queue_cfg.pool == nullptr || cfg.msg_queue_cfg.queue == nullptr )
      {
        mbed_assert_continue_msg( false, "Invalid message queue configuration" );
        return etl::move( new_task );
      }

      /*-----------------------------------------------------------------------
      Configure the message queue
      -----------------------------------------------------------------------*/
      MessageQueue::Config queue_cfg;
      queue_cfg.pool  = cfg.msg_queue_cfg.pool;
      queue_cfg.queue = cfg.msg_queue_cfg.queue;

      cfg.msg_queue_inst->configure( queue_cfg );
    }

    /*-------------------------------------------------------------------------
    Attempt to create the task:
      - Ensure the task ID is not already in use
      - Ensure the task control block is not full
    -------------------------------------------------------------------------*/
    LockGuard lock( s_module_mutex );

    if( s_tsk_control_blocks->full() )
    {
      mbed_assert_continue_msg( false, "Maximum number of tasks already created" );
      return etl::move( new_task );
    }

    if( s_tsk_control_blocks->find( cfg.id ) != s_tsk_control_blocks->end() )
    {
      mbed_assert_continue_msg( false, "Task already created for ID: %d", cfg.id );
      return etl::move( new_task );
    }

    if( auto handle = mb::thread::intf::create_task( cfg ); handle >= 0 )
    {
      /*-----------------------------------------------------------------------
      Reconfigure the returned task information
      -----------------------------------------------------------------------*/
      new_task.mId     = cfg.id;
      new_task.mHandle = handle;
      new_task.mName   = cfg.name;

      /*-----------------------------------------------------------------------
      Create the control block for the task
      -----------------------------------------------------------------------*/
      Internal::ControlBlock tcb;

      tcb.name     = cfg.name;
      tcb.handle   = handle;
      tcb.priority = cfg.priority;
      tcb.msgQueue = cfg.msg_queue_inst;

      s_tsk_control_blocks->insert( { cfg.id, tcb } );
    }
    else
    {
      mbed_assert_continue_msg( false, "Failed to create task id: %d", cfg.id );
    }

    return etl::move( new_task );
  }


  void startScheduler()
  {
    mb::thread::intf::start_scheduler();
  }


  bool sendMessage( const TaskId id, Message &msg, const size_t timeout )
  {
    /*-------------------------------------------------------------------------
    Ensure the destination task ID is valid
    -------------------------------------------------------------------------*/
    mb::thread::LockGuard module_lock( s_module_mutex );

    auto it = s_tsk_control_blocks->find( id );
    if( it == s_tsk_control_blocks->end() )
    {
      mbed_assert_continue_msg( false, "Task ID not found: %d", id );
      return false;
    }

    /*-------------------------------------------------------------------------
    Check the receiver's message queue to ensure it can accept the message
    -------------------------------------------------------------------------*/
    {
      mb::thread::RecursiveLockGuard receiver_queue_lock( it->second.msgRMutex );

      if( !it->second.msgQueue )
      {
        mbed_assert_continue_msg( false, "Dst task [%d] doesn't accept messages", id );
        return false;
      }

      msg.sender = this_thread::id();

      // Switch to a while loop? Maybe wait on the CV with timeout?

      if( !it->second.msgQueue->push( msg ) )
      {
        mbed_assert_continue_msg( false, "Dst task [%d] message queue full", id );
        return false;
      }
    }


    return false;
  }

  namespace this_thread
  {
    bool awaitMessage( Message &msg, const size_t timeout )
    {
      // Check for any message in the queue. Only block if the queue is empty.
      return false;
    }


    bool awaitMessage( Message &msg, MessagePredicate &predicate, const size_t timeout )
    {
      // First check the top of the
      return false;
    }
  }
}    // namespace mb::thread
