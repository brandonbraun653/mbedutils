/******************************************************************************
 *  File Name:
 *    thread.cpp
 *
 *  Description:
 *    System level threading management functions. This was designed with an
 *    embedded RTOS in mind, which usually doesn't dynamically create/destroy
 *    multiple threads within a single power cycle. The design is simple and
 *    expects threads to be set up once, then run indefinitely. If this does
 *    not match your use case, you may need to modify the implementation to
 *    protect access to the control block map.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "mbedutils/drivers/threading/thread.hpp"
#include "mbedutils/drivers/threading/lock.hpp"
#include "mbedutils/drivers/threading/message.hpp"
#include "mbedutils/interfaces/mutex_intf.hpp"
#include "mbedutils/interfaces/time_intf.hpp"
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


  Task &&create( Task::Config &cfg )
  {
    static Task empty_task;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( s_module_ready != DRIVER_INITIALIZED_KEY )
    {
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
      tcb.reset();

      tcb.name     = cfg.name;
      tcb.id       = handle;
      tcb.priority = cfg.priority;
      tcb.msgQueue = cfg.msg_queue_inst;

      mb::osal::buildMutexStrategy( tcb.msgMutex );
      tcb.msgCV.init();

      s_tsk_control_blocks->insert( { cfg.id, tcb } );
    }
    else
    {
      mbed_assert_continue_msg( false, "Failed to create task id: %d", cfg.id );
    }

    return etl::move( new_task );
  }


  void destroy( Task *task )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !task || ( s_module_ready != DRIVER_INITIALIZED_KEY ) )
    {
      return;
    }

    LockGuard lock( s_module_mutex );

    /*-------------------------------------------------------------------------
    Ensure the task ID is valid
    -------------------------------------------------------------------------*/
    auto it = s_tsk_control_blocks->find( task->id() );
    if( it == s_tsk_control_blocks->end() )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Destroy the task control block
    -------------------------------------------------------------------------*/
    mb::osal::destroyMutexStrategy( it->second.msgMutex );
    it->second.msgCV.deinit();
    s_tsk_control_blocks->erase( it );

    /*-------------------------------------------------------------------------
    Destroy the task in the underlying implementation
    -------------------------------------------------------------------------*/
    mb::thread::intf::destroy_task( task->mHandle );

    /*-------------------------------------------------------------------------
    Reset the task object
    -------------------------------------------------------------------------*/
    task->mId     = TASK_ID_INVALID;
    task->mHandle = TASK_ID_INVALID;
    task->pImpl   = nullptr;
    task->mName.clear();
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
    auto it = s_tsk_control_blocks->find( id );
    if( it == s_tsk_control_blocks->end() )
    {
      mbed_assert_continue_msg( false, "Task ID not found: %d", id );
      return false;
    }

    /*-------------------------------------------------------------------------
    Ensure the destination task has a message queue
    -------------------------------------------------------------------------*/
    if( !it->second.msgQueue )
    {
      mbed_assert_continue_msg( false, "Dst task [%d] doesn't accept messages", id );
      return false;
    }

    /*-------------------------------------------------------------------------
    Send the message to the destination task
    -------------------------------------------------------------------------*/
    {
      size_t start_time = mb::time::millis();
      bool   expired    = true;
      msg.sender        = this_thread::id();

      while( ( mb::time::millis() - start_time ) < timeout )
      {
        mb::thread::LockGuard receiver_queue_lock( it->second.msgMutex );
        if( it->second.msgQueue->push( msg ) )
        {
          expired = false;
          break;
        }

        mb::thread::this_thread::yield();
      }

      if( expired )
      {
        mbed_assert_continue_msg( false, "Dst task [%d] message queue full", id );
        return false;
      }

      /*-----------------------------------------------------------------------
      Notify the destination task that a message is available.
      -----------------------------------------------------------------------*/
      it->second.msgCV.notify_one();
    }

    return true;
  }

  namespace this_thread
  {
    bool awaitMessage( Message &msg, const size_t timeout )
    {
      const TaskId id = this_thread::id();

      /*-----------------------------------------------------------------------
      Find the task control block for the current task
      -----------------------------------------------------------------------*/
      auto it = s_tsk_control_blocks->find( id );
      if( it == s_tsk_control_blocks->end() )
      {
        mbed_assert_continue_msg( false, "Task ID not found: %d", id );
        return false;
      }

      /*-----------------------------------------------------------------------
      Ensure the destination task has a message queue
      -----------------------------------------------------------------------*/
      if( !it->second.msgQueue )
      {
        mbed_assert_continue_msg( false, "Dst task [%d] doesn't accept messages", id );
        return false;
      }

      /*-----------------------------------------------------------------------
      Pull out a single message if one is available, waiting if necessary.
      -----------------------------------------------------------------------*/
      {
        mb::thread::LockGuard receiver_queue_lock( it->second.msgMutex );

        if( it->second.msgQueue->empty() )
        {
          it->second.msgCV.wait_for( it->second.msgMutex, timeout );
        }

        if( !it->second.msgQueue->pop( msg ) )
        {
          return false;
        }
      }

      return true;
    }


    bool awaitMessage( Message &msg, MessagePredicate &predicate, const size_t timeout )
    {
      const TaskId id = this_thread::id();

      /*-----------------------------------------------------------------------
      Find the task control block for the current task
      -----------------------------------------------------------------------*/
      auto it = s_tsk_control_blocks->find( id );
      if( it == s_tsk_control_blocks->end() )
      {
        mbed_assert_continue_msg( false, "Task ID not found: %d", id );
        return false;
      }

      /*-----------------------------------------------------------------------
      Ensure the destination task has a message queue
      -----------------------------------------------------------------------*/
      if( !it->second.msgQueue )
      {
        mbed_assert_continue_msg( false, "Dst task [%d] doesn't accept messages", id );
        return false;
      }

      /*-----------------------------------------------------------------------
      Keep checking for messages until the predicate is satisfied or a timeout
      is reached.
      -----------------------------------------------------------------------*/
      {
        mb::thread::RecursiveLockGuard receiver_queue_lock( it->second.msgMutex );

        size_t start_time = mb::time::millis();
        bool   expired    = true;

        while( ( mb::time::millis() - start_time ) < timeout )
        {
          /*-------------------------------------------------------------------
          Wait for a message to arrive
          -------------------------------------------------------------------*/
          if( it->second.msgQueue->empty() )
          {
            it->second.msgCV.wait_for( it->second.msgMutex, timeout );
          }

          /*-------------------------------------------------------------------
          Check if the message is the one we're looking for
          -------------------------------------------------------------------*/
          if( it->second.msgQueue->peek( msg ) && predicate( msg ) )
          {
            it->second.msgQueue->pop( msg );
            expired = false;
            break;
          }
        }

        if( expired )
        {
          return false;
        }
      }

      return true;
    }
  }
}    // namespace mb::thread
