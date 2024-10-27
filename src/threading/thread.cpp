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

  static mb::osal::mb_mutex_t      s_module_mutex;
  static Internal::ControlBlockMap s_tsk_control_blocks;
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
    new_task.taskId = TASK_ID_INVALID;
    new_task.taskImpl  = nullptr;

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

    if( auto handle = mb::thread::intf::create_task( cfg ); handle )
    {
      Internal::ControlBlock tcb;

      tcb.name     = cfg.name;
      tcb.handle   = new_task.taskImpl;
      tcb.priority = cfg.priority;
      tcb.msgQueue = cfg.msg_queue_inst;

      s_tsk_control_blocks->insert( { cfg.id, tcb } );

      new_task.taskId = cfg.id;
      new_task.taskImpl  = handle;
    }

    mbed_assert_continue_msg( new_task.taskImpl, "Failed to create task id: %d", cfg.id );
    return etl::move( new_task );
  }


  void startScheduler()
  {
    mb::thread::intf::start_scheduler();
  }


  bool sendMessage( const TaskId id, Message &msg, const size_t timeout )
  {
    return false;
  }


  namespace this_thread
  {
    void set_name( const char *name )
    {
    }


    TaskName &get_name()
    {
      static TaskName name;
      return name;
    }


    void sleep_for( const size_t timeout )
    {
    }


    void sleep_until( const size_t wakeup )
    {
    }


    void yield()
    {
    }


    void suspend()
    {
    }


    TaskId id()
    {
      return 0;
    }


    bool awaitMessage( Message &msg, const size_t timeout )
    {
      return false;
    }


    bool awaitMessage( Message &msg, MessagePredicate &predicate, const size_t timeout )
    {
      return false;
    }
  }    // namespace this_thread

}    // namespace mb::thread
