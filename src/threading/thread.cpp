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
#include <mbedutils/threading.hpp>
#include <mbedutils/interfaces/thread_intf.hpp>
#include <mbedutils/util.hpp>
#include <etl/flat_map.h>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static mb::osal::mb_mutex_t module_mutex;
  static Internal::ControlBlockMap  tsk_control_blocks;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize( const Internal::ModuleConfig &cfg )
  {
    /*-------------------------------------------------------------------------
    Initialize static memory
    -------------------------------------------------------------------------*/
    mbed_assert( mb::osal::buildMutexStrategy( module_mutex ) );
    mbed_assert( cfg.tsk_control_blocks );
    tsk_control_blocks = cfg.tsk_control_blocks;
    tsk_control_blocks->clear();

    /*-------------------------------------------------------------------------
    Initialize the underlying threading implementation
    -------------------------------------------------------------------------*/
    mb::thread::intf::initialize();
  }


  Task &&create( const Task::Config &cfg )
  {
    mbed_dbg_assert( module_mutex );
    LockGuard lock( module_mutex );

    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    mbed_assert( !tsk_control_blocks->full() );
    mbed_assert( tsk_control_blocks->find( cfg.id ) == tsk_control_blocks->end() );

    /*---------------------------------------------------------------------------
    Create the task using the underlying implementation
    ---------------------------------------------------------------------------*/
    TaskHandle handle = mb::thread::intf::create_task( cfg );
    mbed_assert( handle != nullptr );

    /*-------------------------------------------------------------------------
    Create the task control block
    -------------------------------------------------------------------------*/
    Internal::ControlBlock tcb;

    tcb.name     = cfg.name;
    tcb.handle   = handle;
    tcb.priority = cfg.priority;
    tcb.msgQueue = cfg.msg_queue;

    tsk_control_blocks->insert( { cfg.id, tcb } );

    /*-------------------------------------------------------------------------
    Construct the new task object
    -------------------------------------------------------------------------*/
    Task new_task;
    new_task.taskId = cfg.id;
    new_task.pimpl  = handle;

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
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  Task::Task() noexcept : taskId( -1 ), pimpl( nullptr )
  {
  }


  Task::~Task()
  {
  }


  Task &Task::operator=( Task &&other ) noexcept
  {
    this->taskId = other.taskId;
    this->pimpl  = other.pimpl;
    return *this;
  }


  void start()
  {
    // Send an event to the task to start it.
  }


  void Task::kill()
  {
    // Send an event to the task to kill it.
  }


  void Task::join()
  {
    // Wait for the task to end (intf?)

    // Lock global mutex, then release the task control block.
  }


  bool Task::joinable()
  {
    return false;
  }

}  // namespace mb::thread
