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
#include <mbedutils/thread.hpp>
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
  static TaskCBMap            tsk_control_blocks;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize( const ModuleConfig &cfg )
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


  Task &&create( const TaskConfig &cfg )
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
    TaskControlBlock tcb;

    tcb.name            = cfg.name;
    tcb.handle          = handle;
    tcb.priority        = cfg.priority;
    tcb.msg_pool        = nullptr;
    tcb.msg_queue       = nullptr;
    tcb.msg_queue_mutex = nullptr;

    if( cfg.msg_pool || cfg.msg_queue )
    {
      mbed_assert( cfg.msg_pool );
      mbed_assert( cfg.msg_queue );

      tcb.msg_pool  = cfg.msg_pool;
      tcb.msg_queue = cfg.msg_queue;
      tcb.msg_queue_cv.init();
      tcb.msg_pool->release_all();
      tcb.msg_queue->clear();
      mbed_assert( mb::osal::buildMutexStrategy( tcb.msg_queue_mutex ) );
    }

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


  void sendMessage( const TaskId id, const void *msg, const size_t size, const size_t timeout )
  {
    // TODO BMB: Can I actually pool allocate and initialize from the given thread id queue?
    // The below code comes from Claude, but I'm not sold yet if this is actually valid.
    // I'd need to look up the thread's task structure, pull the queue, lock it.

    // Oh! I should also "notify_one" the queue's CV to wake up that thread if it's waiting.

    // Function to allocate a message of a specific type
    // template<typename T>
    // T* allocateMessage(MessageTypeId type) {
    //     void* memory = messagePool.allocate();
    //     if (memory) {
    //         T* message = new (memory) T();
    //         message->type = type;
    //         return message;
    //     }
    //     return nullptr;
    // }

    // TODO BMB: I think the allocate function can accept a size.

    // // Function to release a message
    // void releaseMessage(BaseMessage* message) {
    //     message->~BaseMessage();
    //     messagePool.release(reinterpret_cast<uint8_t*>(message));
    // }
  }


  bool this_thread::pendTaskMsg( void *msg, const size_t timeout )
  {
    // Look up the thread id, block on the message queue condition variable if empty?
    // Pull one immediately if available.

    return false;
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
