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
#include <etl/flat_map.h>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static ModuleConfig s_cfg;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize( const ModuleConfig &cfg )
  {
    // Need to allocate all the mutexes, condition variables, etc.
  }


  Task &&create( const TaskConfig &cfg )
  {
    return etl::move( Task() );
  }


  void startScheduler()
  {
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

}  // namespace mb::thread
