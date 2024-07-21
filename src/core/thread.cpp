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
#include <mbedutils/drivers/threading/thread.hpp>

namespace mb::thread
{
  /*-------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------*/

  /**
   * @brief Internal representation of a task control block
   */
  struct TaskCB
  {
    // Need a mutex for protecting message queue access. Actually, maybe
    // what I want is a condition variable? I need to get notified when
    // a queue element becomes free.
  };



  void sendMessage( const TaskId id, const TaskMsg &msg, const size_t timeout )
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

    // // Function to release a message
    // void releaseMessage(BaseMessage* message) {
    //     message->~BaseMessage();
    //     messagePool.release(reinterpret_cast<uint8_t*>(message));
    // }
  }

  bool this_thread::pendTaskMsg( TaskMsg &msg, const size_t timeout )
  {
    // Look up the thread id, block on the message queue condition variable if empty?
    // Pull one immediately if available.
  }

}  // namespace mb::thread
