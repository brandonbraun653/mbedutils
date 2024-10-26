/******************************************************************************
 *  File Name:
 *    message.hpp
 *
 *  Description:
 *    Thread message passing interface for mbedutils
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_MESSAGE_HPP
#define MBEDUTILS_THREADING_MESSAGE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/delegate.h>
#include <etl/flat_map.h>
#include <etl/pool.h>
#include <etl/queue.h>
#include <etl/string.h>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/

  struct TaskMessage;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using MessagePredicate = etl::delegate<bool( void * )>;
  using TaskMsgId        = size_t;
  using TaskMsgPool      = etl::ipool *;
  using TaskMsgQueue     = etl::iqueue<TaskMessage> *;


  /**
   * @brief Declares the expected storage for task messages.
   *
   * This allows each thread to maintain it's own local copy of the message
   * in a known accessible memory space.
   *
   * @tparam PayloadType Core data structure being sent as a message.
   * @tparam N           How many elements can be queued
   */
  template<typename PayloadType, size_t N>
  using TaskMsgPoolStorage = etl::pool<PayloadType, N>;

  /**
   * @brief Declares a queue to reference TaskMsgPoolStorage items.
   *
   * This is used to provide type erasure to the queue interface while
   * still being able to order messages. The etl variant of the queue
   * doesn't have an option for a type independent reference like the
   * etl::pool class has.
   *
   * @tparam N How many elements can be queued. Should match TaskMsgPoolStorage<>
   */
  template<size_t N>
  using TaskMsgQueueStorage = etl::queue<void *, N>;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Represents a message that can be sent between threads.
   *
   * In all cases, the user must allocate memory for the data field. When
   * sending, the data field will be copied into the receiving thread's message
   * queue. When receiving, the data field acts as the destination for copying
   * from the executing thread's message queue.
   */
  struct TaskMessage
  {
    TaskMsgId id;       /**< Message identifier */
    uint32_t  priority; /**< Message priority. Lower is more urgent. */
    uint32_t  flags;    /**< Message flags to characterize behavior */
    void     *data;     /**< User data. Must contain POD types only. */
    size_t    size;     /**< Length of user data storage. */
  };

  /**
   * @brief
   *
   * @tparam T
   * @tparam N
   */
  template<typename T, size_t N>
  struct MessageQueueStorage
  {
    // Use a pool of memory to store the messages. POD data only.
    // This would allow for a fixed size message queue that can be
    // used to pass data between threads. Memcpy can be used to pass
    // the data around, which should still work once this is moved
    // to multiple cores/partition schemes.
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class TaskMessageQueue
  {
  public:
    struct Config
    {
      // Message pool
      // Priority queue
    };

    TaskMessageQueue();
    ~TaskMessageQueue();

    void configure( const Config &cfg );

    bool push( TaskMessage &msg );

    bool pop( TaskMessage &msg );

    bool peek( TaskMessage &msg );

    bool empty();

    size_t size();

  private:
    etl::ipriority_queue<TaskMessage> mPriorityQueue;
  };

}    // namespace mb::thread

#endif /* !MBEDUTILS_THREADING_MESSAGE_HPP */
