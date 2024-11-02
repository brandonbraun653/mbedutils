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
#include <etl/deque.h>
#include <etl/flat_map.h>
#include <etl/largest.h>
#include <etl/fixed_sized_memory_block_allocator.h>
#include <etl/string.h>
#include <etl/vector.h>
#include <limits>
#include <mbedutils/drivers/threading/condition.hpp>
#include <mbedutils/drivers/threading/lock.hpp>
#include <mbedutils/drivers/threading/types.hpp>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  struct Message;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using MessageId        = size_t;
  using MessagePredicate = etl::delegate<bool( const Message &msg )>;
  using MessagePriority  = uint32_t;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr MessagePriority MSG_PRIORITY_HIGHEST = std::numeric_limits<MessagePriority>::min();
  static constexpr MessagePriority MSG_PRIORITY_LOWEST  = std::numeric_limits<MessagePriority>::max();

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
  struct Message
  {
    TaskId          sender;   /**< Which task sent the message */
    MessagePriority priority; /**< Message priority. Lower is more urgent. */
    void           *data;     /**< User data. Must contain POD types only. */
    size_t          size;     /**< Length of user data storage. */

    Message();
    Message( const Message &msg );
    ~Message();

    bool is_valid() const;

    bool copy_from( const Message &msg );
  };


  namespace Internal
  {
    /**
     * @brief Comparison function for task messages based on priority.
     */
    struct TskMsgCompare : public etl::binary_function<Message, Message, bool>
    {
      typedef Message value_type;

      ETL_CONSTEXPR bool operator()( const Message &lhs, const Message &rhs ) const
      {
        return ( lhs.priority < rhs.priority );
      }
    };

    /**
     * @brief Size independent task message queue type.
     */
    using ITskMsgQueue = etl::ideque<Message>;

    /**
     * @brief Fixed size task message queue type.
     *
     * @tparam N Number of elements that can be stored in the queue
     */
    template<size_t N>
    using TskMsgQueue  = etl::deque<Message, N>;

    /**
     * @brief Size & type independent reference to a task message memory allocator pool.
     */
    using ITskMsgPool = etl::imemory_block_allocator;

    /**
     * @brief Static pool allocator for task messages.
     *
     * @tparam T User task message type
     * @tparam N Number of elements that can be stored in the pool
     */
    template<typename T, size_t N>
    using TskMsgPool = etl::fixed_sized_memory_block_allocator<etl::largest<T>::size, etl::largest<T>::alignment, N>;
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class MessageQueue : public Lockable<MessageQueue>
  {
  public:
    /**
     * @brief Queue configuration structure. Used to bind the queue to a memory pool.
     *
     * It's highly recommended to used the Storage<> structure from this class to
     * statically allocate the memory pool and queue. This will ensure that the
     * memory pool and queue are properly aligned and sized for the task message.
     *
     * By declaring/binding memory this way, we decouple the task message type
     * from the queue logic, reducing embedded RO code costs due to templates.
     */
    struct Config
    {
      Internal::ITskMsgPool  *pool;  /**< Memory pool for message storage */
      Internal::ITskMsgQueue *queue; /**< Message queue */

      Config() : pool( nullptr ), queue( nullptr ) {}
    };

    /**
     * @brief Declares expected storage requirements for a message queue.
     *
     * This provides thread local storage that can be statically allocated and
     * then bound to a message queue instance. The user task message type is
     * expected to be a POD type or a structure containing only POD types. To
     * save memory space, it's recommended to use a union within the structure
     * to allow for multiple message types to be sent.
     *
     * @tparam T Core user defined task message type
     * @tparam N Number of elements that can be stored in the queue
     */
    template<typename T, size_t N>
    struct Storage
    {
      Internal::TskMsgPool<T, N> pool;  /**< Memory pool for message storage */
      Internal::TskMsgQueue<N>   queue; /**< Message queue */
    };

    MessageQueue();
    ~MessageQueue();

    /**
     * @brief Configures the message with the given configuration settings.
     *
     * @param cfg A constant reference to a Config object containing the configuration settings.
     */
    void configure( const Config &cfg );

    /**
     * @brief Pushes a message onto the message queue.
     *
     * This function attempts to push the provided message onto the message queue.
     *
     * @param msg The message to be pushed onto the queue.
     * @return true if the message was successfully pushed onto the queue.
     * @return false if the message could not be pushed onto the queue.
     */
    bool push( Message &msg );

    /**
     * @brief Removes a message from the queue and assigns it to the provided message object.
     *
     * This function attempts to pop a message from the internal queue. If a message is available,
     * it will be assigned to the provided `msg` parameter, and the function will return true.
     * If no message is available, the function will return false.
     *
     * @param msg A reference to a Message object where the popped message will be stored.
     * @return true if a message was successfully popped from the queue, false otherwise.
     */
    bool pop( Message &msg );

    /**
     * @brief Peek at the next message in the queue without removing it.
     *
     * This function allows you to inspect the next message in the queue without
     * actually removing it from the queue. It is useful for checking the content
     * of the next message without altering the state of the queue.
     *
     * @param msg Reference to a Message object where the peeked message will be stored.
     * @return true if a message was successfully peeked, false if the queue is empty.
     */
    bool peek( Message &msg );

    /**
     * @brief Checks if the message queue is empty.
     *
     * This function returns true if the message queue has no messages,
     * otherwise it returns false.
     *
     * @return true if the message queue is empty, false otherwise.
     */
    bool empty();

    /**
     * @brief Checks the number of elements currently enqueued.
     *
     * @return size_t   The number of elements in the queue.
     */
    size_t size();

  private:
    friend class Lockable<MessageQueue>;

    size_t                  mConfigured; /**< Flag to indicate if the queue has been configured */
    Internal::ITskMsgPool  *mPool;       /**< Memory pool for message storage */
    Internal::ITskMsgQueue *mQueue;      /**< Message queue */
  };

}    // namespace mb::thread

#endif /* !MBEDUTILS_THREADING_MESSAGE_HPP */
