/******************************************************************************
 *  File Name:
 *    thread.hpp
 *
 *  Description:
 *    Threading interface for mbedutils
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_HPP
#define MBEDUTILS_THREADING_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/delegate.h>
#include <etl/string.h>
#include <etl/pool.h>
#include <etl/queue.h>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using TaskId       = size_t;
  using TaskMsgId    = size_t;
  using TaskPriority = uint8_t;
  using TaskAffinity = uint8_t;
  using TaskFunction = etl::delegate<void( void * )>;
  using TaskName     = etl::string<32>;
  using TaskMsgPool  = etl::ipool;
  using TaskMsgQueue = etl::iqueue;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr size_t TIMEOUT_BLOCK     = std::numeric_limits<size_t>::max();
  static constexpr size_t TIMEOUT_DONT_WAIT = 0;
  static constexpr size_t TIMEOUT_1MS       = 1;
  static constexpr size_t TIMEOUT_5MS       = 5;
  static constexpr size_t TIMEOUT_10MS      = 10;
  static constexpr size_t TIMEOUT_25MS      = 25;
  static constexpr size_t TIMEOUT_50MS      = 50;
  static constexpr size_t TIMEOUT_100MS     = 100;
  static constexpr size_t TIMEOUT_500MS     = 500;
  static constexpr size_t TIMEOUT_1S        = 1000;
  static constexpr size_t TIMEOUT_1MIN      = 60 * TIMEOUT_1S;
  static constexpr size_t TIMEOUT_1HR       = 60 * TIMEOUT_1MIN;
  static constexpr TaskId THREAD_ID_INVALID = 0xCCCCCCCC;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  // Define a base message structure
  struct TaskMsg
  {
    TaskMsgId id;
    virtual ~BaseTaskMessage() = default;
  };

  // Template for specific message types
  template<typename PayloadType>
  struct TypedTaskMessage : TaskMsg
  {
    PayloadType payload;
  };

  template<typename PayloadType, size_t N>
  using TaskMsgPoolStorage = etl::pool<PayloadType, N>;

  template<typename PayloadType, size_t N>
  using TaskMsgQueueStorage = etl::queue<PayloadType *, N>;

  /**
   * @brief Configuration for a system thread/task.
   *
   * This is intended to be used with embedded systems that may require static
   * allocation of memory. Also compatible with dynamic allocation if needed.
   */
  struct TaskConfig
  {
    TaskName     name;       /**< Name of the thread */
    TaskFunction func;       /**< Function to run as the task*/
    TaskPriority priority;   /**< System thread priority. Lower == more importance */
    TaskAffinity affinity;   /**< (Optional) Core affinity (multi-core only) */
    TaskMsgPool  msg_pool;   /**< (Optional) Storage for allocating task messages */
    TaskMsgQueue msg_queue;  /**< (Optional) Storage for queueing task messages */
    uint32_t    *stack_buf;  /**< (Optional) Statically allocated stack */
    uint32_t     stack_size; /**< (Optional) Element count of stack buffer */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Create a new task
   *
   * @param cfg   Configuration to use
   * @return Task
   */
  TaskId create( const TaskConfig &cfg );

  /**
   * @brief Find a task ID by name
   *
   * @param name  Expected name of the thread
   * @return TaskId
   */
  TaskId lookup( const TaskName &name );

  /**
   * @brief Send a message to another thread
   *
   * @param id      Task id to send to
   * @param msg     Message to send
   * @param timeout How long to wait in milliseconds
   */
  void sendMessage( const TaskId id, const TaskMsg &msg, const size_t timeout );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class Task
  {
  public:
    Task() noexcept;
    ~Task();

    // Move assignment operator
    Task& operator=(Task&& other) noexcept;

    // Disallow copying
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    /**
     * @brief Starts the thread, returning its generated id.
     *
     * @return TaskId
     */
    TaskId start();

    /**
     * @brief Suspends the thread, assuming it's supported.
     *
     * This is mostly taken from real time operating systems (FreeRTOS) which
     * can commonly suspend a single thread from execution.
     */
    void suspend();

    /**
     * @brief Resumes a previously suspended thread.
     *
     * Does nothing if thread suspension is not supported.
     */
    void resume();

    /**
     * @brief Sends a kills signal to the thread.
     */
    void kill();

    /**
     * @brief Waits for the thread to terminate.
     */
    void join();

    /**
     * @brief Checks if the thread can be joined
     *
     * @return bool
     */
    bool joinable();
  };

  namespace this_thread
  {
    /**
     * @brief Sets the name of the currently executing thread.
     *
     * @param name  Name to be set
     */
    void set_name( const char *name );

    /**
     * @brief Get the name of the currently executing thread.
     *
     * @return TaskName
     */
    TaskName &get_name();

    /**
     * @brief Sleeps the current thread for a number of milliseconds.
     *
     * @param timeout How many milliseconds to wait
     */
    void sleep_for( const size_t timeout );

    /**
     * @brief Sleeps the current thread until an absolute time in the future.
     *
     * @param wakeup System time to wake up at in milliseconds
     */
    void sleep_until( const size_t wakeup );

    /**
     * @brief Yields execution to another thread.
     */
    void yield();

    /**
     * @brief Instructs the scheduler to place this thread in the suspended state.
     */
    void suspend();

    /**
     *  Gets the system identifier of the current thread
     *  @return TaskId
     */
    TaskId id();

    /**
     * @brief Gets the latest task message for the thread.
     *
     * @param msg     Where to place the message, if valid
     * @param timeout How long to wait for a new message
     * @return bool
     */
    bool pendTaskMsg( TaskMsg &msg, const size_t timeout );

  }  // namespace this_thread
}  // namespace mb::thread

#endif  /* !MBEDUTILS_THREADING_HPP */
