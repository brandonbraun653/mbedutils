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
#include <etl/flat_map.h>
#include <etl/pool.h>
#include <etl/queue.h>
#include <etl/string.h>
#include <string>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/

  class Task;
  struct TaskControlBlock;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using TaskId       = size_t;
  using TaskMsgId    = size_t;
  using TaskPriority = uint8_t;
  using TaskAffinity = int8_t;
  using TaskFunction = void ( * )( void * );
  using TaskName     = etl::string_view;
  using TaskMsgPool  = etl::ipool *;
  using TaskMsgQueue = etl::iqueue<void *> *;
  using TaskCBMap    = etl::iflat_map<TaskId, TaskControlBlock> *;
  using TaskHandle   = void *;

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

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Internal representation of a task control block.
   *
   * This normally would not be exposed to the user, but is here to allow for
   * static allocation of memory.
   */
  struct TaskControlBlock
  {
    TaskName             name;            /**< Name of the thread */
    TaskHandle           handle;          /**< Implementation specific handle to the task */
    TaskPriority         priority;        /**< System thread priority. Lower == more importance */
    TaskMsgPool          msg_pool;        /**< Storage for allocating task messages */
    TaskMsgQueue         msg_queue;       /**< Storage for queueing task messages */
    ConditionVariable    msg_queue_cv;    /**< Condition variable for the message queue */
    mb::osal::mb_mutex_t msg_queue_mutex; /**< Mutex for the message queue */
  };

  template<size_t N>
  using TaskControlBlockStorage = etl::flat_map<TaskId, TaskControlBlock, N>;

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

  /**
   * @brief Helper template to allocate memory for a task configuration.
   *
   * @tparam TaskMsgType  Type of the task message
   * @tparam TaskMsgDepth Element count of the task message queue
   * @tparam StackSize    Number of bytes for the thread stack
   */
  template<typename TaskMsgType = uint8_t, size_t TaskMsgDepth = 1, size_t StackSize = 256>
  struct TaskConfigStorage
  {
    // Place stack first to ensure alignment
    static_assert( StackSize % sizeof( uint32_t ) == 0, "Stack must be word aligned" );
    uint32_t stack[ StackSize / sizeof( uint32_t ) ];

    etl::string<32>                               name;
    TaskMsgPoolStorage<TaskMsgType, TaskMsgDepth> msg_pool;
    TaskMsgQueueStorage<TaskMsgDepth>             msg_queue;
  };

  /**
   * @brief Configuration for a system thread/task.
   *
   * This is intended to be used with embedded systems that may require static
   * allocation of memory. Also compatible with dynamic allocation if needed.
   */
  struct TaskConfig
  {
    TaskName     name;       /**< Name of the thread */
    TaskId       id;         /**< System ID of the thread */
    TaskFunction func;       /**< Function to run as the task*/
    TaskPriority priority;   /**< System thread priority. Lower == more importance */
    TaskAffinity affinity;   /**< (Optional) Core affinity bitmask (multi-core only) */
    TaskMsgPool  msg_pool;   /**< (Optional) Storage for allocating task messages */
    TaskMsgQueue msg_queue;  /**< (Optional) Storage for queueing task messages */
    uint32_t    *stack_buf;  /**< (Optional) Statically allocated stack */
    uint32_t     stack_size; /**< (Optional) Element count of stack buffer */
    void        *user_data;  /**< (Optional) User data to pass to the thread */

    void reset()
    {
      name     = "";
      id       = 0;
      func     = nullptr;
      priority = 0;
      affinity = 0;
      msg_pool = nullptr;
      msg_queue = nullptr;
      stack_buf = nullptr;
      stack_size = 0;
      user_data = nullptr;
    }
  };

  /**
   * @brief Configuration to provide resources to the thread library.
   */
  struct ModuleConfig
  {
    TaskCBMap tsk_control_blocks; /**< Control structures for each thread */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the mbedutils threading module.
   *
   * @param cfg Configuration to use.
   */
  void initialize( const ModuleConfig &cfg );

  /**
   * @brief Create a new task
   *
   * @param cfg Configuration to use
   * @return bool
   */
  Task &&create( const TaskConfig &cfg );

  /**
   * @brief Begins the scheduler for multi-threading.
   */
  void startScheduler();

  /**
   * @brief Send a message to another thread
   *
   * @param id      Task id to send to
   * @param msg     Message to send
   * @param timeout How long to wait in milliseconds
   */
  void sendMessage( const TaskId id, const void *msg, const size_t size, const size_t timeout );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Abstraction of a thread, mostly STL compliant.
   */
  class Task
  {
  public:
    Task() noexcept;
    ~Task();

    // Move assignment operator
    Task &operator=( Task &&other ) noexcept;

    // Disallow copying
    Task( const Task & )            = delete;
    Task &operator=( const Task & ) = delete;

    /**
     * @brief Starts the thread.
     */
    void start();

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

  private:
    friend ::mb::thread::Task &&create( const TaskConfig &cfg );

    TaskId     taskId; /**< System identifier for the thread */
    TaskHandle pimpl;  /**< Implementation details */
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
    bool pendTaskMsg( void *msg, const size_t timeout );

  }    // namespace this_thread
}    // namespace mb::thread

#endif /* !MBEDUTILS_THREADING_HPP */
