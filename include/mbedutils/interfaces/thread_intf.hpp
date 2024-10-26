/******************************************************************************
 *  File Name:
 *    thread_intf.hpp
 *
 *  Description:
 *    Threading interface for mbedutils implementers.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_INTERFACE_HPP
#define MBEDUTILS_THREADING_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <mbedutils/drivers/threading/thread.hpp>

namespace mb::thread::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initializes the threading interface.
   *
   * This function sets up any necessary resources or configurations required
   * for the threading interface to function properly.
   */
  void initialize();

  /**
   * @brief Creates a new task with the specified configuration.
   *
   * @param cfg Configuration settings for the task
   * @return TaskHandle Handle to the created task
   */
  mb::thread::TaskHandle create_task( const mb::thread::TaskConfig &cfg );

  /**
   * @brief Destroys the specified task.
   *
   * @param task Handle to the task to be destroyed
   */
  void destroy_task( mb::thread::TaskHandle task );

  /**
   * @brief Sets the CPU core affinity for the specified task.
   *
   * @param task   Handle to the task
   * @param coreId ID of the CPU core to which the task should be bound
   */
  void set_affinity( mb::thread::TaskHandle task, size_t coreId );

  /**
   * @brief Starts the task scheduler.
   *
   * This function starts the task scheduler, allowing tasks to begin execution.
   * Note that in some implementations, this function may be a no-op.
   */
  void start_scheduler();

  /**
   * @brief Callback function for handling stack overflow events.
   *
   * This function is called when a stack overflow is detected.
   * The default implementation is declared as weak, allowing projects to override it as needed.
   */
  void on_stack_overflow();

  /**
   * @brief Callback function for handling memory allocation failure events.
   *
   * This function is called when a memory allocation fails.
   * The default implementation is declared as weak, allowing projects to override it as needed.
   */
  void on_malloc_failed();

  /**
   * @brief Callback function for handling idle state events.
   *
   * This function is called when the system is idle.
   * The default implementation is declared as weak, allowing projects to override it as needed.
   */
  void on_idle();

  /**
   * @brief Callback function for handling tick events.
   *
   * This function is called on each system tick.
   * The default implementation is declared as weak, allowing projects to override it as needed.
   */
  void on_tick();

}  // namespace mb::thread::intf

#endif  /* !MBEDUTILS_THREADING_INTERFACE_HPP */
