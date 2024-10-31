/******************************************************************************
 *  File Name:
 *    condition.hpp
 *
 *  Description:
 *    Condition variable interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_CONDITION_VARIABLE_HPP
#define MBEDUTILS_CONDITION_VARIABLE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/delegate.h>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <mbedutils/interfaces/smphr_intf.hpp>
#include <mbedutils/assert.hpp>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using cv_predicate = etl::delegate<bool(void)>;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum class cv_status
  {
    timeout,
    no_timeout
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Standard condition variable implementation.
   * @see https://en.cppreference.com/w/cpp/thread/condition_variable
   *
   * This class is a wrapper around the mbed OSAL mutex and semaphore
   * primitives. It provides a way to synchronize threads based on a
   * condition predicate, similar to the C++ standard library. The main
   * difference is that this implementation is not as feature rich and
   * is geared towards an embedded MCU RTOS environment.
   */
  class ConditionVariable
  {
  public:
    ConditionVariable();

    /**
     * @brief Acquire condition variable resources.
     *
     * This must be called before using the condition variable. It will
     * allocate the necessary resources for the condition variable to
     * function properly.
     */
    void init();

    /**
     * @brief Release condition variable resources.
     */
    void deinit();

    /**
     * @brief Wait for the condition variable to be signaled.
     *
     * @param mtx Mutex to synchronize with
     */
    void wait( mb::osal::mb_mutex_t &mtx );

    /**
     * @brief Wait for the given predicate to occur
     *
     * @param mtx  Mutex to synchronize with
     * @param pred Condition to look for
     */
    void wait( mb::osal::mb_mutex_t &mtx, const cv_predicate &pred );

    /**
     * @brief Waits a number of milliseconds to be signaled.
     *
     * @param mtx Mutex to synchronize with
     * @param timeout Number of milliseconds to wait
     * @return cv_status
     */
    cv_status wait_for( mb::osal::mb_mutex_t &mtx, const size_t timeout );

    /**
     * @brief Waits for the predicate to become true or a timeout to occur
     *
     * @param mtx Mutex to synchronize with
     * @param timeout Number of milliseconds to wait
     * @param pred Condition to look for
     * @return Status of the predicate upon exiting
     */
    bool wait_for( mb::osal::mb_mutex_t &mtx, const size_t timeout, const cv_predicate &pred );

    /**
     * @brief Notifies a single waiting thread
     */
    void notify_one();

    /**
     * @brief Notifies all waiting threads
     */
    void notify_all();

  private:
    int                  _cv_waiters;
    mb::osal::mb_mutex_t _cv_mtx;
    mb::osal::mb_smphr_t _cv_smphr_signal;
    size_t               _cv_initialized;

    void increment_waiters();
    void decrement_waiters();
  };

}    // namespace mb::thread

#endif /* !MBEDUTILS_CONDITION_VARIABLE_HPP */
