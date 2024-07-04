/******************************************************************************
 *  File Name:
 *    smphr_intf.hpp
 *
 *  Description:
 *    OSAL interface for the semaphore abstraction
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_OSAL_SEMAPHORE_INTERFACE_HPP
#define MBEDUTILS_OSAL_SEMAPHORE_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>
#include <mbedutils/config.hpp>

namespace mb::osal
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Alias for any implementation of a semaphore
   */
  using mb_smphr_t = void *;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the osal semaphore driver.
   *
   * This must be called before any of the API functions are used.
   */
  void initSmphrDriver();

  /**
   * @brief Creates a new semaphore object via dynamic allocation.
   *
   * @param s            The semaphore to create
   * @param maxCount     The maximum number of tokens the semaphore can hold
   * @param initialCount The initial number of tokens the semaphore starts with
   * @return bool        True if the semaphore was created, false otherwise
   */
  bool createSmphr( mb_smphr_t &s, const size_t maxCount, const size_t initialCount );

  /**
   * @brief Destroys a semaphore object created **only** by createSmphr.
   *
   * @param s The semaphore to destroy
   */
  void destroySmphr( mb_smphr_t &s );

  /**
   * @brief Allocates a semaphore object from a fixed size pool.
   *
   * Use this when you know the maximum number of semaphores that will be needed
   * and want to avoid dynamic memory allocation.
   *
   * @param s            The semaphore to allocate into
   * @param maxCount     The maximum number of tokens the semaphore can hold
   * @param initialCount The initial number of tokens the semaphore starts with
   * @return bool        True if the semaphore was allocated, false otherwise
   */
  bool allocateSemahpore( mb_smphr_t &s, const size_t maxCount, const size_t initialCount );

  /**
   * @brief Deallocates the semaphore object back into the pool.
   *
   * @param s The semaphore to deallocate
   */
  void deallocateSemaphore( mb_smphr_t &s );

  /**
   * @brief Builds a new semaphore object based on the configuration settings.
   *
   * @param s            The semaphore to build
   * @param maxCount     The maximum number of tokens the semaphore can hold
   * @param initialCount The initial number of tokens the semaphore starts with
   * @return bool        True if the semaphore was built, false otherwise
   */
  static inline bool buildSmphrStrategy( mb_smphr_t &s, const size_t maxCount, const size_t initialCount )
  {
#if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    return createSmphr( s, maxCount, initialCount );
#else
    static_assert( MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0, "Smphr pool size must be greater than 0" );
    return allocateSmphr( s, maxCount, initialCount);
#endif
  }

  /**
   * @brief Destroys a semaphore object based on the configuration settings.
   *
   * @param s The semaphore to destroy
   */
  static inline void destroySmphrStrategy( mb_smphr_t &s )
  {
#if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    destroySmphr( s );
#else
    deallocateSemaphore( s );
#endif
  }

  /**
   * @brief Get the number of available semaphore tokens to be taken.
   *
   * @param s Semaphore to inspect
   * @return size_t
   */
  size_t getSmphrAvailable( mb_smphr_t &s );

  /**
   * @brief Releases a semaphore token.
   *
   * @param s Semaphore to release on
   */
  void releaseSmphr( mb_smphr_t &s );

  /**
   * @brief Releases a semaphore token from an ISR context.
   *
   * @param s Semaphore to release
   */
  void releaseSmphrFromISR( mb_smphr_t &s );

  /**
   * @brief Acquires the semaphore with blocking.
   *
   * @param s Semaphore to acquire
   */
  void acquireSmphr( mb_smphr_t &s );

  /**
   * @brief Tries to acquire the semaphore, immediately returning.
   *
   * @param s     Semaphore to acquire
   * @return bool True if the semaphore was acquired, false otherwise
   */
  bool tryAcquireSmphr( mb_smphr_t &s );
  /**
   * @brief Acquires the semaphore with a timeout.
   *
   * @param s       Semaphore to acquire
   * @param timeout How long to wait for the semaphore (ms)
   * @return bool   True if the semaphore was acquired, false if timed out
   */
  bool tryAcquireSmphr( mb_smphr_t &s, const size_t timeout );

}  // namespace mb::osal

#endif  /* !MBEDUTILS_OSAL_SEMAPHORE_INTERFACE_HPP */
