/******************************************************************************
 *  File Name:
 *    mutex_intf.hpp
 *
 *  Description:
 *    Standard interface for mutexes of all flavors
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_MUTEX_INTF_HPP
#define MBEDUTILS_MUTEX_INTF_HPP

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

  using mb_mutex_t           = void *;
  using mb_recursive_mutex_t = void *;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the osal mutex driver.
   *
   * This must be called before any of the API functions are used.
   */
  void initMutexDriver();

  /**
   * @brief Creates a new mutex object via dynamic allocation.
   *
   * @param mutex The mutex to create
   * @return bool True if the mutex was created, false otherwise
   */
  bool createMutex( mb::osal::mb_mutex_t &mutex );

  /**
   * @brief Destroys a mutex object created **only** by createMutex.
   *
   * @param mutex The mutex to destroy
   */
  void destroyMutex( mb::osal::mb_mutex_t &mutex );

  /**
   * @brief Allocates a mutex object from a fixed size mutex pool.
   *
   * Use this when you know the maximum number of mutexes that will be needed
   * and want to avoid dynamic memory allocation.
   *
   * @param mutex The mutex to allocate into
   * @return bool True if the mutex was allocated, false otherwise
   */
  bool allocateMutex( mb::osal::mb_mutex_t &mutex );

  /**
   * @brief Deallocates a mutex back to the pool
   *
   * @param mutex Mutex to deallocate
   */
  void deallocateMutex( mb::osal::mb_mutex_t &mutex );

  /**
   * @brief Builds a new mutex object based on the configuration settings.
   *
   * @param mutex Where to put the mutex object
   * @return bool True if the mutex was built, false otherwise
   */
  static inline bool buildMutexStrategy( mb::osal::mb_mutex_t &mutex )
  {
#if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    return createMutex( mutex );
#else
    static_assert( MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0, "Mutex pool size must be greater than 0" );
    return allocateMutex( mutex );
#endif
  }

  /**
   * @brief Destroys a mutex object based on the configuration settings.
   *
   * @param mutex Mutex to destroy
   */
  static inline void destroyMutexStrategy( mb::osal::mb_mutex_t &mutex )
  {
#if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    destroyMutex( mutex );
#else
    deallocateMutex( mutex );
#endif
  }

  /**
   * @brief Locks the mutex
   *
   * @param mutex The mutex to lock
   */
  void lockMutex( mb::osal::mb_mutex_t mutex );

  /**
   * @brief Tries to lock the mutex
   *
   * @param mutex The mutex to try and lock
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockMutex( mb::osal::mb_mutex_t mutex );

  /**
   * @brief Tries to lock the mutex with a timeout
   *
   * @param mutex The mutex to try and lock
   * @param timeout The maximum time to wait for the lock in milliseconds
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockMutex( mb::osal::mb_mutex_t mutex, const size_t timeout );

  /**
   * @brief Unlocks the mutex
   *
   * @param mutex The mutex to unlock
   */
  void unlockMutex( mb::osal::mb_mutex_t mutex );

  /**
   * @brief Creates a new recursive mutex object via dynamic allocation.
   *
   * @param mutex The recursive mutex to create
   * @return bool True if the recursive mutex was created, false otherwise
   */
  bool createRecursiveMutex( mb::osal::mb_recursive_mutex_t &mutex );

  /**
   * @brief Destroys a recursive mutex object created with createRecursiveMutex.
   *
   * @param mutex The recursive mutex to destroy
   */
  void destroyRecursiveMutex( mb::osal::mb_recursive_mutex_t &mutex );

  /**
   * @brief Allocates a recursive mutex object from a fixed size mutex pool.
   *
   * Use this when you know the maximum number of recursive mutexes that will be needed
   * and want to avoid dynamic memory allocation.
   *
   * @param mutex The recursive mutex to allocate into
   * @return bool True if the recursive mutex was allocated, false otherwise
   */
  bool allocateRecursiveMutex( mb::osal::mb_recursive_mutex_t &mutex );

  /**
   * @brief Deallocates a recursive mutex back to the pool
   *
   * @param mutex Recursive mutex to deallocate
   */
  void deallocateRecursiveMutex( mb_recursive_mutex_t &mutex );

  /**
   * @brief Builds a new recursive mutex object based on the configuration settings.
   *
   * @param mutex Where to put the recursive mutex object
   * @return bool True if the recursive mutex was built, false otherwise
   */
  static inline bool buildRecursiveMutexStrategy( mb::osal::mb_recursive_mutex_t &mutex )
  {
#if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    return createRecursiveMutex( mutex );
#else
    static_assert( MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0, "Recursive Mutex pool size must be greater than 0" );
    return allocateRecursiveMutex( mutex );
#endif
  }

  /**
   * @brief Destroys a recursive mutex object based on the configuration settings.
   *
   * @param mutex Recursive mutex to destroy
   */
  static inline void destroyRecursiveMutexStrategy( mb::osal::mb_recursive_mutex_t &mutex )
  {
  #if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    destroyRecursiveMutex( mutex );
  #else
    static_assert( MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0, "Recursive Mutex pool size must be greater than 0" );
    deallocateRecursiveMutex( mutex );
  #endif
  }

  /**
   * @brief Locks the recursive mutex
   *
   * @param mutex The recursive mutex to lock
   */
  void lockRecursiveMutex( mb::osal::mb_recursive_mutex_t mutex );

  /**
   * @brief Tries to lock the recursive mutex
   *
   * @param mutex The recursive mutex to try and lock
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockRecursiveMutex( mb::osal::mb_recursive_mutex_t mutex );

  /**
   * @brief Tries to lock the recursive mutex with a timeout
   *
   * @param mutex The recursive mutex to try and lock
   * @param timeout The maximum time to wait for the lock in milliseconds
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockRecursiveMutex( mb::osal::mb_recursive_mutex_t mutex, const size_t timeout );

  /**
   * @brief Unlocks the recursive mutex
   *
   * @param mutex The recursive mutex to unlock
   */
  void unlockRecursiveMutex( mb::osal::mb_recursive_mutex_t mutex );
}    // namespace mb::osal

#endif /* !MBEDUTILS_MUTEX_INTF_HPP */
