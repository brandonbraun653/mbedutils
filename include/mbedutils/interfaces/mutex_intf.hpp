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
  void initialize();

  /**
   * @brief Creates a new mutex object via dynamic allocation.
   *
   * @param mutex The mutex to create
   */
  void createMutex( mb_mutex_t &mutex );

  /**
   * @brief Destroys a mutex object created **only** by createMutex.
   *
   * @param mutex The mutex to destroy
   */
  void destroyMutex( mb_mutex_t &mutex );

  /**
   * @brief Allocates a mutex object from a fixed size mutex pool.
   *
   * Use this when you know the maximum number of mutexes that will be needed
   * and want to avoid dynamic memory allocation.
   *
   * @param mutex The mutex to allocate into
   */
  void allocateMutex( mb_mutex_t &mutex );

  /**
   * @brief Builds a new mutex object based on the configuration settings.
   *
   * @param mutex Where to put the mutex object
   */
  static inline void buildMutexStrategy( mb_mutex_t &mutex )
  {
#if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    createMutex( mutex );
#else
    static_assert( MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0, "Mutex pool size must be greater than 0" );
    allocateMutex( mutex );
#endif
  }

  /**
   * @brief Initializes the mutex to default values.
   *
   * @param mutex Mutex to initialize
   */
  void initializeMutex( mb_mutex_t mutex );

  /**
   * @brief Locks the mutex
   *
   * @param mutex The mutex to lock
   */
  void lockMutex( mb_mutex_t mutex );

  /**
   * @brief Tries to lock the mutex
   *
   * @param mutex The mutex to try and lock
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockMutex( mb_mutex_t mutex );

  /**
   * @brief Tries to lock the mutex with a timeout
   *
   * @param mutex The mutex to try and lock
   * @param timeout The maximum time to wait for the lock in milliseconds
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockForMutex( mb_mutex_t mutex, const uint32_t timeout );

  /**
   * @brief Unlocks the mutex
   *
   * @param mutex The mutex to unlock
   */
  void unlockMutex( mb_mutex_t mutex );

  /**
   * @brief Creates a new recursive mutex object via dynamic allocation.
   *
   * @return mb_recursive_mutex_t
   */
  void createRecursiveMutex( mb_recursive_mutex_t &mutex );

  /**
   * @brief Destroys a recursive mutex object created with createRecursiveMutex.
   *
   * @param mutex The recursive mutex to destroy
   */
  void destroyRecursiveMutex( mb_recursive_mutex_t &mutex );

  /**
   * @brief Allocates a recursive mutex object from a fixed size mutex pool.
   *
   * Use this when you know the maximum number of recursive mutexes that will be needed
   * and want to avoid dynamic memory allocation.
   *
   * @param mutex The recursive mutex to allocate into
   */
  void allocateRecursiveMutex( mb_recursive_mutex_t &mutex );

  /**
   * @brief Builds a new recursive mutex object based on the configuration settings.
   *
   * @param mutex Where to put the recursive mutex object
   */
  static inline void buildRecursiveMutexStrategy( mb_recursive_mutex_t &mutex )
  {
#if MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION
    createRecursiveMutex( mutex );
#else
    static_assert( MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0, "Recursive Mutex pool size must be greater than 0" );
    allocateRecursiveMutex( mutex );
#endif
  }

  /**
   * @brief Initialize the recursive mutex object
   *
   * @param mutex The recursive mutex to initialize
   */
  void initializeRecursiveMutex( mb_recursive_mutex_t mutex );

  /**
   * @brief Locks the recursive mutex
   *
   * @param mutex The recursive mutex to lock
   */
  void lockRecursiveMutex( mb_recursive_mutex_t mutex );

  /**
   * @brief Tries to lock the recursive mutex
   *
   * @param mutex The recursive mutex to try and lock
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockRecursiveMutex( mb_recursive_mutex_t mutex );

  /**
   * @brief Tries to lock the recursive mutex with a timeout
   *
   * @param mutex The recursive mutex to try and lock
   * @param timeout The maximum time to wait for the lock in milliseconds
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockForRecursiveMutex( mb_recursive_mutex_t mutex, const uint32_t timeout );

  /**
   * @brief Unlocks the recursive mutex
   *
   * @param mutex The recursive mutex to unlock
   */
  void unlockRecursiveMutex( mb_recursive_mutex_t mutex );
}    // namespace mb::osal

#endif /* !MBEDUTILS_MUTEX_INTF_HPP */
