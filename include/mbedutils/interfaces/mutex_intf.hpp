/******************************************************************************
 *  File Name:
 *    mutex_intf.hpp
 *
 *  Description:
 *    Standardd interface for mutexes of all flavors
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

namespace mbedutils::osal
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using mutex_t           = void *;
  using recursive_mutex_t = void *;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Creates a new mutex object
   *
   * @return mutex_t
   */
  mutex_t createMutex();

  /**
   * @brief Initializes the mutex
   *
   * @param mutex Mutex to initialize
   */
  void initializeMutex( mutex_t mutex );

  /**
   * @brief Destroys a mutex object
   *
   * @param mutex The mutex to destroy
   */
  void destroyMutex( mutex_t mutex );

  /**
   * @brief Locks the mutex
   *
   * @param mutex The mutex to lock
   */
  void lockMutex( mutex_t mutex );

  /**
   * @brief Tries to lock the mutex
   *
   * @param mutex The mutex to try and lock
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockMutex( mutex_t mutex );

  /**
   * @brief Tries to lock the mutex with a timeout
   *
   * @param mutex The mutex to try and lock
   * @param timeout The maximum time to wait for the lock in milliseconds
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockForMutex( mutex_t mutex, const uint32_t timeout );

  /**
   * @brief Unlocks the mutex
   *
   * @param mutex The mutex to unlock
   */
  void unlockMutex( mutex_t mutex );

  /**
   * @brief Creates a new recursive mutex object
   *
   * @return recursive_mutex_t
   */
  void createRecursiveMutex( recursive_mutex_t *mutex );

  /**
   * @brief Initialize the recursive mutex object
   *
   * @param mutex The recursive mutex to initialize
   */
  void initializeRecursiveMutex( recursive_mutex_t mutex );

  /**
   * @brief Destroys a recursive mutex object
   *
   * @param mutex The recursive mutex to destroy
   */
  void destroyRecursiveMutex( recursive_mutex_t *mutex );

  /**
   * @brief Locks the recursive mutex
   *
   * @param mutex The recursive mutex to lock
   */
  void lockRecursiveMutex( recursive_mutex_t mutex );

  /**
   * @brief Tries to lock the recursive mutex
   *
   * @param mutex The recursive mutex to try and lock
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockRecursiveMutex( recursive_mutex_t mutex );

  /**
   * @brief Tries to lock the recursive mutex with a timeout
   *
   * @param mutex The recursive mutex to try and lock
   * @param timeout The maximum time to wait for the lock in milliseconds
   * @return bool True if the lock was successful, false otherwise
   */
  bool tryLockForRecursiveMutex( recursive_mutex_t mutex, const uint32_t timeout );

  /**
   * @brief Unlocks the recursive mutex
   *
   * @param mutex The recursive mutex to unlock
   */
  void unlockRecursiveMutex( recursive_mutex_t mutex );
}  // namespace

#endif  /* !MBEDUTILS_MUTEX_INTF_HPP */
