/******************************************************************************
 *  File Name:
 *    lock.hpp
 *
 *  Description:
 *    Threading lock classes for mutex/semaphore based functionality
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_LOCK_HPP
#define MBEDUTILS_THREADING_LOCK_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <mbedutils/assert.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <mbedutils/interfaces/smphr_intf.hpp>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Interfaces
  ---------------------------------------------------------------------------*/
  /**
   *  @brief A generic lock interface that can be attached to objects that need locking
   */
  class LockableInterface
  {
  public:
    virtual ~LockableInterface() = default;

    /**
     * @brief Reserve the inheriting object, blocking if not immediately successful.
     * @warning This function can only run from unprivileged code. Do **not** execute in an ISR.
     */
    virtual void lock() = 0;

    /**
     * @brief Tries to lock the resource without blocking
     * @return bool True if the lock was acquired, false otherwise
     */
    virtual bool try_lock() = 0;

    /**
     * @brief Tries to lock the resource for the given amount of time
     *
     * @param timeout How long to wait to acquire the lock in milliseconds
     * @return bool   True if the lock was acquired, false otherwise
     */
    virtual bool try_lock_for( const size_t timeout ) = 0;

    /**
     * @brief Release the inheriting object, assuming current thread has ownership.
     * @warning This function can only run from unprivileged code. Do **not** execute in an ISR.
     */
    virtual void unlock() = 0;
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief CRTP interface to add lock functionality to a class.
   *
   * This will require "friend-ing" this class:
   * friend ::mb::thread::Lockable<MyInheritingClass>;
   *
   * @tparam T  Base class needing a lock interface
   */
  template<class T>
  class Lockable : public virtual LockableInterface
  {
  public:
    Lockable() = default;
    ~Lockable()
    {
      if( mLockableMutex != nullptr )
      {
        osal::destroyMutexStrategy( mLockableMutex );
      }
    }

    void lock() final override
    {
      osal::lockRecursiveMutex( static_cast<T *>( this )->mLockableMutex );
    }

    bool try_lock() final override
    {
      return osal::tryLockRecursiveMutex( static_cast<T *>( this )->mLockableMutex );
    }

    bool try_lock_for( const size_t timeout ) final override
    {
      return osal::tryLockRecursiveMutex( static_cast<T *>( this )->mLockableMutex, timeout );
    }

    void unlock() final override
    {
      osal::unlockRecursiveMutex( static_cast<T *>( this )->mLockableMutex );
    }

  protected:
    osal::mb_recursive_mutex_t mLockableMutex;

    void initLockable()
    {
      if( mLockableMutex == nullptr )
      {
        mbed_assert( ::mb::osal::buildRecursiveMutexStrategy( mLockableMutex ) );
      }
    }
  };


  /**
   * @brief Analog to the std::lock_guard
   */
  class LockGuard
  {
  public:
    explicit LockGuard( osal::mb_mutex_t &mutex ) : mtx( mutex )
    {
      osal::lockMutex( mtx );
    }

    ~LockGuard()
    {
      osal::unlockMutex( mtx );
    }

    LockGuard( const LockGuard & ) = delete;

  private:
    osal::mb_mutex_t mtx;
  };


  /**
   * @brief Analog to the std::lock_guard
   */
  class RecursiveLockGuard
  {
  public:
    explicit RecursiveLockGuard( osal::mb_recursive_mutex_t &mutex ) : mtx( mutex )
    {
      osal::lockRecursiveMutex( mtx );
    }

    ~RecursiveLockGuard()
    {
      osal::unlockRecursiveMutex( mtx );
    }

    RecursiveLockGuard( const RecursiveLockGuard & ) = delete;

  private:
    osal::mb_recursive_mutex_t mtx;
  };

}    // namespace mb::osal

#endif /* !MBEDUTILS_THREADING_LOCK_HPP */
