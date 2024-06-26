/******************************************************************************
 *  File Name:
 *    mutex_extensions.hpp
 *
 *  Description:
 *    Extension methods for wrapping mutex based functionality
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_MUTEX_EXTENSIONS_HPP
#define MBEDUTILS_MUTEX_EXTENSIONS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <mbedutils/interfaces/mutex_intf.hpp>

namespace mbedutils::osal
{
  /**
   *  @brief A generic lock interface that can be attached to objects that need locking
   */
  class LockableInterface
  {
  public:
    virtual ~LockableInterface() = default;

    /**
     * @brief Reserve the inheriting object, blocking if not immediately successful.
     *
     * @warning This function can only run from unprivileged code. Do **not** execute in an ISR.
     * @return void
     */
    virtual void lock() = 0;

    /**
     * @brief Tries to lock the resource for the given amount of time
     *
     * @param timeout     How long to wait to acquire the lock in milliseconds
     * @return bool
     */
    virtual bool try_lock_for( const size_t timeout ) = 0;

    /**
     * @brief Release the inheriting object, assuming current thread has ownership
     *
     * @warning This function can only run from unprivileged code. Do **not** execute in an ISR.
     * @return void
     */
    virtual void unlock() = 0;
  };


  /**
   * @brief CRTP interface to add lock functionality to a class.
   *
   * This will require "friend-ing" this class:
   * friend ::mbedutils::osal::Lockable<MyInheritingClass>;
   *
   * @tparam T  Base class needing a lock interface
   */
  template<class T>
  class Lockable : public virtual LockableInterface
  {
  public:
    void lock()
    {
      osal::lockRecursiveMutex( static_cast<T *>( this )->mClsMutex );
    }

    bool try_lock_for( const size_t timeout )
    {
      return osal::tryLockForRecursiveMutex( static_cast<T *>( this )->mClsMutex, timeout );
    }

    void unlock()
    {
      osal::unlockRecursiveMutex( static_cast<T *>( this )->mClsMutex );
    }

  protected:
    osal::recursive_mutex_t mClsMutex;
  };


  /**
   * @brief Analog to the std::lock_guard
   */
  class LockGuard
  {
  public:
    explicit LockGuard( osal::mutex_t &mutex ) : mtx( &mutex )
    {
      osal::lockMutex( mtx );
    }

    ~LockGuard()
    {
      osal::unlockMutex( mtx );
    }

    LockGuard( const LockGuard & ) = delete;

  private:
    osal::mutex_t *mtx;
  };


  /**
   * @brief Analog to the std::lock_guard
   */
  class RecursiveLockGuard
  {
  public:
    explicit RecursiveLockGuard( osal::recursive_mutex_t &mutex ) : mtx( &mutex )
    {
      osal::lockRecursiveMutex( mtx );
    }

    ~RecursiveLockGuard()
    {
      osal::unlockRecursiveMutex( mtx );
    }

    RecursiveLockGuard( const RecursiveLockGuard & ) = delete;

  private:
    osal::recursive_mutex_t *mtx;
  };

}  // namespace mbedutils::osal

#endif  /* !MBEDUTILS_MUTEX_EXTENSIONS_HPP */
