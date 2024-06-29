/******************************************************************************
 *  File Name:
 *    extensions.hpp
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
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/threading/event.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <mbedutils/interfaces/smphr_intf.hpp>
#include <mbedutils/interfaces/time_intf.hpp>
#include <mbedutils/interfaces/util_intf.hpp>

namespace mb::osal
{
  /*---------------------------------------------------------------------------
  Interfaces
  ---------------------------------------------------------------------------*/

  /**
   * @brief Asynchronous notification interface
   * @warning Expects to be used as a SPSC type of signaling mechanism
   *
   * Highly useful for signaling that a particular event has happened, such as
   * a USART transaction has completed, or DMA operation is finished.
   */
  class AsyncIOInterface
  {
  public:
    virtual ~AsyncIOInterface() = default;

    /**
     * @brief Wait on an event, or timeout.
     *
     * Asynchronously waits for the given event to occur before the function
     * will return. The is accomplished by blocking the current thread.
     *
     * @param event       The event upon which to be triggered
     * @param timeout     How long to wait for the event to occur
     * @return mb::ErrorCode
     */
    virtual mb::ErrorCode await( const mb::thread::Event event, const size_t timeout ) = 0;

    /**
     * @brief Block on a specific signal and await an event, or timeout.
     *
     * A more explicit version of await that allows selecting the threading
     * primitive on which to block. This could be useful if multiple owners
     * might unblock a process or if the event generator is nested several
     * calls deep in the stack.
     *
     * @param event       The event upon which to be triggered
     * @param notifier    Semaphore to be given to upon the event occurrence
     * @param timeout     How long to wait for the event to occur
     * @return mb::ErrorCode
     */
    virtual mb::ErrorCode await( const mb::thread::Event event, mb::osal::mb_smphr_t &notifier, const size_t timeout ) = 0;

    /**
     * @brief Signal that a particular event has occurred
     *
     * @param trigger   Which event occurred
     * @return void
     */
    virtual void signalAIO( const mb::thread::Event trigger ) = 0;

    /**
     * @brief Signal that a particular event has occurred, but from an ISR safe context
     *
     * @param trigger   Which event occurred
     * @return void
     */
    virtual void signalAIOFromISR( const mb::thread::Event trigger ) = 0;
  };

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
     * @brief Tries to lock the resource without blocking
     *
     * @return bool True if the lock was acquired, false otherwise
     */
    virtual bool try_lock() = 0;

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
   * friend ::mb::osal::Lockable<MyInheritingClass>;
   *
   * @tparam T  Base class needing a lock interface
   */
  template<class T>
  class Lockable : public virtual LockableInterface
  {
  public:
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

  /**
   * @brief CRTP interface to add AsyncIO notification functionality to a class
   * @tparam T  Base class being extended
   */
  template<class T>
  class AsyncIO : public virtual AsyncIOInterface
  {
  public:
    AsyncIO() :
        mAIOAllowedEvents( 0xFFFFFFFF ), mAIOInitialized( ~DRIVER_INITIALIZED_KEY ), mAIOEvent( mb::thread::EVENT_UNKNOWN )
    {
    }

    ~AsyncIO()
    {
    }


    mb::ErrorCode await( const mb::thread::Event event, const size_t timeout ) final override
    {
      mbed_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      /*-----------------------------------------------------------------------
      Check for event support
      -----------------------------------------------------------------------*/
      size_t event_idx = static_cast<size_t>( event );
      if( ( event >= mb::thread::EVENT_NUM_OPTIONS ) || !( mAIOAllowedEvents & ( 1u << event_idx ) ) )
      {
        return ErrorCode::ERR_NOT_SUPPORTED;
      }

      /*-----------------------------------------------------------------------
      Enforce the SPSC idea baked into this AsyncIO topology
      -----------------------------------------------------------------------*/
      LockGuard _lck( mAIOMutex );

      /*-----------------------------------------------------------------------
      Wait for the event to occur. There are two timeouts at play:
        1. High level timeout to ensure no infinite loops.
        2. More fine-grained timeout to accurately block in "tryAcquireSmphr()"
      -----------------------------------------------------------------------*/
      size_t startTime         = time::millis();
      size_t lastWake          = startTime;
      size_t waitTimeRemaining = timeout;

      while( ( time::millis() - startTime ) < timeout )
      {
        osal::tryAcquireSmphr( mAIOSignal, waitTimeRemaining );

        /*---------------------------------------------------------------------
        Expected event (or error) signaled?
        ---------------------------------------------------------------------*/
        if( mAIOEvent == mb::thread::EVENT_SYSTEM_ERROR )
        {
          return ErrorCode::ERR_FAIL;
        }
        else if( mAIOEvent == event )
        {
          return ErrorCode::ERR_OK;
        }

        /*---------------------------------------------------------------------
        Check for a timeout
        ---------------------------------------------------------------------*/
        size_t timeElapsed = time::millis() - lastWake;
        if( timeElapsed >= waitTimeRemaining )
        {
          return ErrorCode::ERR_TIMEOUT;
        }

        /*---------------------------------------------------------------------
        Received a signal, but it was the wrong one. Keep waiting.
        ---------------------------------------------------------------------*/
        lastWake = time::millis();
        waitTimeRemaining -= timeElapsed;
      }

      return ErrorCode::ERR_OK;
    }


    mb::ErrorCode await( const mb::thread::Event event, mb::osal::mb_smphr_t &notifier, const size_t timeout ) final override
    {
      mbed_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      /*-----------------------------------------------------------------------
      Block on the internal semaphore, then notify the user when appropriate
      -----------------------------------------------------------------------*/
      auto result = await( event, timeout );
      if( result == ErrorCode::ERR_OK )
      {
        osal::releaseSmphr( notifier );
      }

      return result;
    }


    void signalAIO( const mb::thread::Event trigger ) final override
    {
      mbed_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      mAIOEvent = trigger;
      osal::releaseSmphr( mAIOSignal );
    }


    void signalAIOFromISR( const mb::thread::Event trigger ) final override
    {
      mbed_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      mAIOEvent = trigger;
      osal::releaseSmphrFromISR( mAIOSignal );
    }

  protected:
    /**
     * @brief Bit mask of events supported by the inheriting class
     *
     * By default all events are supported, but if the inheriting class
     * wants to limit the events it can support, it can do so by setting
     * the appropriate bits in this mask.
     */
    uint32_t mAIOAllowedEvents;

    /**
     * @brief Initialize the AsyncIO runtime
     *
     * This ensures the semaphore is at a known state that will block on first attempt
     * to wait for an event. Only call this method during the inheriting class init sequence.
     */
    void initAIO()
    {
      if( mAIOInitialized == DRIVER_INITIALIZED_KEY )
      {
        return;
      }

      /*-------------------------------------------------------------------------
      Initialize the synchronization primitives
      -------------------------------------------------------------------------*/
      mbed_assert( ::mb::osal::buildSmphrStrategy( mAIOSignal, 1, 0 ) );
      mbed_assert( ::mb::osal::buildMutexStrategy( mAIOMutex ) );
      mAIOEvent       = mb::thread::EVENT_UNKNOWN;
      mAIOInitialized = DRIVER_INITIALIZED_KEY;

      /*-----------------------------------------------------------------------
      Ensure the semaphore is in a known state. This allows the first waiter
      to block immediately.
      -----------------------------------------------------------------------*/
      mbed_assert( osal::tryAcquireSmphr( mAIOSignal ) );
    }

    /**
     * @brief Resets the AIO signals back to a non-triggered state
     */
    void resetAIO()
    {
      mbed_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      mAIOEvent = mb::thread::EVENT_UNKNOWN;
      osal::tryAcquireSmphr( mAIOSignal );
    }

  private:
    size_t            mAIOInitialized; /**< Indicates if the class is initialized */
    mb::thread::Event mAIOEvent;       /**< Which event was triggered by the class */
    osal::mb_smphr_t  mAIOSignal;      /**< Lightweight semaphore to block on */
    osal::mb_mutex_t  mAIOMutex;       /**< Exclusive lock for waiters */
  };

}    // namespace mb::osal

#endif /* !MBEDUTILS_MUTEX_EXTENSIONS_HPP */
