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
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Standard condition variable implementation.
   */
  class CondVar
  {
  private:
    volatile uint32_t    waiters;
    mb::osal::mb_mutex_t mtx;
    mb::osal::mb_smphr_t sem;

  public:
    CondVar() : waiters( 0 ), mtx( nullptr ), sem( nullptr )
    {
    }

    /**
     * @brief Acquire condition variable resources.
     */
    void init()
    {
      mbed_assert( mb::osal::buildMutexStrategy( mtx ) );
      mbed_assert( mb::osal::buildSmphrStrategy( sem, 1, 1 ) );
      waiters = 0;
    }

    /**
     * @brief Wait for the given predicate to occur
     *
     * @param external_mutex  Mutex to synchronize with
     * @param predicate       Condition to look for
     */
    void wait( mb::osal::mb_mutex_t &external_mutex, const etl::delegate<bool( void )> &predicate )
    {
      mbed_dbg_assert( external_mutex != nullptr );
      mbed_dbg_assert( mtx != nullptr );
      mbed_dbg_assert( sem != nullptr );

      while( !predicate() )
      {
        mb::osal::lockMutex( mtx );
        waiters = waiters + 1;
        mb::osal::unlockMutex( mtx );

        mb::osal::unlockMutex( external_mutex );
        mb::osal::acquireSmphr( sem ); // Block until signaled
        mb::osal::lockMutex( external_mutex );

        mb::osal::lockMutex( mtx );
        waiters = waiters - 1;
        mb::osal::unlockMutex( mtx );
      }
    }

    void notify()
    {
      mbed_dbg_assert( mtx != nullptr );
      mbed_dbg_assert( sem != nullptr );

      mb::osal::lockMutex( mtx );
      if( waiters > 0 )
      {
        mb::osal::releaseSmphr( sem );
      }
      mb::osal::unlockMutex( mtx );
    }

    void notifyAll()
    {
      mbed_dbg_assert( mtx != nullptr );
      mbed_dbg_assert( sem != nullptr );

      mb::osal::lockMutex( mtx );
      for( uint32_t i = 0; i < waiters; ++i )
      {
        mb::osal::releaseSmphr( sem );
      }
      mb::osal::unlockMutex( mtx );
    }
  };


  /**
   * @brief Condition variable class with integrated shared mutex.
   */
  class CondVarMtx
  {
  private:
    volatile uint32_t waiters;
    mb::osal::mb_mutex_t mtx;
    mb::osal::mb_smphr_t sem;
    mb::osal::mb_mutex_t shared_mutex;

  public:
    CondVarMtx() : waiters( 0 ), mtx( nullptr ), sem( nullptr ), shared_mutex( nullptr )
    {
    }

    /**
     * @brief Acquire condition variable resources.
     */
    void init()
    {
      mbed_assert( mb::osal::buildMutexStrategy( shared_mutex ) );
      mbed_assert( mb::osal::buildMutexStrategy( mtx ) );
      mbed_assert( mb::osal::buildSmphrStrategy( sem, 1, 1 ) );
      waiters = 0;
    }

    void wait( const std::function<bool()> &predicate )
    {
      mbed_dbg_assert( mtx != nullptr );
      mbed_dbg_assert( sem != nullptr );
      mbed_dbg_assert( shared_mutex != nullptr );

      while( !predicate() )
      {
        mb::osal::lockMutex( mtx );
        waiters = waiters + 1;
        mb::osal::unlockMutex( mtx );

        mb::osal::unlockMutex( shared_mutex );
        mb::osal::acquireSmphr( sem );    // Block until signaled
        mb::osal::lockMutex( shared_mutex );

        mb::osal::lockMutex( mtx );
        waiters = waiters - 1;
        mb::osal::unlockMutex( mtx );
      }
    }

    void notify()
    {
      mbed_dbg_assert( mtx != nullptr );
      mbed_dbg_assert( sem != nullptr );

      mb::osal::lockMutex( mtx );
      if( waiters > 0 )
      {
        mb::osal::releaseSmphr( sem );
      }
      mb::osal::unlockMutex( mtx );
    }

    void notifyAll()
    {
      mbed_dbg_assert( mtx != nullptr );
      mbed_dbg_assert( sem != nullptr );

      mb::osal::lockMutex( mtx );
      for( uint32_t i = 0; i < waiters; ++i )
      {
        mb::osal::releaseSmphr( sem );
      }
      mb::osal::unlockMutex( mtx );
    }

    void lock()
    {
      mb::osal::lockMutex( shared_mutex );
    }

    void unlock()
    {
      mb::osal::unlockMutex( shared_mutex );
    }

    // Helper method to perform an action while holding the lock
    template<typename Func>
    void withLock( Func action )
    {
      lock();
      action();
      unlock();
    }
  };
}    // namespace mb::thread

#endif /* !MBEDUTILS_CONDITION_VARIABLE_HPP */
