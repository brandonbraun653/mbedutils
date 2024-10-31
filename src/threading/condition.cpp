/******************************************************************************
 *  File Name:
 *    condition.cpp
 *
 *  Description:
 *    Condition variable implementation
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "mbedutils/interfaces/util_intf.hpp"
#include <etl/algorithm.h>
#include <mbedutils/threading.hpp>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  ConditionVariable Class
  ---------------------------------------------------------------------------*/

  ConditionVariable::ConditionVariable() : _cv_waiters( 0 ), _cv_mtx( nullptr ), _cv_smphr_signal( nullptr ), _cv_initialized( ~DRIVER_INITIALIZED_KEY )
  {
  }


  void ConditionVariable::init()
  {
    if( _cv_initialized == DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Ensure the resources are allocated. Due to the nature of the mbed OSAL
    primitives, we can't perform RAII style initialization. This is the best
    we can do.
    -------------------------------------------------------------------------*/
    mbed_assert( mb::osal::buildMutexStrategy( _cv_mtx ) );
    mbed_assert( mb::osal::buildSmphrStrategy( _cv_smphr_signal, 1, 0 ) );
    _cv_waiters     = 0;
    _cv_initialized = DRIVER_INITIALIZED_KEY;
  }


  void ConditionVariable::deinit()
  {
    if( _cv_initialized != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Ensure the resources are deallocated.
    -------------------------------------------------------------------------*/
    mb::osal::destroyMutex( _cv_mtx );
    mb::osal::destroySmphr( _cv_smphr_signal );
    _cv_waiters     = 0;
    _cv_initialized = ~DRIVER_INITIALIZED_KEY;
  }


  void ConditionVariable::wait( mb::osal::mb_mutex_t &mtx )
  {
    mbed_dbg_assert( mtx != nullptr );
    mbed_dbg_assert( _cv_mtx != nullptr );
    mbed_dbg_assert( _cv_smphr_signal != nullptr );

    increment_waiters();

    mb::osal::unlockMutex( mtx );
    mb::osal::acquireSmphr( _cv_smphr_signal );    // Block until signaled
    mb::osal::lockMutex( mtx );

    decrement_waiters();
  }


  void ConditionVariable::wait( mb::osal::mb_mutex_t &mtx, const cv_predicate &pred )
  {
    mbed_dbg_assert( mtx != nullptr );
    mbed_dbg_assert( _cv_mtx != nullptr );
    mbed_dbg_assert( _cv_smphr_signal != nullptr );

    increment_waiters();

    while( !pred() )
    {
      mb::osal::unlockMutex( mtx );
      mb::osal::acquireSmphr( _cv_smphr_signal );    // Block until signaled
      mb::osal::lockMutex( mtx );
    }

    decrement_waiters();
  }


  cv_status ConditionVariable::wait_for( mb::osal::mb_mutex_t &mtx, const size_t timeout )
  {
    mbed_dbg_assert( mtx != nullptr );
    mbed_dbg_assert( _cv_mtx != nullptr );
    mbed_dbg_assert( _cv_smphr_signal != nullptr );

    increment_waiters();

    mb::osal::unlockMutex( mtx );
    bool timed_out = !mb::osal::tryAcquireSmphr( _cv_smphr_signal, timeout );    // Block until signaled
    mb::osal::lockMutex( mtx );

    decrement_waiters();

    return timed_out ? cv_status::timeout : cv_status::no_timeout;
  }


  bool ConditionVariable::wait_for( mb::osal::mb_mutex_t &mtx, const size_t timeout, const cv_predicate &pred )
  {
    mbed_dbg_assert( mtx != nullptr );
    mbed_dbg_assert( _cv_mtx != nullptr );
    mbed_dbg_assert( _cv_smphr_signal != nullptr );

    increment_waiters();

    bool last_predicate_result = false;
    while( !last_predicate_result )
    {
      last_predicate_result = pred();
      if( last_predicate_result )
      {
        break;
      }

      mb::osal::unlockMutex( mtx );
      bool timed_out = !mb::osal::tryAcquireSmphr( _cv_smphr_signal, timeout );    // Block until signaled
      mb::osal::lockMutex( mtx );

      if( timed_out )
      {
        decrement_waiters();
        return last_predicate_result;
      }
    }

    decrement_waiters();
    return last_predicate_result;
  }


  void ConditionVariable::notify_one()
  {
    mbed_dbg_assert( _cv_mtx != nullptr );
    mbed_dbg_assert( _cv_smphr_signal != nullptr );

    mb::osal::lockMutex( _cv_mtx );
    if( _cv_waiters > 0 )
    {
      mb::osal::releaseSmphr( _cv_smphr_signal );
    }
    mb::osal::unlockMutex( _cv_mtx );
  }


  void ConditionVariable::notify_all()
  {
    mbed_dbg_assert( _cv_mtx != nullptr );
    mbed_dbg_assert( _cv_smphr_signal != nullptr );

    mb::osal::lockMutex( _cv_mtx );
    for( uint32_t i = 0; i < _cv_waiters; ++i )
    {
      mb::osal::releaseSmphr( _cv_smphr_signal );
    }
    mb::osal::unlockMutex( _cv_mtx );
  }


  void ConditionVariable::increment_waiters()
  {
    mb::osal::lockMutex( _cv_mtx );
    _cv_waiters = _cv_waiters + 1;
    mb::osal::unlockMutex( _cv_mtx );
  }


  void ConditionVariable::decrement_waiters()
  {
    mb::osal::lockMutex( _cv_mtx );
    _cv_waiters = etl::max( _cv_waiters - 1, 0 ); // Ensure we don't go negative
    mb::osal::unlockMutex( _cv_mtx );
  }

}    // namespace mb::thread
