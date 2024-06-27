/******************************************************************************
 *  File Name:
 *    mutex_pico.cpp
 *
 *  Description:
 *    Raspberry Pi Pico specific implementation of the mutex interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/array.h>
#include <mbedutils/config.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <pico/sync.h>

namespace mbedutils::osal
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
  using mt_array = etl::array<mutex_t, MBEDUTILS_OSAL_MUTEX_POOL_SIZE>;
  static mt_array s_mutex_pool;
  static size_t   s_mutex_pool_index = 0;
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  using rmt_array = etl::array<recursive_mutex_t, MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE>;
  static rmt_array s_recursive_mutex_pool;
  static size_t    s_recursive_mutex_pool_index = 0;
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize the mutex pool if it's being used
    -------------------------------------------------------------------------*/
#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
    s_mutex_pool_index = 0;
    for( auto &mutex : s_mutex_pool )
    {
      mutex_init( &mutex );
    }
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

    /*-------------------------------------------------------------------------
    Initialize the recursive mutex pool if it's being used
    -------------------------------------------------------------------------*/
#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    s_recursive_mutex_pool_index = 0;
    for( auto &mutex : s_recursive_mutex_pool )
    {
      recursive_mutex_init( &mutex );
    }
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  }


  void createRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    if( mutex == nullptr )
    {
      mutex = static_cast<mb_recursive_mutex_t>( new recursive_mutex_t() );
    }
  }


  void destroyRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    if( mutex != nullptr )
    {
      delete static_cast<recursive_mutex_t *>( mutex );
      mutex = nullptr;
    }
  }


  void allocateRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    if( s_recursive_mutex_pool_index < MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE )
    {
      mutex = &s_recursive_mutex_pool[ s_recursive_mutex_pool_index++ ];
    }
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  }


  void initializeRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    if( mutex != nullptr )
    {
      recursive_mutex_init( static_cast<recursive_mutex_t *>( mutex ) );
    }
  }


  void lockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    if( mutex != nullptr )
    {
      recursive_mutex_enter_blocking( static_cast<recursive_mutex_t *>( mutex ) );
    }
  }


  bool tryLockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    if( mutex != nullptr )
    {
      return recursive_mutex_try_enter( static_cast<recursive_mutex_t *>( mutex ), nullptr );
    }

    return false;
  }


  bool tryLockForRecursiveMutex( mb_recursive_mutex_t mutex, const uint32_t timeout )
  {
    if( mutex != nullptr )
    {
      return recursive_mutex_enter_timeout_ms( static_cast<recursive_mutex_t *>( mutex ), timeout );
    }

    return false;
  }


  void unlockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    if( mutex != nullptr )
    {
      recursive_mutex_exit( static_cast<recursive_mutex_t *>( mutex ) );
    }
  }
}    // namespace mbedutils::osal
