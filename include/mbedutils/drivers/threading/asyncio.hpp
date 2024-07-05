/******************************************************************************
 *  File Name:
 *    asyncio.hpp
 *
 *  Description:
 *    Asynchronous IO interface for handling non-blocking IO operations.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_ASYNCIO_HPP
#define MBEDUTILS_THREADING_ASYNCIO_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>
#include <etl/delegate.h>
#include <etl/vector.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/threading/event.hpp>
#include <mbedutils/interfaces/irq_intf.hpp>
#include <mbedutils/interfaces/time_intf.hpp>
#include <mbedutils/interfaces/util_intf.hpp>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using AIOCallback = etl::delegate<void()>;

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
     * Waits for the given event to occur before the function will return. This
     * is accomplished by blocking the current thread until the event is signaled.
     *
     * @param event   The event upon which to be triggered
     * @param timeout How long to wait for the event to occur
     * @return mb::ErrorCode
     */
    virtual mb::ErrorCode await( const mb::thread::Event event, const size_t timeout ) = 0;

    /**
     * @brief Block on a specific signal and await an event, or timeout.
     *
     * A more explicit version of await that allows selecting the threading
     * primitive on which to block.
     *
     * @param event     The event upon which to be triggered
     * @param notifier  Semaphore to be given to upon the event occurrence
     * @param timeout   How long to wait for the event to occur
     * @return mb::ErrorCode
     */
    virtual mb::ErrorCode await( const mb::thread::Event event, mb::osal::mb_smphr_t &notifier, const size_t timeout ) = 0;

    /**
     * @brief Signal that a particular event has occurred
     *
     * @param event Which event occurred
     */
    virtual void signalAIO( const mb::thread::Event event ) = 0;

    /**
     * @brief Signal that a particular event has occurred, but from an ISR safe context
     *
     * @param event Which event occurred
     */
    virtual void signalAIOFromISR( const mb::thread::Event event ) = 0;

    /**
     * @brief Register a callback to be invoked when an event occurs.
     *
     * This is useful when you want an action to happen for that event, but don't want to
     * block another thread waiting for the event signaling.
     *
     * @param event    Event to register the callback for
     * @param callback Callback to invoke
     * @param from_isr True if the callback should be invoked from an ISR context, false from user context
     */
    virtual void registerAIO( const mb::thread::Event event, const AIOCallback &callback, const bool from_isr ) = 0;
  };

  /**
   * @brief CRTP interface to add SPSC AsyncIO notification functionality to a class
   * @tparam T      Base class being extended
   * @tparam CBSize Number of callbacks to support
   */
  template<class T, const size_t CBSize = 1>
  class AsyncIO : public virtual AsyncIOInterface
  {
  private:
    struct CallbackData
    {
      bool              isr_ctx;  /**< If true, safe to call from an ISR context */
      mb::thread::Event event;    /**< Event bound to the callback */
      AIOCallback       callback; /**< Callback to invoke */
    };

    size_t                            mAIOInitialized; /**< Indicates if the class is initialized */
    mb::thread::Event                 mAIOEvent;       /**< Which event was triggered by the class */
    osal::mb_smphr_t                  mAIOSignal;      /**< Lightweight semaphore to block on */
    osal::mb_mutex_t                  mAIOMutex;       /**< Exclusive lock for waiters */
    etl::vector<CallbackData, CBSize> mAIOCallbacks;   /**< Registered callbacks */

    /**
     * @brief Invokes a registered callback for an event
     *
     * @param event    Which event occurred
     * @param from_isr True if calling from an ISR, false otherwise
     */
    void invoke_aio_callback( const mb::thread::Event event, const bool from_isr )
    {
      for( auto &cb : mAIOCallbacks )
      {
        if( ( cb.event == event ) && ( cb.isr_ctx == from_isr ) && cb.callback )
        {
          cb.callback();
          break;
        }
      }
    }

  public:
    AsyncIO() :
        mAIOInitialized( ~DRIVER_INITIALIZED_KEY ), mAIOEvent( mb::thread::EVENT_UNKNOWN ), mAIOSignal( nullptr ),
        mAIOMutex( nullptr ), mAIOCallbacks( {} ), mAIOAllowedEvents( 0xFFFFFFFF )
    {
    }

    ~AsyncIO()
    {
      if( mAIOInitialized == DRIVER_INITIALIZED_KEY )
      {
        osal::destroySmphrStrategy( mAIOSignal );
        osal::destroyMutexStrategy( mAIOMutex );
      }
    }


    mb::ErrorCode await( const mb::thread::Event event, const size_t timeout ) final override
    {
      mbed_dbg_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );
      mbed_dbg_assert( !mb::irq::in_isr() );

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
      ErrorCode result            = ErrorCode::ERR_TIMEOUT;
      size_t    startTime         = time::millis();
      size_t    lastWake          = startTime;
      size_t    waitTimeRemaining = timeout;

      while( ( time::millis() - startTime ) < timeout )
      {
        osal::tryAcquireSmphr( mAIOSignal, waitTimeRemaining );

        /*---------------------------------------------------------------------
        Expected event (or error) signaled?
        ---------------------------------------------------------------------*/
        if( mAIOEvent == mb::thread::EVENT_SYSTEM_ERROR )
        {
          result = ErrorCode::ERR_FAIL;
          break;
        }
        else if( mAIOEvent == event )
        {
          result = ErrorCode::ERR_OK;
          invoke_aio_callback( event, false );
          break;
        }

        /*---------------------------------------------------------------------
        Check for a timeout
        ---------------------------------------------------------------------*/
        size_t timeElapsed = time::millis() - lastWake;
        if( timeElapsed >= waitTimeRemaining )
        {
          result = ErrorCode::ERR_TIMEOUT;
          break;
        }

        /*---------------------------------------------------------------------
        Received a signal, but it was the wrong one. Keep waiting.
        ---------------------------------------------------------------------*/
        lastWake = time::millis();
        waitTimeRemaining -= timeElapsed;
      }

      return result;
    }


    mb::ErrorCode await( const mb::thread::Event event, mb::osal::mb_smphr_t &notifier, const size_t timeout ) final override
    {
      mbed_dbg_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

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


    void signalAIO( const mb::thread::Event event ) final override
    {
      mbed_dbg_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      mAIOEvent = event;
      osal::releaseSmphr( mAIOSignal );
    }


    void signalAIOFromISR( const mb::thread::Event event ) final override
    {
      mbed_dbg_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      mAIOEvent = event;
      osal::releaseSmphrFromISR( mAIOSignal );
      invoke_aio_callback( event, mb::irq::in_isr() );
    }


    void registerAIO( const mb::thread::Event event, const AIOCallback &callback, const bool from_isr ) final override
    {
      mbed_dbg_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      LockGuard _lck( mAIOMutex );

      /*-----------------------------------------------------------------------
      Search for a replacement location first
      -----------------------------------------------------------------------*/
      for( auto &cb : mAIOCallbacks )
      {
        if( cb.event == event )
        {
          cb.callback = callback;
          cb.isr_ctx  = from_isr;
          return;
        }
      }

      /*-----------------------------------------------------------------------
      Nothing found. Add a new element if we can.
      -----------------------------------------------------------------------*/
      if( !mAIOCallbacks.full() )
      {
        mAIOCallbacks.push_back( { from_isr, event, callback } );
      }
      else
      {
        mbed_dbg_assert_continue_always();
      }
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
      mbed_assert( ::mb::osal::buildSmphrStrategy( mAIOSignal, 1, 1 ) );
      mbed_assert( ::mb::osal::buildMutexStrategy( mAIOMutex ) );

      /*-----------------------------------------------------------------------
      Initialize member data
      -----------------------------------------------------------------------*/
      mAIOEvent       = mb::thread::EVENT_UNKNOWN;
      mAIOInitialized = DRIVER_INITIALIZED_KEY;
      mAIOCallbacks.clear();

      /*-----------------------------------------------------------------------
      Ensure the semaphore is in a known state. This allows the first waiter
      to block immediately.
      -----------------------------------------------------------------------*/
      mbed_assert( osal::tryAcquireSmphr( mAIOSignal ) );
    }

    /**
     * @brief Resets the AIO signals back to a non-triggered state
     */
    void resetAIOSignals()
    {
      mbed_assert( mAIOInitialized == DRIVER_INITIALIZED_KEY );

      mAIOEvent = mb::thread::EVENT_UNKNOWN;
      osal::tryAcquireSmphr( mAIOSignal );
    }
  };
}  // namespace mb::osal

#endif  /* !MBEDUTILS_THREADING_ASYNCIO_HPP */
