/******************************************************************************
 *  File Name:
 *    buffered_uart.cpp
 *
 *  Description:
 *    Implements the SerialDriver class
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/hardware/serial.hpp>
#include <mbedutils/interfaces/cmn_intf.hpp>
#include <mbedutils/interfaces/serial_intf.hpp>
#include <mbedutils/osal.hpp>
#include <mbedutils/thread.hpp>

namespace mb::hw::serial
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief The maximum amount of time to wait for a read operation to complete.
   */
  static constexpr size_t RX_SCAN_TIMEOUT_MS = 1000;

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  SerialDriver::SerialDriver() : mIsConfigured( false ), mConfig( {} ), mTXControl( {} ), mRXControl( {} )
  {
  }


  SerialDriver::~SerialDriver()
  {
    close();
  }


  bool SerialDriver::open( const ::mb::hw::serial::Config &config )
  {
    /*-------------------------------------------------------------------------
    Validate the input configuration
    -------------------------------------------------------------------------*/
    if( mIsConfigured )
    {
      mbed_assert_continue_msg( false, "UART %d attempted reconfiguration", mConfig.channel );
      return false;
    }

    if( mb::hw::is_driver_available( mb::hw::Driver::UART, config.channel ) == false )
    {
      mbed_assert_continue_msg( false, "UART channel %d is not available", config.channel );
      return -1;
    }

    if( !config.rxBuffer && !config.txBuffer )
    {
      mbed_assert_continue_msg( false, "Buffer pointers are null" );
      return false;
    }

    /*-------------------------------------------------------------------------
    Take ownership of the UART channel
    -------------------------------------------------------------------------*/
    if( !mb::hw::serial::intf::lock( config.channel, 100 ) )
    {
      mbed_assert_continue_msg( false, "Failed to lock UART channel %d", config.channel );
      return false;
    }

    intf::flush( config.channel );

    /*-------------------------------------------------------------------------
    Bind the flow control callbacks for this instance
    -------------------------------------------------------------------------*/
    intf::TXCompleteCallback tx_cb =
        intf::TXCompleteCallback::create<SerialDriver, &SerialDriver::on_tx_complete_callback>( *this );
    intf::RXCompleteCallback rx_cb =
        intf::RXCompleteCallback::create<SerialDriver, &SerialDriver::on_rx_complete_callback>( *this );

    intf::on_tx_complete( config.channel, tx_cb );
    intf::on_rx_complete( config.channel, rx_cb );

    /*-------------------------------------------------------------------------
    Initialize the internal state
    -------------------------------------------------------------------------*/
    /* User specified data */
    mConfig = config;
    if( mConfig.rxBuffer )
    {
      mConfig.rxBuffer->clear();
    }

    if( mConfig.txBuffer )
    {
      mConfig.txBuffer->clear();
    }

    /* Transfer controllers */
    mTXControl.in_progress = false;
    mTXControl.buffer      = {};

    mRXControl.in_progress = false;
    mRXControl.buffer      = {};

    /* CRTP Drivers */
    this->initAIO();
    this->initLockable();

    mIsConfigured = true;

    /*-------------------------------------------------------------------------
    If we've configured an RX buffer, start listening to the hardware
    -------------------------------------------------------------------------*/
    if( mConfig.rxBuffer )
    {
      mRXControl.in_progress = true;
      mRXControl.buffer      = mConfig.rxBuffer->write_reserve_optimal();

      auto start_error =
          intf::read_async( mConfig.channel, mRXControl.buffer.data(), mRXControl.buffer.size(), RX_SCAN_TIMEOUT_MS );
      if( start_error < 0 )
      {
        mbed_assert_continue_msg( false, "Failed to start UART %d RX: %d", mConfig.channel, start_error );
        mRXControl.in_progress = false;
        mRXControl.buffer      = {};
      }
    }

    return true;
  }


  void SerialDriver::close()
  {
    if( !mIsConfigured )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _lock( mLockableMutex );

    intf::flush( mConfig.channel );
    mb::hw::serial::intf::unlock( mConfig.channel );
    mIsConfigured = false;
  }


  int SerialDriver::write( const void *const buffer, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !mIsConfigured || !buffer || !length || !mConfig.txBuffer )
    {
      mbed_assert_continue_always();
      return -1;
    }

    /*-------------------------------------------------------------------------
    Enter critical section to guard against interrupts and other threads
    -------------------------------------------------------------------------*/
    if( !mb::irq::in_isr() )
    {
      this->lock();
      intf::disable_interrupts( mConfig.channel );
    }
    // else we're already inside the ISR, so no need to lock

    /*-------------------------------------------------------------------------
    Push as much data into the queue as possible
    -------------------------------------------------------------------------*/
    size_t write_size = std::min<size_t>( mConfig.txBuffer->available(), length );
    if( write_size == 0 )
    {
      return 0;
    }

    auto write_span = mConfig.txBuffer->write_reserve( write_size );
    memcpy( write_span.data(), buffer, write_size );
    mConfig.txBuffer->write_commit( write_span );

    /*-------------------------------------------------------------------------
    Kick off a write operation if not already in progress. Otherwise the data
    will get transferred when the TX complete callback is triggered.
    -------------------------------------------------------------------------*/
    if( !mTXControl.in_progress && !start_tx_transfer( write_size ) )
    {
      write_size = 0;
    }

    /*-------------------------------------------------------------------------
    Exit critical section and allow the transfer to proceed
    -------------------------------------------------------------------------*/
    if( !mb::irq::in_isr() )
    {
      intf::enable_interrupts( mConfig.channel );
      this->unlock();
    }

    return write_size;
  }


  int SerialDriver::read( void *const buffer, const size_t length, const size_t timeout )
  {
    if( !mIsConfigured || !buffer || !length || !mConfig.rxBuffer )
    {
      mbed_assert_continue_always();
      return -1;
    }

    /*-------------------------------------------------------------------------
    Enter critical section to guard against other threads.
    -------------------------------------------------------------------------*/
    if( !mb::irq::in_isr() )
    {
      this->lock();
      intf::disable_interrupts( mConfig.channel );
    }
    // else we're already inside the ISR, so no need to lock

    /*-------------------------------------------------------------------------
    Either read all data immediately, or wait for it to arrive
    -------------------------------------------------------------------------*/
    size_t read_size = std::min<size_t>( mConfig.rxBuffer->size(), length );
    if( read_size < length )
    {
      if( timeout != 0 && !mb::irq::in_isr() )
      {
        this->await( mb::thread::EVENT_READ_COMPLETE, timeout );
      }

      read_size = std::min<size_t>( mConfig.rxBuffer->size(), length );
    }

    /*-------------------------------------------------------------------------
    Read what we can
    -------------------------------------------------------------------------*/
    if( read_size != 0 )
    {
      auto read_span = mConfig.rxBuffer->read_reserve( read_size );
      memcpy( buffer, read_span.data(), read_size );
      mConfig.rxBuffer->read_commit( read_span );
    }

    /*-------------------------------------------------------------------------
    Exit the critical section
    -------------------------------------------------------------------------*/
    if( !mb::irq::in_isr() )
    {
      intf::enable_interrupts( mConfig.channel );
      this->unlock();
    }

    return read_size;
  }


  size_t SerialDriver::readable()
  {
    if( !mIsConfigured || !mConfig.rxBuffer )
    {
      return 0;
    }

    mb::thread::RecursiveLockGuard _lock( mLockableMutex );
    return mConfig.rxBuffer->size();
  }


  void SerialDriver::flushRX()
  {
    if( !mIsConfigured || !mConfig.rxBuffer || mb::irq::in_isr() )
    {
      mbed_assert_continue_always();
      return;
    }

    mb::thread::RecursiveLockGuard _lock( mLockableMutex );
    mConfig.rxBuffer->clear();
  }


  void SerialDriver::flushTX()
  {
    if( !mIsConfigured || !mConfig.rxBuffer || mb::irq::in_isr() )
    {
      mbed_assert_continue_always();
      return;
    }

    mb::thread::RecursiveLockGuard _lock( mLockableMutex );
    start_tx_transfer();
  }


  void SerialDriver::onWriteComplete( CompletionCallback callback )
  {
    this->registerAIO( mb::thread::EVENT_WRITE_COMPLETE, callback, true );
  }


  void SerialDriver::onReadComplete( CompletionCallback callback )
  {
    this->registerAIO( mb::thread::EVENT_READ_COMPLETE, callback, true );
  }


  /**
   * @brief Starts a new TX transfer if one is not already in progress.
   *
   * If a transfer is already in progress, this function does nothing and the TX complete
   * callback will handle queueing more data as needed.
   *
   * @param num_bytes Hint of how many bytes to transfer.
   * @return size_t Number of actual bytes transferred.
   */
  size_t SerialDriver::start_tx_transfer( const size_t num_bytes )
  {
    int actual_write_size = 0;

    if( !mTXControl.in_progress )
    {
      this->resetAIOSignals();

      mTXControl.in_progress = true;
      mTXControl.buffer      = mConfig.txBuffer->read_reserve( num_bytes );
      actual_write_size      = intf::write_async( mConfig.channel, mTXControl.buffer.data(), mTXControl.buffer.size() );

      /*-----------------------------------------------------------------------
      Due to how the write_async method is supposed to work, we should always
      be able to write the full buffer size. If we can't, something went wrong.
      Leave the data in the buffer so it can be re-transferred.
      -----------------------------------------------------------------------*/
      if( ( actual_write_size < 0 ) || ( static_cast<size_t>( actual_write_size ) != mTXControl.buffer.size() ) )
      {
        mbed_assert_continue_msg( false, "Failed to start TX of %d bytes on UART %d: %d", mTXControl.buffer.size(),
                                  mConfig.channel, actual_write_size );
        mTXControl.in_progress = false;
        mTXControl.buffer      = {};
        actual_write_size      = 0;
      }
    }

    return static_cast<size_t>( actual_write_size );
  }


  /**
   * @brief Handle the completion of a TX operation.
   * @warning Expect this to be called from an ISR context.
   *
   * @param channel   The channel that completed the TX
   * @param num_bytes The number of bytes transmitted
   */
  void SerialDriver::on_tx_complete_callback( const size_t channel, const size_t num_bytes )
  {
    /*-------------------------------------------------------------------------
    Something bad happened, bail out
    -------------------------------------------------------------------------*/
    if( !mIsConfigured || ( channel != mConfig.channel ) )
    {
      mbed_assert_continue_always();
      return;
    }

    /*-------------------------------------------------------------------------
    Transfer more data or finish out the state machine
    -------------------------------------------------------------------------*/
    if( mTXControl.in_progress )
    {
      /*-----------------------------------------------------------------------
      Release the memory back to the buffer
      -----------------------------------------------------------------------*/
      mbed_dbg_assert_msg( mTXControl.buffer.size() == num_bytes, "Serial %d TX size mismatch: %d vs %d", mConfig.channel,
                           mTXControl.buffer.size(), num_bytes );
      mConfig.txBuffer->read_commit( mTXControl.buffer );

      /*-----------------------------------------------------------------------
      If there is more data to send, kick off another transfer
      -----------------------------------------------------------------------*/
      if( mConfig.txBuffer->size() > 0 )
      {
        mTXControl.buffer = mConfig.txBuffer->read_reserve();
        auto write_size   = intf::write_async( mConfig.channel, mTXControl.buffer.data(), mTXControl.buffer.size() );

        if( ( write_size < 0 ) || ( static_cast<size_t>( write_size ) != mTXControl.buffer.size() ) )
        {
          mbed_assert_continue_msg( false, "Lost %d bytes on UART %d", mTXControl.buffer.size(), mConfig.channel );
          mConfig.txBuffer->read_commit( mTXControl.buffer );
          mTXControl.in_progress = false;
          mTXControl.buffer      = {};
        }
      }
      else
      {
        mTXControl.buffer      = {};
        mTXControl.in_progress = false;
      }

      /*-----------------------------------------------------------------------
      Signal the user we've dumped some data and more room may be available.
      -----------------------------------------------------------------------*/
      this->signalAIOFromISR( mb::thread::EVENT_WRITE_COMPLETE );

      if( mConfig.txBuffer->size() == 0 )
      {
        this->signalAIOFromISR( mb::thread::EVENT_WRITE_BUFFER_EMPTY );
      }
    }
  }


  /**
   * @brief Handle the completion or timeout of an RX operation.
   * @warning Expect this to be called from an ISR context.
   *
   * @param channel   The channel that completed the RX
   * @param num_bytes The number of bytes received
   */
  void SerialDriver::on_rx_complete_callback( const size_t channel, const size_t num_bytes )
  {
    /*-------------------------------------------------------------------------
    Something bad happened, bail out
    -------------------------------------------------------------------------*/
    if( !mIsConfigured || ( channel != mConfig.channel ) )
    {
      mbed_assert_continue_always();
      return;
    }

    if( !mRXControl.in_progress )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Commit the data that was received, then reserve space for more RX-ing.
    -------------------------------------------------------------------------*/
    if( num_bytes > 0 )
    {
      etl::span<uint8_t> actual_data = mRXControl.buffer.subspan( 0, num_bytes );
      mConfig.rxBuffer->write_commit( actual_data );

      mRXControl.buffer = mConfig.rxBuffer->write_reserve_optimal();
    }

    /*-------------------------------------------------------------------------
    Restart the RX operation
    -------------------------------------------------------------------------*/
    auto start_error =
        intf::read_async( mConfig.channel, mRXControl.buffer.data(), mRXControl.buffer.size(), RX_SCAN_TIMEOUT_MS );
    if( start_error < 0 )
    {
      mbed_assert_continue_msg( false, "Failed to start UART %d RX: %d", mConfig.channel, start_error );
      mRXControl.in_progress = false;
      mRXControl.buffer      = {};
    }

    /*-------------------------------------------------------------------------
    Signal the user that we've received some data
    -------------------------------------------------------------------------*/
    if( num_bytes > 0 )
    {
      this->signalAIOFromISR( mb::thread::EVENT_READ_COMPLETE );
    }
  }
}    // namespace mb::hw::serial
