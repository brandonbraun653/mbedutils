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

namespace mb::hw::serial
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  SerialDriver::SerialDriver() : mIsConfigured( false ), mConfig( {} ), mTXControl( {} ), mRXControl( {} )
  {
  }

  SerialDriver::~SerialDriver()
  {
    if( mIsConfigured )
    {
      mb::intf::serial::unlock( mConfig.channel );
    }
  }


  int SerialDriver::configure( const ::mb::hw::serial::Config &config )
  {
    /*-------------------------------------------------------------------------
    Validate the input configuration
    -------------------------------------------------------------------------*/
    if( mIsConfigured )
    {
      mbed_assert_continue_msg( false, "UART %d attempted reconfiguration", mConfig.channel );
      return -1;
    }

    if( mb::hw::is_driver_available( mb::hw::Driver::UART, config.channel ) == false )
    {
      mbed_assert_continue_msg( false, "UART channel %d is not available", config.channel );
      return -1;
    }

    if( !mConfig.rxBuffer && !mConfig.txBuffer )
    {
      mbed_assert_continue_msg( false, "Buffer pointers are null" );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Take ownership of the UART channel
    -------------------------------------------------------------------------*/
    if( !mb::intf::serial::lock( config.channel, 100 ) )
    {
      mbed_assert_continue_msg( false, "Failed to lock UART channel %d", config.channel );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Initialize the internal state
    -------------------------------------------------------------------------*/
    mConfig = config;
    if( mConfig.rxBuffer )
    {
      mConfig.rxBuffer->clear();
    }

    if( mConfig.txBuffer )
    {
      mConfig.txBuffer->clear();
    }

    mTXControl.state     = State::READY;
    mTXControl.remaining = 0;
    mTXControl.expected  = 0;
    mTXControl.buffer    = {};

    mRXControl.state     = State::READY;
    mRXControl.remaining = 0;
    mRXControl.expected  = 0;
    mRXControl.buffer    = {};

    mIsConfigured = true;
    return 0;
  }

  int SerialDriver::write( const void *const buffer, const size_t length, const size_t timeout )
  {
    return 0;
  }

  int SerialDriver::read( void *const buffer, const size_t length, const size_t timeout )
  {
    return 0;
  }
}    // namespace mb::hw::serial
