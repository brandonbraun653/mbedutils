/******************************************************************************
 *  File Name:
 *    logging_sink_bare_metal.cpp
 *
 *  Description:
 *    Sink driver implementations that are meant to run on some kind of bare
 *    metal target through consumption of the mb interfaces. May still
 *    work in a hosted environment, but that is not the primary target.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/assert.hpp>
#include <mbedutils/logging.hpp>
#include <mbedutils/interfaces/cmn_intf.hpp>
#include <mbedutils/interfaces/serial_intf.hpp>

namespace mb::logging
{
  /*---------------------------------------------------------------------------
  UART Sink
  ---------------------------------------------------------------------------*/

  SerialSink::SerialSink() : mSerial( nullptr )
  {
  }


  ErrCode SerialSink::open()
  {
    this->initLockable();
    return ErrCode::ERR_OK;
  }


  ErrCode SerialSink::close()
  {
    return ErrCode::ERR_OK;
  }


  ErrCode SerialSink::flush()
  {
    mSerial->flushRX();
    mSerial->flushTX();
    return ErrCode::ERR_OK;
  }


  ErrCode SerialSink::write( const Level level, const void *const message, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Check to see if we should even write
    -------------------------------------------------------------------------*/
    if( !enabled || ( level < logLevel ) || !message || !length )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Write the message to the UART channel
    -------------------------------------------------------------------------*/
    if( static_cast<size_t>( mSerial->write( message, length ) ) == length )
    {
      return ErrCode::ERR_OK;
    }
    else
    {
      return ErrCode::ERR_FAIL;
    }
  }


  void SerialSink::assignDriver( ::mb::hw::serial::ISerial &serial )
  {
    mSerial = &serial;
  }

}    // namespace mb::logging
