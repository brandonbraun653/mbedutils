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

  UARTSink::UARTSink() : mChannel( -1 )
  {
  }


  ErrCode UARTSink::open()
  {
    return ErrCode::ERR_OK;
  }


  ErrCode UARTSink::close()
  {
    return ErrCode::ERR_OK;
  }


  ErrCode UARTSink::flush()
  {
    return ErrCode::ERR_OK;
  }


  ErrCode UARTSink::insert( const Level level, const void *const message, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Check to see if we should even write
    -------------------------------------------------------------------------*/
    if ( !enabled || ( level < logLevel ) || !message || !length || ( mChannel < 0 ) )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Write the message to the UART channel
    -------------------------------------------------------------------------*/
    // size_t written = static_cast<size_t>( intf::uart::write( mChannel, message, length ) );

    // TODO BMB: Need to route this through a buffered uart interface.
    mbed_assert_always();
    size_t written = 0;
    return ( written == length ) ? ErrCode::ERR_OK : ErrCode::ERR_NO_MEM;
  }


  void UARTSink::assignChannel( const size_t channel )
  {
    mbed_assert_msg( hw::is_driver_available( hw::Driver::UART, channel ),
                     "Invalid UART channel assignment");

    mChannel = channel;
  }

}  // namespace mb::logging
