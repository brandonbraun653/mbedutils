/******************************************************************************
 *  File Name:
 *    logging_sink_bare_metal.cpp
 *
 *  Description:
 *    Sink driver implementations that are meant to run on some kind of bare
 *    metal target through consumption of the mbedutils interfaces. May still
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
#include <mbedutils/interfaces/uart_intf.hpp>

namespace mbedutils::logging
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
    auto   pmsg    = reinterpret_cast<const uint8_t *const>( message );
    size_t written = static_cast<size_t>( intf::uart::write( mChannel, pmsg, length ) );

    return ( written == length ) ? ErrCode::ERR_OK : ErrCode::ERR_NO_MEM;
  }


  void UARTSink::assignChannel( const size_t channel )
  {
    mbed_assert( intf::is_driver_available( mbedutils::intf::Driver::UART, channel ),
                 "Invalid UART channel assignment");

    mChannel = channel;
  }

}  // namespace mbedutils::logging
