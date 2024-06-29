/******************************************************************************
 *  File Name:
 *    buffered_uart.cpp
 *
 *  Description:
 *    Implements the BufferedUART class
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/hardware/buffered_uart.hpp>

namespace mbedutils::hw::uart
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  BufferedUART::BufferedUART() : mConfig( {} )
  {
  }


  int BufferedUART::configure( const ::mbedutils::hw::serial::Config &config )
  {
    return 0;
  }

  int BufferedUART::write( const void *const buffer, const size_t length, const size_t timeout )
  {
    return 0;
  }

  int BufferedUART::read( void *const buffer, const size_t length, const size_t timeout )
  {
    return 0;
  }
}  // namespace mbedutils::hw::uart
