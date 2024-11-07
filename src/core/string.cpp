/******************************************************************************
 *  File Name:
 *    string.cpp
 *
 *  Description:
 *    String functions for the mbedutils library
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstring>
#include <mbedutils/string.hpp>

namespace mb::string
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void ensure_crlf_termination( char *const buffer, const size_t buffer_size )
  {
    /*-------------------------------------------------------------------------
    Ensure we can't overrun the buffer
    -------------------------------------------------------------------------*/
    buffer[ buffer_size - 1 ] = '\0';
    const size_t len          = strlen( buffer );

    /*-------------------------------------------------------------------------
    Overwrite the last two characters if the buffer is too small to append to.
    -------------------------------------------------------------------------*/
    if( len >= buffer_size - 2 )
    {
      buffer[ buffer_size - 2 ] = '\r';
      buffer[ buffer_size - 1 ] = '\n';
      return;
    }

    /*-------------------------------------------------------------------------
    Check if the buffer is already terminated with CRLF
    -------------------------------------------------------------------------*/
    if( len >= 2 && ( ( buffer[ len - 2 ] == '\r' && buffer[ len - 1 ] == '\n' ) ||
                      ( buffer[ len - 2 ] == '\n' && buffer[ len - 1 ] == '\r' ) ) )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Append the CRLF sequence to the end of the buffer
    -------------------------------------------------------------------------*/
    buffer[ len ]     = '\r';
    buffer[ len + 1 ] = '\n';
  }

}  // namespace mb::string
