/******************************************************************************
 *  File Name:
 *    assert.cpp
 *
 *  Description:
 *    Assertion driver implementation for the mbedutils library
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdarg>
#include <etl/string.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/config.hpp>
#include <mbedutils/drivers/nanoprintf.hpp>
#include <mbedutils/interfaces/assert_intf.hpp>
#include <mbedutils/interfaces/irq_intf.hpp>
#include <mbedutils/interfaces/time_intf.hpp>

namespace mbedutils::assert
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static volatile bool   s_recursion_guard;
  static volatile size_t s_recurse_event;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    s_recursion_guard = false;
    s_recurse_event   = 0;
  }


  size_t num_recurse_events()
  {
    return s_recurse_event;
  }


  void format_and_log_assert_failure( const bool predicate, const bool halt, const char *file, const int line, const char *fmt,
                                      ... )
  {
    /*-------------------------------------------------------------------------
    Check the predicate! No work to do if the assertion held.
    -------------------------------------------------------------------------*/
    if( predicate )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Prevent recursion from ruining our day
    -------------------------------------------------------------------------*/
    irq::disable_interrupts();
    if( s_recursion_guard )
    {
      s_recurse_event++;
      irq::enable_interrupts();
      return;
    }

    s_recursion_guard = true;
    irq::enable_interrupts();

    /*-------------------------------------------------------------------------
    Format the prefix of the message. This includes the time, file, and line.
    -------------------------------------------------------------------------*/
    etl::string<MBEDUTILS_ASSERT_FMT_BUFFER_SIZE> fmt_buffer;

    const int bytes_written =
        npf_snprintf( fmt_buffer.data(), fmt_buffer.max_size(), "%u | %s:%u | ", time::millis(), file, line );

    /*-------------------------------------------------------------------------
    Format the user message
    -------------------------------------------------------------------------*/
    if( ( 0 < bytes_written ) && ( static_cast<size_t>( bytes_written ) < fmt_buffer.max_size() ) )
    {
      va_list args;
      va_start( args, fmt );
      npf_vsnprintf( fmt_buffer.data(), fmt_buffer.size(), fmt, args );
      va_end( args );
    }

    /*-------------------------------------------------------------------------
    Ensure the string is always null terminated. This technically is done in
    the npf_vsnprintf function, but should that change for some reason...
    -------------------------------------------------------------------------*/
    fmt_buffer.back() = '\0';

    /*-------------------------------------------------------------------------
    Call the interface defined function to handle the failure
    -------------------------------------------------------------------------*/
    on_assert_fail( halt, fmt_buffer );

    /*-------------------------------------------------------------------------
    Unlock now that we're past all points that could throw another assertion
    -------------------------------------------------------------------------*/
    s_recursion_guard = false;
  }
}    // namespace mbedutils::assert
