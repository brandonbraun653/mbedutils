/******************************************************************************
 *  File Name:
 *    util.cpp
 *
 *  Description:
 *    Mbedutils utility functions
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/

#include <mbedutils/util.hpp>
#include <mbedutils/system.hpp>

#if defined( MB_EXEC_SIMULATOR ) && MB_EXEC_SIMULATOR == true
#include <csignal>
#include <sys/ptrace.h>
#endif  /* MB_EXEC_SIMULATOR */

namespace mb::util
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void breakpoint()
  {
#if defined( MB_EXEC_EMBEDDED ) && MB_EXEC_EMBEDDED == true
    __asm volatile( "bkpt #0" );
#elif defined( MB_EXEC_SIMULATOR ) && MB_EXEC_SIMULATOR == true
    if( ptrace( PTRACE_TRACEME, 0, nullptr, 0 ) == -1 )
    {
      std::raise( SIGTRAP );
    }
#endif
  }
}  // namespace mb
