/******************************************************************************
 *  File Name:
 *    system_intf.cpp
 *
 *  Description:
 *    Stubs for default implementations of the system interface functions
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/interfaces/system_intf.hpp>

namespace mb::system::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  __attribute__( ( weak ) ) size_t get_boot_count()
  {
    /*-------------------------------------------------------------------------
    Depends on a project that has some kind of persistent storage. Without
    a common mechanism, the best we can surmise is that the system has booted
    at least once.
    -------------------------------------------------------------------------*/
    return 1;
  }
}    // namespace mb::system::intf
