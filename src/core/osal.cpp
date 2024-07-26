/******************************************************************************
 *  File Name:
 *    osal.cpp
 *
 *  Description:
 *    OSAL driver initialization and management functions
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/osal.hpp>

namespace mb::osal
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initOSALDrivers()
  {
    initMutexDriver();
    initSmphrDriver();
  }


  void enterCritical( mb::osal::mb_mutex_t &lock )
  {
    if( !mb::irq::in_isr() )
    {
      mb::osal::lockMutex( lock );
    }
    mb::irq::disable_interrupts();
  }


  void exitCritical( mb::osal::mb_mutex_t &lock )
  {
    mb::irq::enable_interrupts();
    if( !mb::irq::in_isr() )
    {
      mb::osal::unlockMutex( lock );
    }
  }
}  // namespace mb::osal
