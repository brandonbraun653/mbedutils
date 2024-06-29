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
}  // namespace mb::osal
