/******************************************************************************
 *  File Name:
 *    context.cpp
 *
 *  Description:
 *    Mbedutils application context interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/system.hpp>
#include <mbedutils/interfaces/irq_intf.hpp>


namespace mb::system
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  bool isEmbeddedConext()
  {
    return MB_EXEC_EMBEDDED;
  }


  bool isSimulatorContext()
  {
    return MB_EXEC_SIMULATOR;
  }


  bool isISRContext()
  {
    return mb::irq::in_isr();
  }

}  // namespace mb::system
