/******************************************************************************
 *  File Name:
 *    osal.hpp
 *
 *  Description:
 *    Helper file to include all the OS abstraction layer files in one go.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_OSAL_HPP
#define MBEDUTILS_OSAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <mbedutils/interfaces/smphr_intf.hpp>
#include <mbedutils/interfaces/irq_intf.hpp>

namespace mb::osal
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize all OSAL drivers.
   *
   * This must be called before any of the API functions are used.
   */
  void initOSALDrivers();

  /**
   * @brief Enters a critical section, disabling interrupts and taking a mutex.
   *
   * @param lock Mutex to take
   */
  void enterCritical( mb::osal::mb_mutex_t &lock );

  /**
   * @brief Leaves a critical section, enabling interrupts and releasing a mutex.
   *
   * @param lock Mutex to release
   */
  void exitCritical( mb::osal::mb_mutex_t &lock );
}

#endif  /* !MBEDUTILS_OSAL_HPP */
