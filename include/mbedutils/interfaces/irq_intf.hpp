/******************************************************************************
 *  File Name:
 *    irq_intf.hpp
 *
 *  Description:
 *    Standard interface for interrupt management
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_IRQ_INTF_HPP
#define MBEDUTILS_IRQ_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace mbedutils::irq
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Checks to see if the current execution context is an ISR
   *
   * @return bool True if in an ISR, false otherwise
   */
  bool in_isr();

  /**
   * @brief Disables system wide interrupts
   */
  void disable_interrupts();

  /**
   * @brief Enables system wide interrupts
   */
  void enable_interrupts();

}  // namespace mbedutils::irq

#endif  /* !MBEDUTILS_IRQ_INTF_HPP */
