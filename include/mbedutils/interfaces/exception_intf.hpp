/******************************************************************************
 *  File Name:
 *    exception_intf.hpp
 *
 *  Description:
 *    Mbedutils exception handling interfaces
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef _HPP
#define _HPP

namespace mb::hw::exception::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initializes the hardware exception handling interfaces
   *
   * May or may not be useful, but it provides a nice hook to configure any
   * hardware specific exception handling routines.
   */
  void driver_setup();

}  // namespace mb::hw::exception::intf

#endif  /* !_HPP */