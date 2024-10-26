/******************************************************************************
 *  File Name:
 *    cmn_intf.hpp
 *
 *  Description:
 *    Common interface functions that are shared across multiple modules
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_CMN_INTF_HPP
#define MBEDUTILS_CMN_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mb::hw
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Available interface drivers that are supported.
   *
   * Typically these are hardware peripherals like UART, SPI, etc. But it's
   * not a strict requirement.
   */
  enum class Driver : size_t
  {
    UART,

    NUM_OPTIONS
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Returns the maximum number of drivers available for a given type.
   *
   * For example, if your system has UART 0, 6, and 7 available for use, this
   * function should return 3.
   *
   * @param driver Which driver interface to query
   * @return size_t
   */
  size_t max_drivers( const mb::hw::Driver driver );

  /**
   * @brief Returns the highest index available for a specific driver.
   *
   * For example, if your system has UART 0, 6, and 7 available for use, this
   * function should return 7.
   *
   * @param driver Which driver interface to query
   * @return size_t
   */
  size_t max_driver_index( const mb::hw::Driver driver );

  /**
   * @brief Checks to see if a specific driver instance exists.
   *
   * @param driver Which driver interface to query
   * @param channel Which channel to check for availability
   * @return bool True if the driver is available, false otherwise
   */
  bool is_driver_available( const mb::hw::Driver driver, const size_t channel );

}  // namespace mb::hw

#endif  /* !MBEDUTILS_CMN_INTF_HPP */
