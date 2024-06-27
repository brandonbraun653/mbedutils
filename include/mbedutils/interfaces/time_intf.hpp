/******************************************************************************
 *  File Name:
 *    time_intf.hpp
 *
 *  Description:
 *    Interface for system time functions
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_TIME_HPP
#define MBEDUTILS_TIME_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mbedutils::time
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Gets the current system time in milliseconds
   *
   * @return size_t
   */
  size_t millis();

  /**
   * @brief Gets the current system time in microseconds
   *
   * @return size_t
   */
  size_t micros();

}  // namespace mbedutils::time

#endif  /* !MBEDUTILS_TIME_HPP */
