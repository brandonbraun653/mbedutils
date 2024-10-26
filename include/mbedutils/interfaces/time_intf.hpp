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

namespace mb::time
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

  /**
   * @brief Delays the current thread for a specified number of milliseconds
   *
   * @param val Number of milliseconds to delay
   */
  void delayMilliseconds( const size_t val );

  /**
   * @brief Delays the current thread for a specified number of microseconds
   *
   * @param val Number of microseconds to delay
   */
  void delayMicroseconds( const size_t val );

}  // namespace mb::time

#endif  /* !MBEDUTILS_TIME_HPP */
