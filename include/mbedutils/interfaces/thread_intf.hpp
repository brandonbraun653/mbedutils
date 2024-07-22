/******************************************************************************
 *  File Name:
 *    thread_intf.hpp
 *
 *  Description:
 *    Threading interface for mbedutils implementers.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_INTERFACE_HPP
#define MBEDUTILS_THREADING_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mb::thread::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Total number of physical cores available.
   * @note Expected to be provided by the integrating project.
   *
   * @return constexpr size_t
   */
  extern inline constexpr size_t num_cores();

  /**
   * @brief Max number of threads the system can execute.
   * @note Expected to be provided by the integrating project.
   *
   * @return constexpr size_t
   */
  extern inline constexpr size_t max_tasks();

}  // namespace mb::thread::intf

#endif  /* !MBEDUTILS_THREADING_INTERFACE_HPP */
