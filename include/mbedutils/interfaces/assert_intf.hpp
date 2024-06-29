/******************************************************************************
 *  File Name:
 *    assert_intf.hpp
 *
 *  Description:
 *    Project specific assertion interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_ASSERT_INTF_HPP
#define MBEDUTILS_ASSERT_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/string.h>

namespace mb::assert
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Assertion failure handler
   *
   *
   * This function is the final endpoint for all assertion failures from
   * the mb library. Note that this could be called at any time,
   * including from an ISR. Be sure to keep it as lean as possible.
   *
   * @param halt Should the system halt/restart? This indicates a critical error.
   * @param msg Message of what went wrong. This must by copied as it lives on the stack.
   */
  void on_assert_fail( const bool halt, const etl::string_view &msg );

}  // namespace mb::assert

#endif  /* !MBEDUTILS_ASSERT_INTF_HPP */
