/******************************************************************************
 *  File Name:
 *    atexit.hpp
 *
 *  Description:
 *    System level atexit function for handling cleanup
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_ATEXIT_HPP
#define MBEDUTILS_ATEXIT_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/delegate.h>

namespace mb::system::atexit
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Function signature to use for atexit callbacks
   */
  using Callback = etl::delegate<void( void )>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr uint32_t HIGHEST_PRIORITY = 0;
  static constexpr uint32_t HIGH_PRIORITY    = std::numeric_limits<uint32_t>::max() / 4;
  static constexpr uint32_t NORMAL_PRIORITY  = std::numeric_limits<uint32_t>::max() / 2;
  static constexpr uint32_t LOW_PRIORITY     = std::numeric_limits<uint32_t>::max();

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Prepare the atexit module for operation
   */
  void initialize();

  /**
   * @brief Register a function to be called at system exit.
   *
   * Callbacks registered at the same priority level may be called in any order.
   *
   * @param callback Callback function to invoke
   * @param priority Priority of the callback (0 is highest)
   * @return true   The callback was registered
   * @return false  The callback was not registered (likely full)
   */
  bool registerCallback( mb::system::atexit::Callback &callback, uint32_t priority = 0 );

  /**
   * @brief Unregister a function from the atexit list
   *
   * @param callback Callback function to remove
   * @return true   The callback was removed
   * @return false  The callback was not found
   */
  bool unregisterCallback( mb::system::atexit::Callback &callback );

  /**
   * @brief Call all registered atexit functions
   */
  void exit();

}  // namespace mb::system::atexit

#endif  /* !MBEDUTILS_ATEXIT_HPP */
