/******************************************************************************
 *  File Name:
 *    system_intf.hpp
 *
 *  Description:
 *    Mbedutils interface for system level functionality
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_SYSTEM_INTF_HPP
#define MBEDUTILS_SYSTEM_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mb::system::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Get the number of times the system has booted
   * @return size_t
   */
  size_t get_boot_count();

}    // namespace mb::system::intf

#endif /* !MBEDUTILS_SYSTEM_INTF_HPP */
