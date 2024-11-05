/******************************************************************************
 *  File Name:
 *    context.hpp
 *
 *  Description:
 *    Mbedutils execution context interface for the system
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_SYSTEM_CONTEXT_HPP
#define MBEDUTILS_SYSTEM_CONTEXT_HPP

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

#if defined( MBEDUTILS_SIMULATOR )
  #define MB_EXEC_SIMULATOR ( true )
  #define MB_EXEC_EMBEDDED  ( false )
#else
  #define MB_EXEC_SIMULATOR ( false )
  #define MB_EXEC_EMBEDDED  ( true )
#endif /* MBEDUTILS_SIMULATOR */

namespace mb::system
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Checks if running on a target embedded platform
   *
   * @return bool
   */
  bool isEmbeddedConext();

  /**
   * @brief Checks if running on the host machine.
   *
   * @return bool
   */
  bool isSimulatorContext();

  /**
   * @brief Checks if the caller is in an ISR
   *
   * @return bool
   */
  bool isISRContext();

}    // namespace mb::system

#endif /* !MBEDUTILS_SYSTEM_CONTEXT_HPP */
