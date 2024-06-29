/******************************************************************************
 *  File Name:
 *    logging_types.hpp
 *
 *  Description:
 *    Type declarations for the logging interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_LOGGING_TYPES_HPP
#define MBEDUTILS_LOGGING_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/array.h>

namespace mb::logging
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/

  class SinkInterface;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using SinkHandle_rPtr = SinkInterface *;


  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  namespace console
  {
    /**
     * @brief Standard ANSI escape sequence to clear the screen
     */
    static constexpr etl::array<uint8_t, 4> CmdClearScreen = { 0x1B, 0x5B, 0x32, 0x4A };
  }

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Possible result codes for the logging interface
   */
  enum class ErrCode : size_t
  {
    ERR_OK,
    ERR_IGNORE,
    ERR_FAIL,
    ERR_FAIL_MSG_TOO_LONG,
    ERR_FAIL_BAD_SINK,
    ERR_FAIL_BAD_ARG,
    ERR_FAIL_ISR_CONTEXT,
    ERR_NO_MEM,
    ERR_LOCKED,
    ERR_FULL,

    ERR_INVALID_LEVEL
  };

  /**
   * @brief The supported logging level types for all log sinks
   *
   * Names have to be prefixed with LVL_ to avoid conflicts with other
   * libraries that may define TRACE, DEBUG, etc. C macros can really
   * mess with your day here.
   */
  enum class Level : size_t
  {
    LVL_TRACE,
    LVL_DEBUG,
    LVL_INFO,
    LVL_WARN,
    LVL_ERROR,
    LVL_FATAL,

    LVL_MIN = LVL_TRACE,
    LVL_MAX = LVL_FATAL
  };

}  // namespace mb::logging

#endif  /* !MBEDUTILS_LOGGING_TYPES_HPP */
