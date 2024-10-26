/******************************************************************************
 *  File Name:
 *    util_intf.hpp
 *
 *  Description:
 *    General utilities for things that are not easily categorized elsewhere
 *    in the library but are still pretty useful.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_UTIL_INTF_HPP
#define MBEDUTILS_UTIL_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

/*-----------------------------------------------------------------------------
Create the __SHORT_FILE__ macro, which returns the file name instead of the
whole path from __FILE__ https://blog.galowicz.de/2016/02/20/short_file_macro/
-----------------------------------------------------------------------------*/
namespace mb::internal
{
  using cstr = const char *const;
  static constexpr cstr past_last_slash( cstr str, cstr last_slash )
  {
    return *str == '\0'  ? last_slash
           : *str == '/' ? past_last_slash( str + 1, str + 1 )
                         : past_last_slash( str + 1, last_slash );
  }

  static constexpr cstr past_last_slash( cstr str )
  {
    return past_last_slash( str, str );
  }
}    // namespace mb::internal

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

#define __SHORT_FILE__                                                                  \
  ( {                                                                                   \
    constexpr ::mb::internal::cstr sf__{ ::mb::internal::past_last_slash( __FILE__ ) }; \
    sf__;                                                                               \
  } )

namespace mb
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief Useful for uniquely identifying if a driver has been initialized.
   *
   * Depending on how memory gets laid out/init'd, this could be a stronger check
   * than a simple boolean flag. It's not likely memory will randomly equal this.
   * The data means nothing, it's from random.org.
   */
  static constexpr size_t DRIVER_INITIALIZED_KEY = static_cast<size_t>( 0x560bf751 );

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Standard error type for the library
   */
  using err_t = int;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum ErrorCode : err_t
  {
    ERR_OK      = 0,     /**< No error occurred */
    ERR_GENERIC = -1000, /**< Generic error when nothing else fits */
    ERR_UNKNOWN,         /**< Unknown error, usually for parsing */
    ERR_FAIL,            /**< Operation did not complete nominally */
    ERR_INVALID_ARG,     /**< Invalid arguments were given */
    ERR_NOT_AVAILABLE,   /**< Resource is not available  */
    ERR_NOT_SUPPORTED,   /**< Operation is not supported */
    ERR_MEMORY,          /**< Memory error (oob, invalid access, etc.) */
    ERR_TIMEOUT,         /**< Operation timed out */
    ERR_OVERFLOW,        /**< Data overflow */
    ERR_UNDERFLOW,       /**< Data underflow */
    ERR_BUSY,            /**< Resource is busy */
    ERR_AGAIN,           /**< Try again! May succeed. */
    ERR_BAD_STATE,       /**< Bad state for the operation */
    ERR_CRC,             /**< CRC error */
    ERR_ENCODE,          /**< Data encoding error */
    ERR_DECODE,          /**< Data decoding error */
  };
}

#endif  /* !MBEDUTILS_UTIL_INTF_HPP */
