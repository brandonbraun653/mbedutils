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
#include <cstdint>

/*-----------------------------------------------------------------------------
Create the __SHORT_FILE__ macro, which returns the file name instead of the
whole path from __FILE__ https://blog.galowicz.de/2016/02/20/short_file_macro/
-----------------------------------------------------------------------------*/
using cstr = const char *const;
static constexpr cstr past_last_slash( cstr str, cstr last_slash )
{
  return *str == '\0' ? last_slash : *str == '/' ? past_last_slash( str + 1, str + 1 ) : past_last_slash( str + 1, last_slash );
}

static constexpr cstr past_last_slash( cstr str )
{
  return past_last_slash( str, str );
}

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

#define __SHORT_FILE__                                  \
  ( {                                                   \
    constexpr cstr sf__{ past_last_slash( __FILE__ ) }; \
    sf__;                                               \
  } )

#endif  /* !MBEDUTILS_UTIL_INTF_HPP */
