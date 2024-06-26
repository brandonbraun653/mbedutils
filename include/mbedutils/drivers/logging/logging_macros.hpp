/******************************************************************************
 *  File Name:
 *    logging_macros.hpp
 *
 *  Description:
 *    Helper macros for inserting log messages into the system
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_LOGGING_MACROS_HPP
#define MBEDUTILS_LOGGING_MACROS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/logging/logging_driver.hpp>

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

#define LOG( level, fmt, ... )                                                      \
  do                                                                                \
  {                                                                                 \
    mbedutils::logging::log( level, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ ); \
  } while( 0 )

#define LOG_TRACE( str, ... ) \
  mbedutils::logging::flog( mbedutils::logging::Level::LVL_TRACE, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_TRACE_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_TRACE( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_DEBUG( str, ... ) \
  mbedutils::logging::flog( mbedutils::logging::Level::LVL_DEBUG, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_DEBUG_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_DEBUG( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_INFO( str, ... ) \
  mbedutils::logging::flog( mbedutils::logging::Level::LVL_INFO, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_INFO_IF( predicate, str, ... ) \
  if( ( predicate ) )                      \
  {                                        \
    LOG_INFO( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_WARN( str, ... ) \
  mbedutils::logging::flog( mbedutils::logging::Level::LVL_WARN, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_WARN_IF( predicate, str, ... ) \
  if( ( predicate ) )                      \
  {                                        \
    LOG_WARN( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_ERROR( str, ... ) \
  mbedutils::logging::flog( mbedutils::logging::Level::LVL_ERROR, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_ERROR_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_ERROR( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_FATAL( str, ... ) \
  mbedutils::logging::flog( mbedutils::logging::Level::LVL_FATAL, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_FATAL_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_FATAL( ( str ), ##__VA_ARGS__ );    \
  }

#endif /* !MBEDUTILS_LOGGING_MACROS_HPP */
