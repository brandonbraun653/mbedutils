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
#include <mbedutils/interfaces/util_intf.hpp>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

#define LOG( level, fmt, ... )                                                      \
  do                                                                                \
  {                                                                                 \
    mb::logging::log( level, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ ); \
  } while( 0 )

#define LOG_TRACE( str, ... ) \
  mb::logging::flog( mb::logging::Level::LVL_TRACE, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_TRACE_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_TRACE( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_DEBUG( str, ... ) \
  mb::logging::flog( mb::logging::Level::LVL_DEBUG, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_DEBUG_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_DEBUG( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_INFO( str, ... ) \
  mb::logging::flog( mb::logging::Level::LVL_INFO, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_INFO_IF( predicate, str, ... ) \
  if( ( predicate ) )                      \
  {                                        \
    LOG_INFO( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_WARN( str, ... ) \
  mb::logging::flog( mb::logging::Level::LVL_WARN, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_WARN_IF( predicate, str, ... ) \
  if( ( predicate ) )                      \
  {                                        \
    LOG_WARN( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_ERROR( str, ... ) \
  mb::logging::flog( mb::logging::Level::LVL_ERROR, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_ERROR_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_ERROR( ( str ), ##__VA_ARGS__ );    \
  }


#define LOG_FATAL( str, ... ) \
  mb::logging::flog( mb::logging::Level::LVL_FATAL, __SHORT_FILE__, __LINE__, str, ##__VA_ARGS__ )
#define LOG_FATAL_IF( predicate, str, ... ) \
  if( ( predicate ) )                       \
  {                                         \
    LOG_FATAL( ( str ), ##__VA_ARGS__ );    \
  }

#endif /* !MBEDUTILS_LOGGING_MACROS_HPP */
