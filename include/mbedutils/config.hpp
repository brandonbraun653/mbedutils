/******************************************************************************
 *  File Name:
 *    config.hpp
 *
 *  Description:
 *    Master configuration file for the mbedutils library.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_CONFIG_HPP
#define MBEDUTILS_CONFIG_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#if __has_include("mbedutils_prj_config.hpp")
#include "mbedutils_prj_config.hpp"
#endif

/*-----------------------------------------------------------------------------
Assert Library Configuration
-----------------------------------------------------------------------------*/

#if !defined( MBEDUTILS_ASSERT_FMT_BUFFER_SIZE )
/**
 * @brief Size of the stack buffer used for formatting assertion messages
 */
#define MBEDUTILS_ASSERT_FMT_BUFFER_SIZE ( 256 )
#endif

/*-----------------------------------------------------------------------------
Logging Library Configuration
-----------------------------------------------------------------------------*/

#if !defined( MBEDUTILS_LOGGING_MAX_SINKS )
/**
 * @brief Maximum number of logging sinks that can be registered at once
 */
#define MBEDUTILS_LOGGING_MAX_SINKS ( 4 )
#endif

#if !defined( MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT )
/**
 * @brief How long a log sink will wait for a lock before giving up (ms)
 */
#define MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ( 10 )
#endif

#if !defined( MBEDUTILS_LOGGING_BUFFER_SIZE )
/**
 * @brief Maximum size of a message that can be logged at once (bytes)
 */
#define MBEDUTILS_LOGGING_BUFFER_SIZE ( 512 )
#endif

/*-----------------------------------------------------------------------------
OSAL Library Configuration
-----------------------------------------------------------------------------*/

#if !defined( MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION )
/**
 * @brief Enables dynamic memory allocation for OSAL objects
 */
#define MBEDUTILS_OSAL_USE_DYNAMIC_ALLOCATION ( 1 )
#endif

#if !defined( MBEDUTILS_OSAL_MUTEX_POOL_SIZE )
/**
 * @brief Number of mutexes to preallocate in the pool
 */
#define MBEDUTILS_OSAL_MUTEX_POOL_SIZE ( 0 )
#endif

#if !defined( MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE )
/**
 * @brief Number of recursive mutexes to preallocate in the pool
 */
#define MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE ( 0 )
#endif

#endif  /* !MBEDUTILS_CONFIG_HPP */
