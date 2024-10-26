/******************************************************************************
 *  File Name:
 *    util.hpp
 *
 *  Description:
 *    Miscellaneous utilities
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_UTILITIES_HPP
#define MBEDUTILS_UTILITIES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/interfaces/util_intf.hpp>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

/**
 * @brief Gets the number of elements in an array
 */
#define count_of_array( arr ) ( ( sizeof( arr ) / sizeof( ( arr )[ 0 ] ) ) )

/**
 * @brief Clears an array to zero
 */
#define clear_array( arr ) ( memset( arr, 0, sizeof( arr ) ) )

/**
 * @brief Clears a struct to zero
 */
#define clear_struct( s ) ( memset( &s, 0, sizeof( s ) ) )

#endif  /* !MBEDUTILS_UTILITIES_HPP */
