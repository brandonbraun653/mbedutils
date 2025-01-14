/******************************************************************************
 *  File Name:
 *    memory.hpp
 *
 *  Description:
 *    Mbedutils common memory functionality headers
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_MEMORY_HPP
#define MBEDUTILS_MEMORY_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/

#include <mbedutils/drivers/memory/memory_types.hpp>
#include <mbedutils/drivers/memory/block_device.hpp>
#include <mbedutils/drivers/memory/nvm/nor_flash_device.hpp>
#include <mbedutils/drivers/memory/nvm/nor_flash.hpp>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

/**
 * @brief Aligns a value up to the nearest multiple of the given alignment.
 *
 * @param x     Value to align
 * @param align Alignment value
 */
#define ALIGN_UP( x, align ) ( ( ( x ) + ( ( align ) - 1 ) ) & ~( ( align ) - 1 ) )

/**
 * @brief Aligns a value down to the nearest multiple of the given alignment.
 *
 * @param x     Value to align
 * @param align Alignment value
 */
#define ALIGN_DOWN( x, align ) ( ( x ) & ~( ( align ) - 1 ) )

#endif /* !MBEDUTILS_MEMORY_HPP */
