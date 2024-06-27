/******************************************************************************
 *  File Name:
 *    nanoprintf.hpp
 *
 *  Description:
 *    Interface for the nanoprintf library. This is a lightweight printf
 *   implementation that is designed to be used in embedded systems.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_NANOPRINTF_HPP
#define MBEDUTILS_NANOPRINTF_HPP

/*-----------------------------------------------------------------------------
NanoPrintf configuration for this module. For more information, see:
  https://github.com/charlesnicholson/nanoprintf#configuration
-----------------------------------------------------------------------------*/
#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 0

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "nanoprintf.h"

#endif  /* !MBEDUTILS_NANOPRINTF_HPP */
