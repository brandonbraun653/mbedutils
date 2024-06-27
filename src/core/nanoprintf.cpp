/******************************************************************************
 *  File Name:
 *    nanoprintf.cpp
 *
 *  Description:
 *    Translation unit for compiling the nanoprintf library so we don't get
 *    "multiple definition" errors just from including the header.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/

#define NANOPRINTF_IMPLEMENTATION
#include <mbedutils/drivers/nanoprintf.hpp>
