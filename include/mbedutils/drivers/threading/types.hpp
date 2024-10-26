/******************************************************************************
 *  File Name:
 *    types.hpp
 *
 *  Description:
 *    Shared types between threading modules
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_TYPES_HPP
#define MBEDUTILS_THREADING_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using TaskId = size_t;

}  // namespace mb::thread

#endif  /* !MBEDUTILS_THREADING_TYPES_HPP */
