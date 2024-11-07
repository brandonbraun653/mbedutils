/******************************************************************************
 *  File Name:
 *    string.hpp
 *
 *  Description:
 *    Mbedutils string interfaces
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_STIRNG_HPP
#define MBEDUTILS_STIRNG_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mb::string
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Ensure the string is terminated with a CRLF sequence
   *
   * @param buffer Buffer to check
   * @param buffer_size Size of the buffer, including the null terminator
   */
  void ensure_crlf_termination( char *const buffer, const size_t buffer_size );
}  // namespace mb::string

#endif  /* !MBEDUTILS_STIRNG_HPP */
