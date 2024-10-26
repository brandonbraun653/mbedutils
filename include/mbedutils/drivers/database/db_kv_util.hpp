/******************************************************************************
 *  File Name:
 *    db_kv_util.hpp
 *
 *  Description:
 *    Shared utilities and types for the key-value database module
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_KV_UTIL_HPP
#define MBEDUTILS_KV_UTIL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>
#include <etl/string.h>

namespace mb::db
{
  /*-------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------*/

  /**
   * @brief Primary key type used to identify a parameter node.
   */
  using HashKey = uint32_t;

  /**
   * @brief String representation of a hash key
   *
   * This is used to store the hash key in a human readable format and for
   * literal keying in the NVM backed database w/FlashDB. This is the hex
   * string the 32-bit hash key.
   */
  using HashRepr = etl::string<8>;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum DBError : size_t
  {
    DB_ERR_NONE,
    DB_ERR_NOT_AVAILABLE,
    DB_ERR_BAD_ARG,
    DB_ERR_TRANSCODE_BUFFER_TOO_SMALL,
    DB_ERR_RESOURCES,

    DB_ERR_MAX
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Hashes a string into a 32-bit key
   *
   * @param key String to hash
   * @return HashKey 32-bit hash key
   */
  HashKey hash( etl::string_view key );

  /**
   * @brief Hashes a binary buffer into a 32-bit key
   *
   * @param key Pointer to the buffer
   * @param size Size of the buffer
   * @return HashKey 32-bit hash key
   */
  HashKey hash( const void *key, const size_t size );

  /**
   * @brief Convert a FlashDB error code to a string
   *
   * @param err Error to convert
   * @return const char*
   */
  const char *fdb_err_to_str( const int err );

}    // namespace mb::db

#endif /* !MBEDUTILS_KV_UTIL_HPP */
