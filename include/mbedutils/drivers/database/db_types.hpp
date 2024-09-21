/******************************************************************************
 *  File Name:
 *    db_types.hpp
 *
 *  Description:
 *    Common types for database integration
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_DATABASE_TYPES_HPP
#define MBEDUTILS_DATABASE_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <pb.h>


namespace mb::db
{
  /*-------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------*/

  struct KVNode_t;

  /*-------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------*/

  using KVKey_t = uint32_t;

  /**
   * @brief Validation function for a given KV node.
   *
   * This is a high level function to validate structured data before writing
   * and after reading from the database.
   *
   * @param node Reference to the node being validated
   * @param data Pointer to the data to validate
   * @param size Size of the data to validate
   * @return true Data is valid
   * @return false Data is invalid
   */
  using ValidatorFunc = bool ( * )( const KVNode_t &node, const void *data, const size_t size );

  /**
   * @brief Sanitize data for a given KV node.
   *
   * Inline sanitization function to clean up data before writing to the database.
   *
   * @param node Reference to the node being sanitized
   * @param data Pointer to the data to sanitize
   * @param size Size of the data to sanitize
   */
  using SanitizerFunc = void ( * )( const KVNode_t &node, void *data, const size_t size );

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum KVFlags : uint16_t
  {
    KV_FLAG_READ_ONLY     = 1 << 0, /**< KV data is not writeable */
    KV_FLAG_WRITE_BACK    = 1 << 1, /**< Delay write until later */
    KV_FLAG_WRITE_THROUGH = 1 << 2, /**< Immediately write data to NVM */
    KV_FLAG_CACHEABLE     = 1 << 3, /**< Data is cached in RAM */
    KV_FLAG_LOCKED        = 1 << 4, /**< Data is locked and cannot be modified */
    KV_FLAG_AUTO_SANITIZE = 1 << 5, /**< Automatically sanitize data before writing */

    KV_FLAG_DEFAULT = ( KV_FLAG_WRITE_BACK | KV_FLAG_CACHEABLE | KV_FLAG_AUTO_SANITIZE )
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct KVNode_t
  {
    uint32_t            key;          /**< Software enumeration tied to parameter */
    ValidatorFunc       validator;    /**< Function to validate the data */
    SanitizerFunc       sanitizer;    /**< Function to clean the data before writing */
    void               *pbRAMCopy;    /**< If cached, the where the data lives. Must be the NanoPB type. */
    const pb_msgdesc_t *pbDescriptor; /**< Nanopb descriptor for the data type */
    uint16_t            pbSize;       /**< Size of the nanopb data type */
    uint16_t            flags;        /**< Flags to control behavior */
  };

}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_TYPES_HPP */
