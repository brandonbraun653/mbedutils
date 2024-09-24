/******************************************************************************
 *  File Name:
 *    db_parameter.hpp
 *
 *  Description:
 *    Database parameter interface. This is the volatile component of
 *    structured data storage on an embedded system. It is designed to handle
 *    user facing needs for parameter storage, retrieval, and validation.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_DATABASE_PARAMETERS_HPP
#define MBEDUTILS_DATABASE_PARAMETERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/string.h>
#include <etl/vector.h>
#include <pb.h>

namespace mb::db
{
  /*-------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------*/

  struct KVParamNode;

  /*-------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------*/

  /**
   * @brief Primary key type used to identify a parameter node.
   */
  using HashKey = uint32_t;

  /**
   * @brief Custom updator function for a given KV node.
   *
   * Update the cached state of the parameter data. Used when a binary copy
   * is not appropriate for the underlying data type.
   *
   * @param node  Reference to the node being updated
   * @param data  Pointer to the data containing the update
   * @param size  Size of the new data
   * @param valid Flag indicating if the data is valid
   */
  using UpdateFunc = etl::delegate<void( const KVParamNode &node, const void *data, const size_t size, const bool valid )>;

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
  using ValidateFunc = etl::delegate<bool( const KVParamNode &node )>;

  /**
   * @brief Sanitize data for a given KV node.
   *
   * Inline sanitization function to clean up data before writing to the database.
   *
   * @param node Reference to the node being sanitized
   * @param data Pointer to the data to sanitize
   * @param size Size of the data to sanitize
   */
  using SanitizeFunc = etl::delegate<void( const KVParamNode &node )>;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum KVFlags : uint16_t
  {
    KV_FLAG_READ_ONLY     = 1 << 0, /**< KV data is not writeable */
    KV_FLAG_WRITE_BACK    = 1 << 1, /**< If persistent, delay write until later */
    KV_FLAG_WRITE_THROUGH = 1 << 2, /**< If persistent, immediately write data to NVM */
    KV_FLAG_PERSISTENT    = 1 << 3, /**< Data is persistent and backed in NVM */
    KV_FLAG_LOCKED        = 1 << 4, /**< Data is locked and cannot be modified */
    KV_FLAG_AUTO_SANITIZE = 1 << 5, /**< Automatically sanitize data before writing */
    KV_FLAG_DIRTY         = 1 << 6, /**< Data has been modified and needs to be written */
    KV_FLAG_VALID         = 1 << 7, /**< Data is valid */

    KV_FLAG_DEFAULT_PERSISTENT = ( KV_FLAG_WRITE_BACK | KV_FLAG_PERSISTENT | KV_FLAG_AUTO_SANITIZE ),
    KV_FLAG_DEFAULT_VOLATILE   = ( KV_FLAG_AUTO_SANITIZE )
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Core definition for a key-value parameter node.
   *
   * This provides the necessary information for managing the runtime behavior
   * of a single parameter in a database, such as storage location, validation,
   * and serialization.
   */
  class KVParamNode
  {
  public:
    HashKey             hashKey;      /**< Software enumeration/hash tied to parameter */
    UpdateFunc          updator;      /**< Function to update the data */
    ValidateFunc        validator;    /**< Function to validate the data */
    SanitizeFunc        sanitizer;    /**< Function to clean the data before writing */
    void               *pbRAMCopy;    /**< Where the data lives. Must be the NanoPB type. */
    const pb_msgdesc_t *pbDescriptor; /**< Nanopb descriptor for the data type */
    uint16_t            pbSize;       /**< Size of the nanopb data type */
    uint16_t            flags;        /**< Flags to control behavior */

    KVParamNode() = default;


    int write( const void *data, const size_t size, const bool valid );

    int read( void *data, const size_t size );

    void sanitize();

    bool validate();
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Manages the storage of KV parameter nodes.
   *
   * Allows for discrete control of parameter storage, retrieval, introspection,
   * and modification.
   */
  class KVParamStorageManager
  {
  public:
    KVParamStorageManager();
    ~KVParamStorageManager();

    /**
     * @brief Initialize the manager
     *
     * @param params Base storage for the parameter nodes
     */
    void init( etl::ivector<KVParamNode> &params );

    /**
     * @brief Retrieve a parameter node by key
     *
     * @param key Key to search for
     * @return KVParamNode* Pointer to the parameter node if found, nullptr otherwise
     */
    KVParamNode *find( const HashKey key );

    /**
     * @brief Retrieve a parameter node by key string name
     *
     * @param key
     * @return KVParamNode*
     */
    KVParamNode *find( const etl::string_view &key );


    // Add some iterators for dirty bits, etc

  private:
    etl::ivector<KVParamNode> *mParams;  /**< External storage of parameter nodes */
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_PARAMETERS_HPP */
