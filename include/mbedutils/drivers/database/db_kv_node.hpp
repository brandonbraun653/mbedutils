/******************************************************************************
 *  File Name:
 *    db_kv_node.hpp
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
#include <etl/delegate.h>
#include <etl/string.h>
#include <etl/vector.h>
#include <mbedutils/drivers/database/db_kv_util.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <pb.h>

namespace mb::db
{
  /*-------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------*/

  struct KVNode;

  /*-------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------*/

  /**
   * @brief Vector of parameter nodes.
   */
  template<size_t N>
  using KVNodeVector = etl::vector<KVNode, N>;

  /**
   * @brief Size independent vector of parameter nodes.
   */
  using KVNodeIVector = etl::ivector<KVNode>;

  /**
   * @brief Custom writer function for a given KV node.
   *
   * Update the cached state of the parameter data. Used when a binary copy
   * is not appropriate for the underlying data type.
   *
   * @param node  Reference to the node being updated
   * @param data  Pointer to the data containing the update
   * @param size  Size of the new data
   * @return True if the write was successful, false otherwise
   */
  using WriteFunc = etl::delegate<bool( KVNode &node, const void *data, const size_t size )>;

  /**
   * @brief Custom reader function for a given KV node.
   *
   * Used when a simple binary copy is not the correct action for the data type.
   *
   * @param node  Reference to the node being read
   * @param data  Pointer to the output buffer to read the data into
   * @param size  Number of bytes to read into the buffer
   * @return Number of bytes read, negative on error
   */
  using ReadFunc = etl::delegate<int( const KVNode &node, void *data, const size_t size )>;

  /**
   * @brief Validation function for a given KV node.
   *
   * This is a high level function to validate structured data before writing
   * and after reading from the database.
   *
   * @param node Reference to the node being validated
   * @param data Pointer to the data to validate
   * @param size Size of the data to validate
   * @return Data validity status
   */
  using ValidateFunc = etl::delegate<bool( const KVNode &node, void *data, const size_t size )>;

  /**
   * @brief Sanitize data for a given KV node.
   *
   * Inline sanitization function to clean up data before writing to the database.
   *
   * @param node Reference to the node being sanitized
   * @param data Pointer to the data to sanitize
   * @param size Size of the data to sanitize
   */
  using SanitizeFunc = etl::delegate<void( KVNode &node, void *data, const size_t size )>;

  /**
   * @brief Visitor function for traversing the KV parameter database.
   *
   * @param node Reference to the node being visited
   */
  using VisitorFunc = etl::delegate<void( KVNode &node )>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr HashKey MAX_HASH_KEY = std::numeric_limits<HashKey>::max();

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Descriptive information about a key-value node.
   *
   * These flags control the runtime behavior of a kv node, influencing how
   * data is read, written, and validated. None of this information should
   * be stored in NVM, it is purely for runtime control.
   */
  enum KVFlag : uint16_t
  {
    /*-------------------------------------------------------------------------
    Data Attribute Flags
    -------------------------------------------------------------------------*/
    KV_FLAG_READ_ONLY         = 1 << 0, /**< KV data is not writeable */
    KV_FLAG_PERSISTENT        = 1 << 1, /**< Data is backed by NVM */
    KV_FLAG_SANITIZE_ON_WRITE = 1 << 2, /**< Sanitize data before writing */
    KV_FLAG_SANITIZE_ON_READ  = 1 << 3, /**< Sanitize data after reading */

    /*-------------------------------------------------------------------------
    Data State Flags
    -------------------------------------------------------------------------*/
    KV_FLAG_LOCKED = 1 << 4, /**< Data is locked and cannot be modified */
    KV_FLAG_DIRTY  = 1 << 5, /**< Data has been modified and needs to be written */
    KV_FLAG_VALID  = 1 << 6, /**< Data has been validated and is in a high-trust state */

    /*-------------------------------------------------------------------------
    Insertion Policies
    -------------------------------------------------------------------------*/
    KV_FLAG_INSERT_POLICY_FAIL      = 1 << 8,  /**< Fail the insertion if the key already exists */
    KV_FLAG_INSERT_POLICY_PULL      = 1 << 9,  /**< Pull the key data from NVM if it already exists */
    KV_FLAG_INSERT_POLICY_OVERWRITE = 1 << 10, /**< Overwrite the key if it already exists */

    /*-------------------------------------------------------------------------
    Data Caching Policies
    -------------------------------------------------------------------------*/
    KV_FLAG_CACHE_POLICY_READ_CACHE    = 1 << 11, /**< When read() is called, source from RAM cache */
    KV_FLAG_CACHE_POLICY_READ_THROUGH  = 1 << 12, /**< When read() is called, source from NVM directly */
    KV_FLAG_CACHE_POLICY_READ_SYNC     = 1 << 13, /**< When read() is called, source from NVM and update the RAM cache */
    KV_FLAG_CACHE_POLICY_WRITE_THROUGH = 1 << 14, /**< Write to NVM immediately */
    KV_FLAG_CACHE_POLICY_WRITE_BACK    = 1 << 15, /**< Write to NVM on flushing dirty tags */

    /*-------------------------------------------------------------------------
    Flag Aggregates
    -------------------------------------------------------------------------*/
    /**
     * @brief Default flags for a persistent parameter.
     *
     * This enables quick reads/writes to the cache, delayed write back to NVM, and
     * upon insertion, the data from NVM is pulled into the cache.
     */
    KV_FLAG_DEFAULT_PERSISTENT =
        ( KV_FLAG_PERSISTENT | KV_FLAG_INSERT_POLICY_PULL | KV_FLAG_CACHE_POLICY_READ_CACHE | KV_FLAG_CACHE_POLICY_WRITE_BACK ),

    /**
     * @brief Default flags for a volatile parameter.
     *
     * This enables quick reads/writes to the cache, but no persistent storage.
     */
    KV_FLAG_DEFAULT_VOLATILE = ( KV_FLAG_CACHE_POLICY_READ_CACHE )
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief KV node writer function that uses a memcpy operation.
   * @see ::mb::db::WriteFunc
   */
  bool kv_writer_memcpy( KVNode &node, const void *data, const size_t size );

  /**
   * @brief KV node writer from char* -> etl::string.
   * @see ::mb::db::WriteFunc
   */
  bool kv_writer_char_to_etl_string( KVNode &node, const void *data, const size_t size );

  /**
   * @brief KV node reader using memcpy
   * @see ::mb::db::ReadFunc
   */
  int kv_reader_memcpy( const KVNode &node, void *data, const size_t size );

  /**
   * @brief KV node reader from etl::string -> char *.
   * @see ::mb::db::ReadFunc
   */
  int kv_reader_etl_string_to_char( const KVNode &node, void *data, const size_t size );


  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/

  /* Writers */
  static constexpr auto KVWriter_Memcpy    = WriteFunc::create<kv_writer_memcpy>();
  static constexpr auto KVWriter_EtlString = WriteFunc::create<kv_writer_char_to_etl_string>();

  /* Readers */
  static constexpr auto KVReader_Memcpy    = ReadFunc::create<kv_reader_memcpy>();
  static constexpr auto KVReader_EtlString = ReadFunc::create<kv_reader_etl_string_to_char>();

  /* Validators */

  /* Visitors */

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
  struct KVNode
  {
    HashKey             hashKey   = MAX_HASH_KEY; /**< Software enumeration/hash tied to parameter */
    WriteFunc           writer    = {};           /**< Specify how to write data into the datacache */
    ReadFunc            reader    = {};           /**< Specify how to read data from the datacache */
    ValidateFunc        validator = {};           /**< Specify how the data is validated */
    SanitizeFunc        sanitizer = {};           /**< Specify data sanitization behavior */
    VisitorFunc     onWrite   = {};           /**< Callback for when data is written */
    void               *datacache = nullptr;      /**< Where the data lives in memory */
    const pb_msgdesc_t *pbFields  = nullptr;      /**< NanoPB "<obj>_fields" descriptor if serialization support is desired */
    uint16_t            dataSize  = 0;            /**< Size of the datacache. If NanoPB, use the "<obj>_size" literal. */
    uint16_t            flags     = 0;            /**< Flags to control behavior */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Sanitize the data in the KV node.
   *
   * @param node Reference to the node being sanitized
   */
  void sanitize( KVNode &node );

  /**
   * @brief Checks for validity of the data in the KV node.
   *
   * @param node Reference to the node being checked
   * @return True if the data is valid, false otherwise
   */
  bool data_is_valid( const KVNode &node );

  /**
   * @brief Checks if the node configuration is valid.
   *
   * @param node  Reference to the node being checked
   * @return True if the node is valid, false otherwise
   */
  bool node_is_valid( const KVNode &node );

  /**
   * @brief Writes the given data to the KV node.
   *
   * @param node  Reference to the node being written to
   * @param data  Pointer to the data to write
   * @param size  Size of the data to write
   * @return True if the write was successful, false otherwise
   */
  bool node_write( KVNode &node, const void *data, const size_t size );

  /**
   * @brief Reads the data from the KV node.
   *
   * @param node  Reference to the node being read from
   * @param data  Pointer to the buffer to read the data into
   * @param size  Number of bytes to read into the buffer
   * @return Number of bytes read, negative on error
   */
  int node_read( const KVNode &node, void *data, const size_t size );

  /**
   * @brief Serialize the data in the KV node to a binary format.
   *
   * @param node Reference to the node being serialized
   * @param data Pointer to the buffer to store the serialized data
   * @param size Size of the buffer
   * @return Size of the serialized data. Negative on error.
   */
  int node_serialize( const KVNode &node, void *const data, const size_t size );

  /**
   * @brief Deserialize the given NanoPB data into the KV node.
   *
   * This method decodes NanoPB serialized data into the underlying KVNode
   * RAM storage, if it exists. This expects the exact length of serialized
   * data to decode, due to the possibility of variable length serialization.
   *
   * @param node Reference to the node being deserialized
   * @param data Pointer to the serialized data. Must be NanoPB encoded.
   * @param size Exact size of the serialized data.
   * @return Success or failure
   */
  bool node_deserialize( KVNode &node, const void *data, const size_t size );

}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_PARAMETERS_HPP */
