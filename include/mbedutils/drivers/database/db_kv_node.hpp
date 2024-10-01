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
   * @brief Primary key type used to identify a parameter node.
   */
  using HashKey = uint32_t;

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
   * @param valid Flag indicating if the data is valid
   * @return true Data was updated
   * @return false Data was not updated
   */
  using WriteFunc = etl::delegate<bool( KVNode &node, const void *data, const size_t size, const bool valid )>;

  /**
   * @brief Custom reader function for a given KV node.
   *
   * Used when a simple binary copy is not the correct action for the data type.
   *
   * @param node  Reference to the node being read
   * @param data  Pointer to the output buffer to read the data into
   * @param size  Size of the output buffer
   * @return bool Validity state of the data
   */
  using ReadFunc = etl::delegate<bool( const KVNode &node, void *data, const size_t size )>;

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
  using ValidateFunc = etl::delegate<bool( const KVNode &node )>;

  /**
   * @brief Sanitize data for a given KV node.
   *
   * Inline sanitization function to clean up data before writing to the database.
   *
   * @param node Reference to the node being sanitized
   * @param data Pointer to the data to sanitize
   * @param size Size of the data to sanitize
   */
  using SanitizeFunc = etl::delegate<void( KVNode &node )>;

  /**
   * @brief Visitor function for traversing the KV parameter database.
   *
   * @param node Reference to the node being visited
   */
  using NodeVisitorFunc = etl::delegate<void( KVNode &node )>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr HashKey MAX_HASH_KEY = std::numeric_limits<HashKey>::max();

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
    KV_CLEAN_ON_WRITE     = 1 << 8, /**< Sanitize data before writing */
    KV_CLEAN_ON_READ      = 1 << 9, /**< Sanitize data after reading */

    KV_FLAG_DEFAULT_PERSISTENT = ( KV_FLAG_WRITE_BACK | KV_FLAG_PERSISTENT | KV_FLAG_AUTO_SANITIZE ),
    KV_FLAG_DEFAULT_VOLATILE   = ( KV_FLAG_AUTO_SANITIZE )
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief KV node writer function that uses a memcpy operation.
   * @see ::mb::db::WriteFunc
   */
  bool kv_writer_memcpy( KVNode &node, const void *data, const size_t size, const bool valid );

  /**
   * @brief KV node writer that accepts etl::string data types.
   * @see ::mb::db::WriteFunc
   */
  bool kv_writer_etl_string( KVNode &node, const void *data, const size_t size, const bool valid );

  /**
   * @brief KV node reader using memcpy
   * @see ::mb::db::ReadFunc
   */
  bool kv_reader_memcpy( const KVNode &node, void *data, const size_t size );

  /**
   * @brief KV node reader that accepts etl::string data types.
   * @see ::mb::db::ReadFunc
   */
  bool kv_reader_etl_string( const KVNode &node, void *data, const size_t size );


  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/

  /* Writers */
  static constexpr auto KVWriter_Memcpy    = WriteFunc::create<kv_writer_memcpy>();
  static constexpr auto KVWriter_EtlString = WriteFunc::create<kv_writer_etl_string>();

  /* Readers */
  static constexpr auto KVReader_Memcpy    = ReadFunc::create<kv_reader_memcpy>();
  static constexpr auto KVReader_EtlString = ReadFunc::create<kv_reader_etl_string>();

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
   * @return true   Data is valid
   * @return false  Data is invalid
   */
  bool is_valid( KVNode &node );

  /**
   * @brief Writes the given data to the KV node.
   *
   * @param node  Reference to the node being written to
   * @param data  Pointer to the data to write
   * @param size  Size of the data to write
   * @param valid Flag indicating if the data is valid
   * @return true The write was successful
   * @return false The write failed
   */
  bool write( KVNode &node, const void *data, const size_t size, const bool valid );

  /**
   * @brief Reads the data from the KV node.
   *
   * @param node  Reference to the node being read from
   * @param data  Pointer to the buffer to read the data into
   * @param size  Size of the buffer
   * @return true   The read was successful
   * @return false  The read failed
   */
  bool read( const KVNode &node, void *data, const size_t size );

  /**
   * @brief Serialize the data in the KV node to a binary format.
   *
   * @param node Reference to the node being serialized
   * @param data Pointer to the buffer to store the serialized data
   * @param size Size of the buffer
   * @return Size of the serialized data. Negative on error.
   */
  int serialize( const KVNode &node, void *const data, const size_t size );

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
  bool deserialize( KVNode &node, const void *data, const size_t size );

}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_PARAMETERS_HPP */
