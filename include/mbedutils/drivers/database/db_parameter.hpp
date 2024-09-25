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

  struct KVParamNode;

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
  using KVParamNodeVector = etl::vector<KVParamNode, N>;

  /**
   * @brief Size independent vector of parameter nodes.
   */
  using KVParamNodeIVector = etl::ivector<KVParamNode>;

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
   * @return true Data was updated
   * @return false Data was not updated
   */
  using UpdateFunc = etl::delegate<bool( const KVParamNode &node, const void *data, const size_t size, const bool valid )>;

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

  /**
   * @brief Serialize the data in a KV node's RAM cache to a binary format.
   *
   * @param node Reference to the node being serialized
   * @param data Output pointer to store the serialized data into
   * @param size Size of the output buffer
   * @return true Serialization was successful
   * @return false Serialization failed
   */
  using SerializeFunc = etl::delegate<bool( const KVParamNode &node, void *const data, const size_t size )>;

  /**
   * @brief Deserialize the given data into the KV node's RAM cache.
   *
   * @param node Reference to the node being deserialized
   * @param data Input pointer to the serialized data
   * @param size Size of the input buffer/data
   * @return true Deserialization was successful
   * @return false Deserialization failed
   */
  using DeserializeFunc = etl::delegate<bool( KVParamNode &node, const void *const data, const size_t size )>;

  /**
   * @brief Visitor function for traversing the KV parameter database.
   *
   * @param node Reference to the node being visited
   */
  using NodeVisitorFunc = etl::delegate<void( KVParamNode &node )>;

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
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Shared deserialization function for KV parameter nodes.
   *
   * @param node  Reference to the node being deserialized
   * @param data  Pointer to the serialized data
   * @param size  Size of the serialized data
   * @return true   Deserialization was successful
   * @return false  Deserialization failed
   */
  bool cmn_param_deserializer( KVParamNode &node, const void *const data, const size_t size );

  /**
   * @brief Shared serialization function for KV parameter nodes.
   *
   * @param node  Reference to the node being serialized
   * @param data  Pointer to the buffer to store the serialized data
   * @param size  Size of the buffer
   * @return true   Serialization was successful
   * @return false  Serialization failed
   */
  bool cmn_param_serializer( const KVParamNode &node, void *const data, const size_t size );

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/

  static constexpr auto DefaultDeserializer = DeserializeFunc::create<cmn_param_deserializer>();
  static constexpr auto DefaultSerializer   = SerializeFunc::create<cmn_param_serializer>();

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
  struct KVParamNode
  {
    HashKey             hashKey;      /**< Software enumeration/hash tied to parameter */
    UpdateFunc          updator;      /**< Function to update the data */
    ValidateFunc        validator;    /**< Function to validate the data */
    SanitizeFunc        sanitizer;    /**< Function to clean the data before writing */
    SerializeFunc       serializer;   /**< Function to serialize the data */
    DeserializeFunc     deserializer; /**< Function to deserialize the data */
    void               *pbRAMCopy;    /**< Where the data lives. Must be the NanoPB type. */
    const pb_msgdesc_t *pbDescriptor; /**< Nanopb descriptor for the data type */
    uint16_t            pbSize;       /**< Size of the nanopb data type */
    uint16_t            flags;        /**< Flags to control behavior */

    /**
     * @brief Sanitize the data in the KV node.
     */
    void sanitize();

    /**
     * @brief Checks for validity of the data in the KV node.
     *
     * @return true   Data is valid
     * @return false  Data is invalid
     */
    bool is_valid();

    /**
     * @brief Writes the given data to the KV node.
     *
     * @param data  Pointer to the data to write
     * @param size  Size of the data to write
     * @param valid Flag indicating if the data is valid
     * @return true The write was successful
     * @return false The write failed
     */
    bool write( const void *data, const size_t size, const bool valid );

    /**
     * @brief Reads the data from the KV node.
     *
     * @param data  Pointer to the buffer to read the data into
     * @param size  Size of the buffer
     * @return true   The read was successful
     * @return false  The read failed
     */
    bool read( void *data, const size_t size );

    /**
     * @brief Serialize the data in the KV node to a binary format.
     *
     * @param data Pointer to the buffer to store the serialized data
     * @param size Size of the buffer
     * @return true Serialization was successful
     * @return false Serialization failed
     */
    bool serialize( void *const data, const size_t size );

    /**
     * @brief Deserialize the given data into the KV node.
     *
     * @param data Pointer to the serialized data
     * @param size Size of the serialized data
     * @return true Deserialization was successful
     * @return false Deserialization failed
     */
    bool deserialize( const void *const data, const size_t size );
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
     * @param params Base storage for the parameter nodes (persistent)
     */
    void init( KVParamNodeIVector &params );

    /**
     * @brief Adds a new parameter node to the storage manager.
     *
     * @param node  Node to add
     * @return true   Node was added
     * @return false  Node was not added
     */
    bool insert( const KVParamNode &node );

    /**
     * @brief Remove a parameter node from the storage manager.
     *
     * @param key Key of the node to remove
     */
    void remove( const HashKey key );

    /**
     * @brief Retrieve a parameter node by key
     *
     * @param key Key to search for
     * @return KVParamNode* Pointer to the parameter node if found, nullptr otherwise
     */
    KVParamNode *find( const HashKey key );

    /**
     * @brief Applies a visitor function to all dirty nodes.
     *
     * @param visitor Function to call on each dirty node
     */
    void visit_dirty_nodes( NodeVisitorFunc &visitor );

  private:
    mb::osal::mb_recursive_mutex_t mRMutex;  /**< Mutex for thread safety */
    KVParamNodeIVector            *mParams;  /**< External storage of parameter nodes */
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_PARAMETERS_HPP */
