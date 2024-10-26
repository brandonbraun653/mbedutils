/******************************************************************************
 *  File Name:
 *    db_intf.hpp
 *
 *  Description:
 *    Core database driver interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_DATABASE_INTERFACE_HPP
#define MBEDUTILS_DATABASE_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/string.h>
#include <mbedutils/drivers/database/db_kv_node.hpp>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief High level interface to a Key-Value database
   */
  class IKVDatabase
  {
  public:
    virtual ~IKVDatabase() = default;

    /**
     * @brief Prepare the database for use
     *
     * @return true   The database was initialized successfully
     * @return false  The database failed to initialize
     */
    virtual bool init() = 0;

    /**
     * @brief Tear down the database, putting into a safe state for power off.
     */
    virtual void deinit() = 0;

    /**
     * @brief Adds a new parameter node to the storage manager.
     *
     * @param node  Node to add
     * @return true   Node was added
     * @return false  Node was not added
     */
    virtual bool insert( const KVNode &node ) = 0;

    /**
     * @brief Remove a parameter node from the storage manager.
     *
     * @param key Key of the node to remove
     */
    virtual void remove( const HashKey key ) = 0;

    /**
     * @brief Retrieve a parameter node by key
     *
     * @param key Key to search for
     * @return KVNode* Pointer to the parameter node if found, nullptr otherwise
     */
    virtual KVNode *find( const HashKey key ) = 0;

    /**
     * @brief Checks if a key exists in the database
     *
     * @param key     The key to check for
     * @return true   The key exists in the database
     * @return false  The key does not exist in the database
     */
    virtual bool exists( const HashKey key ) = 0;

    /**
     * @brief Update all keys from the underlying storage, if any.
     */
    virtual void sync() = 0;

    /**
     * @brief Update a specific key from the underlying storage, if any.
     *
     * @param key The key to sync
     */
    virtual void sync( const HashKey key ) = 0;

    /**
     * @brief Flush dirty keys to the underlying storage, if any.
     */
    virtual void flush() = 0;

    /**
     * @brief Reads data from a key in the database.
     *
     * @param key         The key to read
     * @param data        Buffer to read the data into
     * @param data_size   Size of the data buffer
     * @param size        Number of bytes to read
     * @return int        Error code if negative, number of bytes read if positive
     */
    virtual int read( const HashKey key, void *data, const size_t data_size, const size_t size ) = 0;

    /**
     * @brief Writes data to a key in the database.
     *
     * @param key         The key to write to
     * @param data        Data to write
     * @param size        Number of bytes to write
     * @return int        Error code if negative, number of bytes written if positive
     */
    virtual int write( const HashKey key, void *data, const size_t size ) = 0;

    /**
     * @brief Encodes value data with the NanoPB serializer.
     *
     * Used when communicating with an external system over a shared format. It's expected
     * that the underlying KV storage `datacache` matches the `pbFields` and `dataSize` fields
     * of the KVNode object.
     *
     * @param key       The key to encode
     * @param out_data  Output buffer to write the encoded data to
     * @param out_size  Size of the output buffer
     * @return int      Number of bytes written to the output buffer, negative on error.
     */
    virtual int encode( const HashKey key, void *out_data, const size_t out_size ) = 0;

    /**
     * @brief Decode a binary stream into the value data.
     *
     * Used when communicating with an external system over a shared format. It's expected
     * that the underlying KV storage `datacache` matches the decoded data type class.
     *
     * @param key       The key to decode
     * @param in_data   Input buffer containing the encoded data
     * @param in_size   Size of the input buffer
     * @return int      Decoded size in bytes, negative on error
     */
    virtual int decode( const HashKey key, const void *in_data, const size_t in_size ) = 0;
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_INTERFACE_HPP */
