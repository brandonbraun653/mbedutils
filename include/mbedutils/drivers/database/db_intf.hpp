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
#include <mbedutils/drivers/database/db_parameter.hpp>


namespace mb::db
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Hashes a string into a 32-bit key
   *
   * @param key String to hash
   * @return HashKey 32-bit hash key
   */
  HashKey hash( etl::string_view &key );

  /**
   * @brief Hashes a binary buffer into a 32-bit key
   *
   * @param key Pointer to the buffer
   * @param size Size of the buffer
   * @return HashKey 32-bit hash key
   */
  HashKey hash( const void *key, const size_t size );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief High level interface to a database instance.
   *
   * This is written from the perspective of an embedded systems database that
   * really is simple key-value storage. The interface is designed to be as
   * generic as possible to allow for different underlying storage mechanisms.
   */
  class IDatabase
  {
  public:
    virtual ~IDatabase() = default;

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
     * @brief Synchronize the local database with the underlying storage, if any.
     */
    virtual void sync() = 0;

    /**
     * @brief If any keys are dirty, flush them to the underlying storage.
     */
    virtual void flush() = 0;

    /**
     * @brief Checks if a key exists in the database
     *
     * @param key     The key to check for
     * @return true   The key exists in the database
     * @return false  The key does not exist in the database
     */
    virtual bool exists( const HashKey key ) = 0;

    /**
     * @brief Read the data associated with a key
     *
     * @param key         The key to read
     * @param data        Buffer to read the data into
     * @param data_size   Size of the data buffer
     * @param size        Number of bytes to read
     * @return int        Error code if negative, number of bytes read if positive
     */
    virtual int read( const HashKey key, void *data, const size_t data_size, const size_t size ) = 0;

    /**
     * @brief Writes data to a key in the database
     *
     * @param key         The key to write to
     * @param data        Data to write
     * @param data_size   Size of the data buffer
     * @param size        Number of bytes to write
     * @return int        Error code if negative, number of bytes written if positive
     */
    virtual int write( const HashKey key, const void *data, const size_t data_size, const size_t size ) = 0;

    /**
     * @brief Erases a key from the database
     *
     * @param key The key to remove
     */
    virtual void remove( const HashKey key ) = 0;
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_INTERFACE_HPP */
