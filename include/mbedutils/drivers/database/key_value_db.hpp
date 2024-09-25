/******************************************************************************
 *  File Name:
 *    key_value_db.hpp
 *
 *  Description:
 *    Key-Value database interfaces
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_KEY_VALUE_DATABASE_HPP
#define MBEDUTILS_KEY_VALUE_DATABASE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/array.h>
#include <etl/list.h>
#include <etl/span.h>
#include <etl/string.h>
#include <mbedutils/drivers/database/db_intf.hpp>

extern "C"
{
#include <fal.h>
#include <flashdb.h>
}

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Helper struct to declare storage data for a persistent KVDB
   *
   * @tparam N Number of KV pairs to manage (elements)
   * @tparam M Size of the transcode buffer (bytes)
   */
  template<size_t N, size_t M>
  struct PersistenKVDBStorage
  {
    KVParamStorageManager  param_manager;          /**< RAM manager for KV pair cache */
    KVParamNodeVector<N>   param_nodes;            /**< Storage for KV pair descriptors */
    etl::array<uint8_t, M> param_transcode_buffer; /**< Storage for encoding/decoding largest data */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief A flash memory backed key-value database with RAM caching.
   *
   * This database is designed to be used with a flash memory device that
   * supports the FAL interface. The database will cache all key-value pairs
   * in RAM to reduce the number of flash writes. The database will also
   * automatically flush the cache as needed to ensure consistency between
   * the RAM and flash storage.
   */
  class PersistentKVDB : public virtual IDatabase
  {
  public:
    struct Config
    {
      etl::string<FAL_DEV_NAME_MAX> dev_name;               /**< Which device is being accessed */
      etl::string<FAL_DEV_NAME_MAX> partition_name;         /**< Sub-partition being accessed */
      KVParamStorageManager        *param_manager;          /**< RAM manager for KV pairs located on the device/partition */
      KVParamNodeIVector           *param_nodes;            /**< RAM storage for KV pairs */
      etl::span<uint8_t>            param_transcode_buffer; /**< RAM buffer for encoding/decoding KV pair data */
    };

    PersistentKVDB();
    ~PersistentKVDB();

    /**
     * @brief Bind the provided configuration to the database
     *
     * @param config  Configuration data for the database
     * @return true   The database was initialized successfully
     * @return false  The database failed to initialize
     */
    bool configure( Config &config );

    /*-------------------------------------------------------------------------
    IDatabase Interface
    -------------------------------------------------------------------------*/
    bool init() override;
    void deinit() override;
    void sync() override;
    void flush() override;
    bool exists( const HashKey key ) override;
    int read( const HashKey key, void *data, const size_t data_size, const size_t size ) override;
    int write( const HashKey key, const void *data, const size_t data_size, const size_t size ) override;
    void remove( const HashKey key ) override;

  protected:

    /**
     * @brief System atexit handler to ensure the cache is flushed before exit
     */
    void flush_on_exit();

  private:
    fdb_kvdb mDB;
    Config   mConfig;

    // Add a reader/writer lock here to protect the cache? Maybe...
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_KEY_VALUE_DATABASE_HPP */
