/******************************************************************************
 *  File Name:
 *    db_kv_mgr.hpp
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
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Helper struct to declare storage data for a RAM based KVDB
   *
   * @tparam N Number of KV pairs to manage (elements)
   * @tparam M Size of the transcode buffer (bytes)
   */
  template<size_t N, size_t M>
  struct RamKVDBStorage
  {
    KVNodeVector<N>        nodes;            /**< Storage for KV pair descriptors */
    etl::array<uint8_t, M> transcode_buffer; /**< Storage for encoding/decoding largest data */
  };

  class RamKVDB : public virtual IKVDatabase
  {
  public:
    struct Config
    {
      KVNodeIVector     *node_storage;     /**< External storage of parameter nodes */
      etl::span<uint8_t> transcode_buffer; /**< RAM buffer for encoding/decoding KV pair data */
    };

    RamKVDB();
    ~RamKVDB();

    /**
     * @brief Initialize the manager
     *
     * @param params Base storage for the parameter nodes (persistent)
     * @return Error code indicating success or failure
     */
    DBError configure( Config &params );

    /**
     * @brief Applies a visitor function to all dirty nodes.
     *
     * @param visitor Function to call on each dirty node
     */
    void visit_dirty_nodes( NodeVisitorFunc &visitor );

    /*-------------------------------------------------------------------------
    IKVDatabase Interface
    -------------------------------------------------------------------------*/
    bool    init() override;
    void    deinit() override;
    bool    insert( const KVNode &node ) override;
    void    remove( const HashKey key ) override;
    KVNode *find( const HashKey key ) override;
    bool    exists( const HashKey key ) override;
    void    sync() override;
    void    flush() override;
    int     read( const HashKey key, void *data, const size_t data_size, const size_t size ) override;
    int     write( const HashKey key, const void *data, const size_t data_size, const size_t size ) override;
    int     encode( const HashKey key, void *out_data, const size_t out_size ) override;
    int     decode( const HashKey key, const void *in_data, const size_t in_size ) override;

  private:
    mb::osal::mb_recursive_mutex_t mRMutex; /**< Mutex for thread safety */
    Config                         mConfig; /**< Configuration data for the instance */
  };


/**
   * @brief Helper struct to declare storage data for a persistent KVDB
   *
   * @tparam N Number of KV pairs to manage (elements)
   * @tparam M Size of the transcode buffer (bytes)
   */
  template<size_t N, size_t M>
  struct NvmKVDBStorage
  {
    RamKVDB                ramdb;          /**< RAM manager for KV pair cache */
    KVNodeVector<N>        nodes;            /**< Storage for KV pair descriptors */
    etl::array<uint8_t, M> transcode_buffer; /**< Storage for encoding/decoding largest data */
  };

  /**
   * @brief A non-volatile memory backed key-value database with RAM caching.
   *
   * This database is designed to be used with a flash memory device that
   * supports the FAL interface. For fast access, the KV data is cached in
   * RAM and only written to the flash memory when necessary.
   */
  class NvmKVDB : public virtual IKVDatabase
  {
  public:
    struct Config
    {
      etl::string<FAL_DEV_NAME_MAX> dev_name;         /**< Which device is being accessed */
      etl::string<FAL_DEV_NAME_MAX> partition_name;   /**< Sub-partition being accessed */
      RamKVDB                      *manager;          /**< RAM manager for KV pairs located on the device/partition */
      KVNodeIVector                *nodes;            /**< RAM storage for KV pairs */
      etl::span<uint8_t>            transcode_buffer; /**< RAM buffer for encoding/decoding KV pair data */
    };

    NvmKVDB();
    ~NvmKVDB();

    /**
     * @brief Bind the provided configuration to the database
     *
     * @param config  Configuration data for the database
     * @return true   The database was initialized successfully
     * @return false  The database failed to initialize
     */
    bool configure( Config &config );

    /*-------------------------------------------------------------------------
    IKVDatabase Interface
    -------------------------------------------------------------------------*/
    bool    init() override;
    void    deinit() override;
    bool    insert( const KVNode &node ) override;
    void    remove( const HashKey key ) override;
    KVNode *find( const HashKey key ) override;
    bool    exists( const HashKey key ) override;
    void    sync() override;
    void    flush() override;
    int     read( const HashKey key, void *data, const size_t data_size, const size_t size ) override;
    int     write( const HashKey key, const void *data, const size_t data_size, const size_t size ) override;
    int     encode( const HashKey key, void *out_data, const size_t out_size ) override;
    int     decode( const HashKey key, const void *in_data, const size_t in_size ) override;

  protected:
    /**
     * @brief System atexit handler to ensure the cache is flushed before exit
     */
    void flush_on_exit();

  private:
    mb::osal::mb_recursive_mutex_t mRMutex; /**< Mutex for thread safety */
    fdb_kvdb                       mDB;     /**< FlashDB management layer */
    Config                         mConfig; /**< Instance configuration data */
  };
}    // namespace mb::db

#endif /* !MBEDUTILS_KEY_VALUE_DATABASE_HPP */
