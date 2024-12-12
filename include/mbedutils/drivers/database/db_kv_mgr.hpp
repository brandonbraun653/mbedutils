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
  Forward Declarations
  ---------------------------------------------------------------------------*/

  class NvmKVDB;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Helper struct to declare storage data for a RAM based KVDB
   *
   * @tparam N Number of KV pairs to manage (elements)
   * @tparam M Size of the transcode buffer (bytes)
   */
  template<size_t N, size_t M>
  struct Storage
  {
    KVNodeVector<N>        node_dsc;         /**< Storage for KV pair descriptors */
    etl::array<uint8_t, M> transcode_buffer; /**< Storage for encoding/decoding largest data */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief RAM based Key-Value database with no persistent storage.
   */
  class RamKVDB : public virtual IKVDatabase
  {
  public:
    /**
     * @brief Configuration data for the RAM based KVDB
     *
     * This should be built via binding to the Storage struct members.
     */
    struct Config
    {
      KVNodeIVector     *ext_node_dsc;         /**< External storage of parameter nodes */
      etl::span<uint8_t> ext_transcode_buffer; /**< RAM buffer for encoding/decoding KV pair data */
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
    void visit_dirty_nodes( VisitorFunc &visitor );

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
    void    sync( const HashKey key ) override;
    void    flush() override;
    int     read( const HashKey key, void *data, const size_t data_size, const size_t size = 0 ) override;
    int     write( const HashKey key, void *data, const size_t size ) override;
    int     encode( const HashKey key, void *out_data, const size_t out_size ) override;
    int     decode( const HashKey key, const void *in_data, const size_t in_size ) override;

  protected:
    friend class NvmKVDB;

    mb::osal::mb_recursive_mutex_t mRMutex;          /**< Mutex for thread safety */
    KVNodeIVector                 *mNodeDsc;         /**< External storage of parameter nodes */
    etl::span<uint8_t>             mTranscodeBuffer; /**< RAM buffer for encoding/decoding KV pair data */

  private:
    size_t mRamDBReady; /**< Flag to indicate the manager is ready */
  };

  /**
   * @brief A non-volatile memory backed key-value database with RAM caching.
   *
   * This database is designed to be used with a flash memory device that
   * supports the FAL interface. For fast access, the KV data is cached in
   * RAM and only written to the flash memory when necessary.
   */
  class NvmKVDB : public RamKVDB
  {
  public:
    using FALString = etl::string<FAL_DEV_NAME_MAX>;

    /**
     * @brief Configuration data for the NVM based KVDB
     *
     * This should be built via binding to the Storage struct members.
     */
    struct Config
    {
      FALString          dev_name;             /**< Which device is being accessed */
      FALString          part_name;            /**< Sub-partition being accessed */
      uint32_t           dev_sector_size;      /**< Size of a sector on the device */
      KVNodeIVector     *ext_node_dsc;         /**< External storage of parameter nodes */
      etl::span<uint8_t> ext_transcode_buffer; /**< RAM buffer for encoding/decoding KV pair data */
    };

    NvmKVDB();
    ~NvmKVDB();

    /**
     * @brief Bind the provided configuration to the database
     *
     * @param config  Configuration data for the database
     * @return Error code indicating success or failure
     */
    DBError configure( Config &config );

    /*-------------------------------------------------------------------------
    IKVDatabase Interface
    -------------------------------------------------------------------------*/
    bool    init() final override;
    void    deinit() final override;
    bool    insert( const KVNode &node ) final override;
    void    remove( const HashKey key ) final override;
    KVNode *find( const HashKey key ) final override;
    bool    exists( const HashKey key ) final override;
    void    sync() final override;
    void    sync( const HashKey key ) final override;
    void    flush() final override;
    int     read( const HashKey key, void *data, const size_t data_size, const size_t size = 0 ) final override;
    int     write( const HashKey key, void *data, const size_t size ) final override;
    int     encode( const HashKey key, void *out_data, const size_t out_size ) final override;
    int     decode( const HashKey key, const void *in_data, const size_t in_size ) final override;

  protected:
    /**
     * @brief System atexit handler to ensure the cache is flushed before exit
     */
    void flush_on_exit();

  private:
    FALString mDeviceName;    /**< Which device is being accessed */
    FALString mPartitionName; /**< Sub-partition being accessed */
    uint32_t  mSectorSize;    /**< Size of a sector on the device */
    size_t    mNvmDBReady;    /**< Flag to indicate the manager is ready */

    /*-------------------------------------------------------------------------
    FlashDB Management. This declaration is a bit odd. For some reason, the
    gcc-arm-none-eabi compiler won't properly link the structure, causing the
    start address of the next variable in the map file == &mDB.user_data.
    This does not happen with your typical x86_64 compiler and a large amount
    of investigation didn't reveal the root cause. To get around this, I've
    provided a 4 byte padding to ensure the structure is properly aligned for
    whatever data is next in the map file. This is a hack, but it works since I
    don't control the FlashDB library.
    -------------------------------------------------------------------------*/
    fdb_kvdb mDB; /**< FlashDB state management for NVM */
    alignas( 4 ) uint32_t _pad;

    int  write_fdb_node( const KVNode &node, const void *data, const size_t size );
    int  write_fdb_blob( const HashKey key, const void *data, const size_t size );
    bool sync_node( const KVNode &node );
  };
}    // namespace mb::db

#endif /* !MBEDUTILS_KEY_VALUE_DATABASE_HPP */
