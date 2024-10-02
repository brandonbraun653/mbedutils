/******************************************************************************
 *  File Name:
 *    db_kv_mgr.cpp
 *
 *  Description:
 *    Key-Value database manager implementation classes
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/database/db_kv_mgr.hpp>
#include <pb_encode.h>
#include <pb_decode.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  RamKVDB Class
  ---------------------------------------------------------------------------*/

  RamKVDB::RamKVDB()
  {
  }


  RamKVDB::~RamKVDB()
  {
  }


  DBError RamKVDB::configure( Config &config )
  {
    /*-------------------------------------------------------------------------
    Validate the input configuration
    -------------------------------------------------------------------------*/
    /* Simple sizing/existence tests */
    if ( config.node_storage == nullptr || config.transcode_buffer.empty() )
    {
      return DB_ERR_BAD_ARG;
    }

    /* Make sure the transcode buffer can handle all known encode/decode ops */
    for ( auto &node : *config.node_storage )
    {
      if ( node.pbFields && ( node.dataSize > config.transcode_buffer.max_size() ) )
      {
        return DB_ERR_TRANSCODE_BUFFER_TOO_SMALL;
      }
    }

    return DB_ERR_NONE;
  }


  bool RamKVDB::init()
  {
    /*-------------------------------------------------------------------------
    Integration requirement. Nothing needs to occur.
    -------------------------------------------------------------------------*/
    return true;
  }


  void RamKVDB::deinit()
  {
    /*-------------------------------------------------------------------------
    Integration requirement. Nothing needs to occur.
    -------------------------------------------------------------------------*/
  }


  bool RamKVDB::insert( const KVNode &node )
  {
    // Check transcode buffer size
    return false;
  }


  void RamKVDB::remove( const HashKey key )
  {
  }


  KVNode *RamKVDB::find( const HashKey key )
  {
    return nullptr;
  }


  bool RamKVDB::exists( const HashKey key )
  {
    return false;
  }


  void RamKVDB::sync()
  {
  }


  void RamKVDB::flush()
  {
  }


  int RamKVDB::read( const HashKey key, void *data, const size_t data_size, const size_t size )
  {
    return -1;
  }


  int RamKVDB::write( const HashKey key, const void *data, const size_t data_size, const size_t size )
  {
    return -1;
  }


  int RamKVDB::encode( const HashKey key, void *out_data, const size_t out_size )
  {
    return -1;
  }


  int RamKVDB::decode( const HashKey key, const void *in_data, const size_t in_size )
  {
    return -1;
  }

  /*---------------------------------------------------------------------------
  NvmKVDB Class
  ---------------------------------------------------------------------------*/

  NvmKVDB::NvmKVDB() : mDB( {} ), mConfig( {} )
  {
  }


  NvmKVDB::~NvmKVDB()
  {
  }


  bool NvmKVDB::configure( Config &config )
  {
    return false;
  }


  bool NvmKVDB::init()
  {
    // FDB_NO_ERR == fdb_kvdb_init( &mDB, config.dev_name.c_str(), config.partition_name.c_str(), &config.default_kv_table, this );
    // Pull information from flash memory
    // register atexit call

    return false;
  }


  void NvmKVDB::deinit()
  {
    // flush
    // de-register atexit call
  }


  bool NvmKVDB::insert( const KVNode &node )
  {
    return false;
  }


  void NvmKVDB::remove( const HashKey key )
  {
  }


  KVNode *NvmKVDB::find( const HashKey key )
  {
    return nullptr;
  }


  bool NvmKVDB::exists( const HashKey key )
  {
    return false;
  }


  void NvmKVDB::sync()
  {
  }


  void NvmKVDB::flush()
  {
    // Acquire lock
    this->flush_on_exit();
  }


  int NvmKVDB::read( const HashKey key, void *data, const size_t data_size, const size_t size )
  {
    return -1;
  }


  int NvmKVDB::write( const HashKey key, const void *data, const size_t data_size, const size_t size )
  {
    return -1;
  }


  int NvmKVDB::encode( const HashKey key, void *out_data, const size_t out_size )
  {
    return -1;
  }


  int NvmKVDB::decode( const HashKey key, const void *in_data, const size_t in_size )
  {
    return -1;
  }


  void NvmKVDB::flush_on_exit()
  {
    // Likely don't need to acquire a lock here
  }

}  // namespace mb::db
