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
#include <mbedutils/database.hpp>
#include <mbedutils/thread.hpp>
#include <mbedutils/system.hpp>
#include <pb_decode.h>
#include <pb_encode.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Private Data
  ---------------------------------------------------------------------------*/

  // static const fdb_default_kv_node s_dflt_kv_table[] = { {
  //     .key       = "mbedutils_db_ver",
  //     .value     = "1.0.0",
  //     .value_len = 6,
  // } };

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  static const char *fdb_err_to_str( const fdb_err_t err )
  {
    switch( err )
    {
      case FDB_NO_ERR:
        return "No error";
      case FDB_ERASE_ERR:
        return "Erase error";
      case FDB_READ_ERR:
        return "Read error";
      case FDB_WRITE_ERR:
        return "Write error";
      case FDB_PART_NOT_FOUND:
        return "Partition not found";
      case FDB_KV_NAME_ERR:
        return "Key-Value name error";
      case FDB_KV_NAME_EXIST:
        return "Key-Value name exists";
      case FDB_SAVED_FULL:
        return "Saved full";
      case FDB_INIT_FAILED:
        return "Initialization failed";
      default:
        return "Unknown error";
    }
  }

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
      if( !node_is_valid( node ) )
      {
        return DB_ERR_BAD_ARG;
      }

      if ( node.pbFields && ( node.dataSize > config.transcode_buffer.max_size() ) )
      {
        return DB_ERR_TRANSCODE_BUFFER_TOO_SMALL;
      }
    }

    mConfig = config;
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
    /*-------------------------------------------------------------------------
    Validate the input node
    -------------------------------------------------------------------------*/
    const bool pbFieldsTooLarge = node.pbFields && ( node.dataSize > mConfig.transcode_buffer.max_size() );
    if( !node_is_valid( node ) || pbFieldsTooLarge )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Safely insert the node
    -------------------------------------------------------------------------*/
    mb::thread::RecursiveLockGuard lock( mRMutex );

    if( exists( node.hashKey ) )
    {
      return false;
    }

    mConfig.node_storage->push_back( node );
    etl::sort( mConfig.node_storage->begin(), mConfig.node_storage->end(),
               []( const KVNode &lhs, const KVNode &rhs ) -> bool { return lhs.hashKey < rhs.hashKey; } );

    return true;
  }


  void RamKVDB::remove( const HashKey key )
  {
    mb::thread::RecursiveLockGuard lock( mRMutex );

    /*-------------------------------------------------------------------------
    Search for the node
    -------------------------------------------------------------------------*/
    auto node = etl::find_if( mConfig.node_storage->begin(), mConfig.node_storage->end(),
                              [ key ]( const KVNode &node ) -> bool { return node.hashKey == key; } );

    /*-------------------------------------------------------------------------
    Remove the node if it exists
    -------------------------------------------------------------------------*/
    if( node != mConfig.node_storage->end() )
    {
      mConfig.node_storage->erase( node );
    }
  }


  KVNode *RamKVDB::find( const HashKey key )
  {
    mb::thread::RecursiveLockGuard lock( mRMutex );

    /*-------------------------------------------------------------------------
    Search for the node
    -------------------------------------------------------------------------*/
    auto node = etl::find_if( mConfig.node_storage->begin(), mConfig.node_storage->end(),
                              [ key ]( const KVNode &node ) -> bool { return node.hashKey == key; } );

    /*-------------------------------------------------------------------------
    Return the node if it exists
    -------------------------------------------------------------------------*/
    if( node != mConfig.node_storage->end() )
    {
      return node;
    }

    return nullptr;
  }


  bool RamKVDB::exists( const HashKey key )
  {
    return this->find( key ) != nullptr;
  }


  void RamKVDB::sync()
  {
    /*-------------------------------------------------------------------------
    No need to sync a RAM database
    -------------------------------------------------------------------------*/
  }


  void RamKVDB::flush()
  {
    /*-------------------------------------------------------------------------
    No need to flush a RAM database
    -------------------------------------------------------------------------*/
  }


  int RamKVDB::read( const HashKey key, void *data, const size_t data_size, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Auto-adjust the read size if necessary
    -------------------------------------------------------------------------*/
    auto read_size = size ? size : data_size;

    /*-------------------------------------------------------------------------
    Search for the node and read it
    -------------------------------------------------------------------------*/
    mb::thread::RecursiveLockGuard lock( mRMutex );
    if( auto node = this->find( key ); node && ( read_size <= data_size ) )
    {
      return node_read( *node, data, read_size );
    }

    return -1;
  }


  int RamKVDB::write( const HashKey key, const void *data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Search for the node and update it
    -------------------------------------------------------------------------*/
    mb::thread::RecursiveLockGuard lock( mRMutex );
    if( auto node = this->find( key ); node  )
    {
      return node_write( *node, data, size ) ? size : -1;
    }

    return -1;
  }


  int RamKVDB::encode( const HashKey key, void *out_data, const size_t out_size )
  {
    /*-------------------------------------------------------------------------
    Search for the node and encode it
    -------------------------------------------------------------------------*/
    mb::thread::RecursiveLockGuard lock( mRMutex );
    if( auto node = this->find( key ); node )
    {
      return node_serialize( *node, out_data, out_size );
    }

    return -1;
  }


  int RamKVDB::decode( const HashKey key, const void *in_data, const size_t in_size )
  {
    /*-------------------------------------------------------------------------
    Search for the node and decode it
    -------------------------------------------------------------------------*/
    mb::thread::RecursiveLockGuard lock( mRMutex );
    if( auto node = this->find( key ); node )
    {
      return node_deserialize( *node, in_data, in_size ) ? in_size : -1;
    }

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


  DBError NvmKVDB::configure( Config &config )
  {
    /*-------------------------------------------------------------------------
    Validate the input configuration
    -------------------------------------------------------------------------*/
    if( config.dev_name.empty() || config.part_name.empty() || config.ram_kvdb == nullptr ||
        ( ( config.dev_sector_size % 32 ) != 0 ) )
    {
      return DB_ERR_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    Store the configuration
    -------------------------------------------------------------------------*/
    mConfig = config;
    return DB_ERR_NONE;
  }


  bool NvmKVDB::init()
  {
    using namespace mb::system::atexit;

    /*-------------------------------------------------------------------------
    Reset the system memory
    -------------------------------------------------------------------------*/
    mDB = {};
    fdb_kvdb_control( &mDB, FDB_KVDB_CTRL_SET_SEC_SIZE, &mConfig.dev_sector_size );

    /*-------------------------------------------------------------------------
    Initialize the NVM database
    -------------------------------------------------------------------------*/
    auto fdb_err = fdb_kvdb_init( &mDB, mConfig.dev_name.c_str(), mConfig.part_name.c_str(), nullptr, this );
    if( fdb_err != FDB_NO_ERR )
    {
      mbed_assert_continue_msg( false, "Failed to initialize the NVM database: %s", fdb_err_to_str( fdb_err ) );
      return false;
    }

    /*-------------------------------------------------------------------------
    Check the integrity of the NVM database
    -------------------------------------------------------------------------*/
    fdb_err = fdb_kvdb_check( &mDB );
    if( fdb_err != FDB_NO_ERR )
    {
      mbed_assert_continue_msg( false, "NVM database integrity failure: %s", fdb_err_to_str( fdb_err ) );
      return false;
    }

    /*-------------------------------------------------------------------------
    Register the teardown function with atexit to ensure the cache is flushed
    -------------------------------------------------------------------------*/
    auto cb_stub = mb::system::atexit::Callback::create<NvmKVDB, &NvmKVDB::flush_on_exit>( *this );
    return mb::system::atexit::registerCallback( cb_stub, NORMAL_PRIORITY );
  }


  void NvmKVDB::deinit()
  {
    // flush
    // de-register atexit call
  }


  bool NvmKVDB::insert( const KVNode &node )
  {
    // Sync the node to the NVM
    return false;
  }


  void NvmKVDB::remove( const HashKey key )
  {
    // Sync the node to the NVM
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
    // Pull all data from the NVM and load it into the RAM cache
  }


  void NvmKVDB::flush()
  {
    // Acquire lock
    this->flush_on_exit();
  }


  int NvmKVDB::read( const HashKey key, void *data, const size_t data_size, const size_t size )
  {
    // Read flag to determine if read from cache or always fetch from NVM
    return -1;
  }


  int NvmKVDB::write( const HashKey key, const void *data, const size_t size )
  {
    // Optionally, sync the node to the NVM
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
