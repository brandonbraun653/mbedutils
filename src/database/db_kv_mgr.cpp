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
#include <etl/to_arithmetic.h>
#include <etl/to_string.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/database.hpp>
#include <mbedutils/system.hpp>
#include <mbedutils/thread.hpp>
#include <mbedutils/util.hpp>
#include <pb_decode.h>
#include <pb_encode.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief String representation of a hash key
   *
   * This is used to store the hash key in a human readable format and for
   * literal keying in the NVM backed database w/FlashDB. This is the hex
   * string the 32-bit hash key.
   */
  using HashRepr = etl::string<8>;

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Convert an FDB error code to a string
   *
   * @param err Error to convert
   * @return const char*
   */
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

  /**
   * @brief Convert the mbedutils key to an FDB key
   *
   * @param key   Hash key to convert
   * @return HashRepr
   */
  static inline HashRepr hash_to_fdb_key( const HashKey key )
  {
    static_assert( sizeof( HashKey ) == 4, "HashKey is not 32-bits" );

    /* Format to 8 character hex */
    etl::format_spec format;
    format.hex().width( 8 ).fill( '0' );

    /* Transform */
    HashRepr repr;
    etl::to_string( key, repr, format );

    return repr;
  }

  /**
   * @brief Convert the FDB key to the mbedutils key
   *
   * @param key Key to convert
   * @return HashKey
   */
  static inline HashKey fdb_key_to_hash( const HashRepr &key )
  {
    return etl::to_arithmetic<HashKey>( key, etl::hex );
  }

  /*---------------------------------------------------------------------------
  RamKVDB Class
  ---------------------------------------------------------------------------*/

  RamKVDB::RamKVDB() : mRMutex( nullptr ), mConfig(), mInitialized( ~DRIVER_INITIALIZED_KEY )
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
    if( mInitialized == DRIVER_INITIALIZED_KEY )
    {
      return DB_ERR_NOT_AVAILABLE;
    }

    /* Simple sizing/existence tests */
    if( config.node_storage == nullptr || config.transcode_buffer.empty() )
    {
      return DB_ERR_BAD_ARG;
    }

    /* Make sure the transcode buffer can handle all known encode/decode ops */
    for( auto &node : *config.node_storage )
    {
      if( !node_is_valid( node ) )
      {
        return DB_ERR_BAD_ARG;
      }

      if( node.pbFields && ( node.dataSize > config.transcode_buffer.max_size() ) )
      {
        return DB_ERR_TRANSCODE_BUFFER_TOO_SMALL;
      }
    }

    mConfig = config;
    return DB_ERR_NONE;
  }


  bool RamKVDB::init()
  {
    if( mInitialized == DRIVER_INITIALIZED_KEY )
    {
      return true;
    }

    if( !mRMutex && !mb::osal::buildRecursiveMutexStrategy( mRMutex ) )
    {
      mbed_assert_continue_msg( false, "Not enough resources to init RamKVDB" );
      return false;
    }

    mInitialized = DRIVER_INITIALIZED_KEY;
    return true;
  }


  void RamKVDB::deinit()
  {
    if( mInitialized != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::osal::destroyRecursiveMutex( mRMutex );
    mInitialized = ~DRIVER_INITIALIZED_KEY;
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
    if( auto node = this->find( key ); node )
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

  NvmKVDB::NvmKVDB() : mRMutex( nullptr ), mDB( {} ), mConfig( {} ), mInitialized( ~DRIVER_INITIALIZED_KEY )
  {
  }


  NvmKVDB::~NvmKVDB()
  {
  }


  DBError NvmKVDB::configure( Config &config )
  {
    /*-------------------------------------------------------------------------
    Ensure the driver is not already initialized
    -------------------------------------------------------------------------*/
    if( mInitialized == DRIVER_INITIALIZED_KEY )
    {
      return DB_ERR_NOT_AVAILABLE;
    }

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
    Ensure the driver is not already initialized
    -------------------------------------------------------------------------*/
    if( mInitialized == DRIVER_INITIALIZED_KEY )
    {
      return true;
    }

    /*-------------------------------------------------------------------------
    Validate the data configuration. If this is failing, the consumer hasn't
    called configure() successfully yet.
    -------------------------------------------------------------------------*/
    if( mConfig.ram_kvdb == nullptr )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Ensure the RAM cache is initialized first
    -------------------------------------------------------------------------*/
    if( !mConfig.ram_kvdb->init() )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Allocate the recursive mutex lock
    -------------------------------------------------------------------------*/
    if( !mRMutex && !mb::osal::buildRecursiveMutexStrategy( mRMutex ) )
    {
      mbed_assert_continue_msg( false, "Not enough resources to init NvmKVDB" );
      return false;
    }

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
    if( !mb::system::atexit::registerCallback( cb_stub, NORMAL_PRIORITY ) )
    {
      mbed_assert_continue_msg( false, "Failed to register atexit callback" );
      return false;
    }

    mInitialized = DRIVER_INITIALIZED_KEY;
    return true;
  }


  void NvmKVDB::deinit()
  {
    /*-------------------------------------------------------------------------
    Ensure the driver is initialized
    -------------------------------------------------------------------------*/
    if( mInitialized != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Perform destruction in a thread safe manner
    -------------------------------------------------------------------------*/
    mb::osal::lockRecursiveMutex( mRMutex );
    {
      this->flush();
      fdb_kvdb_deinit( &mDB );
      mConfig.ram_kvdb->deinit();

      auto cb = mb::system::atexit::Callback::create<NvmKVDB, &NvmKVDB::flush_on_exit>( *this );
      mb::system::atexit::unregisterCallback( cb );
    }
    mb::osal::unlockRecursiveMutex( mRMutex );
    mb::osal::destroyRecursiveMutex( mRMutex );

    mInitialized = ~DRIVER_INITIALIZED_KEY;
  }


  bool NvmKVDB::insert( const KVNode &node )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mInitialized != DRIVER_INITIALIZED_KEY )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _nvm_lock( mRMutex );

    /*-------------------------------------------------------------------------
    Insert the node into the RAM cache
    -------------------------------------------------------------------------*/
    if( !mConfig.ram_kvdb->insert( node ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Sync the node to flash if it's persistent. To ensure consistency, remove
    the node if the write fails.
    -------------------------------------------------------------------------*/
    if( ( node.flags & KV_FLAG_PERSISTENT ) &&
        ( 0 >= this->write( node.hashKey, node.datacache, node.dataSize ) ) )
    {
      this->remove( node.hashKey );
      return false;
    }

    /*-------------------------------------------------------------------------
    Flush the cache if necessary. Write back policies won't actually create the
    node in NVM until the cache is flushed.
    -------------------------------------------------------------------------*/
    if( node.flags & KV_FLAG_CACHE_POLICY_WRITE_BACK )
    {
      this->flush();
    }

    return true;
  }


  void NvmKVDB::remove( const HashKey key )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mInitialized != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _nvm_lock( mRMutex );
    mb::thread::RecursiveLockGuard _ram_lock( mConfig.ram_kvdb->mRMutex );

    /*-------------------------------------------------------------------------
    Remove the node from the RAM cache
    -------------------------------------------------------------------------*/
    mConfig.ram_kvdb->remove( key );

    /*-------------------------------------------------------------------------
    Remove the node from the NVM cache
    -------------------------------------------------------------------------*/
    auto key_repr = hash_to_fdb_key( key );
    auto error    = fdb_kv_del( &mDB, key_repr.c_str() );

    mbed_assert_continue_msg( error == FDB_NO_ERR, "FDB remove key %d failure: %s", key, fdb_err_to_str( error ) );
  }


  KVNode *NvmKVDB::find( const HashKey key )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mInitialized != DRIVER_INITIALIZED_KEY )
    {
      return nullptr;
    }

    return mConfig.ram_kvdb->find( key );
  }


  bool NvmKVDB::exists( const HashKey key )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mInitialized != DRIVER_INITIALIZED_KEY )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Find the node in the RAM cache
    -------------------------------------------------------------------------*/
    auto node = mConfig.ram_kvdb->find( key );
    if( !node )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Check the NVM cache for the node. This is the strong consistency check.
    We have to provide some type of storage and since we don't want to actually
    read the data into the RAM cache, we'll have to use the transcode buffer
    and introduce a synchronization point.
    -------------------------------------------------------------------------*/
    if( node->flags & KV_FLAG_PERSISTENT )
    {
      mb::thread::RecursiveLockGuard _nvm_lock( mRMutex );

      fdb_blob blob;
      blob.buf  = mConfig.ram_kvdb->mConfig.transcode_buffer.data();
      blob.size = mConfig.ram_kvdb->mConfig.transcode_buffer.size();

      auto key_repr = hash_to_fdb_key( key );
      return static_cast<bool>( fdb_kv_get_blob( &mDB, key_repr.c_str(), &blob ) );
    }

    return true;
  }


  void NvmKVDB::sync()
  {
    // Pull all data from the NVM and load it into the RAM cache
  }


  void NvmKVDB::flush()
  {
    mb::thread::RecursiveLockGuard _nvm_lock( mRMutex );
    mb::thread::RecursiveLockGuard _ram_lock( mConfig.ram_kvdb->mRMutex );
    this->flush_on_exit();
  }


  int NvmKVDB::read( const HashKey key, void *data, const size_t data_size, const size_t size )
  {
    const size_t act_read_size = size ? size : data_size;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( ( mInitialized != DRIVER_INITIALIZED_KEY ) || ( data == nullptr ) || ( act_read_size == 0 ) )
    {
      return -1;
    }

    mb::thread::RecursiveLockGuard _nvm_lock( mRMutex );
    mb::thread::RecursiveLockGuard _ram_lock( mConfig.ram_kvdb->mRMutex );

    /*-------------------------------------------------------------------------
    Find the node in the RAM cache
    -------------------------------------------------------------------------*/
    auto node = mConfig.ram_kvdb->find( key );
    if( node == nullptr )
    {
      mbed_assert_continue_msg( false, "Key %d not found", key );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Read the data according to the cache policy
    -------------------------------------------------------------------------*/
    int ret_read_size = 0;

    if( node->flags & KV_FLAG_PERSISTENT &&
        ( node->flags & ( KV_FLAG_CACHE_POLICY_READ_SYNC | KV_FLAG_CACHE_POLICY_READ_THROUGH ) ) )
    {
      /*-----------------------------------------------------------------------
      Read the data from NVM
      -----------------------------------------------------------------------*/
      fdb_blob blob;
      if( !node->pbFields )
      {
        blob.buf  = data;
        blob.size = act_read_size;
      }
      else
      {
        mbed_assert( act_read_size <= mConfig.ram_kvdb->mConfig.transcode_buffer.size() );
        blob.buf  = mConfig.ram_kvdb->mConfig.transcode_buffer.data();
        blob.size = act_read_size;
      }

      HashRepr key_repr = hash_to_fdb_key( key );
      if( !fdb_kv_get_blob( &mDB, key_repr.c_str(), &blob ) )
      {
        mbed_assert_continue_msg( false, "Failed to read key %d from NVM", key );
        return -1;
      }

      /*-----------------------------------------------------------------------
      Decode the data if necessary
      -----------------------------------------------------------------------*/
      if( node->pbFields )
      {
        pb_istream_t stream = pb_istream_from_buffer( static_cast<const pb_byte_t *>( blob.buf ), blob.saved.len );
        if( !pb_decode( &stream, node->pbFields, data ) )
        {
          mbed_assert_continue_msg( false, "KVNode %d decode failure: %s", key, stream.errmsg );
          return -1;
        }
      }

      /*-----------------------------------------------------------------------
      Synchronize the RAM cache with NVM if requested
      -----------------------------------------------------------------------*/
      if( node->flags & KV_FLAG_CACHE_POLICY_READ_SYNC )
      {
        node_write( *node, blob.buf, blob.saved.len );
      }

      ret_read_size = blob.saved.len;
    }

    if( node->flags & KV_FLAG_CACHE_POLICY_READ_CACHE )
    {
      ret_read_size = node_read( *node, data, act_read_size );
    }

    /*-------------------------------------------------------------------------
    Auto-validate the data?
    -------------------------------------------------------------------------*/
    // TODO

    return ret_read_size;
  }


  int NvmKVDB::write( const HashKey key, const void *data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( ( mInitialized != DRIVER_INITIALIZED_KEY ) || ( data == nullptr ) || ( size == 0 ) )
    {
      return -1;
    }

    mb::thread::RecursiveLockGuard _nvm_lock( mRMutex );
    mb::thread::RecursiveLockGuard _ram_lock( mConfig.ram_kvdb->mRMutex );

    /*-------------------------------------------------------------------------
    Find the node in the RAM cache
    -------------------------------------------------------------------------*/
    auto node = mConfig.ram_kvdb->find( key );
    if( node == nullptr )
    {
      mbed_assert_continue_msg( false, "Key %d not found", key );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Auto-validate the data?
    -------------------------------------------------------------------------*/
    // TODO: Add data validation node call

    /*-------------------------------------------------------------------------
    Commit the data to NVM first to ensure consistency
    -------------------------------------------------------------------------*/
    if( node->flags & KV_FLAG_PERSISTENT )
    {
      if( node->flags & KV_FLAG_CACHE_POLICY_WRITE_THROUGH )
      {
        auto nvm_size = write_fdb_node( *node, data, size );
        if( nvm_size < 0 )
        {
          mbed_assert_continue_msg( false, "Failed to write key %d to NVM", key );
          return -1;
        }
      }
      else if( node->flags & KV_FLAG_CACHE_POLICY_WRITE_BACK )
      {
        node->flags |= KV_FLAG_DIRTY; // Write on next call to flush()
      }
      else
      {
        mbed_assert_continue_msg( false, "Missing write policy for key %d", key );
        return -1;
      }
    }

    /*-------------------------------------------------------------------------
    Update the RAM cache second now that it's likely data was committed.
    -------------------------------------------------------------------------*/
    return mConfig.ram_kvdb->write( key, data, size );
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
    //! No lock acquired here b/c we're assuming an atexit call context.
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !mConfig.ram_kvdb->mConfig.node_storage )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Flush all dirty nodes to NVM
    -------------------------------------------------------------------------*/
    const auto needs_write_bits = KV_FLAG_DIRTY | KV_FLAG_PERSISTENT;

    for( auto &node : *mConfig.ram_kvdb->mConfig.node_storage )
    {
      if( ( node.flags & needs_write_bits ) == needs_write_bits )
      {
        auto size = write_fdb_node( node, node.datacache, node.dataSize );
        mbed_assert_continue_msg( size >= 0, "Flush key %d to NVM failure", node.hashKey );
      }
    }
  }


  /**
   * @brief Writes data to the FlashDB database, encoding through NanoPB it if necessary.
   *
   * This allows for the most optimal storage format to be used. Sometimes compresssion or
   * other size reduction techniques can be applied to the data before writing.
   *
   * @param node  Node encoding information
   * @param data  Data to write
   * @param size  Size of the data to write
   * @return int  Number of bytes written if successful, negative error code otherwise
   */
  int NvmKVDB::write_fdb_node( const KVNode &node, const void *data, const size_t size )
  {
    if( node.pbFields )
    {
      pb_ostream_t stream = pb_ostream_from_buffer( mConfig.ram_kvdb->mConfig.transcode_buffer.data(),
                                                    mConfig.ram_kvdb->mConfig.transcode_buffer.max_size() );

      if( !pb_encode( &stream, node.pbFields, data ) )
      {
        mbed_assert_continue_msg( false, "KVNode %d encode failure: %s", node.hashKey, stream.errmsg );
        return -1;
      }

      return write_fdb_blob( node.hashKey, mConfig.ram_kvdb->mConfig.transcode_buffer.data(), stream.bytes_written );
    }
    else
    {
      return write_fdb_blob( node.hashKey, data, size );
    }
  }


  /**
   * @brief Writes a blob of data to the NVM database through the FlashDB layer
   *
   * @param key   Which key to write
   * @param data  Data to write
   * @param size  Size of the data to write
   * @return int  Number of bytes written if successful, negative error code otherwise
   */
  int NvmKVDB::write_fdb_blob( const HashKey key, const void *data, const size_t size )
  {
    fdb_blob  blob;
    HashRepr  key_repr = hash_to_fdb_key( key );
    fdb_err_t error    = fdb_kv_set_blob( &mDB, key_repr.c_str(), fdb_blob_make( &blob, data, size ) );

    if( error != FDB_NO_ERR )
    {
      mbed_assert_continue_msg( false, "Failed to write key %d to NVM: %s", key, fdb_err_to_str( error ) );
      return -1;
    }

    return static_cast<int>( size );
  }
}    // namespace mb::db
