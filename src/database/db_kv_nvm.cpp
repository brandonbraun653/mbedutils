/******************************************************************************
 *  File Name:
 *    db_kv_nvm.cpp
 *
 *  Description:
 *    Non-volatile memory backed KV database
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/crc.h>
#include <etl/to_arithmetic.h>
#include <etl/to_string.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/database.hpp>
#include <mbedutils/logging.hpp>
#include <mbedutils/system.hpp>
#include <mbedutils/thread.hpp>
#include <mbedutils/util.hpp>
#include <nanoprintf.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <fdb_def.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr bool MODULE_TRACE_ENABLED = false;

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

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
   * @brief Helper method to print bytes in hex format.
   *
   * Not meant to be a serious function, just for debugging purposes.
   *
   * @param data Data to format
   * @param len  Number of bytes to format
   * @return char*  Formatted string
   */
  static char *format_array( void *data, size_t len )
  {
    static char data_buffer[ 128 ];

    memset( data_buffer, 0, sizeof( data_buffer ) );
    if( len > sizeof( data_buffer ) )
    {
      len = sizeof( data_buffer );
    }

    auto p8_data            = reinterpret_cast<uint8_t *>( data );
    int  remaining_buf_size = sizeof( data_buffer );
    int  bytes_written      = 0;

    for( size_t i = 0; i < len; i++ )
    {
      auto fmt_size = npf_snprintf( data_buffer + bytes_written, remaining_buf_size, "%02X ", p8_data[ i ] );
      bytes_written += fmt_size;
      remaining_buf_size -= fmt_size;
    }

    return data_buffer;
  }

  /*---------------------------------------------------------------------------
  NvmKVDB Class
  ---------------------------------------------------------------------------*/

  NvmKVDB::NvmKVDB() :
      mDeviceName( "" ), mPartitionName( "" ), mSectorSize( 0 ), mNvmDBReady( ~DRIVER_INITIALIZED_KEY ), mDB( {} )
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
    if( mNvmDBReady == DRIVER_INITIALIZED_KEY )
    {
      return DB_ERR_NOT_AVAILABLE;
    }

    /*-------------------------------------------------------------------------
    Configure the RAM cache components
    -------------------------------------------------------------------------*/
    RamKVDB::Config ram_cfg;
    ram_cfg.ext_node_dsc     = config.ext_node_dsc;
    ram_cfg.ext_transcode_buffer = config.ext_transcode_buffer;

    auto ram_cfg_error = RamKVDB::configure( ram_cfg );
    if( ram_cfg_error != DB_ERR_NONE )
    {
      return ram_cfg_error;
    }

    /*-------------------------------------------------------------------------
    Validate the input configuration
    -------------------------------------------------------------------------*/
    if( config.dev_name.empty() || config.part_name.empty() || ( ( config.dev_sector_size % 32 ) != 0 ) )
    {
      return DB_ERR_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    Store the configuration
    -------------------------------------------------------------------------*/
    mDeviceName    = config.dev_name;
    mPartitionName = config.part_name;
    mSectorSize    = config.dev_sector_size;

    return DB_ERR_NONE;
  }


  bool NvmKVDB::init()
  {
    using namespace mb::system::atexit;

    /*-------------------------------------------------------------------------
    Ensure the driver is not already initialized
    -------------------------------------------------------------------------*/
    if( mNvmDBReady == DRIVER_INITIALIZED_KEY )
    {
      return true;
    }

    /*-------------------------------------------------------------------------
    Ensure the RAM cache is initialized first
    -------------------------------------------------------------------------*/
    if( !RamKVDB::init() )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Reset the system memory
    -------------------------------------------------------------------------*/
    mDB = {};
    fdb_kvdb_control( &mDB, FDB_KVDB_CTRL_SET_SEC_SIZE, &mSectorSize );

    /*-------------------------------------------------------------------------
    Initialize the NVM database
    -------------------------------------------------------------------------*/
    auto fdb_err = fdb_kvdb_init( &mDB, mDeviceName.c_str(), mPartitionName.c_str(), nullptr, this );
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
      fdb_kvdb_deinit( &mDB );
      LOG_DEBUG( "NVM database integrity failure: %s", fdb_err_to_str( fdb_err ) );
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

    mNvmDBReady = DRIVER_INITIALIZED_KEY;
    return true;
  }


  void NvmKVDB::deinit()
  {
    /*-------------------------------------------------------------------------
    Ensure the driver is initialized
    -------------------------------------------------------------------------*/
    if( mNvmDBReady != DRIVER_INITIALIZED_KEY )
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

      auto cb = mb::system::atexit::Callback::create<NvmKVDB, &NvmKVDB::flush_on_exit>( *this );
      mb::system::atexit::unregisterCallback( cb );
    }
    mb::osal::unlockRecursiveMutex( mRMutex );
    RamKVDB::deinit();

    mNvmDBReady = ~DRIVER_INITIALIZED_KEY;
  }


  bool NvmKVDB::insert( const KVNode &node )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mNvmDBReady != DRIVER_INITIALIZED_KEY )
    {
      return false;
    }

    mb::thread::RecursiveLockGuard _lock( mRMutex );

    /*-------------------------------------------------------------------------
    Insert the node into the RAM cache
    -------------------------------------------------------------------------*/
    if( !RamKVDB::insert( node ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Apply the node's insertion policy to NVM
    -------------------------------------------------------------------------*/
    if( node.flags & KV_FLAG_PERSISTENT )
    {
      bool node_exists = this->exists( node.hashKey );

      /*-----------------------------------------------------------------------
      Fail the insertion if the key already exists and policy is set to fail
      -----------------------------------------------------------------------*/
      if( node_exists && ( node.flags & KV_FLAG_INSERT_POLICY_FAIL ) )
      {
        mbed_assert_continue_msg( false, "Key %d already exists", node.hashKey );
        return false;
      }

      /*-----------------------------------------------------------------------
      Write back to NVM as needed
      -----------------------------------------------------------------------*/
      if( !node_exists || ( node_exists && ( node.flags & KV_FLAG_INSERT_POLICY_OVERWRITE ) ) )
      {
        if( write( node.hashKey, node.datacache, node.dataSize ) <= 0 )
        {
          remove( node.hashKey );
          return false;
        }

        if( node.flags & KV_FLAG_CACHE_POLICY_WRITE_BACK )
        {
          this->flush();
        }
      }
      /*-----------------------------------------------------------------------
      Otherwise, update the RAM cache with the NVM data
      -----------------------------------------------------------------------*/
      else if( ( node.flags & KV_FLAG_INSERT_POLICY_PULL ) && !sync_node( node ) )
      {
        remove( node.hashKey );
        return false;
      }
    }

    return true;
  }


  void NvmKVDB::remove( const HashKey key )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mNvmDBReady != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _lock( mRMutex );

    /*-------------------------------------------------------------------------
    Remove the node from the RAM cache
    -------------------------------------------------------------------------*/
    RamKVDB::remove( key );

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
    if( mNvmDBReady != DRIVER_INITIALIZED_KEY )
    {
      return nullptr;
    }

    return RamKVDB::find( key );
  }


  bool NvmKVDB::exists( const HashKey key )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mNvmDBReady != DRIVER_INITIALIZED_KEY )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Find the node in the RAM cache
    -------------------------------------------------------------------------*/
    auto node = RamKVDB::find( key );
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
      mb::thread::RecursiveLockGuard _lock( mRMutex );

      fdb_blob blob;
      blob.buf  = mTranscodeBuffer.data();
      blob.size = mTranscodeBuffer.size();

      auto key_repr = hash_to_fdb_key( key );
      return static_cast<bool>( fdb_kv_get_blob( &mDB, key_repr.c_str(), &blob ) );
    }

    return true;
  }


  void NvmKVDB::sync()
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mNvmDBReady != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _lock( mRMutex );

    /*-------------------------------------------------------------------------
    Sync the entire known RAM cache with NVM
    -------------------------------------------------------------------------*/
    for( auto &node : *mNodeDsc )
    {
      this->sync_node( node );
    }
  }


  void NvmKVDB::sync( const HashKey key )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mNvmDBReady != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::thread::RecursiveLockGuard _lock( mRMutex );

    /*-------------------------------------------------------------------------
    Find the node in the RAM cache
    -------------------------------------------------------------------------*/
    auto node = RamKVDB::find( key );
    if( node == nullptr )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Sync the node
    -------------------------------------------------------------------------*/
    this->sync_node( *node );
  }

  void NvmKVDB::flush()
  {
    mb::thread::RecursiveLockGuard _lock( mRMutex );
    this->flush_on_exit();
  }


  int NvmKVDB::read( const HashKey key, void *data, const size_t data_size, const size_t size )
  {
    const size_t act_read_size = size ? size : data_size;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( ( mNvmDBReady != DRIVER_INITIALIZED_KEY ) || ( data == nullptr ) || ( act_read_size == 0 ) )
    {
      return -1;
    }

    mb::thread::RecursiveLockGuard _lock( mRMutex );

    /*-------------------------------------------------------------------------
    Find the node in the RAM cache
    -------------------------------------------------------------------------*/
    auto node = RamKVDB::find( key );
    if( node == nullptr )
    {
      mbed_assert_continue_msg( false, "Key %d not found", key );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Validate the cache policy is supported
    -------------------------------------------------------------------------*/
    constexpr uint32_t invalid_policy = KV_FLAG_CACHE_POLICY_READ_THROUGH | KV_FLAG_CACHE_POLICY_READ_CACHE;
    if( ( node->flags & invalid_policy ) == invalid_policy )
    {
      mbed_assert_continue_msg( false, "Invalid read policy for key %d", key );
      return -1;
    }

    constexpr uint32_t missing_policy =
        KV_FLAG_CACHE_POLICY_READ_SYNC | KV_FLAG_CACHE_POLICY_READ_THROUGH | KV_FLAG_CACHE_POLICY_READ_CACHE;
    if( ( node->flags & missing_policy ) == 0 )
    {
      mbed_assert_continue_msg( false, "Missing read policy for key %d", key );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Read the data according to the cache policy
    -------------------------------------------------------------------------*/
    int ret_read_size = -1;

    if( ( node->flags & KV_FLAG_PERSISTENT ) &&
        ( node->flags & ( KV_FLAG_CACHE_POLICY_READ_SYNC | KV_FLAG_CACHE_POLICY_READ_THROUGH ) ) )
    {
      /*-----------------------------------------------------------------------
      Read the raw data from NVM into the user's data buffer
      -----------------------------------------------------------------------*/
      mbed_assert( act_read_size <= mTranscodeBuffer.size() );

      fdb_blob blob;
      if( !fdb_kv_get_blob( &mDB, hash_to_fdb_key( key ).c_str(), fdb_blob_make( &blob, data, act_read_size ) ) )
      {
        mbed_assert_continue_msg( false, "Failed to read key %d from NVM", key );
        return -1;
      }

      ret_read_size = blob.saved.len;

      /*-----------------------------------------------------------------------
      Decode if the data was stored with NanoPB serialization. This will update
      the user's data buffer with the decoded data.
      -----------------------------------------------------------------------*/
      if( node->pbFields )
      {
        /* Transcode Buffer Prep */
        mbed_assert( blob.saved.len <= mTranscodeBuffer.size() );
        memset( mTranscodeBuffer.data(), 0, blob.saved.len );

        /* Decode User Data Buffer -> Transcode Buffer */
        auto stream = pb_istream_from_buffer( static_cast<const pb_byte_t *>( blob.buf ), blob.saved.len );
        if( !pb_decode( &stream, node->pbFields, mTranscodeBuffer.data() ) )
        {
          mbed_assert_continue_msg( false, "KVNode %d decode failure: %s", key, stream.errmsg );
          return -1;
        }

        /* Update data references & sizing to ensure later ops have latest information */
        blob.buf       = mTranscodeBuffer.data();
        blob.saved.len = blob.saved.len - stream.bytes_left;

        /* Write decoded data back to the user buffer */
        memcpy( data, blob.buf, blob.saved.len );
        ret_read_size = blob.saved.len;
      }

      /*-----------------------------------------------------------------------
      Sanitize the data in-place before a potential cache write next.
      -----------------------------------------------------------------------*/
      if( ( ret_read_size > 0 ) && ( node->flags & KV_FLAG_SANITIZE_ON_READ ) )
      {
        mbed_assert_continue_msg( node->sanitizer, "Missing sanitize callback for key %d", key );
        if( node->sanitizer )
        {
          node->sanitizer( *node, data, ret_read_size );
        }
      }

      /*-----------------------------------------------------------------------
      Synchronize NVM -> RAM cache if tagged to do so
      -----------------------------------------------------------------------*/
      if( node->flags & KV_FLAG_CACHE_POLICY_READ_SYNC )
      {
        node_write( *node, blob.buf, blob.saved.len );
      }
    }
    else if( node->flags & KV_FLAG_CACHE_POLICY_READ_CACHE )
    {
      ret_read_size = node_read( *node, data, act_read_size );

      /*-----------------------------------------------------------------------
      Sanitize the data in-place
      -----------------------------------------------------------------------*/
      if( ( ret_read_size > 0 ) && ( node->flags & KV_FLAG_SANITIZE_ON_READ ) )
      {
        mbed_assert_continue_msg( node->sanitizer, "Missing sanitize callback for key %d", key );
        if( node->sanitizer )
        {
          node->sanitizer( *node, data, ret_read_size );
        }
      }
    }

    return ret_read_size;
  }


  int NvmKVDB::write( const HashKey key, void *data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( ( mNvmDBReady != DRIVER_INITIALIZED_KEY ) || ( data == nullptr ) || ( size == 0 ) )
    {
      return -1;
    }

    mb::thread::RecursiveLockGuard _lock( mRMutex );

    /*-------------------------------------------------------------------------
    Find the node in the RAM cache
    -------------------------------------------------------------------------*/
    auto node = RamKVDB::find( key );
    if( node == nullptr )
    {
      mbed_assert_continue_msg( false, "Key %d not found", key );
      return -1;
    }

    /*-------------------------------------------------------------------------
    Sanitize the data before writing to NVM?
    -------------------------------------------------------------------------*/
    if( node->flags & KV_FLAG_SANITIZE_ON_WRITE )
    {
      mbed_assert_continue_msg( node->sanitizer, "Missing sanitize callback for key %d", key );
      if( node->sanitizer )
      {
        node->sanitizer( *node, data, size );
      }
    }

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
        node->flags |= KV_FLAG_DIRTY;    // Write on next call to flush()
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
    return RamKVDB::write( key, data, size );
  }


  int NvmKVDB::encode( const HashKey key, void *out_data, const size_t out_size )
  {
    return this->RamKVDB::encode( key, out_data, out_size );
  }


  int NvmKVDB::decode( const HashKey key, const void *in_data, const size_t in_size )
  {
    return this->RamKVDB::decode( key, in_data, in_size );
  }


  void NvmKVDB::flush_on_exit()
  {
    //! No lock acquired here b/c we're assuming an atexit call context.
    /*-------------------------------------------------------------------------
    Flush all dirty nodes to NVM
    -------------------------------------------------------------------------*/
    const auto needs_write_bits = KV_FLAG_DIRTY | KV_FLAG_PERSISTENT;

    for( auto &node : *mNodeDsc )
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
      pb_ostream_t stream = pb_ostream_from_buffer( mTranscodeBuffer.data(), mTranscodeBuffer.max_size() );

      if( !pb_encode( &stream, node.pbFields, data ) )
      {
        mbed_assert_continue_msg( false, "KVNode %d encode failure: %s", node.hashKey, stream.errmsg );
        return -1;
      }

      return write_fdb_blob( node.hashKey, mTranscodeBuffer.data(), stream.bytes_written );
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
    fdb_err_t error    = fdb_kv_set_blob( &mDB, key_repr.data(), fdb_blob_make( &blob, data, size ) );

    if( error != FDB_NO_ERR )
    {
      mbed_assert_continue_msg( false, "Failed to write key %d to NVM: %s", key, fdb_err_to_str( error ) );
      return -1;
    }

    LOG_TRACE_IF( MODULE_TRACE_ENABLED, "Write %d bytes to NVM for key %d: %s", size, key,
                  format_array( blob.buf, blob.size ) );
    return static_cast<int>( size );
  }


  /**
   * @brief Sync a single node from NVM to the RAM cache.
   *
   * It's important to note that this function is not thread safe and should only be
   * called from within a locked context.
   *
   * @param node Which node to sync
   * @return true If the sync was successful, false otherwise
   */
  bool NvmKVDB::sync_node( const KVNode &node )
  {
    if( node.flags & KV_FLAG_PERSISTENT )
    {
      /*-----------------------------------------------------------------------
      Read the raw data from NVM into the user's data buffer
      -----------------------------------------------------------------------*/
      fdb_blob blob;
      if( !fdb_kv_get_blob( &mDB, hash_to_fdb_key( node.hashKey ).c_str(),
                            fdb_blob_make( &blob, mTranscodeBuffer.data(), mTranscodeBuffer.max_size() ) ) )
      {
        mbed_assert_continue_msg( false, "Key %d not present in NVM", node.hashKey );
        return false;
      }

      LOG_TRACE_IF( MODULE_TRACE_ENABLED, "Read %d bytes from NVM for key %d: %s", blob.saved.len, node.hashKey,
                    format_array( blob.buf, blob.saved.len ) );

      /*-----------------------------------------------------------------------
      Decode if the data was stored with NanoPB serialization. This will update
      the user's data buffer with the decoded data.
      -----------------------------------------------------------------------*/
      if( node.pbFields )
      {
        /* Decode User Data Buffer -> Transcode Buffer */
        auto stream = pb_istream_from_buffer( static_cast<const pb_byte_t *>( blob.buf ), blob.saved.len );
        if( !pb_decode( &stream, node.pbFields, node.datacache ) )
        {
          mbed_assert_continue_msg( false, "KVNode %d decode failure: %s", node.hashKey, stream.errmsg );
          return false;
        }
      }
      else
      {
        /* Copy the raw data into the cache */
        node_read( node, blob.buf, blob.saved.len );
      }

      return true;
    }
    else
    {
      return false;
    }
  }
}    // namespace mb::db
