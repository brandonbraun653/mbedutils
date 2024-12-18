/******************************************************************************
 *  File Name:
 *    db_kv_ram.cpp
 *
 *  Description:
 *    RAM based KV storage
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "mbedutils/drivers/database/db_kv_util.hpp"
#include <etl/crc.h>
#include <etl/to_arithmetic.h>
#include <etl/to_string.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/database.hpp>
#include <mbedutils/logging.hpp>
#include <mbedutils/system.hpp>
#include <mbedutils/threading.hpp>
#include <mbedutils/util.hpp>
#include <nanoprintf.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <fdb_def.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
    RamKVDB Class
    ---------------------------------------------------------------------------*/

  RamKVDB::RamKVDB() : mRMutex( nullptr ), mNodeDsc( nullptr ), mRamDBReady( ~DRIVER_INITIALIZED_KEY )
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
    if( mRamDBReady == DRIVER_INITIALIZED_KEY )
    {
      return DB_ERR_NOT_AVAILABLE;
    }

    /* Simple sizing/existence tests */
    if( config.ext_node_dsc == nullptr || config.ext_transcode_buffer.empty() )
    {
      return DB_ERR_BAD_ARG;
    }

    /* Make sure the transcode buffer can handle all known encode/decode ops */
    for( auto &node : *config.ext_node_dsc )
    {
      if( !node_is_valid( node ) )
      {
        return DB_ERR_BAD_ARG;
      }

      if( node.pbFields && ( node.dataSize > config.ext_transcode_buffer.max_size() ) )
      {
        return DB_ERR_TRANSCODE_BUFFER_TOO_SMALL;
      }
    }

    /*-------------------------------------------------------------------------
    Build a new mutex if necessary
    -------------------------------------------------------------------------*/
    if( !mRMutex && !mb::osal::buildRecursiveMutexStrategy( mRMutex ) )
    {
      return DB_ERR_RESOURCES;
    }

    /*-------------------------------------------------------------------------
    Store the configuration
    -------------------------------------------------------------------------*/
    mNodeDsc         = config.ext_node_dsc;
    mTranscodeBuffer = config.ext_transcode_buffer;
    mRamDBReady      = DRIVER_INITIALIZED_KEY;
    return DB_ERR_NONE;
  }


  bool RamKVDB::init()
  {
    return mRamDBReady == DRIVER_INITIALIZED_KEY;
  }


  void RamKVDB::deinit()
  {
    if( mRamDBReady != DRIVER_INITIALIZED_KEY )
    {
      return;
    }

    mb::osal::destroyRecursiveMutex( mRMutex );
    mNodeDsc         = nullptr;
    mTranscodeBuffer = {};
    mRamDBReady      = ~DRIVER_INITIALIZED_KEY;
  }


  bool RamKVDB::insert( const KVNode &node )
  {
    /*-------------------------------------------------------------------------
    Validate the input node
    -------------------------------------------------------------------------*/
    const bool pbFieldsTooLarge = node.pbFields && ( node.dataSize > mTranscodeBuffer.max_size() );
    if( !node_is_valid( node ) || pbFieldsTooLarge )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Safely insert the node
    -------------------------------------------------------------------------*/
    mb::thread::RecursiveLockGuard lock( mRMutex );

    if( RamKVDB::exists( node.hashKey ) || mNodeDsc->full() )
    {
      return false;
    }

    mNodeDsc->push_back( node );
    etl::sort( mNodeDsc->begin(), mNodeDsc->end(),
               []( const KVNode &lhs, const KVNode &rhs ) -> bool { return lhs.hashKey < rhs.hashKey; } );

    return true;
  }


  void RamKVDB::remove( const HashKey key )
  {
    mb::thread::RecursiveLockGuard lock( mRMutex );

    /*-------------------------------------------------------------------------
    Search for the node
    -------------------------------------------------------------------------*/
    auto node = etl::find_if( mNodeDsc->begin(), mNodeDsc->end(),
                              [ key ]( const KVNode &node ) -> bool { return node.hashKey == key; } );

    /*-------------------------------------------------------------------------
    Remove the node if it exists
    -------------------------------------------------------------------------*/
    if( node != mNodeDsc->end() )
    {
      mNodeDsc->erase( node );
    }
  }


  KVNode *RamKVDB::find( const HashKey key )
  {
    mb::thread::RecursiveLockGuard lock( mRMutex );

    /*-------------------------------------------------------------------------
    Search for the node
    -------------------------------------------------------------------------*/
    auto node = etl::find_if( mNodeDsc->begin(), mNodeDsc->end(),
                              [ key ]( const KVNode &node ) -> bool { return node.hashKey == key; } );

    /*-------------------------------------------------------------------------
    Return the node if it exists
    -------------------------------------------------------------------------*/
    if( node != mNodeDsc->end() )
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


  void RamKVDB::sync( const HashKey key )
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


  int RamKVDB::write( const HashKey key, void *data, const size_t size )
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
}    // namespace mb::db
