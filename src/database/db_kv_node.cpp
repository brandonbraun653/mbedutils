/******************************************************************************
 *  File Name:
 *    db_kv_node.cpp
 *
 *  Description:
 *    Parameter database implementation details
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/database/db_kv_node.hpp>
#include <pb_encode.h>
#include <pb_decode.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  bool kv_writer_memcpy( KVNode &node, const void *const data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !size || !node.datacache || ( node.dataSize < size ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Update the data
    -------------------------------------------------------------------------*/
    memcpy( node.datacache, data, size );
    return true;
  }


  bool kv_writer_char_to_etl_string( KVNode &node, const void *data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !size || !node.datacache || ( node.dataSize < size ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Update the data
    -------------------------------------------------------------------------*/
    auto val = reinterpret_cast<etl::istring *>( node.datacache );
    val->assign( reinterpret_cast<const char *>( data ), size );

    return true;
  }


  int kv_reader_memcpy( const KVNode &node, void *data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !size || !node.datacache )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Copy the data
    -------------------------------------------------------------------------*/
    const size_t copy_size = node.dataSize > size ? size : node.dataSize;
    memcpy( data, node.datacache, copy_size );
    return static_cast<int>( copy_size );
  }


  int kv_reader_etl_string_to_char( const KVNode &node, void *data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !size || !node.datacache )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Copy the data and return its validity
    -------------------------------------------------------------------------*/
    const auto   val       = reinterpret_cast<const etl::istring *>( node.datacache );
    const size_t copy_size = val->size() > size ? size : val->size();

    memcpy( data, val->c_str(), copy_size );
    return static_cast<int>( copy_size );
  }


  void sanitize( KVNode &node )
  {
    if( node.sanitizer )
    {
      node.sanitizer( node, node.datacache, node.dataSize );
    }
  }


  bool data_is_valid( const KVNode &node )
  {
    /*-------------------------------------------------------------------------
    Perform basic checks
    -------------------------------------------------------------------------*/
    if( !node_is_valid( node ) )
    {
      return false;
    }

    if( node.validator )
    {
      return node.validator( node, node.datacache, node.dataSize );
    }

    return static_cast<bool>( node.flags & KV_FLAG_VALID );
  }


  bool node_is_valid( const KVNode &node )
  {
    return ( node.datacache                      /* Must have memory backing */
             || node.dataSize                    /* Memory backing must have some size */
             || ( node.hashKey != MAX_HASH_KEY ) /* Hash key must be valid */
             || ( node.writer || node.reader )   /* Has at least a writer or reader */
    );
  }


  bool node_write( KVNode &node, const void *data, const size_t size )
  {
    if( !node.writer )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Perform the write operation, running the callback if successful
    -------------------------------------------------------------------------*/
    bool writer_success = node.writer( node, data, size );
    if( writer_success && node.onWrite )
    {
      node.onWrite( node );
    }

    return writer_success;
  }


  int node_read( const KVNode &node, void *data, const size_t size )
  {
    if( !node.reader )
    {
      return 0;
    }

    return node.reader( node, data, size );
  }


  int node_serialize( const KVNode &node, void *const data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !size || !node.pbFields )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Serialize the data
    -------------------------------------------------------------------------*/
    pb_ostream_t stream = pb_ostream_from_buffer( static_cast<pb_byte_t *>( data ), size );
    if( !pb_encode( &stream, node.pbFields, node.datacache ) )
    {
      mbed_assert_continue_msg( false, "KVNode %d encode failure: %s", node.hashKey, stream.errmsg );
      return -1;
    }

    return static_cast<int>( stream.bytes_written );
  }


  bool node_deserialize( KVNode &node, const void *data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !node.pbFields || !node.datacache )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Deserialize the data
    -------------------------------------------------------------------------*/
    pb_istream_t stream = pb_istream_from_buffer( static_cast<const pb_byte_t *>( data ), size );
    if( !pb_decode( &stream, node.pbFields, node.datacache ) )
    {
      mbed_assert_continue_msg( false, "KVNode %d decode failure: %s", node.hashKey, stream.errmsg );
      return false;
    }

    return true;
  }

}    // namespace mb::db
