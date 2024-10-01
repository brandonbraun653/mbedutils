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

  bool kv_writer_memcpy( KVNode &node, const void *const data, const size_t size, const bool valid )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !size || !node.datacache || ( node.dataSize < size ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Update the data and flags
    -------------------------------------------------------------------------*/
    memcpy( node.datacache, data, size );

    if( valid )
    {
      node.flags |= KV_FLAG_VALID;
    }
    else
    {
      node.flags &= ~KV_FLAG_VALID;
    }

    return true;
  }


  bool kv_writer_char_to_etl_string( KVNode &node, const void *data, const size_t size, const bool valid )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if( !data || !size || !node.datacache || ( node.dataSize < size ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Update the data and flags
    -------------------------------------------------------------------------*/
    auto val = reinterpret_cast<etl::istring *>( node.datacache );
    val->assign( reinterpret_cast<const char *>( data ), size );

    if( valid )
    {
      node.flags |= KV_FLAG_VALID;
    }
    else
    {
      node.flags &= ~KV_FLAG_VALID;
    }

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
      node.sanitizer( node );
    }
  }


  bool is_valid( const KVNode &node )
  {
    if( node.validator )
    {
      return node.validator( node );
    }

    return static_cast<bool>( node.flags & KV_FLAG_VALID );
  }


  bool write( KVNode &node, const void *data, const size_t size, const bool valid )
  {
    if( node.writer )
    {
      return node.writer( node, data, size, valid );
    }

    return false;
  }


  int read( const KVNode &node, void *data, const size_t size )
  {
    if( node.reader )
    {
      return node.reader( node, data, size );
    }

    return 0;
  }


  int serialize( const KVNode &node, void *const data, const size_t size )
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


  bool deserialize( KVNode &node, const void *data, const size_t size )
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
