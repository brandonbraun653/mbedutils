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
#include <mbedutils/drivers/database/db_kv_node.hpp>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  bool kv_writer_memcpy( KVNode &node, const void *const data, const size_t size, const bool valid )
  {
    return false;
  }


  bool kv_writer_etl_string( KVNode &node, const void *const data, const size_t size, const bool valid )
  {
    return false;
  }


  bool kv_reader_memcpy( const KVNode &node, void *data, const size_t size )
  {
    return false;
  }


  bool kv_reader_etl_string( const KVNode &node, void *data, const size_t size )
  {
    return false;
  }


  void sanitize( KVNode &node )
  {
  }


  bool is_valid( KVNode &node )
  {
    return false;
  }


  bool write( KVNode &node, const void *data, const size_t size, const bool valid )
  {
    return false;
  }


  bool read( KVNode &node, void *data, const size_t size )
  {
    return false;
  }


  bool serialize( KVNode &node, void *const data, const size_t size )
  {
    return false;
  }


  bool deserialize( KVNode &node, const void *const data, const size_t size )
  {
    return false;
  }

}    // namespace mb::db
