/******************************************************************************
 *  File Name:
 *    db_parameter.cpp
 *
 *  Description:
 *    Parameter database implementation details
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/database/db_parameter.hpp>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  bool cmn_param_deserializer( KVParamNode &node, const void *const data, const size_t size )
  {
    return false;
  }


  bool cmn_param_serializer( const KVParamNode &node, void *const data, const size_t size )
  {
    return false;
  }

  /*---------------------------------------------------------------------------
  KVParamNode Class
  ---------------------------------------------------------------------------*/

  void KVParamNode::sanitize()
  {
  }


  bool KVParamNode::is_valid()
  {
    return false;
  }


  bool KVParamNode::write( const void *data, const size_t size, const bool valid )
  {
    return false;
  }


  bool KVParamNode::read( void *data, const size_t size )
  {
    return false;
  }


  bool KVParamNode::serialize( void *const data, const size_t size )
  {
    return false;
  }


  bool KVParamNode::deserialize( const void *const data, const size_t size )
  {
    return false;
  }


  /*---------------------------------------------------------------------------
  KVParamStorageManager Class
  ---------------------------------------------------------------------------*/

  KVParamStorageManager::KVParamStorageManager() : mParams( nullptr )
  {
  }


  KVParamStorageManager::~KVParamStorageManager()
  {
  }


  void KVParamStorageManager::init( KVParamNodeIVector &params )
  {
  }


  bool KVParamStorageManager::insert( const KVParamNode &node )
  {
    return false;
  }


  void KVParamStorageManager::remove( const HashKey key )
  {
  }


  KVParamNode *KVParamStorageManager::find( const HashKey key )
  {
    return nullptr;
  }


  void KVParamStorageManager::visit_dirty_nodes( NodeVisitorFunc &visitor )
  {
  }

}    // namespace mb::db
