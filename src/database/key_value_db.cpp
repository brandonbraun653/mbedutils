/******************************************************************************
 *  File Name:
 *    key_value_db.cpp
 *
 *  Description:
 *    Key-Value database interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/database/key_value_db.hpp>
#include <pb_encode.h>
#include <pb_decode.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  PersistentKVDB::PersistentKVDB()
  {
  }


  PersistentKVDB::~PersistentKVDB()
  {
  }


  bool PersistentKVDB::init( Config &config )
  {
    return FDB_NO_ERR == fdb_kvdb_init( &mDB, config.dev_name.c_str(), config.partition_name.c_str(), &config.default_kv_table, this );
  }


}  // namespace mb::db
