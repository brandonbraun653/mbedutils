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

  PersistentKVDB::PersistentKVDB() : mDB( {} ), mConfig( {} )
  {
  }


  PersistentKVDB::~PersistentKVDB()
  {
  }


  bool PersistentKVDB::configure( Config &config )
  {
    return false;
  }


  bool PersistentKVDB::init()
  {
    // FDB_NO_ERR == fdb_kvdb_init( &mDB, config.dev_name.c_str(), config.partition_name.c_str(), &config.default_kv_table, this );
    // Pull information from flash memory
    // register atexit call

    return false;
  }


  void PersistentKVDB::deinit()
  {
    // flush
    // de-register atexit call
  }


  void PersistentKVDB::sync()
  {
  }


  void PersistentKVDB::flush()
  {
    // Acquire lock
    this->flush_on_exit();
  }


  bool PersistentKVDB::exists( const HashKey key )
  {
    return false;
  }


  int PersistentKVDB::read( const HashKey key, void *data, const size_t data_size, const size_t size )
  {
    return -1;
  }


  int PersistentKVDB::write( const HashKey key, const void *data, const size_t data_size, const size_t size )
  {
    return -1;
  }


  int PersistentKVDB::erase( const HashKey key )
  {
    return -1;
  }


  void PersistentKVDB::flush_on_exit()
  {
    // Likely don't need to acquire a lock here
  }

}  // namespace mb::db
