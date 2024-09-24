/******************************************************************************
 *  File Name:
 *    key_value_db.hpp
 *
 *  Description:
 *    Key-Value database interfaces
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_KEY_VALUE_DATABASE_HPP
#define MBEDUTILS_KEY_VALUE_DATABASE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>
#include <etl/string.h>
#include <etl/list.h>
#include <mbedutils/drivers/database/db_intf.hpp>
#include <mbedutils/drivers/database/db_types.hpp>

extern "C"
{
#include <fal.h>
#include <flashdb.h>
}

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  template<size_t N>
  struct KVParamStorageManager
  {
    // mabye need this?
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief A flash memory backed key-value database with RAM caching.
   *
   * This database is designed to be used with a flash memory device that
   * supports the FAL interface. The database will cache all key-value pairs
   * in RAM to reduce the number of flash writes. The database will also
   * automatically flush the cache as needed to ensure consistency between
   * the RAM and flash storage.
   */
  class PersistentKVDB : public virtual IDatabase
  {
  public:
    struct Config
    {
      etl::string<FAL_DEV_NAME_MAX> dev_name;       /**< Which device is being accessed */
      etl::string<FAL_DEV_NAME_MAX> partition_name; /**< Sub-partition being accessed */
      KVParamStorageManager        *param_manager;  /**< RAM manager for KV pairs located on the device/partition */
    };

    PersistentKVDB();
    ~PersistentKVDB();

    bool init( Config &config );


  protected:

    /**
     * @brief System atexit handler to ensure the cache is flushed before exit
     */
    void flush_on_exit();

  private:
    fdb_kvdb mDB;
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_KEY_VALUE_DATABASE_HPP */
