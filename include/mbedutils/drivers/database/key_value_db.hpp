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
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief A flash memory backed key-value database
   *
   */
  class PersistentKVDB : public virtual IDatabase
  {
  public:
    struct Config
    {
      etl::string<FAL_DEV_NAME_MAX> dev_name;         /**< Which device is being accessed */
      etl::string<FAL_DEV_NAME_MAX> partition_name;   /**< Sub-partition being accessed */
      etl::ivector<KVNode_t>        kv_descriptors;   /**< Descriptors for KV pairs located on the device/partition */
    };

    PersistentKVDB();
    ~PersistentKVDB();

    bool init( Config &config );

  private:
    fdb_kvdb mDB;
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_KEY_VALUE_DATABASE_HPP */
