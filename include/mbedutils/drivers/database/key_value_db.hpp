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
#include <cstddef>
#include <mbedutils/drivers/memory/block_device.hpp>
#include <flashdb.h>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief A flash memory backed key-value database
   *
   */
  class KVDBFlash
  {
  public:
    struct Config
    {
      mb::memory::block_device::IBlockDeviceDriver *block_device;
      fdb_default_kv_node * default_kv_table;
      size_t default_kv_table_size;
    };

    KVDBFlash();
    ~KVDBFlash();

  private:
    struct State
    {
      fdb_kvdb db;

    } mState;
  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_KEY_VALUE_DATABASE_HPP */
