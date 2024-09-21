/******************************************************************************
 *  File Name:
 *    db_intf.hpp
 *
 *  Description:
 *    Core database driver interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_DATABASE_INTERFACE_HPP
#define MBEDUTILS_DATABASE_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <mbedutils/drivers/database/db_types.hpp>


namespace mb::db
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class IDatabase
  {
  public:
    virtual ~IDatabase() = default;

    virtual bool init() = 0;
    virtual bool deinit() = 0;

    /**
     * @brief If any keys are dirty, flush them to the underlying storage.
     */
    virtual void flush() = 0;

    virtual bool exists( const KVKey_t key ) = 0;
    virtual int read( const KVKey_t key, void *data, const size_t size ) = 0;
    virtual int write( const KVKey_t key, const void *data, const size_t size ) = 0;
    virtual int erase( const KVKey_t key ) = 0;

  };
}  // namespace mb::db

#endif  /* !MBEDUTILS_DATABASE_INTERFACE_HPP */
