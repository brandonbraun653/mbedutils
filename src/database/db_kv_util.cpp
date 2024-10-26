/******************************************************************************
 *  File Name:
 *    db_kv_mgr.cpp
 *
 *  Description:
 *    Key-Value database manager implementation classes
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/crc.h>
#include <etl/to_arithmetic.h>
#include <etl/to_string.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/database.hpp>
#include <mbedutils/logging.hpp>
#include <mbedutils/system.hpp>
#include <mbedutils/threading.hpp>
#include <mbedutils/util.hpp>
#include <pb_decode.h>
#include <pb_encode.h>
#include <fdb_def.h>
#include <nanoprintf.h>

namespace mb::db
{

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  const char *fdb_err_to_str( const int err )
  {
    switch( err )
    {
      case FDB_NO_ERR:
        return "No error";
      case FDB_ERASE_ERR:
        return "Erase error";
      case FDB_READ_ERR:
        return "Read error";
      case FDB_WRITE_ERR:
        return "Write error";
      case FDB_PART_NOT_FOUND:
        return "Partition not found";
      case FDB_KV_NAME_ERR:
        return "Key-Value name error";
      case FDB_KV_NAME_EXIST:
        return "Key-Value name exists";
      case FDB_SAVED_FULL:
        return "Saved full";
      case FDB_INIT_FAILED:
        return "Initialization failed";
      default:
        return "Unknown error";
    }
  }


  HashKey hash( etl::string_view key )
  {
    etl::crc32 crc_calculator;
    etl::copy( key.begin(), key.end(), crc_calculator.input() );
    uint32_t crc = crc_calculator.value();
    return crc;
  }


  HashKey hash( const void *key, const size_t size )
  {
    etl::crc32 crc_calculator;
    auto       p8_key = reinterpret_cast<const uint8_t *>( key );
    for( size_t i = 0; i < size; i++ )
    {
      crc_calculator.add( p8_key[ i ] );
    }

    uint32_t crc = crc_calculator.value();
    return crc;
  }

}    // namespace mb::db
