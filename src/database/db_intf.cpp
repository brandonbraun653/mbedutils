/******************************************************************************
 *  File Name:
 *    db_intf.cpp
 *
 *  Description:
 *    Database interface implementation details
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/database/db_intf.hpp>

namespace mb::db
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  HashKey hash( etl::string_view &key )
  {
    return hash( key.data(), key.size() );
  }


  HashKey hash( const void *key, const size_t size )
  {
    return 0;
  }
}    // namespace mb::db
