/******************************************************************************
 *  File Name:
 *    flash_db.hpp
 *
 *  Description:
 *    Core database driver layer backed by flash memory
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_FLASH_DATABASE_HPP
#define MBEDUTILS_FLASH_DATABASE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <fal_def.h>


namespace mb::db::flash
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the driver layer
   */
  void initialize();

  /**
   * @brief Set a partition table for a device
   *
   * @param device_name
   * @param partition
   * @return true
   * @return false
   */
  bool set_partition_table( const char *device_name, const fal_partition &partition );

}  // namespace mb::db::flash

#endif  /* !MBEDUTILS_FLASH_DATABASE_HPP */
