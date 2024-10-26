/******************************************************************************
 *  File Name:
 *    nor_flash_device.hpp
 *
 *  Description:
 *    Device specific interfaces for NOR flash memory devices.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_NOR_FLASH_DEVICE_HPP
#define MBEDUTILS_NOR_FLASH_DEVICE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <mbedutils/drivers/memory/nvm/nor_flash.hpp>

namespace mb::memory::nor::device
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Waits for a specific event to complete on an Adesto NOR device.
   *
   * @param cfg     Configuration information for the NOR device
   * @param event   Event that is pending
   * @param timeout Maximum time to wait for the event to complete in milliseconds
   * @return Status
   */
  mb::memory::Status adesto_at25sfxxx_pend_event( const mb::memory::nor::DeviceConfig &cfg, const mb::memory::nor::Event event,
                                                  const size_t timeout );

}    // namespace mb::memory::nor::device

#endif /* !MBEDUTILS_NOR_FLASH_DEVICE_HPP */
