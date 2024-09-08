/******************************************************************************
 *  File Name:
 *    nor_flash.hpp
 *
 *  Description:
 *    Interface for NOR flash memory devices.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_NOR_FLASH_HPP
#define MBEDUTILS_NOR_FLASH_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/array.h>
#include <mbedutils/drivers/memory/block_device.hpp>
#include <mbedutils/interfaces/gpio_intf.hpp>
#include <mbedutils/interfaces/spi_intf.hpp>
#include <mbedutils/memory/memory_types.hpp>
#include <mbedutils/memory/nvm/jedec_cfi_cmds.hpp>
#include <mbedutils/threading/asyncio.hpp>
#include <mbedutils/threading/lock.hpp>

namespace mb::memory::nor
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Configuration information for a NOR flash device.
   *
   * Provides all the necessary information to initialize and communicate with
   * a generic NOR flash device.
   */
  struct DeviceConfig
  {
    block_device::Attributes dev_attr;    /**< NOR device descriptor */
    mb::hw::spi::Port_t      spi_port;    /**< Which SPI device to use */
    mb::hw::gpio::Pin_t      spi_cs_pin;  /**< Chip select IO line */
    bool                     use_hs_read; /**< Use high speed read mode */
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief NOR Flash Memory Driver
   *
   * This is a low level driver for interfacing with NOR flash memory devices.
   */
  class DeviceDriver : public block_device::IBlockDeviceDriver,
                       public mb::thread::LockableInterface<DeviceDriver>,
                       public mb::thread::AsyncIOInterface<DeviceDriver>
  {
  public:
    DeviceDriver();
    ~DeviceDriver();

    /*-------------------------------------------------------------------------
    Block Device Interface
    -------------------------------------------------------------------------*/
    Status write( const size_t block_idx, const size_t offset, const void *const data, const size_t length ) final override;
    Status write( const uint64_t address, const void *const data, const size_t length ) final override;
    Status read( const size_t block_idx, const size_t offset, void *const data, const size_t length ) final override;
    Status read( const uint64_t address, void *const data, const size_t length ) final override;
    Status erase( const size_t block_idx ) final override;
    Status erase() final override;
    Status flush() final override;

    /*-------------------------------------------------------------------------
    NOR Driver Interface
    -------------------------------------------------------------------------*/

    /**
     * @brief Configures the driver for a specific NOR device.
     *
     * This will setup the driver to communicate with the NOR device using the
     * provided configuration information. The driver will not be fully operational
     * until the open function is called.
     *
     * This function expects that all dependencies like the SPI and GPIO interfaces
     * have been properly initialized before calling.
     *
     * @param cfg  Configuration information for the NOR device
     * @return true   If the configuration was successful
     * @return false  If the configuration failed
     */
    bool open( const DeviceConfig &cfg );

    /**
     * @brief Tears down the driver and releases any resources.
     */
    void close();

    /**
     * @brief Exposes the raw data bus interface to the user.
     *
     * This is useful for performing direct memory operations on the NOR device
     * in a full duplex manner. The user is responsible for ensuring the command
     * and data buffers are properly formatted for the device.
     *
     * @param cmd     Command buffer to send to the NOR chip
     * @param output  Buffer to read in the RX half of the transfer
     * @param size    Size of both buffers (must be the same)
     */
    void transfer( const void *const cmd, void *const output, const size_t size );

  private:
    friend class mb::thread::LockableInterface<DeviceDriver>;
    friend class mb::thread::AsyncIOInterface<DeviceDriver>;

    DeviceConfig                          mConfig;    /**< Device configuration attributes */
    bool                                  mIsOpen;    /**< Status flag for driver operability */
    etl::array<uint8_t, cfi::MAX_CMD_LEN> mCmdBuffer; /**< Command buffer for NOR operations */
  };
}  // namespace mb::memory::nor

#endif  /* !MBEDUTILS_NOR_FLASH_HPP */
