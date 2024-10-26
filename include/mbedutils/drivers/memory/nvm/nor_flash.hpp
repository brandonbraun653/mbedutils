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
#include <mbedutils/drivers/memory/memory_types.hpp>
#include <mbedutils/drivers/memory/nvm/jedec_cfi_cmds.hpp>
#include <mbedutils/drivers/threading/asyncio.hpp>
#include <mbedutils/drivers/threading/lock.hpp>

namespace mb::memory::nor
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/

  struct DeviceConfig;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------
  Address Byte Positions
  -------------------------------------------------------------------*/

  static constexpr size_t ADDRESS_BYTE_1_POS = 0;
  static constexpr size_t ADDRESS_BYTE_1_MSK = 0x000000FF;
  static constexpr size_t ADDRESS_BYTE_2_POS = 8;
  static constexpr size_t ADDRESS_BYTE_2_MSK = 0x0000FF00;
  static constexpr size_t ADDRESS_BYTE_3_POS = 16;
  static constexpr size_t ADDRESS_BYTE_3_MSK = 0x00FF0000;

  /*-------------------------------------------------------------------
  Manufacturer ID Bit Masks
  -------------------------------------------------------------------*/

  static constexpr uint8_t MFR_MSK          = 0xFF;
  static constexpr uint8_t FAMILY_CODE_POS  = 5;
  static constexpr uint8_t FAMILY_CODE_MSK  = 0x07;
  static constexpr uint8_t DENSITY_CODE_POS = 0;
  static constexpr uint8_t DENSITY_CODE_MSK = 0x1F;
  static constexpr uint8_t SUB_CODE_POS     = 5;
  static constexpr uint8_t SUB_CODE_MSK     = 0x07;
  static constexpr uint8_t PROD_VERSION_POS = 0;
  static constexpr uint8_t PROD_VERSION_MSK = 0x1F;

  /*-------------------------------------------------------------------
  Common Block Sizes
  -------------------------------------------------------------------*/

  static constexpr size_t BLOCK_SIZE_256 = 256;
  static constexpr size_t BLOCK_SIZE_4K  = 4 * 1024;
  static constexpr size_t BLOCK_SIZE_32K = 32 * 1024;
  static constexpr size_t BLOCK_SIZE_64K = 64 * 1024;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Common events that can be triggered by the NOR driver.
   */
  enum Event : uint32_t
  {
    MEM_WRITE_COMPLETE = 0,
    MEM_READ_COMPLETE,
    MEM_ERASE_COMPLETE,
    MEM_ERROR
  };

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /**
   * @brief Device specific callback for handling NOR flash events.
   *
   * Most chips have a non-standard set of registers that need to be read to
   * determine the status of a pending operation. This callback is used to
   * provide that functionality to the driver.
   *
   * It's safe to assume that any resources needed to perform the callback
   * (aka SPI, GPIO, etc) are already locked by the driver and can be used
   * directly.
   *
   * @param cfg      Configuration information for the NOR device
   * @param event    Event that is pending
   * @param timeout  Maximum time to wait for the event to complete in milliseconds
   * @return Status  Result of the event operation, ERR_OK if successful.
   */
  using PendEventFunc_t = Status ( * )( const DeviceConfig &cfg, const Event event, const size_t timeout );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Configuration information for a NOR flash device.
   *
   * Provides all the necessary information to initialize and communicate with
   * a generic NOR flash device. Some of this information is specific to the
   * target device and must be provided by the user, especially the device
   * pending event callback.
   */
  struct DeviceConfig
  {
    block_device::Attributes dev_attr;      /**< NOR device descriptor */
    mb::hw::spi::Port_t      spi_port;      /**< Which SPI device to use */
    mb::hw::gpio::Port_t     spi_cs_port;   /**< Chip select IO port */
    mb::hw::gpio::Pin_t      spi_cs_pin;    /**< Chip select IO line */
    bool                     use_hs_read;   /**< Use high speed read mode */
    PendEventFunc_t          pend_event_cb; /**< Event callback function */

    bool is_valid() const
    {
      bool validity = dev_attr.is_valid();
      validity &= ( pend_event_cb != nullptr );

      return validity;
    }
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief NOR Flash Memory Driver
   *
   * This is a low level driver for interfacing with NOR flash memory devices.
   * All operations are blocking for the current thread, but the driver is
   * thread safe and can be used in a multi-threaded environment.
   */
  class DeviceDriver : public block_device::IBlockDeviceDriver,
                       public mb::thread::Lockable<DeviceDriver>
  {
  public:
    DeviceDriver();
    ~DeviceDriver();

    /*-------------------------------------------------------------------------
    Block Device Interface
    -------------------------------------------------------------------------*/
    mb::memory::Status write( const size_t block_idx, const size_t offset, const void *const data, const size_t length ) final override;
    mb::memory::Status write( const uint64_t address, const void *const data, const size_t length ) final override;
    mb::memory::Status read( const size_t block_idx, const size_t offset, void *const data, const size_t length ) final override;
    mb::memory::Status read( const uint64_t address, void *const data, const size_t length ) final override;
    mb::memory::Status erase( const uint64_t address, const size_t size ) final override;
    mb::memory::Status erase( const size_t block_idx ) final override;
    mb::memory::Status erase() final override;
    mb::memory::Status flush() final override;

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
     */
    void open( const mb::memory::nor::DeviceConfig &cfg );

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
    mb::memory::Status transfer( const void *const cmd, void *const output, const size_t size );

    /**
     * @brief Get the JEDEC assigned device information
     *
     * @return mb::memory::cfi::JEDECDeviceInfo
     */
    mb::memory::cfi::JEDECDeviceInfo getDeviceInfo();

  private:
    friend class mb::thread::Lockable<mb::memory::nor::DeviceDriver>;

    size_t                                mReadyStatus; /**< Status of the device */
    mb::memory::nor::DeviceConfig         mConfig;      /**< Device configuration attributes */
    etl::array<uint8_t, cfi::MAX_CMD_LEN> mCmdBuffer;   /**< Command buffer for NOR operations */
  };
}  // namespace mb::memory::nor

#endif  /* !MBEDUTILS_NOR_FLASH_HPP */
