/******************************************************************************
 *  File Name:
 *    nor_flash.cpp
 *
 *  Description:
 *    NOR flash memory driver implementation details.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/drivers/memory/nvm/nor_flash.hpp>
#include <mbedutils/interfaces/gpio_intf.hpp>
#include <mbedutils/interfaces/spi_intf.hpp>

namespace mb::memory::nor
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  DeviceDriver::DeviceDriver() : mConfig(), mIsOpen( false ), mCmdBuffer()
  {
  }


  DeviceDriver::~DeviceDriver()
  {
  }


  bool DeviceDriver::open( const DeviceConfig &cfg )
  {
    mConfig = cfg;
    mIsOpen = true;
    return true;
  }


  void DeviceDriver::close()
  {
    mIsOpen = false;
  }


  void DeviceDriver::transfer( const void *const cmd, void *const output, const size_t size )
  {
  }


  Status DeviceDriver::write( const size_t block_idx, const size_t offset, const void *const data, const size_t length )
  {
    return Status::ERR_NOT_SUPPORTED;
  }


  Status DeviceDriver::write( const uint64_t address, const void *const data, const size_t length )
  {
    return Status::ERR_NOT_SUPPORTED;
  }


  Status DeviceDriver::read( const size_t block_idx, const size_t offset, void *const data, const size_t length )
  {
    return Status::ERR_NOT_SUPPORTED;
  }


  Status DeviceDriver::read( const uint64_t address, void *const data, const size_t length )
  {
    return Status::ERR_NOT_SUPPORTED;
  }


  Status DeviceDriver::erase( const size_t block_idx )
  {
    return Status::ERR_NOT_SUPPORTED;
  }


  Status DeviceDriver::erase()
  {
    return Status::ERR_NOT_SUPPORTED;
  }


  Status DeviceDriver::flush()
  {
    return Status::ERR_NOT_SUPPORTED;
  }

}  // namespace mb::memory::nor
