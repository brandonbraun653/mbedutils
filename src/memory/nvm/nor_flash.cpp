/******************************************************************************
 *  File Name:
 *    nor_flash.cpp
 *
 *  Description:
 *    Generic NOR flash memory driver implementation details. This implements
 *    the cfi (Common Flash Interface) commands for interfacing with NOR flash
 *    memory devices.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/assert.hpp>
#include <mbedutils/drivers/memory/nvm/jedec_cfi_cmds.hpp>
#include <mbedutils/drivers/memory/nvm/nor_flash.hpp>
#include <mbedutils/drivers/threading/thread.hpp>
#include <mbedutils/interfaces/gpio_intf.hpp>
#include <mbedutils/interfaces/spi_intf.hpp>

namespace mb::memory::nor
{
  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Send the write enable command to the device.
   *
   * This expects to be called from a locked SPI interface context.
   *
   * @param cfg Configuration information for the NOR device
   */
  static void issue_write_enable( const DeviceConfig &cfg )
  {
    using namespace mb::hw;
    using namespace mb::memory;

    gpio::intf::write( cfg.spi_cs_port, cfg.spi_cs_pin, gpio::State_t::STATE_LOW );
    spi::intf::write( cfg.spi_port, &cfi::WRITE_ENABLE, cfi::WRITE_ENABLE_OPS_LEN );
    gpio::intf::write( cfg.spi_cs_port, cfg.spi_cs_pin, gpio::State_t::STATE_HIGH );
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  DeviceDriver::DeviceDriver() : mConfig(), mCmdBuffer()
  {
  }


  DeviceDriver::~DeviceDriver()
  {
  }


  void DeviceDriver::open( const DeviceConfig &cfg )
  {
    mbed_assert( cfg.pend_event_cb );

    mConfig = cfg;
    mCmdBuffer.fill( 0 );
    this->initLockable();
  }


  void DeviceDriver::close()
  {
    mConfig = DeviceConfig();
    mCmdBuffer.fill( 0 );
  }


  void DeviceDriver::transfer( const void *const cmd, void *const output, const size_t size )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !cmd || !output || !size )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Perform the SPI transaction
    -------------------------------------------------------------------------*/
    LockGuard _driverLock( this->mLockableMutex );
    spi::intf::lock( mConfig.spi_port );
    {
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::transfer( mConfig.spi_port, cmd, output, size );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
    }
    spi::intf::unlock( mConfig.spi_port );
  }


  Status DeviceDriver::write( const size_t block_idx, const size_t offset, const void *const data, const size_t length )
  {
    return this->write( ( block_idx * mConfig.dev_attr.block_size + offset ), data, length );
  }


  Status DeviceDriver::write( const uint64_t address, const void *const data, const size_t length )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    LockGuard _driverLock( this->mLockableMutex );
    if( !data || !length || ( ( address + length ) > mConfig.dev_attr.size ) )
    {
      return Status::ERR_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    Setup the command buffer
    -------------------------------------------------------------------------*/
    mCmdBuffer[ 0 ] = cfi::PAGE_PROGRAM;
    mCmdBuffer[ 1 ] = ( address & ADDRESS_BYTE_3_MSK ) >> ADDRESS_BYTE_3_POS;
    mCmdBuffer[ 2 ] = ( address & ADDRESS_BYTE_2_MSK ) >> ADDRESS_BYTE_2_POS;
    mCmdBuffer[ 3 ] = ( address & ADDRESS_BYTE_1_MSK ) >> ADDRESS_BYTE_1_POS;

    /*-------------------------------------------------------------------------
    Perform the SPI transaction
    -------------------------------------------------------------------------*/
    spi::intf::lock( mConfig.spi_port );
    {
      // Ensure the write enable command is sent first
      issue_write_enable( mConfig );

      // Send the command and address
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::write( mConfig.spi_port, mCmdBuffer.data(), cfi::PAGE_PROGRAM_OPS_LEN );
      spi::intf::write( mConfig.spi_port, data, length );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
    }
    spi::intf::unlock( mConfig.spi_port );

    /*-------------------------------------------------------------------------
    Wait for the hardware to finish the operation
    -------------------------------------------------------------------------*/
    return mConfig.pend_event_cb( mConfig, Event::MEM_WRITE_COMPLETE, TIMEOUT_BLOCK );
  }


  Status DeviceDriver::read( const size_t block_idx, const size_t offset, void *const data, const size_t length )
  {
    return this->read( ( block_idx * mConfig.dev_attr.read_size + offset ), data, length );
  }


  Status DeviceDriver::read( const uint64_t address, void *const data, const size_t length )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    LockGuard _driverLock( this->mLockableMutex );
    if( !data || !length || ( ( address + length ) > mConfig.dev_attr.size ) )
    {
      return Status::ERR_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    Init the cmd sequence. The high speed cmd works for all frequency ranges.
    -------------------------------------------------------------------------*/
    mCmdBuffer[ 0 ] = cfi::READ_ARRAY_HS;
    mCmdBuffer[ 1 ] = ( address & ADDRESS_BYTE_3_MSK ) >> ADDRESS_BYTE_3_POS;
    mCmdBuffer[ 2 ] = ( address & ADDRESS_BYTE_2_MSK ) >> ADDRESS_BYTE_2_POS;
    mCmdBuffer[ 3 ] = ( address & ADDRESS_BYTE_1_MSK ) >> ADDRESS_BYTE_1_POS;
    mCmdBuffer[ 4 ] = 0;    // Dummy byte

    /*-------------------------------------------------------------------------
    Perform the SPI transaction
    -------------------------------------------------------------------------*/
    spi::intf::lock( mConfig.spi_port );
    {
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::write( mConfig.spi_port, mCmdBuffer.data(), cfi::READ_ARRAY_HS_OPS_LEN );
      spi::intf::read( mConfig.spi_port, data, length );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
    }
    spi::intf::unlock( mConfig.spi_port );

    return Status::ERR_OK;
  }


  Status DeviceDriver::erase( const size_t block_idx )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    LockGuard _driverLock( this->mLockableMutex );

    uint32_t address = block_idx * mConfig.dev_attr.block_size;
    if( ( address + mConfig.dev_attr.block_size ) > mConfig.dev_attr.size )
    {
      return Status::ERR_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    Determine the op-code to use based on the requested chunk size to erase.
    -------------------------------------------------------------------------*/
    size_t eraseOpsLen = cfi::BLOCK_ERASE_OPS_LEN;
    switch( mConfig.dev_attr.block_size )
    {
      case BLOCK_SIZE_4K:
        mCmdBuffer[ 0 ] = cfi::BLOCK_ERASE_4K;
        break;

      case BLOCK_SIZE_32K:
        mCmdBuffer[ 0 ] = cfi::BLOCK_ERASE_32K;
        break;

      case BLOCK_SIZE_64K:
        mCmdBuffer[ 0 ] = cfi::BLOCK_ERASE_64K;
        break;

      default:    // Weird erase size
        mbed_assert_always();
        break;
    }

    /*-------------------------------------------------------------------------
    Initialize the command sequence. If whole chip erase command, these bytes
    will be ignored anyways.
    -------------------------------------------------------------------------*/
    mCmdBuffer[ 1 ] = ( address & ADDRESS_BYTE_3_MSK ) >> ADDRESS_BYTE_3_POS;
    mCmdBuffer[ 2 ] = ( address & ADDRESS_BYTE_2_MSK ) >> ADDRESS_BYTE_2_POS;
    mCmdBuffer[ 3 ] = ( address & ADDRESS_BYTE_1_MSK ) >> ADDRESS_BYTE_1_POS;

    /*-------------------------------------------------------------------------
    Perform the SPI transaction
    -------------------------------------------------------------------------*/
    spi::intf::lock( mConfig.spi_port );
    {
      issue_write_enable( mConfig );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::write( mConfig.spi_port, mCmdBuffer.data(), eraseOpsLen );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
    }
    spi::intf::unlock( mConfig.spi_port );

    return mConfig.pend_event_cb( mConfig, Event::MEM_ERASE_COMPLETE, TIMEOUT_BLOCK );
  }


  Status DeviceDriver::erase()
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    LockGuard _driverLock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Perform the SPI transaction
    -------------------------------------------------------------------------*/
    spi::intf::lock( mConfig.spi_port );
    {
      issue_write_enable( mConfig );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::write( mConfig.spi_port, &cfi::CHIP_ERASE, cfi::CHIP_ERASE_OPS_LEN );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
    }
    spi::intf::unlock( mConfig.spi_port );

    return mConfig.pend_event_cb( mConfig, Event::MEM_ERASE_COMPLETE, TIMEOUT_BLOCK );
  }


  Status DeviceDriver::flush()
  {
    return Status::ERR_OK;
  }

}    // namespace mb::memory::nor
