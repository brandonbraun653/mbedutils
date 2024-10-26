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

  DeviceDriver::DeviceDriver() : mReadyStatus( ~DRIVER_INITIALIZED_KEY ), mConfig(), mCmdBuffer()
  {
  }


  DeviceDriver::~DeviceDriver()
  {
  }


  void DeviceDriver::open( const DeviceConfig &cfg )
  {
    if( ( mReadyStatus != DRIVER_INITIALIZED_KEY ) && cfg.is_valid() )
    {
      mReadyStatus = DRIVER_INITIALIZED_KEY;
      mConfig      = cfg;
      mCmdBuffer.fill( 0 );
      this->initLockable();
    }
    else
    {
      mbed_assert_continue_msg( false, "Unable to open NOR device, invalid state" );
    }
  }


  void DeviceDriver::close()
  {
    if( mReadyStatus == DRIVER_INITIALIZED_KEY )
    {
      mReadyStatus = ~DRIVER_INITIALIZED_KEY;
      mConfig      = DeviceConfig();
      mCmdBuffer.fill( 0 );
    }
  }


  Status DeviceDriver::transfer( const void *const cmd, void *const output, const size_t size )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      return Status::ERR_BAD_STATE;
    }

    if( !cmd || !output || !size )
    {
      return Status::ERR_BAD_ARG;
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

    return Status::ERR_OK;
  }


  Status DeviceDriver::write( const size_t block_idx, const size_t offset, const void *const data, const size_t length )
  {
    if( offset > mConfig.dev_attr.write_size )
    {
      return Status::ERR_BAD_ARG;
    }

    return this->write( ( ( block_idx * mConfig.dev_attr.write_size ) + offset ), data, length );
  }


  Status DeviceDriver::write( const uint64_t address, const void *const data, const size_t length )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      return Status::ERR_BAD_STATE;
    }

    LockGuard _driverLock( this->mLockableMutex );
    if( !data || !length || ( ( address + length ) > mConfig.dev_attr.size ) || ( length > mConfig.dev_attr.write_size ) )
    {
      return Status::ERR_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    Do page aligned writes. Most devices cannot cross page boundaries on write.
    -------------------------------------------------------------------------*/
    int      bytes_remaining = length;
    uint64_t write_addr      = address;
    uint8_t *data_ptr        = reinterpret_cast<uint8_t *>( const_cast<void *>( data ) );

    while( bytes_remaining > 0 )
    {
      /*-----------------------------------------------------------------------
      Compute how far away we are from the next page boundary
      -----------------------------------------------------------------------*/
      const uint32_t page_offset = write_addr % mConfig.dev_attr.write_size;
      const uint32_t bytes_to_write =
          std::min( bytes_remaining, static_cast<int>( mConfig.dev_attr.write_size - page_offset ) );

      /*-----------------------------------------------------------------------
      Set up the command buffer
      -----------------------------------------------------------------------*/
      mCmdBuffer[ 0 ] = cfi::PAGE_PROGRAM;
      mCmdBuffer[ 1 ] = ( write_addr & ADDRESS_BYTE_3_MSK ) >> ADDRESS_BYTE_3_POS;
      mCmdBuffer[ 2 ] = ( write_addr & ADDRESS_BYTE_2_MSK ) >> ADDRESS_BYTE_2_POS;
      mCmdBuffer[ 3 ] = ( write_addr & ADDRESS_BYTE_1_MSK ) >> ADDRESS_BYTE_1_POS;

      /*-----------------------------------------------------------------------
      Perform the SPI transaction
      -----------------------------------------------------------------------*/
      spi::intf::lock( mConfig.spi_port );
      {
        // Ensure the write enable command is sent first
        issue_write_enable( mConfig );

        // Send the command and address
        gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_LOW );
        spi::intf::write( mConfig.spi_port, mCmdBuffer.data(), cfi::PAGE_PROGRAM_OPS_LEN );
        spi::intf::write( mConfig.spi_port, data_ptr, bytes_to_write );
        gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
      }
      spi::intf::unlock( mConfig.spi_port );

      /*-----------------------------------------------------------------------
      Wait for the hardware to finish the operation
      -----------------------------------------------------------------------*/
      auto status = mConfig.pend_event_cb( mConfig, Event::MEM_WRITE_COMPLETE, mConfig.dev_attr.write_latency );
      if( status != Status::ERR_OK )
      {
        return status;
      }

      /*-----------------------------------------------------------------------
      Update the pointers and counters
      -----------------------------------------------------------------------*/
      write_addr += bytes_to_write;
      data_ptr += bytes_to_write;
      bytes_remaining -= bytes_to_write;
    }

    return Status::ERR_OK;
  }


  Status DeviceDriver::read( const size_t block_idx, const size_t offset, void *const data, const size_t length )
  {
    if( offset > mConfig.dev_attr.read_size )
    {
      return Status::ERR_BAD_ARG;
    }

    return this->read( ( ( block_idx * mConfig.dev_attr.read_size ) + offset ), data, length );
  }


  Status DeviceDriver::read( const uint64_t address, void *const data, const size_t length )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      return Status::ERR_BAD_STATE;
    }

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


  Status DeviceDriver::erase( const uint64_t address, const size_t size )
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      return Status::ERR_BAD_STATE;
    }

    LockGuard _driverLock( this->mLockableMutex );

    if( ( address + size ) > mConfig.dev_attr.size )
    {
      return Status::ERR_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    Determine the op-code to use based on the requested chunk size to erase.
    -------------------------------------------------------------------------*/
    switch( size )
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

      default:    // Weird erase size, bail out
        return Status::ERR_BAD_CFG;
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
      spi::intf::write( mConfig.spi_port, mCmdBuffer.data(), cfi::BLOCK_ERASE_OPS_LEN );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
    }
    spi::intf::unlock( mConfig.spi_port );

    return mConfig.pend_event_cb( mConfig, Event::MEM_ERASE_COMPLETE, mConfig.dev_attr.erase_latency );
  }


  Status DeviceDriver::erase( const size_t block_idx )
  {
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      return Status::ERR_BAD_STATE;
    }

    LockGuard _driverLock( this->mLockableMutex );
    return this->erase( ( block_idx * mConfig.dev_attr.erase_size ), mConfig.dev_attr.erase_size );
  }


  Status DeviceDriver::erase()
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      return Status::ERR_BAD_STATE;
    }

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

    return mConfig.pend_event_cb( mConfig, Event::MEM_ERASE_COMPLETE, mConfig.dev_attr.erase_chip_latency );
  }


  Status DeviceDriver::flush()
  {
    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      return Status::ERR_BAD_STATE;
    }

    return Status::ERR_OK;
  }


  cfi::JEDECDeviceInfo DeviceDriver::getDeviceInfo()
  {
    using namespace mb::hw;
    using namespace mb::memory;
    using namespace mb::thread;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( mReadyStatus != DRIVER_INITIALIZED_KEY )
    {
      mbed_assert_continue_always();
      return { 0, 0, 0 };
    }

    LockGuard _driverLock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Get the device information
    -------------------------------------------------------------------------*/
    mCmdBuffer.fill( 0 );
    mCmdBuffer[ 0 ] = cfi::READ_DEV_INFO;

    uint8_t info[ cfi::READ_DEV_INFO_OPS_LEN ] = { 0 };

    spi::intf::lock( mConfig.spi_port );
    {
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::transfer( mConfig.spi_port, mCmdBuffer.data(), info, cfi::READ_DEV_INFO_OPS_LEN );
      gpio::intf::write( mConfig.spi_cs_port, mConfig.spi_cs_pin, gpio::State_t::STATE_HIGH );
    }
    spi::intf::unlock( mConfig.spi_port );

    /*-------------------------------------------------------------------------
    Copy out the data
    -------------------------------------------------------------------------*/
    return { info[ 1 ], info[ 2 ], info[ 3 ] };
  }

}    // namespace mb::memory::nor
