/******************************************************************************
 *  File Name:
 *    nor_adesto.cpp
 *
 *  Description:
 *    Adesto NOR flash memory driver implementation details.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/array.h>
#include <mbedutils/drivers/memory/nvm/nor_flash.hpp>
#include <mbedutils/drivers/memory/nvm/nor_flash_device.hpp>
#include <mbedutils/drivers/threading/thread.hpp>
#include <mbedutils/interfaces/time_intf.hpp>

namespace mb::memory::nor::device
{
  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Reads the status register of the Adesto NOR flash device.
   *
   * Returned status register is in the format: [byte2, byte1]
   *
   * @param cfg Configuration information for the NOR device
   * @return uint16_t   Status register contents with byte 1 in the LSB
   */
  static uint16_t read_status_register( const DeviceConfig &cfg )
  {
    using namespace mb::hw;

    /*-------------------------------------------------------------------------
    Status Register Read Commands
    -------------------------------------------------------------------------*/
    static constexpr uint8_t READ_SR_BYTE1         = 0x05;
    static constexpr uint8_t READ_SR_BYTE1_CMD_LEN = 1;
    static constexpr uint8_t READ_SR_BYTE1_RSP_LEN = 1;
    static constexpr uint8_t READ_SR_BYTE1_OPS_LEN = READ_SR_BYTE1_CMD_LEN + READ_SR_BYTE1_RSP_LEN;

    static constexpr uint8_t READ_SR_BYTE2         = 0x35;
    static constexpr uint8_t READ_SR_BYTE2_CMD_LEN = 1;
    static constexpr uint8_t READ_SR_BYTE2_RSP_LEN = 1;
    static constexpr uint8_t READ_SR_BYTE2_OPS_LEN = READ_SR_BYTE2_CMD_LEN + READ_SR_BYTE2_RSP_LEN;

    /*-------------------------------------------------------------------------
    Initialize the command sequence
    -------------------------------------------------------------------------*/
    uint16_t               result = 0;
    etl::array<uint8_t, 5> cmdBuffer;
    etl::array<uint8_t, 5> rxBuffer;
    cmdBuffer.fill( 0 );
    rxBuffer.fill( 0 );

    /*-------------------------------------------------------------------------
    Perform the SPI transaction
    -------------------------------------------------------------------------*/
    spi::intf::lock( cfg.spi_port );
    {
      // Read out byte 1
      cmdBuffer[ 0 ] = READ_SR_BYTE1;
      gpio::intf::write( cfg.spi_cs_port, cfg.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::transfer( cfg.spi_port, cmdBuffer.data(), rxBuffer.data(), READ_SR_BYTE1_OPS_LEN );
      gpio::intf::write( cfg.spi_cs_port, cfg.spi_cs_pin, gpio::State_t::STATE_HIGH );

      result |= rxBuffer[ 1 ];

      // Read out byte 2
      cmdBuffer[ 0 ] = READ_SR_BYTE2;
      cmdBuffer[ 1 ] = 0;

      gpio::intf::write( cfg.spi_cs_port, cfg.spi_cs_pin, gpio::State_t::STATE_LOW );
      spi::intf::transfer( cfg.spi_port, cmdBuffer.data(), rxBuffer.data(), READ_SR_BYTE2_OPS_LEN );
      gpio::intf::write( cfg.spi_cs_port, cfg.spi_cs_pin, gpio::State_t::STATE_HIGH );

      result |= ( rxBuffer[ 1 ] << 8 );
    }
    spi::intf::unlock( cfg.spi_port );

    return result;
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  Status adesto_at25sfxxx_pend_event( const DeviceConfig &cfg, const Event event, const size_t timeout )
  {
    /*-------------------------------------------------------------------------
    Status Register Byte 1 Definitions
    -------------------------------------------------------------------------*/
    static constexpr uint32_t SR_RDY_BUSY_POS = 0;
    static constexpr uint32_t SR_RDY_BUSY_MSK = 0x01;
    static constexpr uint32_t SR_RDY_BUSY     = SR_RDY_BUSY_MSK << SR_RDY_BUSY_POS;

    /*-------------------------------------------------------------------------
    Decide the bits used to indicate events occurred.
    -------------------------------------------------------------------------*/
    uint16_t eventBitMask = 0;  // Indicates bits to look at

    switch ( event )
    {
      case Event::MEM_ERASE_COMPLETE:
      case Event::MEM_WRITE_COMPLETE:
        eventBitMask = SR_RDY_BUSY;
        break;

      default:
        return Status::ERR_NOT_SUPPORTED;
    };

    /*-------------------------------------------------------------------------
    For the AT25SF081, the device is busy with an operation when the RDY/BSY
    flag is set. Assuming this extends to other AT25 devices as well.

    See Table 10-1 of device datasheet.
    -------------------------------------------------------------------------*/
    uint16_t     statusRegister = read_status_register( cfg );
    const size_t startTime      = mb::time::millis();

    while ( ( statusRegister & eventBitMask ) == eventBitMask )
    {
      /*-----------------------------------------------------------------------
      If the timeout has been exceeded, return an error.
      -----------------------------------------------------------------------*/
      if ( ( mb::time::millis() - startTime ) > timeout )
      {
        mbed_assert_continue_msg( false, "Timeout waiting for event: %d", event );
        return Status::ERR_TIMEOUT;
      }

      /*-----------------------------------------------------------------------
      Wait a bit before polling again.
      -----------------------------------------------------------------------*/
      mb::time::delayMilliseconds( mb::thread::TIMEOUT_1MS );
      statusRegister = read_status_register( cfg );
    };

    return Status::ERR_OK;
  }

}    // namespace mb::memory::nor::device
