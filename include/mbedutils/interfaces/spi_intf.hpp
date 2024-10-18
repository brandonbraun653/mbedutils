/******************************************************************************
 *  File Name:
 *    spi_intf.hpp
 *
 *  Description:
 *    Mbedutils interface for SPI hardware
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_SPI_INTF_HPP
#define MBEDUTILS_SPI_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/delegate.h>

namespace mb::hw::spi
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using Port_t     = uint8_t;                       /**< SPI peripheral number */
  using BitWidth_t = uint32_t;                      /**< SPI data bus width */
  using Speed_t    = uint32_t;                      /**< SPI clock speed */
  using Mode_t     = uint32_t;                      /**< SPI clock mode */
  using Polarity_t = uint32_t;                      /**< SPI clock polarity */
  using Phase_t    = uint32_t;                      /**< SPI clock phase */
  using Order_t    = uint32_t;                      /**< SPI data order */
  using Callback_t = etl::delegate<void( void * )>; /**< SPI interrupt callback */

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Provides a register level configuration for an SPI peripheral.
   *
   * The settings provided here are intented to be written directly to the
   * SPI peripheral registers to configure the bus. The actual usage of this
   * information is platform dependent.
   */
  struct SpiConfig
  {
    Port_t     port;         /**< SPI peripheral number */
    BitWidth_t width;        /**< SPI data bus width */
    Speed_t    speed;        /**< SPI clock speed in Hz */
    Mode_t     mode;         /**< SPI clock mode */
    Polarity_t polarity;     /**< SPI clock polarity */
    Phase_t    phase;        /**< SPI clock phase */
    Order_t    order;        /**< SPI data order */
    Callback_t callback;     /**< SPI interrupt callback */
    uint32_t   impl_options; /**< Additional configuration options */
  };

}    // namespace mb::hw::spi


namespace mb::hw::spi::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the implementation specific driver.
   */
  void driver_setup();

  /**
   * @brief Clears any resources used by the driver.
   */
  void driver_teardown();

  /**
   * @brief Configures the SPI peripheral for communication.
   *
   * @param config  Configuration settings for the SPI peripheral
   */
  void init( const mb::hw::spi::SpiConfig &config );

  /**
   * @brief Deinitializes the SPI peripheral.
   *
   * @param port SPI peripheral number
   */
  void deinit( const mb::hw::spi::Port_t port );

  /**
   * @brief Transmits data over the SPI bus.
   *
   * @param port   SPI peripheral number
   * @param data   Data to transmit
   * @param length Number of bytes to transmit
   * @return int   Number of bytes written
   */
  int write( const mb::hw::spi::Port_t port, const void * data, const size_t length );

  /**
   * @brief Receives data over the SPI bus.
   *
   * @param port   SPI peripheral number
   * @param data   Buffer to store received data
   * @param length Number of bytes to receive
   * @return int   Number of bytes read
   */
  int read( const mb::hw::spi::Port_t port, void * data, const size_t length );

  /**
   * @brief Transmits and receives data over the SPI bus.
   *
   * @param port   SPI peripheral number
   * @param tx     Data to transmit
   * @param rx     Buffer to store received data
   * @param length Number of bytes to transfer
   * @return int   Number of bytes actually transferred
   */
  int transfer( const mb::hw::spi::Port_t port, const void * tx, void * rx, const size_t length );

  /**
   * @brief Lock the SPI peripheral for exclusive use by the calling thread.
   *
   * @param port  SPI peripheral number
   */
  void lock( const mb::hw::spi::Port_t port );

  /**
   * @brief Unlock the SPI peripheral for use by other threads.
   *
   * @param port  SPI peripheral number
   */
  void unlock( const mb::hw::spi::Port_t port );
}    // namespace mb::hw::spi::intf

#endif /* !MBEDUTILS_SPI_INTF_HPP */
