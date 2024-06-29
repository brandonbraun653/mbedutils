/******************************************************************************
 *  File Name:
 *    pico_serial.hpp
 *
 *  Description:
 *    RPI Pico SDK based serial driver. This is a wrapper around the Pico SDK
 *    UART peripheral and meant to handle more complex setup and configuration.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_PICO_SERIAL_HPP
#define MBEDUTILS_PICO_SERIAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "hardware/uart.h"

namespace mb::hw::serial::pico
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Configuration descriptor for a Pico SDK UART peripheral
   */
  struct UartConfig
  {
    uart_inst_t  *uart;        /**< UART peripheral instance */
    uart_parity_t parity;      /**< Parity setting */
    uint          baudrate;    /**< Baud rate to operate at */
    uint          tx_pin;      /**< TX pin number */
    uint          rx_pin;      /**< RX pin number */
    uint          dma_channel; /**< DMA channel to use for TX/RX */
    bool          dma_enabled; /**< Use DMA if true, else use interrupts */
    size_t        usr_channel; /**< User's logical channel to identify as */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the Pico SDK UART serial driver
   */
  void initialize();

  /**
   * @brief Configure one of the Pico SDK UART peripherals.
   *
   * This will handle setting up the UART peripheral with the given configuration
   * data, including IO pin assignments, baud rate, and DMA settings. Once this
   * function exits, the UART peripheral should be ready to use.
   *
   * @param config Configuration data to use.
   */
  void configure( const UartConfig &config );

}    // namespace mb::hw::serial::pico

#endif /* !MBEDUTILS_PICO_SERIAL_HPP */
