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
    uint          data_bits;   /**< Number of data bits */
    uint          stop_bits;   /**< Number of stop bits */
    uint          tx_pin;      /**< TX pin number */
    uint          rx_pin;      /**< RX pin number */
    size_t        usr_channel; /**< User's logical channel to identify as */

    /**
     * @brief Reset the configuration to an empty state
     */
    void reset()
    {
      uart        = nullptr;
      parity      = UART_PARITY_NONE;
      baudrate    = 9600;
      data_bits   = 8;
      stop_bits   = 1;
      tx_pin      = 0;
      rx_pin      = 0;
      usr_channel = 0;
    }
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
