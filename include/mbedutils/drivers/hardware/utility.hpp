/******************************************************************************
 *  File Name:
 *    utility.hpp
 *
 *  Description:
 *    Utility functions and classes for the hardware drivers
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_HARDWARE_UTILITY_HPP
#define MBEDUTILS_HARDWARE_UTILITY_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mb::hw
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Available hardware peripherals on the system
   */
  enum Peripheral : size_t
  {
    PERIPH_ADC = 0, /**< Analog to Digital Peripheral */
    PERIPH_CAN,     /**< Controller Area Network */
    PERIPH_CRC,     /**< Cyclic Redundancy Check */
    PERIPH_CRS,     /**< Clock Recovery System */
    PERIPH_DAC,     /**< Digital to Analog Peripheral */
    PERIPH_DMA,     /**< Direct Memory Access */
    PERIPH_EXTI,    /**< External Interrupt */
    PERIPH_FLASH,   /**< System Flash Controller */
    PERIPH_FMC,     /**< Flexible Memory Controller */
    PERIPH_GPIO,    /**< General Purpose Input Output */
    PERIPH_I2C,     /**< Inter-Integrated Circuit */
    PERIPH_I2S,     /**< Inter-IC Sound */
    PERIPH_IWDG,    /**< Independent Watchdog */
    PERIPH_NVIC,    /**< Nested Vector Interrupt Controller */
    PERIPH_PWM,     /**< Pulse Width Modulation Controller */
    PERIPH_PWR,     /**< Power Controller */
    PERIPH_RCC,     /**< Reset and Clock Controller */
    PERIPH_RTC,     /**< Real Time Clock */
    PERIPH_SDIO,    /**< Secure Digital Input Output */
    PERIPH_SPI,     /**< Serial Peripheral Interface */
    PERIPH_SYSCFG,  /**< System Configuration */
    PERIPH_TIMER,   /**< Timer */
    PERIPH_UART,    /**< Universal Asynchronous Receiver Transmitter */
    PERIPH_USART,   /**< Universal Synchronous/Asynchronous Receiver Transmitter */
    PERIPH_VIC,     /**< Vector Interrupt Controller */
    PERIPH_WWDG,    /**< Window Watchdog */
    PERIPH_USB,     /**< Universal Serial Bus */

    NUM_OPTIONS,
    PERIPH_NONE,    /**< Not a peripheral */
    UNKNOWN
  };

}  // namespace mb::hw

#endif  /* !MBEDUTILS_HARDWARE_UTILITY_HPP */
