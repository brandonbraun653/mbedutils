/******************************************************************************
 *  File Name:
 *    gpio_intf.hpp
 *
 *  Description:
 *    Mbedutils interface for GPIO hardware
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_GPIO_INTF_HPP
#define MBEDUTILS_GPIO_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/delegate.h>

namespace mb::hw::gpio
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using Pin_t       = uint32_t;                      /**< GPIO pin number */
  using Port_t      = uint32_t;                      /**< GPIO port number */
  using Pull_t      = uint32_t;                      /**< GPIO pull configuration */
  using Speed_t     = uint32_t;                      /**< GPIO speed configuration */
  using Drive_t     = uint32_t;                      /**< GPIO drive configuration */
  using Mode_t      = uint32_t;                      /**< GPIO mode configuration */
  using State_t     = uint32_t;                      /**< GPIO pin state */
  using Alternate_t = uint32_t;                      /**< GPIO alternate function */
  using Trigger_t   = uint32_t;                      /**< GPIO interrupt edge trigger */
  using Callback_t  = etl::delegate<void( void  *)>; /**< GPIO interrupt callback */

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Provides a register level configuration for a GPIO pin.
   *
   * The settings provided here are intented to be written directly to the
   * GPIO peripheral registers to configure the pin. The actual usage of this
   * information is platform dependent.
   */
  struct PinConfig
  {
    Port_t      port;      /**< GPIO port number */
    Pin_t       pin;       /**< GPIO pin number */
    Pull_t      pull;      /**< GPIO pull configuration */
    Speed_t     speed;     /**< GPIO speed configuration */
    Drive_t     drive;     /**< GPIO drive configuration */
    Mode_t      mode;      /**< GPIO mode configuration */
    Alternate_t alternate; /**< GPIO alternate function */
  };

}    // namespace mb::hw::gpio


namespace mb::hw::gpio::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Configures a GPIO pin using the provided settings.
   *
   * @param config    Configuration settings for the pin
   * @return true     If the configuration was successful
   * @return false    If the configuration failed
   */
  bool init( const PinConfig &config );

  /**
   * @brief Writes the state of a GPIO pin.
   *
   * @param port    GPIO port number
   * @param pin     GPIO pin number
   * @param state   Desired state of the pin
   */
  void write( const Port_t port, const Pin_t pin, const State_t state );

  /**
   * @brief Toggles the state of a GPIO pin.
   *
   * @param port    GPIO port number
   * @param pin     GPIO pin number
   */
  void toggle( const Port_t port, const Pin_t pin );

  /**
   * @brief Reads the current state of a GPIO pin.
   *
   * @param port    GPIO port number
   * @param pin     GPIO pin number
   * @return State of the pin
   */
  State_t read( const Port_t port, const Pin_t pin );

  /**
   * @brief Configures the alternate function of a GPIO pin.
   *
   * @param port      GPIO port number
   * @param pin       GPIO pin number
   * @param alternate Desired alternate function
   */
  void setAlternate( const Port_t port, const Pin_t pin, const Alternate_t alternate );

  /**
   * @brief Configures the pull configuration of a GPIO pin.
   *
   * @param port  GPIO port number
   * @param pin   GPIO pin number
   * @param pull  Desired pull configuration
   */
  void setPull( const Port_t port, const Pin_t pin, const Pull_t pull );

  /**
   * @brief Configures the drive strength of a GPIO pin.
   *
   * @param port  GPIO port number
   * @param pin   GPIO pin number
   * @param drive Desired drive strength
   */
  void setDrive( const Port_t port, const Pin_t pin, const Drive_t drive );

  /**
   * @brief Configures the speed of a GPIO pin.
   *
   * @param port  GPIO port number
   * @param pin   GPIO pin number
   * @param speed Desired speed setting
   */
  void setSpeed( const Port_t port, const Pin_t pin, const Speed_t speed );

  /**
   * @brief Sets the mode of a GPIO pin.
   *
   * @param port  GPIO port number
   * @param pin   GPIO pin number
   * @param mode  Desired mode setting
   */
  void setMode( const Port_t port, const Pin_t pin, const Mode_t mode );

  /**
   * @brief Configures the GPIO pin to trigger an interrupt on a specific edge.
   *
   * @param port      GPIO port number
   * @param pin       GPIO pin number
   * @param trigger   Desired edge trigger
   * @param callback  Function to call when the interrupt is triggered
   */
  void attachInterrupt( const Port_t port, const Pin_t pin, const Trigger_t trigger, const Callback_t &callback );

  /**
   * @brief Detaches the interrupt from a GPIO pin.
   *
   * @param port  GPIO port number
   * @param pin   GPIO pin number
   */
  void detachInterrupt( const Port_t port, const Pin_t pin );

  /**
   * @brief Retrieves the interrupt line number for a specific GPIO pin.
   *
   * @param port  GPIO port number
   * @param pin   GPIO pin number
   * @return uint32_t
   */
  uint32_t getInterruptLine( const Port_t port, const Pin_t pin );
}    // namespace mb::hw::gpio::intf

#endif /* !MBEDUTILS_GPIO_INTF_HPP */
