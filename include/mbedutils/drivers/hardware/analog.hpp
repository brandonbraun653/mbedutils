/******************************************************************************
 *  File Name:
 *    analog.hpp
 *
 *  Description:
 *    Analog processing interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_ANALOG_HPP
#define MBEDUTILS_ANALOG_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cmath>

namespace Analog
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Computes the voltage input to a voltage divider circuit
   *
   * @param vOut    Measured output voltage
   * @param r1      Resistor 1 value
   * @param r2      Resistor 2 value
   * @return float  The input voltage to the divider
   */
  static inline float calculateVoltageDividerInput( const float vOut, const float r1, const float r2 )
  {
    return vOut * ( r1 + r2 ) / r2;
  }

  /**
   * @brief Calculates temperature using the beta parameter method for a thermistor.
   *
   * This function uses the beta parameter method to calculate the temperature
   * based on the voltage output of a voltage divider circuit containing a thermistor.
   * It assumes an NTC (Negative Temperature Coefficient) thermistor is being used.
   *
   * @param vOut    The measured output voltage of the voltage divider.
   * @param vin     The input voltage to the voltage divider.
   * @param beta    The beta coefficient of the thermistor, typically provided by the manufacturer.
   * @param r_fixed The resistance of the fixed resistor in the voltage divider.
   * @param r0      The resistance of the thermistor at the reference temperature.
   * @param t0      The reference temperature in Celsius.
   * @return The calculated temperature in Celsius.
   *
   * @note This function assumes the thermistor is the lower resistor in the voltage divider.
   *       If it's the upper resistor, the voltage divider equation needs to be adjusted.
   *
   * @warning The beta parameter method is an approximation and may not be accurate
   *          over a wide temperature range. Consult the thermistor's datasheet for
   *          the valid temperature range for the beta value.
   *
   * @see For more information on the beta parameter method:
   *      https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
   */
  static inline float calculateTempBeta( const float vOut, const float vin, const float beta, const float r_fixed,
                                         const float r0, const float t0 )
  {
    const float r_thermistor = r_fixed * ( vin / vOut - 1.0f );
    const float invT         = 1.0f / ( t0 + 273.15f ) + ( 1.0f / beta ) * logf( r_thermistor / r0 );
    return ( 1.0f / invT ) - 273.15f;    // Convert back to Celsius
  }
}    // namespace Analog

#endif  /* !MBEDUTILS_ANALOG_HPP */
