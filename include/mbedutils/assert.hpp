/******************************************************************************
 *  File Name:
 *    assert.hpp
 *
 *  Description:
 *    Assertion interface for the mbedutils library
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_ASSERT_HPP
#define MBEDUTILS_ASSERT_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

/**
 * @brief Throw an assertion failure if the predicate is false.
 *
 * @param expr Predicate to check. Halts system if false.
 * @param fmt Format string for the assertion message
 * @param ... Arguments to the format string
 */
#define mbed_assert( expr, fmt, ... ) \
  mbedutils::assert::format_and_log_assert_failure( ( expr ), true, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

/**
 * @brief Throw an assertion failure and immediately halt the system.
 *
 * @param fmt Format string for the assertion message
 * @param ... Arguments to the format string
 */
#define mbed_assert_always( fmt, ... ) \
  mbedutils::assert::format_and_log_assert_failure( true, true, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

/**
 * @brief Throws an assertion if the predicate is false, then continues normal operation.
 *
 * @param expr Predicate to check
 * @param fmt Format string for the assertion message
 * @param ... Arguments to the format string
 */
#define mbed_assert_continue( expr, fmt, ... ) \
  mbedutils::assert::format_and_log_assert_failure( ( expr ), false, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

namespace mbedutils::assert
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the assertion driver.
   *
   * This must be called before any of the API functions are used. If not
   * done, you may get some strange behavior when asserts cross power
   * cycling.
   */
  void initialize();

  /**
   * @brief Number of unloggable asserts that occurred this power cycle.
   *
   * Use this to check if you've accidentally introduced an infinite loop
   * of asserts. Should always be zero if your code is correct.
   *
   * @return size_t
   */
  size_t num_recurse_events();

  /**
   * @brief Logs an assertion failure if the predicate is false.
   *
   * This function is reentrant and can be logged from any context. Recursive
   * calls are locked out to prevent infinite loops.
   *
   * @param predicate Assertion to check
   * @param halt Should the system halt on failure. This indicates a critical error.
   * @param file Which file the assertion occurred in
   * @param line Which line the assertion occurred on
   * @param fmt Format string for the assertion message
   * @param ... Arguments to the format string
   */
  void format_and_log_assert_failure( const bool predicate, const bool halt, const char *const file, const int line,
                                      const char *fmt, ... );
}    // namespace mbedutils::assert

#endif /* !MBEDUTILS_ASSERT_HPP */
