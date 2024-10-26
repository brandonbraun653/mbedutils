/******************************************************************************
 *  File Name:
 *    assert.hpp
 *
 *  Description:
 *    Assertion interface for the mb library
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
#include <mbedutils/interfaces/util_intf.hpp>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/

/**
 * @brief Throw an assertion failure if the predicate is false.
 *
 * @param expr Predicate to check. Halts system if false.
 */
#define mbed_assert( expr ) \
  ::mb::assert::log_assert_failure( ( expr ), true, __SHORT_FILE__, __LINE__ )


/**
 * @brief Throw an assertion failure if the predicate is false.
 *
 * @param expr Predicate to check. Halts system if false.
 * @param fmt Format string for the assertion message
 * @param ... Arguments to the format string
 */
#define mbed_assert_msg( expr, fmt, ... ) \
  ::mb::assert::format_and_log_assert_failure( ( expr ), true, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )


/**
 * @brief Throw an assertion failure and immediately halt the system.
 */
#define mbed_assert_always() \
  ::mb::assert::log_assert_failure( false, true, __SHORT_FILE__, __LINE__ )

/**
 * @brief Throw an assertion failure and immediately halt the system.
 *
 * @param fmt Format string for the assertion message
 * @param ... Arguments to the format string
 */
#define mbed_assert_always_msg( fmt, ... ) \
  ::mb::assert::format_and_log_assert_failure( false, true, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

/**
 * @brief Throws an assertion if the predicate is false, then continues normal operation.
 *
 * May be used in an 'if' statement to perform addition logic if the assertion fails.
 *
 * @param expr Predicate to check
 * @param fmt Format string for the assertion message
 * @param ... Arguments to the format string
 * @return bool Result of the assertion
 */
#define mbed_assert_continue_msg( expr, fmt, ... ) \
  ::mb::assert::format_and_log_assert_failure( ( expr ), false, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

/**
 * @brief Throws an assertion if the predicate is false, then continues normal operation.
 *
 * May be used in an 'if' statement to perform addition logic if the assertion fails.
 *
 * @return bool Result of the assertion
 */
#define mbed_assert_continue( expr ) \
  ::mb::assert::log_assert_failure( ( expr ), false, __SHORT_FILE__, __LINE__ )

/**
 * @brief Throws an assertion and then continues normal operation.
 *
 * May be used in an 'if' statement to perform addition logic if the assertion fails.
 *
 * @return bool Result of the assertion
 */
#define mbed_assert_continue_always() \
  ::mb::assert::log_assert_failure( false, false, __SHORT_FILE__, __LINE__ )

#if !defined( DEBUG )

#define mbed_dbg_assert( expr ) ( void )0
#define mbed_dbg_assert_msg( expr, fmt, ... ) ( void )0
#define mbed_dbg_assert_always() ( void )0
#define mbed_dbg_assert_always_msg( fmt, ... ) ( void )0
#define mbed_dbg_assert_continue_msg( expr, fmt, ... ) ( void )0
#define mbed_dbg_assert_continue( expr ) ( void )0
#define mbed_dbg_assert_continue_always() ( void )0

#else /* DEBUG */

#define mbed_dbg_assert( expr ) \
  ::mb::assert::log_assert_failure( ( expr ), true, __SHORT_FILE__, __LINE__ )

#define mbed_dbg_assert_msg( expr, fmt, ... ) \
  ::mb::assert::format_and_log_assert_failure( ( expr ), true, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

#define mbed_dbg_assert_always() \
  ::mb::assert::log_assert_failure( false, true, __SHORT_FILE__, __LINE__ )

#define mbed_dbg_assert_always_msg( fmt, ... ) \
  ::mb::assert::format_and_log_assert_failure( false, true, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

#define mbed_dbg_assert_continue_msg( expr, fmt, ... ) \
  ::mb::assert::format_and_log_assert_failure( ( expr ), false, __SHORT_FILE__, __LINE__, fmt, ##__VA_ARGS__ )

#define mbed_dbg_assert_continue( expr ) \
  ::mb::assert::log_assert_failure( ( expr ), false, __SHORT_FILE__, __LINE__ )

#define mbed_dbg_assert_continue_always() \
  ::mb::assert::log_assert_failure( false, false, __SHORT_FILE__, __LINE__ )

#endif /* !DEBUG */

namespace mb::assert
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
   * @param predicate   Assertion to check
   * @param halt    Should the system halt on failure. This indicates a critical error.
   * @param file  Which file the assertion occurred in
   * @param line  Which line the assertion occurred on
   * @return true     If the assertion failed
   * @return false  If the assertion passed
   */
  bool log_assert_failure( const bool predicate, const bool halt, const char *const file, const int line );

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
   * @return bool True if the assertion failed
   */
  bool format_and_log_assert_failure( const bool predicate, const bool halt, const char *const file, const int line,
                                      const char *fmt, ... );
}    // namespace mb::assert

#endif /* !MBEDUTILS_ASSERT_HPP */
