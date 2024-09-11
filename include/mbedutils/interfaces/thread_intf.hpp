/******************************************************************************
 *  File Name:
 *    thread_intf.hpp
 *
 *  Description:
 *    Threading interface for mbedutils implementers.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_INTERFACE_HPP
#define MBEDUTILS_THREADING_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <mbedutils/drivers/threading/thread.hpp>

namespace mb::thread::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize();

  mb::thread::TaskHandle create_task( const mb::thread::TaskConfig &cfg );

  void destroy_task( mb::thread::TaskHandle task );

  void set_affinity( mb::thread::TaskHandle task, size_t coreId );

  void start_scheduler();

  void on_stack_overflow();

  void on_malloc_failed();

  void on_idle();

  void on_tick();

}  // namespace mb::thread::intf

#endif  /* !MBEDUTILS_THREADING_INTERFACE_HPP */
