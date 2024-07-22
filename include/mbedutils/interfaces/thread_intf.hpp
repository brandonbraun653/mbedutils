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
  Aliases
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  TaskHandle_t create_task( const TaskConfig &cfg );

  void set_affinity( TaskHandle_t task, size_t coreId );

  void start_scheduler();

}  // namespace mb::thread::intf

#endif  /* !MBEDUTILS_THREADING_INTERFACE_HPP */
