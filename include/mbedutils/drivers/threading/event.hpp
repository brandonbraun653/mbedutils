/******************************************************************************
 *  File Name:
 *    event.hpp
 *
 *  Description:
 *    Mutlithreaded event signaling mechanism
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_THREADING_EVENT_HPP
#define MBEDUTILS_THREADING_EVENT_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum Event : size_t
  {
    EVENT_INVALID,           /**< Special case used for initialization */
    EVENT_READ_COMPLETE,     /**< A read was completed */
    EVENT_WRITE_COMPLETE,    /**< A write was completed */
    EVENT_TXFR_COMPLETE,     /**< A transfer of some sort completed (bi-directional) */
    EVENT_SYSTEM_ERROR,      /**< Catch all error case */
    EVENT_DATA_AVAILABLE,    /**< Some kind of data is ready for processing */
    EVENT_ACK,               /**< Acknowledgement event */
    EVENT_NACK,              /**< Not-Acknowledged event */

    EVENT_NUM_OPTIONS,
    EVENT_UNKNOWN
  };
}  // namespace mb::thread

#endif  /* !MBEDUTILS_THREADING_EVENT_HPP */
