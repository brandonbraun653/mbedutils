/******************************************************************************
 *  File Name:
 *    memory_types.hpp
 *
 *  Description:
 *    High level shared types for the memory interface.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_MEMORY_TYPES_HPP
#define MBEDUTILS_MEMORY_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace mb::memory
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Possible status codes returned by a block device driver
   */
  enum Status : uint8_t
  {
    ERR_OK,                 /**< No error occurred */
    ERR_BAD_ARG,            /**< A bad value was passed as a parameter */
    ERR_BAD_CFG,            /**< An invalid configuration setting was used */
    ERR_BAD_STATE,          /**< The driver is in an invalid state */
    ERR_DRIVER_ERR,         /**< A generic driver error occurred */
    ERR_HF_INIT_FAIL,       /**< High frequency initialization failed */
    ERR_NOT_PAGE_ALIGNED,   /**< Memory address is not page aligned */
    ERR_FAIL,               /**< A general failure occurred */
    ERR_OUT_OF_MEMORY,      /**< Not enough memory available */
    ERR_OVERRUN,            /**< Data overrun occurred */
    ERR_PGM_ALIGNMENT,      /**< Program alignment error */
    ERR_PGM_PARALLEL,       /**< Parallel programming error */
    ERR_PGM_SEQUENCE,       /**< Programming sequence error */
    ERR_READ_PROTECT,       /**< Read protection error */
    ERR_TIMEOUT,            /**< Operation timed out */
    ERR_UNALIGNED_MEM,      /**< Unaligned memory access */
    ERR_UNKNOWN_JEDEC,      /**< Unknown JEDEC ID */
    ERR_NOT_SUPPORTED,      /**< Operation not supported */
    ERR_WRITE_PROTECT,      /**< Write protection error */
    ERR_NOT_READY,          /**< Device is not ready */
    ERR_GENERIC,            /**< Generic uncategorized error */
  };

}  // namespace mb::memory

#endif  /* !MBEDUTILS_MEMORY_TYPES_HPP */
