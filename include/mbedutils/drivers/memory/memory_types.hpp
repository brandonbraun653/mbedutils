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
    ERR_OK,
    ERR_BAD_ARG,
    ERR_DRIVER_ERR,
    ERR_HF_INIT_FAIL,
    ERR_NOT_PAGE_ALIGNED,
    ERR_FAIL,
    ERR_OUT_OF_MEMORY,
    ERR_OVERRUN,
    ERR_PGM_ALIGNMENT,
    ERR_PGM_PARALLEL,
    ERR_PGM_SEQUENCE,
    ERR_READ_PROTECT,
    ERR_TIMEOUT,
    ERR_UNALIGNED_MEM,
    ERR_UNKNOWN_JEDEC,
    ERR_NOT_SUPPORTED,
    ERR_WRITE_PROTECT,
  };

}  // namespace mb::memory

#endif  /* !MBEDUTILS_MEMORY_TYPES_HPP */
