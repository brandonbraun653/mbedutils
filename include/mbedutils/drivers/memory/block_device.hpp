/******************************************************************************
 *  File Name:
 *    block_device.hpp
 *
 *  Description:
 *    Memory interface for a block device driver
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef MBEDUTILS_BLOCK_DEVICE_HPP
#define MBEDUTILS_BLOCK_DEVICE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>
#include <mbedutils/drivers/memory/memory_types.hpp>
#include <mbedutils/drivers/threading/asyncio.hpp>
#include <mbedutils/drivers/threading/lock.hpp>

namespace mb::memory::block_device
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Descriptor for block device attributes.
   *
   * Provides tuning parameters to model a particular device and optimize
   * access patterns for the IBlockDeviceDriver implementations.
   */
  struct Attributes
  {
    uint64_t size;               /**< Total size of the block device in bytes */
    uint32_t read_size;          /**< Optimal block read size in bytes */
    uint32_t write_size;         /**< Optimal block write size in bytes */
    uint32_t erase_size;         /**< Optimal block erase size in bytes */
    uint32_t block_size;         /**< Size of a block in bytes */
    size_t   erase_latency;      /**< Upper bound in milliseconds to erase a block */
    size_t   erase_chip_latency; /**< Upper bound in milliseconds to erase the entire chip */
    size_t   write_latency;      /**< Upper bound in milliseconds to write a 'write_size'-ed block */

    bool is_valid() const
    {
      /* Base validity is non-zero values */
      bool validity = ( size && read_size && write_size && erase_size && erase_latency && erase_chip_latency && write_latency &&
                        block_size );

      /* Ensure erase size is a power of two */
      validity &= ( ( erase_size & ( erase_size - 1 ) ) == 0 );

      /* Ensure multiples of block size */
      if( validity )
      {
        validity &= ( ( size % block_size ) == 0 );
        validity &= ( ( erase_size % block_size ) == 0 );
      }

      return validity;
    }
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief A common interface for block device type memory drivers.
   *
   * All operations are expected to be asynchronous and thread safe, but this
   * requires the driver implementation to take advantage of the provided
   * locking and async IO mechanisms.
   */
  class IBlockDeviceDriver
  {
  public:
    virtual ~IBlockDeviceDriver() = default;

    /**
     * @brief Writes data into the given block.
     *
     * @param block_idx The block id to write data into
     * @param offset    Byte offset into the block
     * @param data      The buffer of data that will be written
     * @param length    Number of bytes to be written
     * @return Status
     */
    virtual Status write( const size_t block_idx, const size_t offset, const void *const data, const size_t length ) = 0;

    /**
     * @brief Writes data at an absolute address.
     *
     * @param address Address to start writing at
     * @param data    The buffer of data that will be written
     * @param length  Number of bytes to be written
     * @return Status
     */
    virtual Status write( const uint64_t address, const void *const data, const size_t length ) = 0;

    /**
     * @brief Reads a contiguous length of memory starting at the given block.
     *
     * @param block_idx The block id to start the read from
     * @param offset    Byte offset into the block
     * @param data      Buffer of data to read into
     * @param length    How many bytes to read out
     * @return Status
     */
    virtual Status read( const size_t block_idx, const size_t offset, void *const data, const size_t length ) = 0;

    /**
     * @brief Reads a contiguous length of memory from an absolute address.
     *
     * @param address Address to begin reading from
     * @param data    Buffer of data to read into
     * @param length  How many bytes to read out
     * @return Status
     */
    virtual Status read( const uint64_t address, void *const data, const size_t length ) = 0;

    /**
     * @brief Erase a range of memory.
     *
     * This must be aligned with the erase size of the device.
     *
     * @param address Address to start erasing from
     * @param size    Number of bytes to erase
     * @return Status
     */
    virtual Status erase( const uint64_t address, const size_t size ) = 0;

    /**
     * @brief Erase an entire block of memory.
     *
     * @param block Which block to erase
     * @return Status
     */
    virtual Status erase( const size_t block_idx ) = 0;

    /**
     * @brief Erases the entire chip.
     * @return Status
     */
    virtual Status erase() = 0;

    /**
     * Flushes any buffered memory to the device.
     * @return Status
     */
    virtual Status flush() = 0;
  };
}    // namespace mb::memory::block_device

#endif /* !MBEDUTILS_BLOCK_DEVICE_HPP */
