/******************************************************************************
 *  File Name:
 *    jedec_cfi_cmds.hpp
 *
 *  Description:
 *    Flash command definitions which conform to the JEDEC CFI (Common Flash
 *    Interface) described in document JESD68-01. This can be downloaded here:
 *
 *    https://www.jedec.org/standards-documents/docs/jesd-6801
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef FLASH_JEDEC_CFI_HPP
#define FLASH_JEDEC_CFI_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>

namespace mb::memory::cfi
{
  /*---------------------------------------------------------------------------
  General Constants
  ---------------------------------------------------------------------------*/

  static constexpr size_t MAX_CMD_LEN = 5; /**< Max bytes used in cmd sequence. Doesn't include data payload lengths. */

  /*---------------------------------------------------------------------------
  Read Commands
  ---------------------------------------------------------------------------*/

  static constexpr uint8_t READ_ARRAY_HS         = 0x0B;
  static constexpr uint8_t READ_ARRAY_HS_CMD_LEN = 1;
  static constexpr uint8_t READ_ARRAY_HS_OPS_LEN = 5; /**< CMD + 3 address bytes + 1 dummy byte */

  static constexpr uint8_t READ_ARRAY_LS         = 0x03;
  static constexpr uint8_t READ_ARRAY_LS_CMD_LEN = 1;
  static constexpr uint8_t READ_ARRAY_LS_OPS_LEN = 4; /**< CMD + 3 address bytes */

  /*---------------------------------------------------------------------------
  Write Commands
  ---------------------------------------------------------------------------*/

  static constexpr uint8_t PAGE_PROGRAM         = 0x02;
  static constexpr uint8_t PAGE_PROGRAM_CMD_LEN = 1;
  static constexpr uint8_t PAGE_PROGRAM_OPS_LEN = 4; /**< CMD + 3 address bytes */

  /*---------------------------------------------------------------------------
  Erase Commands
  ---------------------------------------------------------------------------*/

  static constexpr uint8_t BLOCK_ERASE_4K  = 0x20;
  static constexpr uint8_t BLOCK_ERASE_32K = 0x52;
  static constexpr uint8_t BLOCK_ERASE_64K = 0xD8;

  static constexpr uint8_t BLOCK_ERASE_CMD_LEN = 1;
  static constexpr uint8_t BLOCK_ERASE_OPS_LEN = 4;

  static constexpr uint8_t CHIP_ERASE         = 0xC7;
  static constexpr uint8_t CHIP_ERASE_CMD_LEN = 1;
  static constexpr uint8_t CHIP_ERASE_OPS_LEN = 1;

  /*---------------------------------------------------------------------------
  Protection Commands
  ---------------------------------------------------------------------------*/

  static constexpr uint8_t WRITE_ENABLE         = 0x06;
  static constexpr uint8_t WRITE_ENABLE_CMD_LEN = 1;
  static constexpr uint8_t WRITE_ENABLE_OPS_LEN = 1;

  static constexpr uint8_t WRITE_DISABLE = 0x04;

  /*---------------------------------------------------------------------------
  Status Commands
  ---------------------------------------------------------------------------*/

  static constexpr uint8_t READ_SR_BYTE1         = 0x05;
  static constexpr uint8_t READ_SR_BYTE1_CMD_LEN = 1;
  static constexpr uint8_t READ_SR_BYTE1_RSP_LEN = 1;
  static constexpr uint8_t READ_SR_BYTE1_OPS_LEN = READ_SR_BYTE1_CMD_LEN + READ_SR_BYTE1_RSP_LEN;

  static constexpr uint8_t READ_SR_BYTE2         = 0x35;
  static constexpr uint8_t READ_SR_BYTE2_CMD_LEN = 1;
  static constexpr uint8_t READ_SR_BYTE2_RSP_LEN = 1;
  static constexpr uint8_t READ_SR_BYTE2_OPS_LEN = READ_SR_BYTE2_CMD_LEN + READ_SR_BYTE2_RSP_LEN;

  static constexpr uint8_t WRITE_SR         = 0x01;
  static constexpr uint8_t WRITE_SR_CMD_LEN = 1;

  static constexpr uint8_t WRITE_EN_VOLATILE_SR         = 0x50;
  static constexpr uint8_t WRITE_EN_VOLATILE_SR_CMD_LEN = 1;

  /*---------------------------------------------------------------------------
  Additional Commands
  ---------------------------------------------------------------------------*/

  static constexpr uint8_t READ_DEV_INFO         = 0x9F;
  static constexpr uint8_t READ_DEV_INFO_CMD_LEN = 1;
  static constexpr uint8_t READ_DEV_INFO_RSP_LEN = 3;
  static constexpr uint8_t READ_DEV_INFO_OPS_LEN = READ_DEV_INFO_CMD_LEN + READ_DEV_INFO_RSP_LEN;

  /**
   * @brief JEDEC Device Info Response
   */
  struct JEDECDeviceInfo
  {
    uint8_t  manufacturer; /**< Manufacturer ID */
    uint8_t  type;         /**< Memory type */
    uint8_t  capacity;     /**< Memory capacity */
  };

}    // namespace mb::memory::cfi

#endif /* !NOR_FLASH_JEDEC_CFI_HPP */
