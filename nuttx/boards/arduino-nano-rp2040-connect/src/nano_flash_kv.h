/* SPDX-License-Identifier: Apache-2.0 */
/* The OS-independent parts of nano_flash_kv.c (the MTD over the last 8 KiB
 * of the flash that holds the key-value store), kept apart so that the host
 * tests (nuttx/tests/nano_flash_kv.c) can check them:
 *
 * - the range checks done before every erase and program: a request is
 *   refused unless it lies entirely inside the key-value area,
 * - the split of a byte-addressed write into 256-byte flash pages,
 * - the check of what the area holds before NuttX's NVS
 *   (drivers/mtd/mtd_config_nvs.c) mounts it: data in NuttX's own layout,
 *   nothing (all erased) or anything else, such as the Zephyr build's NVS.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_FLASH_KV_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_FLASH_KV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* AT25SF128A: program in 256-byte pages, erase in 4 KiB sectors. */

#define NANO_KV_PAGE_SIZE    256u
#define NANO_KV_SECTOR_SIZE  4096u
#define NANO_KV_ERASE_VALUE  0xffu

/* NuttX NVS with a 1-byte program unit (the MTD reports blocksize 1): an
 * allocation table entry (ATE) is 16 bytes, struct nvs_ate (12 bytes:
 * id u32, offset u16, len u16, key_len u16, data_crc8, crc8) followed by
 * the "expired" byte and 3 bytes of padding that stay erased.  The last
 * ATE of a block (the "close" slot) is written when the block is full; the
 * one before it is the first ATE written into a block (a gc-done marker or
 * an entry moved by garbage collection).
 */

#define NANO_KV_ATE_SIZE     16u
#define NANO_KV_ATE_CRC_LEN  11u  /* offsetof(struct nvs_ate, crc8) */
#define NANO_KV_ATE_SPECIAL  0xffffffffu

static inline uint32_t nano_kv_le32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint16_t nano_kv_le16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (p[1] << 8));
}

/* True if [offset, offset + length) lies inside [lo, hi), is not empty
 * and starts and ends on a multiple of align (align 1: any byte range).
 * Written so that no sum can wrap around.
 */

static inline bool nano_kv_range_ok(uint32_t offset, uint32_t length,
                                    uint32_t lo, uint32_t hi,
                                    uint32_t align)
{
  if (align == 0 || lo >= hi || length == 0)
    {
      return false;
    }

  if (offset < lo || offset >= hi || length > hi - offset)
    {
      return false;
    }

  return (offset % align) == 0 && (length % align) == 0;
}

/* The number of bytes of a write at offset (of length remaining) that go
 * into the flash page holding offset.
 */

static inline uint32_t nano_kv_page_chunk(uint32_t offset,
                                          uint32_t remaining)
{
  uint32_t room = NANO_KV_PAGE_SIZE - (offset % NANO_KV_PAGE_SIZE);

  return remaining < room ? remaining : room;
}

/* Write length bytes from buffer at flash offset `offset`, one 256-byte
 * page at a time: page is filled with NANO_KV_ERASE_VALUE around the bytes
 * that fall into it and handed to program() with the page's flash offset.
 * Stops at the first negative return of program() and returns it; else 0.
 */

typedef int (*nano_kv_program_t)(void *arg, uint32_t page_offset,
                                 const uint8_t *page);

static inline int nano_kv_write_pages(uint32_t offset, const uint8_t *buffer,
                                      uint32_t length,
                                      uint8_t page[NANO_KV_PAGE_SIZE],
                                      nano_kv_program_t program, void *arg)
{
  while (length > 0)
    {
      uint32_t chunk = nano_kv_page_chunk(offset, length);
      uint32_t in_page = offset % NANO_KV_PAGE_SIZE;
      uint32_t i;
      int ret;

      for (i = 0; i < NANO_KV_PAGE_SIZE; i++)
        {
          page[i] = NANO_KV_ERASE_VALUE;
        }

      for (i = 0; i < chunk; i++)
        {
          page[in_page + i] = buffer[i];
        }

      ret = program(arg, offset - in_page, page);
      if (ret < 0)
        {
          return ret;
        }

      buffer += chunk;
      offset += chunk;
      length -= chunk;
    }

  return 0;
}

/* Whether the 16 bytes at slot are an ATE that NuttX's NVS could have
 * written there (crc is crc8part(slot, NANO_KV_ATE_CRC_LEN, 0xff), computed
 * by the caller): nvs_ate_valid() of mtd_config_nvs.c (CRC, offset inside
 * the block, a key unless it is a special ATE, nvs_ate_crc8_check()'s rule
 * for data_crc8 when there is no data) and, which NuttX does not check but
 * always leaves so, the 3 padding bytes after the expired byte still
 * erased.  The expired byte itself is not checked (it is cleared when the
 * entry is replaced and could be half written after a power loss).
 * With close set, the slot must instead hold a block close ATE as
 * nvs_close_ate_valid() accepts it, never expired.
 */

static inline bool nano_kv_ate_plausible(const uint8_t slot[16],
                                         uint8_t crc, uint32_t blocksize,
                                         bool close)
{
  uint32_t id = nano_kv_le32(&slot[0]);
  uint16_t offset = nano_kv_le16(&slot[4]);
  uint16_t len = nano_kv_le16(&slot[6]);
  uint16_t key_len = nano_kv_le16(&slot[8]);
  uint8_t data_crc8 = slot[10];

  if (crc != slot[11])
    {
      return false;
    }

  if (len == 0 && data_crc8 != 0 && data_crc8 != NANO_KV_ERASE_VALUE)
    {
      return false;
    }

  if (offset >= blocksize - NANO_KV_ATE_SIZE)
    {
      return false;
    }

  if (key_len == 0 && id != NANO_KV_ATE_SPECIAL)
    {
      return false;
    }

  if (slot[13] != NANO_KV_ERASE_VALUE || slot[14] != NANO_KV_ERASE_VALUE ||
      slot[15] != NANO_KV_ERASE_VALUE)
    {
      return false;
    }

  if (close)
    {
      return id == NANO_KV_ATE_SPECIAL && len == 0 && key_len == 0 &&
             slot[12] == NANO_KV_ERASE_VALUE &&
             ((blocksize - offset) % NANO_KV_ATE_SIZE) == 0;
    }

  return true;
}

/* What the area holds. */

enum nano_kv_layout_e
{
  NANO_KV_LAYOUT_NUTTX = 0,  /* at least one block in NuttX's NVS layout */
  NANO_KV_LAYOUT_EMPTY,      /* all erased */
  NANO_KV_LAYOUT_FOREIGN     /* something else: erase before mounting */
};

/* signed_blocks: the number of blocks whose close slot or first ATE slot
 * is plausible (nano_kv_ate_plausible()); erased: the whole area reads as
 * NANO_KV_ERASE_VALUE.  Any NuttX block keeps the area as it is, so that
 * NuttX's own recovery (an interrupted garbage collection, a half-written
 * entry) is never replaced by an erase.
 */

static inline enum nano_kv_layout_e nano_kv_layout(unsigned signed_blocks,
                                                   bool erased)
{
  if (signed_blocks > 0)
    {
      return NANO_KV_LAYOUT_NUTTX;
    }

  return erased ? NANO_KV_LAYOUT_EMPTY : NANO_KV_LAYOUT_FOREIGN;
}

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_FLASH_KV_H */
