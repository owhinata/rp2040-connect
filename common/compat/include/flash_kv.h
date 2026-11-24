/* SPDX-License-Identifier: Apache-2.0 */
/* The key-value area: the last 8 KiB of the 16 MiB flash (flash offsets
 * 0xffe000-0xffffff, XIP 0x10ffe000-0x10ffffff, two 4 KiB erase sectors),
 * the Zephyr build's storage_partition, which holds Zephyr's settings on
 * NVS in every build (the copies of Zephyr's NVS and settings in
 * common/compat/subsys on the flash device of zephyr_flash_kv.c).
 *
 * The OS's part, which reads, programs and erases the flash, is declared
 * in flash_kv_os.h.  This header has the area's constants and the
 * OS-independent helpers, kept here so that the host tests
 * (common/tests/flash_kv.c) can check them: the range check done before
 * every erase and program, the split of a byte-addressed write into
 * 256-byte flash pages, and the check of what the area holds before NVS
 * mounts it: Zephyr's NVS, nothing (all erased), the NuttX build's earlier
 * format (NuttX's NVS, drivers/mtd/mtd_config_nvs.c) or anything else (an
 * OpenMV file system, ...).  Only Zephyr's NVS and an erased area are
 * mounted; the rest is left as it is (<zephyr/storage/flash_map.h>).
 */

#ifndef __COMMON_COMPAT_FLASH_KV_H
#define __COMMON_COMPAT_FLASH_KV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The area as flash offsets, [FLASH_KV_OFFSET, FLASH_KV_END). */

#define FLASH_KV_FLASH_SIZE   0x1000000u   /* 16 MiB */
#define FLASH_KV_OFFSET       0xffe000u
#define FLASH_KV_SIZE         0x2000u
#define FLASH_KV_END          (FLASH_KV_OFFSET + FLASH_KV_SIZE)

/* AT25SF128A: program in 256-byte pages, erase in 4 KiB sectors. */

#define FLASH_KV_PAGE_SIZE    256u
#define FLASH_KV_SECTOR_SIZE  4096u
#define FLASH_KV_SECTORS      (FLASH_KV_SIZE / FLASH_KV_SECTOR_SIZE)
#define FLASH_KV_ERASE_VALUE  0xffu

#if FLASH_KV_END != FLASH_KV_FLASH_SIZE || FLASH_KV_OFFSET % 4096u != 0 || \
    FLASH_KV_SIZE != 8192u
#  error "the key-value area must be the last two 4 KiB sectors"
#endif

/* OS-independent helpers -------------------------------------------------*/

static inline uint32_t flash_kv_le32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint16_t flash_kv_le16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (p[1] << 8));
}

/* True if [offset, offset + length) lies inside [lo, hi), is not empty
 * and starts and ends on a multiple of align (align 1: any byte range).
 * Written so that no sum can wrap around.
 */

static inline bool flash_kv_range_ok(uint32_t offset, uint32_t length,
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

static inline uint32_t flash_kv_page_chunk(uint32_t offset,
                                           uint32_t remaining)
{
  uint32_t room = FLASH_KV_PAGE_SIZE - (offset % FLASH_KV_PAGE_SIZE);

  return remaining < room ? remaining : room;
}

/* Write length bytes from buffer at flash offset `offset`, one 256-byte
 * page at a time: page is filled with FLASH_KV_ERASE_VALUE around the bytes
 * that fall into it and handed to program() with the page's flash offset.
 * Stops at the first negative return of program() and returns it; else 0.
 */

typedef int (*flash_kv_program_t)(void *arg, uint32_t page_offset,
                                  const uint8_t *page);

static inline int flash_kv_write_pages(uint32_t offset,
                                       const uint8_t *buffer,
                                       uint32_t length,
                                       uint8_t page[FLASH_KV_PAGE_SIZE],
                                       flash_kv_program_t program,
                                       void *arg)
{
  while (length > 0)
    {
      uint32_t chunk = flash_kv_page_chunk(offset, length);
      uint32_t in_page = offset % FLASH_KV_PAGE_SIZE;
      uint32_t i;
      int ret;

      for (i = 0; i < FLASH_KV_PAGE_SIZE; i++)
        {
          page[i] = FLASH_KV_ERASE_VALUE;
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

/* Zephyr's NVS (common/compat/subsys/kvss/nvs) with a 1-byte write block:
 * an allocation table entry (ATE) is 8 bytes, struct nvs_ate (id u16,
 * offset u16, len u16, part u8, crc8 u8; crc8 is crc8_ccitt(0xff, ...)
 * over the first 7), written from the end of a sector downwards; data
 * from the start upwards.  The last ATE of a sector (offset 4088, the
 * "close" slot) is written when the sector is full; the one before it
 * (offset 4080) is the first ATE written into a sector: a gc-done marker
 * (id 0xffff, len 0) or an entry moved there by garbage collection.
 */

#define FLASH_KV_NVS_ATE_SIZE     8u
#define FLASH_KV_NVS_ATE_CRC_LEN  7u      /* offsetof(struct nvs_ate, crc8) */
#define FLASH_KV_NVS_ID_SPECIAL   0xffffu

/* Whether the 8 bytes at slot are an ATE that this NVS could have written
 * at sector offset slot_pos (crc is crc8_ccitt(0xff, slot, 7), computed
 * by the caller): nvs_ate_valid() (the CRC, offset + len not past the
 * ATE) and, which NVS does not check but always writes, part 0xff.  With
 * close set, the slot must instead hold a sector close ATE as
 * nvs_close_ate_valid() accepts it (also id 0xffff, len 0, an offset a
 * whole number of ATEs below the sector's end).
 */

static inline bool flash_kv_nvs_ate_valid(const uint8_t slot[8],
                                          uint8_t crc, uint32_t slot_pos,
                                          bool close)
{
  uint16_t id = flash_kv_le16(&slot[0]);
  uint16_t offset = flash_kv_le16(&slot[2]);
  uint16_t len = flash_kv_le16(&slot[4]);

  if (crc != slot[7] || slot[6] != 0xff ||
      (uint32_t)offset + len > slot_pos)
    {
      return false;
    }

  if (close)
    {
      return id == FLASH_KV_NVS_ID_SPECIAL && len == 0 &&
             ((FLASH_KV_SECTOR_SIZE - offset) % FLASH_KV_NVS_ATE_SIZE) == 0;
    }

  return true;
}

/* The NuttX build's earlier format: NuttX's NVS with a 1-byte program
 * unit: an ATE is 16 bytes, struct nvs_ate (12 bytes: id u32, offset u16,
 * len u16, key_len u16, data_crc8, crc8) followed by the "expired" byte
 * and 3 bytes of padding that stay erased.  The last ATE of a block (the
 * "close" slot) is written when the block is full; the one before it is
 * the first ATE written into a block (a gc-done marker or an entry moved
 * by garbage collection).
 */

#define FLASH_KV_NUTTX_ATE_SIZE     16u
#define FLASH_KV_NUTTX_ATE_CRC_LEN  11u  /* offsetof(struct nvs_ate, crc8) */
#define FLASH_KV_NUTTX_ATE_SPECIAL  0xffffffffu

/* NuttX's crc8part(src, len, 0xff) (libs/libc/misc/lib_crc8.c: the
 * reflected polynomial 0xB2, input and output inverted), bit by bit.
 */

static inline uint8_t flash_kv_nuttx_crc8(const uint8_t *src, size_t len)
{
  uint8_t crc = 0xff ^ 0xff;      /* the initial value 0xff, inverted */
  size_t i;
  int bit;

  for (i = 0; i < len; i++)
    {
      crc ^= src[i];
      for (bit = 0; bit < 8; bit++)
        {
          crc = (crc & 1) ? (uint8_t)(0xb2 ^ (crc >> 1)) : (uint8_t)(crc >> 1);
        }
    }

  return crc ^ 0xff;
}

/* Whether the 16 bytes at slot are an ATE that NuttX's NVS could have
 * written there (crc is NuttX's crc8part(slot,
 * FLASH_KV_NUTTX_ATE_CRC_LEN, 0xff), computed by the caller):
 * nvs_ate_valid() of mtd_config_nvs.c (CRC,
 * offset inside the block, a key unless it is a special ATE,
 * nvs_ate_crc8_check()'s rule for data_crc8 when there is no data) and,
 * which NuttX does not check but always leaves so, the 3 padding bytes
 * after the expired byte still erased.  The expired byte itself is not
 * checked.  With close set, the slot must instead hold a block close ATE
 * as nvs_close_ate_valid() accepts it, never expired.
 */

static inline bool flash_kv_nuttx_ate_plausible(const uint8_t slot[16],
                                                uint8_t crc,
                                                uint32_t blocksize,
                                                bool close)
{
  uint32_t id = flash_kv_le32(&slot[0]);
  uint16_t offset = flash_kv_le16(&slot[4]);
  uint16_t len = flash_kv_le16(&slot[6]);
  uint16_t key_len = flash_kv_le16(&slot[8]);
  uint8_t data_crc8 = slot[10];

  if (crc != slot[11])
    {
      return false;
    }

  if (len == 0 && data_crc8 != 0 && data_crc8 != FLASH_KV_ERASE_VALUE)
    {
      return false;
    }

  if (offset >= blocksize - FLASH_KV_NUTTX_ATE_SIZE)
    {
      return false;
    }

  if (key_len == 0 && id != FLASH_KV_NUTTX_ATE_SPECIAL)
    {
      return false;
    }

  if (slot[13] != FLASH_KV_ERASE_VALUE || slot[14] != FLASH_KV_ERASE_VALUE ||
      slot[15] != FLASH_KV_ERASE_VALUE)
    {
      return false;
    }

  if (close)
    {
      return id == FLASH_KV_NUTTX_ATE_SPECIAL && len == 0 && key_len == 0 &&
             slot[12] == FLASH_KV_ERASE_VALUE &&
             ((blocksize - offset) % FLASH_KV_NUTTX_ATE_SIZE) == 0;
    }

  return true;
}

/* What the area holds. */

enum flash_kv_layout_e
{
  FLASH_KV_LAYOUT_NVS = 0,    /* at least one sector in Zephyr's NVS layout */
  FLASH_KV_LAYOUT_EMPTY,      /* all erased */
  FLASH_KV_LAYOUT_NUTTX,      /* the NuttX build's earlier format */
  FLASH_KV_LAYOUT_FOREIGN     /* something else */
};

/* nvs_sectors: the number of sectors whose close slot or first ATE slot
 * holds a valid ATE of Zephyr's NVS (flash_kv_nvs_ate_valid());
 * nuttx_blocks: the number of 4 KiB blocks whose last two 16-byte slots
 * hold a plausible ATE of NuttX's NVS (flash_kv_nuttx_ate_plausible(),
 * with flash_kv_nuttx_crc8()); erased: the whole area reads as
 * FLASH_KV_ERASE_VALUE.  Any sector of Zephyr's NVS makes the area
 * Zephyr's, so that NVS's own recovery (an interrupted
 * garbage collection, a half-written entry, a sector being erased) is
 * left to NVS.  The two layouts do not both match: where NuttX looks
 * (byte 13 of the last 32 and of the last 16 bytes of a block, padding
 * that it requires to be 0xff), a Zephyr sector holds the high byte of an
 * ATE's length, never 0xff, or erased slots, which NuttX rejects by their
 * offset; where Zephyr looks, a NuttX block holds parts of its close ATE,
 * which give an offset of 0xffff or a length of 0xff00 or more, or erased
 * bytes.  common/tests/flash_kv.c checks both formats.
 */

static inline enum flash_kv_layout_e
flash_kv_layout_classify(unsigned nvs_sectors, unsigned nuttx_blocks,
                         bool erased)
{
  if (nvs_sectors > 0)
    {
      return FLASH_KV_LAYOUT_NVS;
    }

  if (nuttx_blocks > 0)
    {
      return FLASH_KV_LAYOUT_NUTTX;
    }

  return erased ? FLASH_KV_LAYOUT_EMPTY : FLASH_KV_LAYOUT_FOREIGN;
}

/* The check itself, through flash_kv_os_read() and crc8_ccitt()
 * (zephyr_flash_kv.c): reads the last 32 bytes of each sector and, unless
 * a sector is Zephyr's, the whole area.  Negative errno if a read fails.
 */

int flash_kv_layout(enum flash_kv_layout_e *layout);

/* Erase the whole area (zephyr_flash_kv.c) and check it again at the next
 * flash_area_open().  For kv_store_wipe(), which holds the settings' lock
 * and initializes the settings again afterwards.
 */

int flash_kv_wipe(void);

#endif /* __COMMON_COMPAT_FLASH_KV_H */
