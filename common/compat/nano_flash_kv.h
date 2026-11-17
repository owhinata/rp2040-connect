/* SPDX-License-Identifier: Apache-2.0 */
/* The key-value area: the last 8 KiB of the 16 MiB flash (flash offsets
 * 0xffe000-0xffffff, XIP 0x10ffe000-0x10ffffff, two 4 KiB erase sectors),
 * the Zephyr build's storage_partition, which holds Zephyr's settings on
 * NVS in every build (the copies of Zephyr's NVS and settings in
 * common/compat/subsys on the flash device of zephyr_flash_kv.c).
 *
 * This header has two parts:
 *
 * 1. The boundary to the OS (below, nano_flash_kv_os_*()): three small
 *    functions that each OS implements on its flash (NuttX:
 *    nuttx/boards/arduino-nano-rp2040-connect/src/nano_flash_kv.c, with
 *    the boot ROM; the Pico SDK: flash_range_program(),
 *    flash_range_erase() with the interrupts off).  zephyr_flash_kv.c
 *    calls them only with requests inside the area, aligned as stated,
 *    from threads, one at a time (it holds its lock); the OS checks the
 *    range again itself and refuses anything else without touching the
 *    flash: an erase or a program outside the area destroys the firmware.
 *
 * 2. OS-independent helpers, kept here so that the host tests
 *    (common/tests/kv_flash.c) can check them: the range check done before
 *    every erase and program, the split of a byte-addressed write into
 *    256-byte flash pages, and the check of what the area holds before
 *    NVS mounts it: Zephyr's NVS, nothing (all erased), the NuttX build's
 *    earlier format (NuttX's NVS, drivers/mtd/mtd_config_nvs.c) or
 *    anything else (an OpenMV file system, ...).  Only Zephyr's NVS and an
 *    erased area are mounted; the rest is left as it is
 *    (<zephyr/storage/flash_map.h>).
 */

#ifndef __COMMON_COMPAT_NANO_FLASH_KV_H
#define __COMMON_COMPAT_NANO_FLASH_KV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The area as flash offsets, [NANO_KV_OFFSET, NANO_KV_END). */

#define NANO_KV_FLASH_SIZE   0x1000000u   /* 16 MiB */
#define NANO_KV_OFFSET       0xffe000u
#define NANO_KV_SIZE         0x2000u
#define NANO_KV_END          (NANO_KV_OFFSET + NANO_KV_SIZE)

/* AT25SF128A: program in 256-byte pages, erase in 4 KiB sectors. */

#define NANO_KV_PAGE_SIZE    256u
#define NANO_KV_SECTOR_SIZE  4096u
#define NANO_KV_SECTORS      (NANO_KV_SIZE / NANO_KV_SECTOR_SIZE)
#define NANO_KV_ERASE_VALUE  0xffu

#if NANO_KV_END != NANO_KV_FLASH_SIZE || NANO_KV_OFFSET % 4096u != 0 || \
    NANO_KV_SIZE != 8192u
#  error "the key-value area must be the last two 4 KiB sectors"
#endif

/* 1. The boundary to the OS ------------------------------------------------
 *
 * All three return 0 or a negative errno (-EINVAL for a request outside
 * the area, without touching the flash), are called from threads only,
 * never two at a time, and are not called before nano_flash_kv_os_ready()
 * is true.
 */

/* True once the OS can read, program and erase the area (NuttX: after the
 * board has checked the linker script and found the boot ROM's functions).
 * False makes the flash map report the device as not ready (-ENODEV).
 */

bool nano_flash_kv_os_ready(void);

/* Copy LEN bytes of the flash at OFFSET (a flash offset, inside the area)
 * to BUF.  Reads what the flash holds now, also right after a program or
 * an erase (an uncached read, or a cache that the program and the erase
 * flush).
 */

int nano_flash_kv_os_read(uint32_t offset, void *buf, uint32_t len);

/* Program the 256-byte flash page at PAGE_OFFSET (a multiple of 256 inside
 * the area) with PAGE (256 bytes; 0xff where nothing is to be written:
 * programming only clears bits).  Interrupts are off while the flash is
 * out of XIP mode (about 0.4-0.5 ms, nuttx/PORTING_STATUS.md); the code
 * that runs then must not be in the flash.
 */

int nano_flash_kv_os_program(uint32_t page_offset,
                             const uint8_t page[NANO_KV_PAGE_SIZE]);

/* Erase the 4 KiB sector at SECTOR_OFFSET (a multiple of 4096 inside the
 * area) to 0xff.  Interrupts are off for the whole erase (34-40 ms
 * measured on the NuttX build).
 */

int nano_flash_kv_os_erase(uint32_t sector_offset);

/* 2. OS-independent helpers ------------------------------------------------*/

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

/* Zephyr's NVS (common/compat/subsys/kvss/nvs) with a 1-byte write block:
 * an allocation table entry (ATE) is 8 bytes, struct nvs_ate (id u16,
 * offset u16, len u16, part u8, crc8 u8; crc8 is crc8_ccitt(0xff, ...)
 * over the first 7), written from the end of a sector downwards; data
 * from the start upwards.  The last ATE of a sector (offset 4088, the
 * "close" slot) is written when the sector is full; the one before it
 * (offset 4080) is the first ATE written into a sector: a gc-done marker
 * (id 0xffff, len 0) or an entry moved there by garbage collection.
 */

#define NANO_KV_NVS_ATE_SIZE     8u
#define NANO_KV_NVS_ATE_CRC_LEN  7u      /* offsetof(struct nvs_ate, crc8) */
#define NANO_KV_NVS_ID_SPECIAL   0xffffu

/* Whether the 8 bytes at slot are an ATE that this NVS could have written
 * at sector offset slot_pos (crc is crc8_ccitt(0xff, slot, 7), computed
 * by the caller): nvs_ate_valid() (the CRC, offset + len not past the
 * ATE) and, which NVS does not check but always writes, part 0xff.  With
 * close set, the slot must instead hold a sector close ATE as
 * nvs_close_ate_valid() accepts it (also id 0xffff, len 0, an offset a
 * whole number of ATEs below the sector's end).
 */

static inline bool nano_kv_nvs_ate_valid(const uint8_t slot[8], uint8_t crc,
                                         uint32_t slot_pos, bool close)
{
  uint16_t id = nano_kv_le16(&slot[0]);
  uint16_t offset = nano_kv_le16(&slot[2]);
  uint16_t len = nano_kv_le16(&slot[4]);

  if (crc != slot[7] || slot[6] != 0xff ||
      (uint32_t)offset + len > slot_pos)
    {
      return false;
    }

  if (close)
    {
      return id == NANO_KV_NVS_ID_SPECIAL && len == 0 &&
             ((NANO_KV_SECTOR_SIZE - offset) % NANO_KV_NVS_ATE_SIZE) == 0;
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

#define NANO_KV_ATE_SIZE     16u
#define NANO_KV_ATE_CRC_LEN  11u  /* offsetof(struct nvs_ate, crc8) */
#define NANO_KV_ATE_SPECIAL  0xffffffffu

/* NuttX's crc8part(src, len, 0xff) (libs/libc/misc/lib_crc8.c: the
 * reflected polynomial 0xB2, input and output inverted), bit by bit.
 */

static inline uint8_t nano_kv_nuttx_crc8(const uint8_t *src, size_t len)
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
 * written there (crc is NuttX's crc8part(slot, NANO_KV_ATE_CRC_LEN, 0xff),
 * computed by the caller): nvs_ate_valid() of mtd_config_nvs.c (CRC,
 * offset inside the block, a key unless it is a special ATE,
 * nvs_ate_crc8_check()'s rule for data_crc8 when there is no data) and,
 * which NuttX does not check but always leaves so, the 3 padding bytes
 * after the expired byte still erased.  The expired byte itself is not
 * checked.  With close set, the slot must instead hold a block close ATE
 * as nvs_close_ate_valid() accepts it, never expired.
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
  NANO_KV_LAYOUT_NVS = 0,    /* at least one sector in Zephyr's NVS layout */
  NANO_KV_LAYOUT_EMPTY,      /* all erased */
  NANO_KV_LAYOUT_NUTTX,      /* the NuttX build's earlier format */
  NANO_KV_LAYOUT_FOREIGN     /* something else */
};

/* nvs_sectors: the number of sectors whose close slot or first ATE slot
 * holds a valid ATE of Zephyr's NVS (nano_kv_nvs_ate_valid());
 * nuttx_blocks: the number of 4 KiB blocks whose last two 16-byte slots
 * hold a plausible ATE of NuttX's NVS (nano_kv_ate_plausible(), with
 * nano_kv_nuttx_crc8()); erased:
 * the whole area reads as NANO_KV_ERASE_VALUE.  Any sector of Zephyr's NVS
 * makes the area Zephyr's, so that NVS's own recovery (an interrupted
 * garbage collection, a half-written entry, a sector being erased) is
 * left to NVS.  The two layouts do not both match: where NuttX looks
 * (byte 13 of the last 32 and of the last 16 bytes of a block, padding
 * that it requires to be 0xff), a Zephyr sector holds the high byte of an
 * ATE's length, never 0xff, or erased slots, which NuttX rejects by their
 * offset; where Zephyr looks, a NuttX block holds parts of its close ATE,
 * which give an offset of 0xffff or a length of 0xff00 or more, or erased
 * bytes.  common/tests/kv_flash.c checks both formats.
 */

static inline enum nano_kv_layout_e nano_kv_layout(unsigned nvs_sectors,
                                                   unsigned nuttx_blocks,
                                                   bool erased)
{
  if (nvs_sectors > 0)
    {
      return NANO_KV_LAYOUT_NVS;
    }

  if (nuttx_blocks > 0)
    {
      return NANO_KV_LAYOUT_NUTTX;
    }

  return erased ? NANO_KV_LAYOUT_EMPTY : NANO_KV_LAYOUT_FOREIGN;
}

/* The check itself, through nano_flash_kv_os_read() and crc8_ccitt()
 * (zephyr_flash_kv.c): reads the last 32 bytes of each sector and, unless
 * a sector is Zephyr's, the whole area.  Negative errno if a read fails.
 */

int nano_flash_kv_layout(enum nano_kv_layout_e *layout);

/* Erase the whole area (zephyr_flash_kv.c) and check it again at the next
 * flash_area_open().  For kv_store_wipe(), which holds the settings' lock
 * and initializes the settings again afterwards.
 */

int nano_flash_kv_wipe(void);

#endif /* __COMMON_COMPAT_NANO_FLASH_KV_H */
