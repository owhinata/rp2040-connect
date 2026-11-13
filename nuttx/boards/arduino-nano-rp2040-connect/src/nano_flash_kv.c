/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_flash_kv.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* The key-value store's flash: an MTD over the last 8 KiB of the 16 MiB
 * flash (0x10ffe000-0x10ffffff, two 4 KiB erase sectors), the place of the
 * Zephyr build's storage_partition, with NuttX's NVS
 * (drivers/mtd/mtd_config_nvs.c) on top as /dev/config.  The image never
 * reaches the area (scripts/nano-flash.ld leaves it out of the flash
 * region; nuttx/script/build.sh checks the image end).
 *
 * Geometry: blocksize 1, erasesize 4096, 2 erase blocks.  mtd_config_nvs.c
 * takes blocksize as its program unit, and with 1 an allocation table
 * entry is 16 bytes and data is not padded (with the flash's real 256-byte
 * page, one entry would take at least 768 bytes).  Writes are done in
 * 256-byte pages, the bytes outside the request set to 0xff, which leaves
 * those bytes of the flash as they are (programming only clears bits).
 * NVS relies on that when it marks an entry as replaced by clearing one
 * byte of its table entry.
 *
 * Safety: an erase or a program outside the area would destroy the
 * firmware.  Every request is checked twice before the flash is touched:
 * in the caller (nano_kv_range_ok() against the board.h constants and
 * against the bounds taken from the linker script), and again in the RAM
 * function itself right before the boot ROM is called.  The bounds are
 * set only after the linker symbols __kv_start/__kv_end have been found
 * equal to the constants and the image found to end before the area;
 * until then every request is refused.
 *
 * Erasing and programming stop XIP, so they run from RAM with interrupts
 * disabled: the boot ROM's connect_internal_flash, flash_exit_xip,
 * flash_range_erase/flash_range_program and flash_flush_cache, then a RAM
 * copy of boot2 to restart XIP in its fast mode (the way pico-sdk's
 * hardware_flash and NuttX's rp2040_flash_mtd.c do it).  Interrupts are off
 * for one 4 KiB sector erase or one 256-byte page program at a time.  The
 * Zephyr driver (drivers/flash/flash_rpi_pico.c) disables them (irq_lock)
 * for one erase or write call: NVS erases one sector per call and writes
 * one entry's data (up to two pages) or one table entry per call.
 * Reads go through the uncached XIP alias (0x13000000).
 *
 * Before NuttX's NVS mounts the area, the area is checked
 * (nano_kv_layout()): if neither block shows NuttX's layout and the area
 * is not erased, it holds something else, typically the Zephyr build's NVS,
 * which NuttX would read as damaged NuttX data.  The area is then erased.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/compiler.h>
#include <nuttx/crc8.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>
#include <arch/board/board.h>

#include "rp2040_rom.h"

#include "app_log.h"
#include "nano_connect.h"
#include "nano_flash_kv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MTD_CONFIG_NVS
#  error "nano_flash_kv.c is the MTD of mtd_config_nvs.c: set MTD_CONFIG_NVS"
#endif

#if CONFIG_MTD_CONFIG_BLOCKSIZE_MULTIPLE != 1
#  error "nano_flash_kv.c expects MTD_CONFIG_BLOCKSIZE_MULTIPLE 1"
#endif

#ifdef CONFIG_SMP
#  error "nano_flash_kv.c does not stop the other core while XIP is off"
#endif

#define KV_XIP_BASE          0x10000000u
#define KV_XIP_NOCACHE_BASE  0x13000000u  /* XIP_NOCACHE_NOALLOC_BASE */
#define KV_BOOT2_SIZE        256u

/* The area as flash offsets, [KV_LO, KV_HI). */

#define KV_LO                ((uint32_t)BOARD_KV_OFFSET)
#define KV_HI                ((uint32_t)(BOARD_KV_OFFSET + BOARD_KV_SIZE))
#define KV_NBLOCKS           (BOARD_KV_SIZE / NANO_KV_SECTOR_SIZE)

#if (BOARD_KV_OFFSET % 4096) != 0 || BOARD_KV_SIZE != 8192 || \
    BOARD_KV_OFFSET + BOARD_KV_SIZE != BOARD_FLASH_SIZE
#  error "the key-value area must be the last two 4 KiB sectors"
#endif

/* pico-sdk's flash_range_erase(): 64 KiB block erases (0xd8) where the
 * range allows, else 4 KiB sector erases (0x20).  Requests here are always
 * one sector.
 */

#define KV_ROM_BLOCK_SIZE    65536u
#define KV_ROM_BLOCK_CMD     0xd8u

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*kv_rom_void_fn)(void);
typedef void (*kv_rom_erase_fn)(uint32_t addr, size_t count,
                                uint32_t block_size, uint8_t block_cmd);
typedef void (*kv_rom_program_fn)(uint32_t addr, const uint8_t *data,
                                  size_t count);

struct kv_rom_s
{
  kv_rom_void_fn    connect_internal_flash;
  kv_rom_void_fn    flash_exit_xip;
  kv_rom_erase_fn   flash_range_erase;
  kv_rom_program_fn flash_range_program;
  kv_rom_void_fn    flash_flush_cache;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     kv_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks);
static ssize_t kv_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks, FAR uint8_t *buffer);
static ssize_t kv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks, FAR const uint8_t *buffer);
static ssize_t kv_read(FAR struct mtd_dev_s *dev, off_t offset,
                       size_t nbytes, FAR uint8_t *buffer);
static int     kv_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* scripts/nano-flash.ld and the startup code. */

extern const uint8_t __kv_start[];
extern const uint8_t __kv_end[];
extern const uint8_t _eronly[];
extern uint8_t _sdata[];
extern uint8_t _edata[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

LOG_MODULE_REGISTER(fs_nvs, LOG_LEVEL_INF);

static struct mtd_dev_s g_kv_mtd =
{
  .erase  = kv_erase,
  .bread  = kv_bread,
  .bwrite = kv_bwrite,
  .read   = kv_read,
#ifdef CONFIG_MTD_BYTE_WRITE
  .write  = kv_bwrite,
#endif
  .ioctl  = kv_ioctl,
  .name   = "nano_kv",
};

/* Serializes the page buffer (NVS already serializes its callers). */

static mutex_t g_kv_lock = NXMUTEX_INITIALIZER;

/* Everything the RAM function touches is in RAM. */

static struct kv_rom_s g_kv_rom;
static uint32_t g_kv_boot2[KV_BOOT2_SIZE / 4];
static uint8_t g_kv_page[NANO_KV_PAGE_SIZE] aligned_data(4);

/* The area's bounds as flash offsets once the linker script has been
 * checked; 0 and 0 refuse everything.
 */

static uint32_t g_kv_lo;
static uint32_t g_kv_hi;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kv_flash_op
 *
 * Description:
 *   Program one page (page != NULL) or erase one sector (page == NULL) at
 *   flash offset addr.  Runs from RAM with interrupts disabled and calls
 *   nothing but the boot ROM and the RAM copy of boot2.  Refuses (returns
 *   -EINVAL without touching the flash) anything outside the area.
 *
 ****************************************************************************/

static int locate_code(".ram_code.kv_flash_op") noinline_function
kv_flash_op(uint32_t addr, FAR const uint8_t *page, uint32_t count)
{
  uint32_t lo = g_kv_lo;
  uint32_t hi = g_kv_hi;

  if (lo == 0 || lo >= hi || addr < lo || addr >= hi || count == 0 ||
      count > hi - addr)
    {
      return -EINVAL;
    }

  __asm__ volatile ("" : : : "memory");

  g_kv_rom.connect_internal_flash();
  g_kv_rom.flash_exit_xip();

  if (page != NULL)
    {
      g_kv_rom.flash_range_program(addr, page, count);
    }
  else
    {
      g_kv_rom.flash_range_erase(addr, count, KV_ROM_BLOCK_SIZE,
                                 KV_ROM_BLOCK_CMD);
    }

  /* Also releases the chip select that the ROM forced. */

  g_kv_rom.flash_flush_cache();

  /* boot2 returns to its caller when called with a non-zero lr. */

  ((kv_rom_void_fn)((uintptr_t)g_kv_boot2 + 1))();

  __asm__ volatile ("" : : : "memory");
  return OK;
}

/****************************************************************************
 * Name: kv_flash
 *
 * Description:
 *   Check a program (page != NULL, count 256) or erase (page == NULL,
 *   count 4096) request at flash offset addr against both copies of the
 *   area's bounds and run it with interrupts disabled.
 *
 ****************************************************************************/

static int kv_flash(uint32_t addr, FAR const uint8_t *page, uint32_t count)
{
  uint32_t align = page != NULL ? NANO_KV_PAGE_SIZE : NANO_KV_SECTOR_SIZE;
  irqstate_t flags;
  int ret;

  if (count != align ||
      !nano_kv_range_ok(addr, count, KV_LO, KV_HI, align) ||
      !nano_kv_range_ok(addr, count, g_kv_lo, g_kv_hi, align))
    {
      LOG_ERR("Refused flash %s at 0x%08lx, %lu bytes",
              page != NULL ? "write" : "erase", (unsigned long)addr,
              (unsigned long)count);
      return -EINVAL;
    }

  flags = up_irq_save();
  ret = kv_flash_op(addr, page, count);
  up_irq_restore(flags);

  return ret;
}

/****************************************************************************
 * Name: kv_erase
 ****************************************************************************/

static int kv_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                    size_t nblocks)
{
  size_t i;
  int ret;

  UNUSED(dev);

  if (startblock < 0 || nblocks > KV_NBLOCKS ||
      (size_t)startblock > KV_NBLOCKS - nblocks)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_kv_lock);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < nblocks; i++)
    {
      ret = kv_flash(KV_LO + ((uint32_t)startblock + i) *
                     NANO_KV_SECTOR_SIZE, NULL, NANO_KV_SECTOR_SIZE);
      if (ret < 0)
        {
          break;
        }
    }

  nxmutex_unlock(&g_kv_lock);
  return ret < 0 ? ret : (int)nblocks;
}

/****************************************************************************
 * Name: kv_read
 ****************************************************************************/

static ssize_t kv_read(FAR struct mtd_dev_s *dev, off_t offset,
                       size_t nbytes, FAR uint8_t *buffer)
{
  UNUSED(dev);

  if (nbytes == 0)
    {
      return 0;
    }

  if (offset < 0 || offset >= BOARD_KV_SIZE || nbytes > BOARD_KV_SIZE ||
      !nano_kv_range_ok(KV_LO + (uint32_t)offset, (uint32_t)nbytes,
                        KV_LO, KV_HI, 1))
    {
      return -EINVAL;
    }

  memcpy(buffer, (FAR const void *)(uintptr_t)(KV_XIP_NOCACHE_BASE + KV_LO +
                                               (uint32_t)offset), nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: kv_bread
 ****************************************************************************/

static ssize_t kv_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks, FAR uint8_t *buffer)
{
  /* blocksize is 1: blocks are bytes. */

  return kv_read(dev, startblock, nblocks, buffer);
}

/****************************************************************************
 * Name: kv_bwrite
 *
 * Description:
 *   Write nblocks bytes (blocksize 1) at offset startblock of the area, a
 *   256-byte page at a time with 0xff around the data.
 *
 ****************************************************************************/

static int kv_program_page(FAR void *arg, uint32_t page_offset,
                           FAR const uint8_t *page)
{
  UNUSED(arg);
  return kv_flash(page_offset, page, NANO_KV_PAGE_SIZE);
}

static ssize_t kv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks, FAR const uint8_t *buffer)
{
  int ret;

  UNUSED(dev);

  if (nblocks == 0)
    {
      return 0;
    }

  if (startblock < 0 || startblock >= BOARD_KV_SIZE ||
      nblocks > BOARD_KV_SIZE ||
      !nano_kv_range_ok(KV_LO + (uint32_t)startblock, (uint32_t)nblocks,
                        KV_LO, KV_HI, 1))
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_kv_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = nano_kv_write_pages(KV_LO + (uint32_t)startblock, buffer,
                            (uint32_t)nblocks, g_kv_page, kv_program_page,
                            NULL);

  nxmutex_unlock(&g_kv_lock);
  return ret < 0 ? ret : (ssize_t)nblocks;
}

/****************************************************************************
 * Name: kv_ioctl
 ****************************************************************************/

static int kv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo == NULL)
            {
              return -EINVAL;
            }

          memset(geo, 0, sizeof(*geo));
          geo->blocksize    = 1;
          geo->erasesize    = NANO_KV_SECTOR_SIZE;
          geo->neraseblocks = KV_NBLOCKS;
          strlcpy(geo->model, "nano_kv", sizeof(geo->model));
          return OK;
        }

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *value = (FAR uint8_t *)((uintptr_t)arg);

          if (value == NULL)
            {
              return -EINVAL;
            }

          *value = NANO_KV_ERASE_VALUE;
          return OK;
        }

      case MTDIOC_BULKERASE:
        {
          int ret = kv_erase(dev, 0, KV_NBLOCKS);

          return ret < 0 ? ret : OK;
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: kv_layout
 *
 * Description:
 *   What the area holds (see nano_flash_kv.h).
 *
 ****************************************************************************/

static enum nano_kv_layout_e kv_layout(void)
{
  unsigned signed_blocks = 0;
  bool erased = true;
  uint8_t tail[2 * NANO_KV_ATE_SIZE];
  uint32_t block;
  uint32_t i;

  for (block = 0; block < KV_NBLOCKS; block++)
    {
      FAR const uint8_t *first = &tail[0];
      FAR const uint8_t *close = &tail[NANO_KV_ATE_SIZE];

      kv_read(&g_kv_mtd, (block + 1) * NANO_KV_SECTOR_SIZE - sizeof(tail),
              sizeof(tail), tail);

      if (nano_kv_ate_plausible(first,
                                crc8part(first, NANO_KV_ATE_CRC_LEN, 0xff),
                                NANO_KV_SECTOR_SIZE, false) ||
          nano_kv_ate_plausible(close,
                                crc8part(close, NANO_KV_ATE_CRC_LEN, 0xff),
                                NANO_KV_SECTOR_SIZE, true))
        {
          signed_blocks++;
        }
    }

  for (i = 0; i < BOARD_KV_SIZE && erased; i += sizeof(tail))
    {
      uint32_t j;

      kv_read(&g_kv_mtd, i, sizeof(tail), tail);
      for (j = 0; j < sizeof(tail); j++)
        {
          if (tail[j] != NANO_KV_ERASE_VALUE)
            {
              erased = false;
              break;
            }
        }
    }

  return nano_kv_layout(signed_blocks, erased);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_flash_kv_initialize
 *
 * Description:
 *   Check the linker script, set up the MTD, erase the area if it holds
 *   something other than NuttX's NVS, and register /dev/config.  Called
 *   once from nano_bringup().
 *
 * Returned Value:
 *   OK, or a negated errno value (then there is no /dev/config).
 *
 ****************************************************************************/

int nano_flash_kv_initialize(void)
{
  uintptr_t image_end = (uintptr_t)_eronly +
                        (uintptr_t)(_edata - _sdata);
  int ret;

  if ((uintptr_t)__kv_start != KV_XIP_BASE + KV_LO ||
      (uintptr_t)__kv_end != KV_XIP_BASE + KV_HI ||
      image_end > (uintptr_t)__kv_start)
    {
      LOG_ERR("Key-value area: linker script disagrees "
              "(0x%08lx-0x%08lx, image end 0x%08lx)",
              (unsigned long)(uintptr_t)__kv_start,
              (unsigned long)(uintptr_t)__kv_end,
              (unsigned long)image_end);
      return -EFAULT;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
  g_kv_rom.connect_internal_flash =
    (kv_rom_void_fn)ROM_LOOKUP(ROM_FLASH_CONNECT);
  g_kv_rom.flash_exit_xip =
    (kv_rom_void_fn)ROM_LOOKUP(ROM_FLASH_EXIT_XIP);
  g_kv_rom.flash_range_erase =
    (kv_rom_erase_fn)ROM_LOOKUP(ROM_FLASH_ERASE);
  g_kv_rom.flash_range_program =
    (kv_rom_program_fn)ROM_LOOKUP(ROM_FLASH_PROGRAM);
  g_kv_rom.flash_flush_cache =
    (kv_rom_void_fn)ROM_LOOKUP(ROM_FLASH_FLUSH_CACHE);
#pragma GCC diagnostic pop

  if (g_kv_rom.connect_internal_flash == NULL ||
      g_kv_rom.flash_exit_xip == NULL ||
      g_kv_rom.flash_range_erase == NULL ||
      g_kv_rom.flash_range_program == NULL ||
      g_kv_rom.flash_flush_cache == NULL)
    {
      LOG_ERR("Key-value area: boot ROM functions not found");
      return -ENOSYS;
    }

  /* boot2 restarts XIP in its fast (quad, continuous read) mode; it must
   * run from RAM because the flash is not readable until it has.
   */

  memcpy(g_kv_boot2, (FAR const void *)KV_XIP_BASE, KV_BOOT2_SIZE);

  g_kv_lo = KV_LO;
  g_kv_hi = KV_HI;

  if (kv_layout() == NANO_KV_LAYOUT_FOREIGN)
    {
      LOG_WRN("Erasing the key-value area: not in this build's format");
      ret = kv_erase(&g_kv_mtd, 0, KV_NBLOCKS);
      if (ret < 0)
        {
          LOG_ERR("Could not erase the key-value area: %d", ret);
          return ret;
        }
    }

  ret = mtdconfig_register(&g_kv_mtd);
  if (ret < 0)
    {
      LOG_ERR("Could not mount the key-value area: %d", ret);
      return ret;
    }

  LOG_INF("%d Sectors of %d bytes", (int)KV_NBLOCKS,
          (int)NANO_KV_SECTOR_SIZE);
  return OK;
}
