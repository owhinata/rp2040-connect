/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/rp2040_flash_kv_os.c
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

/* The OS boundary of the key-value area (common/compat/flash_kv_os.h):
 * reads, 256-byte page programs and 4 KiB sector erases of the last 8 KiB
 * of the 16 MiB flash (0x10ffe000-0x10ffffff, two sectors), the place of
 * the Zephyr build's storage_partition.  The copies of Zephyr's NVS and
 * settings (common/compat/subsys) keep the store there in the Zephyr
 * build's format through the flash device of
 * common/compat/zephyr_flash_kv.c, which calls these functions.  The image
 * never reaches the area (scripts/arduino-nano-rp2040-connect-flash.ld
 * leaves it out of the flash region; nuttx/script/build.sh checks the image
 * end).
 *
 * Safety: an erase or a program outside the area would destroy the
 * firmware.  Every request is checked three times before the flash is
 * touched: in zephyr_flash_kv.c (against the constants of flash_kv.h),
 * here (flash_kv_range_ok() against the board.h constants and against the
 * bounds taken from the linker script), and again in the RAM function
 * itself right before the boot ROM is called.
 * The bounds are set only after the linker symbols __kv_start/__kv_end
 * have been found equal to the constants and the image found to end
 * before the area; until then every request is refused and
 * flash_kv_os_ready() is false.
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
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "rp2040_rom.h"

#include "app_log.h"
#include "rp2040_pico.h"
#include "flash_kv.h"
#include "flash_kv_os.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP
#  error "rp2040_flash_kv_os.c does not stop the other core while XIP is off"
#endif

#define KV_XIP_BASE          0x10000000u
#define KV_XIP_NOCACHE_BASE  0x13000000u  /* XIP_NOCACHE_NOALLOC_BASE */
#define KV_BOOT2_SIZE        256u

/* The area as flash offsets, [KV_LO, KV_HI). */

#define KV_LO                ((uint32_t)BOARD_KV_OFFSET)
#define KV_HI                ((uint32_t)(BOARD_KV_OFFSET + BOARD_KV_SIZE))

#if BOARD_KV_OFFSET != FLASH_KV_OFFSET || BOARD_KV_SIZE != FLASH_KV_SIZE || \
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
 * Public Data
 ****************************************************************************/

/* scripts/arduino-nano-rp2040-connect-flash.ld and the startup code. */

extern const uint8_t __kv_start[];
extern const uint8_t __kv_end[];
extern const uint8_t _eronly[];
extern uint8_t _sdata[];
extern uint8_t _edata[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

LOG_MODULE_REGISTER(flash_kv, LOG_LEVEL_INF);

/* Everything the RAM function touches is in RAM. */

static struct kv_rom_s g_kv_rom;
static uint32_t g_kv_boot2[KV_BOOT2_SIZE / 4];
static uint8_t g_kv_page[FLASH_KV_PAGE_SIZE] aligned_data(4);

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
  uint32_t align = page != NULL ? FLASH_KV_PAGE_SIZE : FLASH_KV_SECTOR_SIZE;
  irqstate_t flags;
  int ret;

  if (count != align ||
      !flash_kv_range_ok(addr, count, KV_LO, KV_HI, align) ||
      !flash_kv_range_ok(addr, count, g_kv_lo, g_kv_hi, align))
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flash_kv_os_ready, flash_kv_os_read,
 *       flash_kv_os_program, flash_kv_os_erase
 *
 * Description:
 *   The boundary of common/compat/flash_kv_os.h.  The program copies
 *   the page into RAM first: the caller's page may be anywhere, and the
 *   ROM reads it while XIP is off.
 *
 ****************************************************************************/

bool flash_kv_os_ready(void)
{
  return g_kv_lo != 0;
}

int flash_kv_os_read(uint32_t offset, FAR void *buf, uint32_t len)
{
  if (!flash_kv_range_ok(offset, len, KV_LO, KV_HI, 1))
    {
      LOG_ERR("Refused flash read at 0x%08lx, %lu bytes",
              (unsigned long)offset, (unsigned long)len);
      return -EINVAL;
    }

  memcpy(buf, (FAR const void *)(uintptr_t)(KV_XIP_NOCACHE_BASE + offset),
         len);
  return OK;
}

int flash_kv_os_program(uint32_t page_offset,
                        FAR const uint8_t page[FLASH_KV_PAGE_SIZE])
{
  memcpy(g_kv_page, page, FLASH_KV_PAGE_SIZE);
  return kv_flash(page_offset, g_kv_page, FLASH_KV_PAGE_SIZE);
}

int flash_kv_os_erase(uint32_t sector_offset)
{
  return kv_flash(sector_offset, NULL, FLASH_KV_SECTOR_SIZE);
}

/****************************************************************************
 * Name: flash_kv_os_initialize
 *
 * Description:
 *   Check the linker script, find the boot ROM's flash functions and copy
 *   boot2, then accept requests.  Called once from rp2040_bringup().  What
 *   the area holds is checked when the settings open it
 *   (common/compat/zephyr_flash_kv.c); nothing is erased here.
 *
 * Returned Value:
 *   OK, or a negated errno value (then every request is refused).
 *
 ****************************************************************************/

int flash_kv_os_initialize(void)
{
  uintptr_t image_end = (uintptr_t)_eronly +
                        (uintptr_t)(_edata - _sdata);

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
  return OK;
}
