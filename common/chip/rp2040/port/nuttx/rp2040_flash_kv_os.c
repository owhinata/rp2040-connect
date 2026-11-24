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

/* The flash operations of the key-value area on NuttX
 * (common/chip/rp2040/port/rp2040_flash_kv_os.h): one 256-byte page
 * program or one 4 KiB sector erase with XIP off and interrupts disabled,
 * with the boot ROM, for the shared part
 * (common/chip/rp2040/rp2040_flash_kv.c), which implements the OS boundary
 * of the area (common/compat/port/flash_kv_os.h) with them and checks every
 * request against the area and the linker script's bounds first.  NuttX's
 * own flash driver for this chip (arch/arm/src/rp2040/rp2040_flash_mtd.c,
 * RP2040_FLASH_FILE_SYSTEM) is not used: its area is the SmartFS region
 * after the image, which it erases to the end of the flash at the first
 * start, and it is an MTD device, not the area's page and sector
 * operations.  The image never reaches the area
 * (scripts/arduino-nano-rp2040-connect-flash.ld leaves it out of the flash
 * region; nuttx/script/build.sh checks the image end).
 *
 * The last check of a request, against the bounds the shared part passes,
 * is in the RAM function itself (kv_flash_op()) right before the boot ROM
 * is called; it refuses without touching the flash.
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
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "rp2040_rom.h"

#include "app_log.h"
#include "flash_kv.h"
#include "rp2040_flash_kv_os.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP
#  error "rp2040_flash_kv_os.c does not stop the other core while XIP is off"
#endif

#define KV_XIP_BASE          0x10000000u
#define KV_BOOT2_SIZE        256u

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

/* The startup code's symbols (the end of the image in the flash). */

extern const uint8_t _eronly[];
extern uint8_t _sdata[];
extern uint8_t _edata[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

LOG_MODULE_REGISTER(flash_kv, LOG_LEVEL_INF);

/* Everything the RAM function touches is in RAM (the page as well: the
 * shared part's copy).
 */

static struct kv_rom_s g_kv_rom;
static uint32_t g_kv_boot2[KV_BOOT2_SIZE / 4];

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
 *   -EINVAL without touching the flash) anything outside [lo, hi).
 *
 ****************************************************************************/

static int locate_code(".ram_code.kv_flash_op") noinline_function
kv_flash_op(uint32_t addr, FAR const uint8_t *page, uint32_t count,
            uint32_t lo, uint32_t hi)
{
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
 *   Run a program (page != NULL, count 256) or erase (page == NULL, count
 *   4096) request at flash offset addr with interrupts disabled.
 *
 ****************************************************************************/

static int kv_flash(uint32_t addr, FAR const uint8_t *page, uint32_t count,
                    uint32_t lo, uint32_t hi)
{
  irqstate_t flags;
  int ret;

  flags = up_irq_save();
  ret = kv_flash_op(addr, page, count, lo, hi);
  up_irq_restore(flags);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_flash_kv_os_init
 *
 * Description:
 *   Find the boot ROM's flash functions and copy boot2
 *   (rp2040_flash_kv_os.h).  Called once by the shared part's
 *   flash_kv_os_initialize() (from rp2040_bringup()) after the linker
 *   script has been checked.  What the area holds is checked when the
 *   settings open it (common/compat/zephyr_flash_kv.c); nothing is erased
 *   here.
 *
 * Returned Value:
 *   OK, or -ENOSYS (then every request is refused).
 *
 ****************************************************************************/

int rp2040_flash_kv_os_init(void)
{
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
  return OK;
}

/****************************************************************************
 * Name: rp2040_flash_kv_os_image_end
 *
 * Description:
 *   The XIP address where the image ends: the code and read-only data
 *   (_eronly) and the initial values of .data stored after them.
 *
 ****************************************************************************/

uintptr_t rp2040_flash_kv_os_image_end(void)
{
  return (uintptr_t)_eronly + (uintptr_t)(_edata - _sdata);
}

/****************************************************************************
 * Name: rp2040_flash_kv_os_program, rp2040_flash_kv_os_erase
 *
 * Description:
 *   One page program, one sector erase (rp2040_flash_kv_os.h).  The page
 *   is the shared part's copy in RAM; the ROM reads it while XIP is off.
 *
 ****************************************************************************/

int rp2040_flash_kv_os_program(uint32_t page_offset,
                               FAR const uint8_t *page, uint32_t lo,
                               uint32_t hi)
{
  return kv_flash(page_offset, page, FLASH_KV_PAGE_SIZE, lo, hi);
}

int rp2040_flash_kv_os_erase(uint32_t sector_offset, uint32_t lo,
                             uint32_t hi)
{
  return kv_flash(sector_offset, NULL, FLASH_KV_SECTOR_SIZE, lo, hi);
}
