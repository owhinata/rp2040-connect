/* SPDX-License-Identifier: Apache-2.0 */
/* The OS's part of the key-value area (flash_kv.h): five functions that
 * each build implements on its flash.  On the RP2040 the NuttX and
 * FreeRTOS builds share them (common/chip/rp2040/rp2040_flash_kv.c), on
 * each build's page program and sector erase (rp2040_flash_kv_os.h: NuttX
 * with the boot ROM, FreeRTOS with the Pico SDK's flash_range_program(),
 * flash_range_erase() and the interrupts off).  zephyr_flash_kv.c calls
 * them only with requests inside the area, aligned as stated, from
 * threads, one at a time (it holds its lock); the OS checks the range again
 * itself and refuses anything else without touching the flash: an erase or
 * a program outside the area destroys the firmware.
 */

#ifndef COMMON_COMPAT_PORT_FLASH_KV_OS_H
#define COMMON_COMPAT_PORT_FLASH_KV_OS_H

#include <stdbool.h>
#include <stdint.h>

#include "flash_kv.h"

/* Called once by the build's start, before any thread uses the store:
 * checks the linker script's bounds of the area and gets the flash
 * operations ready.  0, or a negative errno (then every erase and program
 * is refused and flash_kv_os_ready() stays false).  Erases nothing.
 */

int flash_kv_os_initialize(void);

/* The other four return 0 or a negative errno (-EINVAL for a request
 * outside the area, without touching the flash), are called from threads
 * only, never two at a time, and are not called before flash_kv_os_ready()
 * is true.
 */

/* True once the OS can read, program and erase the area (after
 * flash_kv_os_initialize() has succeeded).  False makes the flash map
 * report the device as not ready (-ENODEV).
 */

bool flash_kv_os_ready(void);

/* Copy LEN bytes of the flash at OFFSET (a flash offset, inside the area)
 * to BUF.  Reads what the flash holds now, also right after a program or
 * an erase (an uncached read, or a cache that the program and the erase
 * flush).
 */

int flash_kv_os_read(uint32_t offset, void *buf, uint32_t len);

/* Program the 256-byte flash page at PAGE_OFFSET (a multiple of 256 inside
 * the area) with PAGE (256 bytes; 0xff where nothing is to be written:
 * programming only clears bits).  Interrupts are off while the flash is
 * out of XIP mode (about 0.4-0.5 ms, nuttx/PORTING_STATUS.md); the code
 * that runs then must not be in the flash.
 */

int flash_kv_os_program(uint32_t page_offset,
                        const uint8_t page[FLASH_KV_PAGE_SIZE]);

/* Erase the 4 KiB sector at SECTOR_OFFSET (a multiple of 4096 inside the
 * area) to 0xff.  Interrupts are off for the whole erase (34-40 ms
 * measured on the NuttX build).
 */

int flash_kv_os_erase(uint32_t sector_offset);

#endif /* COMMON_COMPAT_PORT_FLASH_KV_OS_H */
