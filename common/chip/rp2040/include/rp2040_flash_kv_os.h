/* SPDX-License-Identifier: MIT */
/* The flash operations of the key-value area on the RP2040 that a build
 * writes for the shared part (common/chip/rp2040/rp2040_flash_kv.c, which
 * implements common/compat/flash_kv_os.h with them): the page program and
 * the sector erase, which stop XIP, and their start.  The NuttX build
 * writes them with the boot ROM
 * (nuttx/boards/arduino-nano-rp2040-connect/src/rp2040_flash_kv_os.c), the
 * FreeRTOS build with the Pico SDK's hardware_flash and pico_flash
 * (freertos/drivers/kv/rp2040_flash_kv_os.c).
 *
 * The shared part calls them only with a request it has checked against
 * the area (common/compat/flash_kv.h) and against the bounds it took from
 * the linker script, from threads, one at a time.  The build checks the
 * request once more against the bounds it is given (lo, hi: flash offsets,
 * [lo, hi); 0 and 0 while the area is not ready) with the interrupts off,
 * right before the flash is touched, and returns -EINVAL without touching
 * it if the request is not inside: an erase or a program outside the area
 * destroys the firmware.
 *
 * Erasing and programming stop XIP: from the boot ROM's
 * connect_internal_flash() and flash_exit_xip() until boot2 (a copy in RAM)
 * has restarted it, nothing may be fetched from the flash.  So the
 * interrupts are off (PRIMASK) for one page program or one sector erase,
 * and the code that runs while XIP is off and the data it reads (the page)
 * are in RAM.  Core 1 is not used by any build (it stays in the boot ROM,
 * which does not fetch from the flash).
 *
 * The build's linker script defines __kv_start and __kv_end, the XIP
 * addresses of the area (0x10ffe000, 0x11000000), and keeps the image out
 * of it.
 */

#ifndef COMMON_CHIP_RP2040_INCLUDE_RP2040_FLASH_KV_OS_H
#define COMMON_CHIP_RP2040_INCLUDE_RP2040_FLASH_KV_OS_H

#include <stdint.h>

/* Gets the flash operations ready (NuttX: finds the boot ROM's functions
 * and copies boot2 into RAM), once, from flash_kv_os_initialize() after
 * the linker script has been checked.  0, or a negative errno (logged;
 * then every erase and program is refused).
 */
int rp2040_flash_kv_os_init(void);

/* The XIP address where the image's contents in the flash end (the start
 * plus the code, the read-only data and the initial values of the data).
 */
uintptr_t rp2040_flash_kv_os_image_end(void);

/* Programs the 256-byte page PAGE (in RAM) at the flash offset PAGE_OFFSET
 * (a multiple of 256) with XIP off and the interrupts off, if
 * [PAGE_OFFSET, PAGE_OFFSET + 256) lies inside [LO, HI) (LO not 0).  0, or
 * a negative errno (-EINVAL: refused without touching the flash).
 */
int rp2040_flash_kv_os_program(uint32_t page_offset, const uint8_t *page, uint32_t lo, uint32_t hi);

/* Erases the 4 KiB sector at the flash offset SECTOR_OFFSET (a multiple of
 * 4096) to 0xff in the same way, if it lies inside [LO, HI).
 */
int rp2040_flash_kv_os_erase(uint32_t sector_offset, uint32_t lo, uint32_t hi);

#endif /* COMMON_CHIP_RP2040_INCLUDE_RP2040_FLASH_KV_OS_H */
