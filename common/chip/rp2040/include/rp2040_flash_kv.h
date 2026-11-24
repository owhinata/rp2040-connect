/* SPDX-License-Identifier: MIT */
/* The key-value area's flash on the RP2040, shared by the builds that have
 * no flash driver of their own for it (NuttX, FreeRTOS, ThreadX):
 * common/chip/rp2040/rp2040_flash_kv.c implements common/compat/port/flash_kv_os.h
 * on the build's flash operations (rp2040_flash_kv_os.h).  This header has
 * what the host tests call besides flash_kv_os.h.
 */

#ifndef COMMON_CHIP_RP2040_INCLUDE_RP2040_FLASH_KV_H
#define COMMON_CHIP_RP2040_INCLUDE_RP2040_FLASH_KV_H

#include <stdint.h>

/* The RP2040's flash in the address map: XIP_BASE (cached) and
 * XIP_NOCACHE_NOALLOC_BASE (neither looks up nor fills the XIP cache), as
 * in the Pico SDK's hardware/regs/addressmap.h.
 */
#define RP2040_FLASH_KV_XIP_BASE         0x10000000u
#define RP2040_FLASH_KV_XIP_NOCACHE_BASE 0x13000000u

/* What flash_kv_os_initialize() does with the linker script's __kv_start,
 * __kv_end and rp2040_flash_kv_os_image_end(): refuses every erase and
 * program from now on, then, if KV_START and KV_END are the XIP addresses
 * of the area (0x10ffe000, 0x11000000) and the image ends (IMAGE_END)
 * inside the flash and before KV_START, calls rp2040_flash_kv_os_init()
 * and, if that succeeds, accepts requests inside the area.  0, -EFAULT
 * (the linker script, logged) or the error of rp2040_flash_kv_os_init().
 * Erases nothing.
 */
int rp2040_flash_kv_set_bounds(uintptr_t kv_start, uintptr_t kv_end, uintptr_t image_end);

#endif /* COMMON_CHIP_RP2040_INCLUDE_RP2040_FLASH_KV_H */
