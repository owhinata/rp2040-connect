/* SPDX-License-Identifier: MIT */
/* The start of the FreeRTOS build's flash boundary of the key-value area
 * (nano_flash_kv.c, the boundary of common/compat/nano_flash_kv.h on the
 * Pico SDK).  Part of the FreeRTOS build (freertos/drivers/kv; the NuttX
 * build's counterpart is the board's nano_flash_kv_initialize()).
 */

#ifndef FREERTOS_DRIVERS_KV_NANO_FLASH_KV_OS_H
#define FREERTOS_DRIVERS_KV_NANO_FLASH_KV_OS_H

#include <stdint.h>

/* Checks the linker script's bounds of the area (__kv_start, __kv_end of
 * freertos/drivers/kv/ld/section_extra_post_end.incl) and the end of the
 * image (the SDK's __flash_binary_end) with nano_flash_kv_set_bounds().
 * Call once, from main() before the scheduler starts.  Erases nothing.
 */
int nano_flash_kv_initialize(void);

/* Lets nano_flash_kv_os_*() erase and program the area if the XIP addresses
 * KV_START and KV_END are the area's (0x10ffe000, 0x11000000) and the image
 * ends (IMAGE_END) inside the flash before KV_START; else refuses every
 * erase and program from then on.  0, or -EFAULT (logged).  Until it has
 * succeeded, nano_flash_kv_os_ready() is false and every erase and program
 * is refused.  nano_flash_kv_initialize() calls it with the linker's
 * symbols; the host test (freertos/tests/kv) with its own values.
 */
int nano_flash_kv_set_bounds(uintptr_t kv_start, uintptr_t kv_end, uintptr_t image_end);

#endif /* FREERTOS_DRIVERS_KV_NANO_FLASH_KV_OS_H */
