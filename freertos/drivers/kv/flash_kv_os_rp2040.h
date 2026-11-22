/* SPDX-License-Identifier: MIT */
/* The start of the FreeRTOS build's flash boundary of the key-value area
 * (flash_kv_os_rp2040.c, the boundary of common/compat/flash_kv_os.h on the
 * Pico SDK), for its host test.  Part of the FreeRTOS build
 * (freertos/drivers/kv; flash_kv_os_initialize() is declared in
 * flash_kv_os.h).
 */

#ifndef FREERTOS_DRIVERS_KV_FLASH_KV_OS_RP2040_H
#define FREERTOS_DRIVERS_KV_FLASH_KV_OS_RP2040_H

#include <stdint.h>

/* Lets flash_kv_os_*() erase and program the area if the XIP addresses
 * KV_START and KV_END are the area's (0x10ffe000, 0x11000000) and the image
 * ends (IMAGE_END) inside the flash before KV_START; else refuses every
 * erase and program from then on.  0, or -EFAULT (logged).  Until it has
 * succeeded, flash_kv_os_ready() is false and every erase and program
 * is refused.  flash_kv_os_initialize() calls it with the linker's
 * symbols (__kv_start, __kv_end of
 * freertos/drivers/kv/ld/section_extra_post_end.incl, the SDK's
 * __flash_binary_end); the host test (freertos/tests/kv) with its own
 * values.
 */
int flash_kv_os_rp2040_set_bounds(uintptr_t kv_start, uintptr_t kv_end, uintptr_t image_end);

#endif /* FREERTOS_DRIVERS_KV_FLASH_KV_OS_RP2040_H */
