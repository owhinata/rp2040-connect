/* SPDX-License-Identifier: MIT */
/* The Pico SDK's <hardware/regs/addressmap.h> as flash_kv_os_rp2040.c uses it,
 * for its host test: XIP_BASE as on the RP2040 (the linker symbols are
 * compared with it), and the uncached XIP alias placed so that the area's
 * flash offsets 0xffe000-0xffffff read the test's RAM copy of the area
 * (kv_test_flash, freertos/tests/kv/main.c).
 */

#ifndef FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_REGS_ADDRESSMAP_H
#define FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_REGS_ADDRESSMAP_H

#include <stdint.h>

extern uint8_t kv_test_flash[];

#define XIP_BASE                 0x10000000u
#define XIP_NOCACHE_NOALLOC_BASE ((uintptr_t)kv_test_flash - 0xffe000u)

#endif /* FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_REGS_ADDRESSMAP_H */
