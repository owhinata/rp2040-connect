/* SPDX-License-Identifier: MIT */
/* The Pico SDK's <hardware/flash.h> as flash_kv_os_rp2040.c uses it, for its host
 * test (freertos/tests/kv/main.c implements the functions on a RAM copy of
 * the key-value area).
 */

#ifndef FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_FLASH_H
#define FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_FLASH_H

#include "pico.h"

#define FLASH_PAGE_SIZE   (1u << 8)
#define FLASH_SECTOR_SIZE (1u << 12)

void flash_range_erase(uint32_t flash_offs, size_t count);
void flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);

#endif /* FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_FLASH_H */
