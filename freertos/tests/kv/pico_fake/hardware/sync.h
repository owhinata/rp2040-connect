/* SPDX-License-Identifier: MIT */
/* The Pico SDK's <hardware/sync.h> as flash_kv_os_rp2040.c uses it, for its host
 * test: the interrupt mask is a variable of the test (main.c), 1 while the
 * "interrupts" are off, as PRIMASK.
 */

#ifndef FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_SYNC_H
#define FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_SYNC_H

#include <stdint.h>

uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t status);

#endif /* FREERTOS_TESTS_KV_PICO_FAKE_HARDWARE_SYNC_H */
