/* SPDX-License-Identifier: MIT */
/* The Pico SDK's <pico/flash.h> as flash_kv_os_rp2040.c uses it, for its host
 * test (main.c implements flash_safe_execute(): it disables the test's
 * "interrupts" around the function, as the SDK's does with
 * PICO_FLASH_ASSUME_CORE1_SAFE=1, or fails or forgets to on request).
 */

#ifndef FREERTOS_TESTS_KV_PICO_FAKE_PICO_FLASH_H
#define FREERTOS_TESTS_KV_PICO_FAKE_PICO_FLASH_H

#include "pico.h"

#ifndef PICO_FLASH_ASSUME_CORE1_SAFE
#define PICO_FLASH_ASSUME_CORE1_SAFE 0
#endif

int flash_safe_execute(void (*func)(void *), void *param, uint32_t enter_exit_timeout_ms);

#endif /* FREERTOS_TESTS_KV_PICO_FAKE_PICO_FLASH_H */
