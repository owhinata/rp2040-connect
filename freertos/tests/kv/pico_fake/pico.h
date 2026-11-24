/* SPDX-License-Identifier: MIT */
/* The part of the Pico SDK that common/chip/rp2040/port/pico_sdk/rp2040_flash_kv_os.c uses,
 * for its host test (freertos/tests/kv/main.c, which implements the
 * functions): the board's flash size and the SDK's result codes.  The
 * hardware/ and pico/ headers next to this one are the test's as well.
 */

#ifndef FREERTOS_TESTS_KV_PICO_FAKE_PICO_H
#define FREERTOS_TESTS_KV_PICO_FAKE_PICO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* boards/arduino_nano_rp2040_connect.h */
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)

/* pico/error.h */
#define PICO_OK                  0
#define PICO_ERROR_NOT_PERMITTED (-4)

#endif /* FREERTOS_TESTS_KV_PICO_FAKE_PICO_H */
