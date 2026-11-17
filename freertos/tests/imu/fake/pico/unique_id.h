/* SPDX-License-Identifier: MIT */
/* A fake of the Pico SDK's pico/unique_id.h for the host test of
 * freertos/tests/imu: the test sets the ID that
 * pico_get_unique_board_id() returns (main.c).
 */

#ifndef FREERTOS_TESTS_IMU_PICO_UNIQUE_ID_H
#define FREERTOS_TESTS_IMU_PICO_UNIQUE_ID_H

#include <stdint.h>

#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8

typedef struct {
	uint8_t id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
} pico_unique_board_id_t;

void pico_get_unique_board_id(pico_unique_board_id_t *id_out);

#endif /* FREERTOS_TESTS_IMU_PICO_UNIQUE_ID_H */
