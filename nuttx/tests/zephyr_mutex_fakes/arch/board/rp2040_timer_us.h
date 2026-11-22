/* SPDX-License-Identifier: Apache-2.0 */
/* The board's clock that <zephyr/kernel.h> names, for the host test of
 * zephyr_mutex.c (not called). */
#ifndef TEST_ZMUTEX_RP2040_TIMER_US_H
#define TEST_ZMUTEX_RP2040_TIMER_US_H
#include <stdint.h>
uint64_t rp2040_timer_us(void);
#endif
