/* SPDX-License-Identifier: Apache-2.0 */
/* The board's clock that <zephyr/kernel.h> names, for the host test of
 * zephyr_mutex.c (not called). */
#ifndef TEST_ZMUTEX_NANO_TIMER_H
#define TEST_ZMUTEX_NANO_TIMER_H
#include <stdint.h>
uint64_t nano_mono_us(void);
#endif
