/* SPDX-License-Identifier: MIT */
/* The per-OS <zephyr/sys/clock.h> of the layer for the ThreadX host test
 * (the FreeRTOS build's is freertos/drivers/compat/zephyr/sys/clock.h, with
 * its realtime clock in zephyr_sys_clock.c): the constants, and the
 * realtime clock declared (the kernel test does not call it).
 */

#ifndef THREADX_TESTS_COMPAT_ZEPHYR_SYS_CLOCK_H
#define THREADX_TESTS_COMPAT_ZEPHYR_SYS_CLOCK_H

#include <time.h>

#define NSEC_PER_USEC 1000U
#define USEC_PER_MSEC 1000U
#define MSEC_PER_SEC  1000U
#define USEC_PER_SEC  1000000
#define NSEC_PER_SEC  1000000000U

#define SYS_CLOCK_REALTIME  1
#define SYS_CLOCK_MONOTONIC 4

int sys_clock_gettime(int clock_id, struct timespec *ts);
int sys_clock_settime(int clock_id, const struct timespec *ts);

#endif /* THREADX_TESTS_COMPAT_ZEPHYR_SYS_CLOCK_H */
