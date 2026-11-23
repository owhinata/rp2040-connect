/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/sys/clock.h> (Zephyr v4.5.0) that the layer
 * and the application use, with Zephyr's names and values: the constants
 * (the layer's <zephyr/kernel.h> includes this header, as Zephyr's includes
 * its <zephyr/sys_clock.h>), and the realtime clock, declared (the FreeRTOS
 * build's freertos/drivers/compat/zephyr/sys/clock.h states its results; its
 * implementation on the RP2040 TIMER comes with the time synchronization).
 * Part of the ThreadX implementation of the Zephyr compatibility layer
 * (threadx/drivers/compat).
 */

#ifndef THREADX_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H
#define THREADX_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H

#include <errno.h>
#include <time.h>

#ifndef NSEC_PER_USEC
#define NSEC_PER_USEC 1000U
#endif

#ifndef USEC_PER_MSEC
#define USEC_PER_MSEC 1000U
#endif

#ifndef MSEC_PER_SEC
#define MSEC_PER_SEC 1000U
#endif

#ifndef USEC_PER_SEC
#define USEC_PER_SEC 1000000
#endif

#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC 1000000000U
#endif

#define SYS_CLOCK_REALTIME  1
#define SYS_CLOCK_MONOTONIC 4

int sys_clock_gettime(int clock_id, struct timespec *ts);
int sys_clock_settime(int clock_id, const struct timespec *ts);

#endif /* THREADX_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H */
