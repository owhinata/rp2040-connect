/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/sys/clock.h> (Zephyr v4.5.0) that the time
 * synchronization (common/imu/time_sync.c) uses, with Zephyr's names,
 * values and results, on the FreeRTOS build (zephyr_sys_clock.c):
 *
 *   SYS_CLOCK_REALTIME (1), SYS_CLOCK_MONOTONIC (4)
 *   sys_clock_gettime(id, ts)  0, or -EINVAL for another clock
 *   sys_clock_settime(id, ts)  0, or -EINVAL for a clock other than
 *                              SYS_CLOCK_REALTIME or a time whose tv_nsec
 *                              is not in 0..999999999
 *   NSEC_PER_USEC, USEC_PER_SEC, NSEC_PER_SEC
 *
 * The monotonic clock is the RP2040 TIMER (nano_mono_us(), microseconds
 * since the start), and the realtime clock is that plus a base that
 * sys_clock_settime() sets, as Zephyr's realtime clock is its uptime plus an
 * offset (lib/os/clock.c) and NuttX's CLOCK_REALTIME a base added to its
 * monotonic clock (the TIMER, nuttx/drivers/compat/zephyr/sys/clock.h).
 * Both read in microseconds: tv_nsec is a whole number of microseconds.
 * Zephyr's uptime is its kernel tick (100 us, driven by SysTick, which falls
 * behind under load); this one follows the TIMER, as the NuttX build's
 * does.  Part of the FreeRTOS implementation of the small Zephyr
 * compatibility layer of the port (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H

#include <errno.h>
#include <time.h>

#ifndef NSEC_PER_USEC
#define NSEC_PER_USEC 1000U
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

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H */
