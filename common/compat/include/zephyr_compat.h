/* SPDX-License-Identifier: Apache-2.0 */
/* The start of the Zephyr compatibility layer (common/compat), for
 * the board's bring-up and the init task, which do not include the layer's
 * Zephyr headers (they take over some of the OS's names, see
 * <zephyr/sys/atomic.h>).  zephyr_compat_initialize() is OS-independent
 * (zephyr_init.c), on the OS's start of a thread.
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_COMPAT_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_COMPAT_H

/* Starts the system work queue ("sysworkq"), the RTIO work queue
 * ("RTIO WQ 0", zephyr_rtio_workq.c) and, with the LSM6DSO's FIFO stream
 * (CONFIG_LSM6DSO_STREAM), the gpio0 device: what Zephyr starts at
 * boot (POST_KERNEL) before the sensors.  The board calls it in
 * its bring-up, before the LSM6DSOX.  0 or a negative errno.
 */

int zephyr_compat_initialize(void);

/* Starts the thread that a source defined with K_THREAD_DEFINE(name, ...)
 * of <zephyr/kernel.h>; 0 or a negative errno.  Zephyr starts every such
 * thread before main(); the application's table of threads starts each one
 * with this, before the part that does what main() does
 * (app_threads_start(), common/imu/app_init.c).
 */

#define K_THREAD_DEFINE_START(name) \
  ({ \
    extern int z_thread_start_##name(void); \
    z_thread_start_##name(); \
  })

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_COMPAT_H */
