/* SPDX-License-Identifier: Apache-2.0 */
/* Placeholder for Zephyr's <zephyr/rtio/rtio.h>: only the two names that
 * lsm6dso_fifo_stats.h (an unchanged copy) declares
 * lsm6dso_fifo_request_stop() with.  The RTIO part that the FIFO stream
 * uses comes with the stream (nuttx/PORTING_PLAN.md T11, imu_rtio.[ch]),
 * which replaces this file.  Part of the small Zephyr compatibility layer
 * of the port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_RTIO_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_RTIO_H

#include <stdint.h>

struct rtio;
struct rtio_iodev_sqe;

/* zephyr/rtio/sqe.h */

typedef uint32_t rtio_sqe_handle_t;

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_RTIO_H */
