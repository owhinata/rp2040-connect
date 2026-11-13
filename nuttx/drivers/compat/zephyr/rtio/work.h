/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's RTIO work queue (<zephyr/rtio/work.h>, subsys/rtio/rtio_workq.c):
 * a pool of CONFIG_RTIO_WORKQ_POOL_ITEMS (4) requests run in submission
 * order by CONFIG_RTIO_WORKQ_THREADS_POOL (1) thread "RTIO WQ 0" at
 * priority CONFIG_RTIO_WORKQ_THREADS_POOL_PRIO (0, here 118) with a
 * CONFIG_RTIO_WORKQ_THREADS_POOL_STACK_SIZE (1024) stack.  A sensor driver
 * uses it to run a blocking read for an RTIO submission (the LSM6DSO's
 * single reads, lsm6dso_compat.c).  Implemented in zephyr_rtio_workq.c.
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_WORK_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_WORK_H

#include <stdint.h>

#include <zephyr/rtio/rtio.h>

#define CONFIG_RTIO_WORKQ_POOL_ITEMS 4

typedef void (*rtio_work_submit_t)(struct rtio_iodev_sqe *iodev_sqe);

struct rtio_work_req
{
  struct rtio_work_req *next;   /* in the queue */
  struct rtio_iodev_sqe *iodev_sqe;
  rtio_work_submit_t handler;
};

/* A free request, or NULL (all CONFIG_RTIO_WORKQ_POOL_ITEMS in use). */

struct rtio_work_req *rtio_work_req_alloc(void);

/* Queues the handler for the submission; the request is freed after the
 * handler has run (or at once if iodev_sqe or handler is NULL).
 */

void rtio_work_req_submit(struct rtio_work_req *req,
                          struct rtio_iodev_sqe *iodev_sqe,
                          rtio_work_submit_t handler);

uint32_t rtio_work_req_used_count_get(void);

/* Starts the thread (zephyr_compat_initialize()). */

void rtio_workq_initialize(void);

/* Runs the oldest queued request in the caller; false if none (host tests
 * and the thread).
 */

bool rtio_workq_process(void);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_WORK_H */
