/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of the RTIO work queue (common/compat/zephyr_rtio_workq.c,
 * which includes this file at its end): the thread "RTIO WQ 0", a NuttX
 * kernel thread on a static stack (NANO_PRIO_RTIO_WQ,
 * NANO_STACK_RTIO_WQ), started by rtio_workq_initialize()
 * (zephyr_compat_initialize()).  Part of the NuttX implementation of the
 * Zephyr compatibility layer (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_WORKQ_OS_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_WORKQ_OS_H

#include <nuttx/kthread.h>

#include "priorities.h"

/* rtio_workq.c rtio_workq_thread_fn(): k_queue_get(.., K_FOREVER). */

static int rtio_workq_thread(int argc, FAR char *argv[])
{
  (void)argc;
  (void)argv;

  for (; ; )
    {
      (void)k_sem_take(&rtio_workq_sem, K_FOREVER);
      (void)rtio_workq_process();
    }

  return 0;
}

static K_THREAD_STACK_DEFINE(rtio_workq_stack, NANO_STACK_RTIO_WQ);

void rtio_workq_initialize(void)
{
  int pid = kthread_create_with_stack("RTIO WQ 0", NANO_PRIO_RTIO_WQ,
                                      rtio_workq_stack,
                                      K_THREAD_STACK_SIZEOF(rtio_workq_stack),
                                      rtio_workq_thread, NULL);

  DEBUGASSERT(pid > 0);
  (void)pid;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_WORKQ_OS_H */
