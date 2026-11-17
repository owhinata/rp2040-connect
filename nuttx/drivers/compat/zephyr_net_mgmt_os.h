/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of the net_mgmt events (common/compat/zephyr_net_mgmt.c,
 * which includes this file at its end): the thread "net_mgmt", a NuttX
 * kernel thread (NANO_PRIO_NET_MGMT) on the file's mgmt_stack, and its
 * start, net_mgmt_event_init().  Part of the NuttX implementation of the
 * Zephyr compatibility layer (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_MGMT_OS_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_MGMT_OS_H

#include <nuttx/kthread.h>

static int mgmt_event_thread(int argc, FAR char *argv[])
{
  static struct mgmt_event_entry mgmt_event;
  k_spinlock_key_t key;

  (void)argc;
  (void)argv;
  for (; ; )
    {
      (void)k_sem_take(&g_queue_used, K_FOREVER);
      key = k_spin_lock(&g_queue_lock);
      mgmt_event = g_queue[g_queue_head];
      g_queue_head = (g_queue_head + 1) % NET_MGMT_EVENT_QUEUE_SIZE;
      k_spin_unlock(&g_queue_lock, key);
      k_sem_give(&g_queue_free);

      mgmt_run_callbacks(&mgmt_event);
    }

  return 0;
}

int net_mgmt_event_init(void)
{
  int pid;

  if (g_started)
    {
      return 0;
    }

  pid = kthread_create_with_stack("net_mgmt", NANO_PRIO_NET_MGMT,
                                  mgmt_stack, sizeof(mgmt_stack),
                                  mgmt_event_thread, NULL);
  if (pid < 0)
    {
      return pid;
    }

  g_started = true;
  return 0;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_MGMT_OS_H */
