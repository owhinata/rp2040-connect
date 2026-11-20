/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the net_mgmt events (common/compat/zephyr_net_mgmt.c,
 * which includes this file at its end): the thread "net_mgmt", a task on the
 * file's static mgmt_stack (NANO_PRIO_NET_MGMT, Zephyr's -1 -> 21;
 * NANO_STACK_NET_MGMT 2048 bytes), and its start, net_mgmt_event_init()
 * (freertos/drivers/compat/zephyr_net_init.c, before the interface is initialized,
 * as Zephyr's net_init()).  Part of the FreeRTOS implementation of the
 * Zephyr compatibility layer (freertos/drivers/compat; NuttX's is
 * nuttx/drivers/compat/zephyr_net_mgmt_os.h).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_MGMT_OS_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_MGMT_OS_H

#include "FreeRTOS.h"
#include "task.h"

#include "zephyr_os_internal.h"

/* subsys/net/ip/net_mgmt.c mgmt_event_thread(): the events in order, each
 * copied out of the queue before its callbacks run.
 */
static void mgmt_event_thread(void *arg)
{
	static struct mgmt_event_entry mgmt_event;
	k_spinlock_key_t key;

	(void)arg;
	for (;;) {
		(void)k_sem_take(&g_queue_used, K_FOREVER);
		key = k_spin_lock(&g_queue_lock);
		mgmt_event = g_queue[g_queue_head];
		g_queue_head = (g_queue_head + 1) % NET_MGMT_EVENT_QUEUE_SIZE;
		k_spin_unlock(&g_queue_lock, key);
		k_sem_give(&g_queue_free);

		mgmt_run_callbacks(&mgmt_event);
	}
}

static TaskHandle_t mgmt_task;

int net_mgmt_event_init(void)
{
	int ret;

	if (g_started) {
		return 0;
	}

	ret = z_os_task_start(mgmt_event_thread, "net_mgmt", mgmt_stack,
			      K_THREAD_STACK_SIZEOF(mgmt_stack), NANO_PRIO_NET_MGMT, NULL,
			      &mgmt_task);
	if (ret != 0) {
		return ret;
	}

	g_started = true;
	return 0;
}

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_MGMT_OS_H */
