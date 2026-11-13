/* SPDX-License-Identifier: Apache-2.0 */
/* The event part of <zephyr/net/net_mgmt.h>: Zephyr's
 * subsys/net/ip/net_mgmt.c with CONFIG_NET_MGMT_EVENT_THREAD,
 * _EVENT_QUEUE, _EVENT_INFO, _EVENT_QUEUE_SIZE=5 and _QUEUE_TIMEOUT=10 (the
 * Zephyr build's values).  The event is copied into a static entry under a
 * lock (so that the caller's stack does not hold it) and put in the queue,
 * waiting up to 10 ms for room; the "net_mgmt" thread takes the events in
 * order and runs the callbacks under the callback lock.  See the header.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/kthread.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "priorities.h"

#define NET_MGMT_EVENT_QUEUE_SIZE     5    /* CONFIG_NET_MGMT_EVENT_QUEUE_SIZE */
#define NET_MGMT_EVENT_QUEUE_TIMEOUT  10   /* ..._QUEUE_TIMEOUT, ms */

/* The largest information an event carries (Zephyr:
 * NET_EVENT_INFO_MAX_SIZE, the largest of the event structures).
 */

#define NET_EVENT_INFO_MAX_SIZE \
  (sizeof(struct wifi_scan_result) > sizeof(struct wifi_status) ? \
   sizeof(struct wifi_scan_result) : sizeof(struct wifi_status))

struct mgmt_event_entry
{
  uint64_t event;
  struct net_if *iface;
  uint8_t info[NET_EVENT_INFO_MAX_SIZE] __aligned(4);
  size_t info_length;
};

/* The queue: k_msgq of Zephyr, as a ring with a count of free and used
 * slots.
 */

static struct mgmt_event_entry g_queue[NET_MGMT_EVENT_QUEUE_SIZE];
static unsigned int g_queue_head;
static unsigned int g_queue_tail;
static K_SEM_DEFINE(g_queue_free, NET_MGMT_EVENT_QUEUE_SIZE,
                    NET_MGMT_EVENT_QUEUE_SIZE);
static K_SEM_DEFINE(g_queue_used, 0, NET_MGMT_EVENT_QUEUE_SIZE);
static struct k_spinlock g_queue_lock;

static K_MUTEX_DEFINE(net_mgmt_event_lock);
static struct mgmt_event_entry new_event;

static K_MUTEX_DEFINE(net_mgmt_callback_lock);
static struct net_mgmt_event_callback *g_callbacks;

static K_THREAD_STACK_DEFINE(mgmt_stack, NANO_STACK_NET_MGMT);
static bool g_started;

static void mgmt_push_event(uint64_t mgmt_event, struct net_if *iface,
                            const void *info, size_t length)
{
  k_spinlock_key_t key;

  (void)k_mutex_lock(&net_mgmt_event_lock, K_FOREVER);
  memset(&new_event, 0, sizeof(new_event));
  if (info != NULL && length > 0)
    {
      if (length > NET_EVENT_INFO_MAX_SIZE)
        {
          (void)k_mutex_unlock(&net_mgmt_event_lock);
          return;
        }

      memcpy(new_event.info, info, length);
      new_event.info_length = length;
    }

  new_event.event = mgmt_event;
  new_event.iface = iface;

  /* k_msgq_put(&event_msgq, &new_event, K_MSEC(10)): dropped silently
   * when there is no room by then (NET_WARN() is compiled out in the
   * Zephyr build).
   */

  if (k_sem_take(&g_queue_free, K_MSEC(NET_MGMT_EVENT_QUEUE_TIMEOUT)) == 0)
    {
      key = k_spin_lock(&g_queue_lock);
      g_queue[g_queue_tail] = new_event;
      g_queue_tail = (g_queue_tail + 1) % NET_MGMT_EVENT_QUEUE_SIZE;
      k_spin_unlock(&g_queue_lock, key);
      k_sem_give(&g_queue_used);
    }

  (void)k_mutex_unlock(&net_mgmt_event_lock);
}

static void mgmt_run_callbacks(const struct mgmt_event_entry *mgmt_event)
{
  struct net_mgmt_event_callback *cb;
  struct net_mgmt_event_callback *next;

  (void)k_mutex_lock(&net_mgmt_callback_lock, K_FOREVER);
  for (cb = g_callbacks; cb != NULL; cb = next)
    {
      next = cb->next;
      if ((cb->event_mask & mgmt_event->event) == 0)
        {
          continue;
        }

      if (mgmt_event->info_length > 0)
        {
          cb->info = mgmt_event->info;
          cb->info_length = mgmt_event->info_length;
        }
      else
        {
          cb->info = NULL;
          cb->info_length = 0;
        }

      cb->handler(cb, mgmt_event->event, mgmt_event->iface);
    }

  (void)k_mutex_unlock(&net_mgmt_callback_lock);
}

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

void net_mgmt_add_event_callback(struct net_mgmt_event_callback *cb)
{
  (void)k_mutex_lock(&net_mgmt_callback_lock, K_FOREVER);
  cb->next = g_callbacks;
  g_callbacks = cb;
  (void)k_mutex_unlock(&net_mgmt_callback_lock);
}

void net_mgmt_del_event_callback(struct net_mgmt_event_callback *cb)
{
  struct net_mgmt_event_callback **p;

  (void)k_mutex_lock(&net_mgmt_callback_lock, K_FOREVER);
  for (p = &g_callbacks; *p != NULL; p = &(*p)->next)
    {
      if (*p == cb)
        {
          *p = cb->next;
          break;
        }
    }

  (void)k_mutex_unlock(&net_mgmt_callback_lock);
}

void net_mgmt_event_notify_with_info(uint64_t mgmt_event,
                                     struct net_if *iface,
                                     const void *info, size_t length)
{
  mgmt_push_event(mgmt_event, iface, info, length);
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
