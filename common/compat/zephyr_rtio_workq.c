/* SPDX-License-Identifier: Apache-2.0 */
/* The RTIO work queue of <zephyr/rtio/work.h>: Zephyr's
 * subsys/rtio/rtio_workq.c with its k_mem_slab of requests as a fixed
 * array and its k_queue as a first-in first-out list and a semaphore.  The
 * list and the pool are OS-independent; the thread ("RTIO WQ 0") is the
 * OS's, in "zephyr_rtio_workq_os.h", which this file includes at its end.
 * Part of the small Zephyr compatibility layer of the port, its
 * OS-independent part (common/compat).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/rtio/work.h>

static struct rtio_work_req rtio_work_items[CONFIG_RTIO_WORKQ_POOL_ITEMS];
static bool rtio_work_used[CONFIG_RTIO_WORKQ_POOL_ITEMS];
static struct rtio_work_req *rtio_workq_head;
static struct rtio_work_req *rtio_workq_tail;
static struct k_spinlock rtio_workq_lock;
static K_SEM_DEFINE(rtio_workq_sem, 0, K_SEM_MAX_LIMIT);

/* k_mem_slab_alloc(.., K_NO_WAIT) */

struct rtio_work_req *rtio_work_req_alloc(void)
{
  k_spinlock_key_t key = k_spin_lock(&rtio_workq_lock);
  struct rtio_work_req *req = NULL;

  for (int i = 0; i < CONFIG_RTIO_WORKQ_POOL_ITEMS; i++)
    {
      if (!rtio_work_used[i])
        {
          rtio_work_used[i] = true;
          req = &rtio_work_items[i];
          break;
        }
    }

  k_spin_unlock(&rtio_workq_lock, key);
  return req;
}

static void rtio_work_req_free(struct rtio_work_req *req)
{
  k_spinlock_key_t key = k_spin_lock(&rtio_workq_lock);

  rtio_work_used[req - rtio_work_items] = false;
  k_spin_unlock(&rtio_workq_lock, key);
}

void rtio_work_req_submit(struct rtio_work_req *req,
                          struct rtio_iodev_sqe *iodev_sqe,
                          rtio_work_submit_t handler)
{
  k_spinlock_key_t key;

  if (!req)
    {
      return;
    }

  if (!iodev_sqe || !handler)
    {
      rtio_work_req_free(req);
      return;
    }

  req->iodev_sqe = iodev_sqe;
  req->handler = handler;

  /* For now we're simply treating this as a FIFO queue (k_queue_append()). */

  key = k_spin_lock(&rtio_workq_lock);
  req->next = NULL;
  if (rtio_workq_tail != NULL)
    {
      rtio_workq_tail->next = req;
    }
  else
    {
      rtio_workq_head = req;
    }

  rtio_workq_tail = req;
  k_spin_unlock(&rtio_workq_lock, key);
  k_sem_give(&rtio_workq_sem);
}

uint32_t rtio_work_req_used_count_get(void)
{
  k_spinlock_key_t key = k_spin_lock(&rtio_workq_lock);
  uint32_t used = 0;

  for (int i = 0; i < CONFIG_RTIO_WORKQ_POOL_ITEMS; i++)
    {
      used += rtio_work_used[i];
    }

  k_spin_unlock(&rtio_workq_lock, key);
  return used;
}

bool rtio_workq_process(void)
{
  k_spinlock_key_t key = k_spin_lock(&rtio_workq_lock);
  struct rtio_work_req *req = rtio_workq_head;

  if (req != NULL)
    {
      rtio_workq_head = req->next;
      if (rtio_workq_head == NULL)
        {
          rtio_workq_tail = NULL;
        }
    }

  k_spin_unlock(&rtio_workq_lock, key);
  if (req == NULL)
    {
      return false;
    }

  req->handler(req->iodev_sqe);
  rtio_work_req_free(req);
  return true;
}

/* The OS part: the thread "RTIO WQ 0" that takes rtio_workq_sem and runs
 * rtio_workq_process(), started by rtio_workq_initialize() (on NuttX a
 * kernel thread on a static stack, nuttx/drivers/compat).
 */

#include "zephyr_rtio_workq_os.h"
