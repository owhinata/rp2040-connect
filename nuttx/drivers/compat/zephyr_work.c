/* SPDX-License-Identifier: Apache-2.0 */
/* Work items and work queues of <zephyr/kernel.h>: the parts of Zephyr's
 * kernel/work.c (Zephyr v4.5.0) that the port uses, with the same flags,
 * decisions and return values.  OS-independent: it uses k_spin_lock() (one
 * lock for all items, Zephyr's work_lock), k_sem_give()/k_sem_take() and
 * z_add_timeout()/z_abort_timeout(), which zephyr_kernel.c provides on
 * NuttX and a fake provides in host tests.
 *
 * Kept from kernel/work.c:
 * - submit_to_queue_locked(): an item that is already queued is not queued
 *   again (0); a running item is queued again on the queue that runs it
 *   (2); a canceling one is refused (-EBUSY).
 * - work_queue_main(): RUNNING is set and QUEUED cleared before the
 *   handler runs, so the handler (or anyone) can queue the item again;
 *   after it, RUNNING is cleared and a cancellation is finished; the
 *   thread yields if more items are pending (unless no_yield).
 * - k_work_schedule_for_queue(): nothing if the item is delayed or queued
 *   (running is allowed); K_NO_WAIT submits at once, else DELAYED and a
 *   timer.  k_work_reschedule_for_queue(): stops the timer first.
 * - work_timeout(): whoever clears DELAYED first (the timer or an
 *   unschedule) decides; the timer submits to the queue it was scheduled
 *   on.
 * - k_work_cancel_sync(): removes a queued item; if it is still running,
 *   CANCELING is set (submissions get -EBUSY) and the caller waits until
 *   the handler has returned.
 * Not kept: flushing, draining, plugging, stopping a queue, the work
 * timeout watchdog, k_work_cancel(), k_work_cancel_delayable(), more than
 * one thread waiting in k_work_cancel_sync() for the same item.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

/* kernel/work.c work_lock: guards every item's flags and every queue. */

static struct k_spinlock work_lock;

static inline bool flag_test(const uint32_t *flagp, uint32_t bit)
{
  return (*flagp & BIT(bit)) != 0U;
}

static inline void flag_set(uint32_t *flagp, uint32_t bit)
{
  *flagp |= BIT(bit);
}

static inline void flag_clear(uint32_t *flagp, uint32_t bit)
{
  *flagp &= ~BIT(bit);
}

static inline bool flag_test_and_clear(uint32_t *flagp, int bit)
{
  bool ret = flag_test(flagp, bit);

  flag_clear(flagp, bit);
  return ret;
}

static inline int work_busy_get_locked(const struct k_work *work)
{
  return work->flags & K_WORK_MASK;
}

/* The queue's pending list (Zephyr: a sys_slist). */

static void pending_append(struct k_work_q *queue, struct k_work *work)
{
  work->next = NULL;
  if (queue->tail != NULL)
    {
      queue->tail->next = work;
    }
  else
    {
      queue->head = work;
    }

  queue->tail = work;
}

static bool pending_remove(struct k_work_q *queue, struct k_work *work)
{
  struct k_work *prev = NULL;
  struct k_work *cur;

  for (cur = queue->head; cur != NULL; prev = cur, cur = cur->next)
    {
      if (cur == work)
        {
          if (prev != NULL)
            {
              prev->next = cur->next;
            }
          else
            {
              queue->head = cur->next;
            }

          if (queue->tail == cur)
            {
              queue->tail = prev;
            }

          cur->next = NULL;
          return true;
        }
    }

  return false;
}

static struct k_work *pending_get(struct k_work_q *queue)
{
  struct k_work *work = queue->head;

  if (work != NULL)
    {
      queue->head = work->next;
      if (queue->head == NULL)
        {
          queue->tail = NULL;
        }

      work->next = NULL;
    }

  return work;
}

/* kernel/work.c finalize_cancel_locked(): the handler has returned. */

static void finalize_cancel_locked(struct k_work *work)
{
  flag_clear(&work->flags, K_WORK_CANCELING_BIT);
  if (work->canceller != NULL)
    {
      k_sem_give(&work->canceller->sem);
      work->canceller = NULL;
    }
}

/* kernel/work.c queue_remove_locked() */

static inline void queue_remove_locked(struct k_work_q *queue,
                                       struct k_work *work)
{
  if (flag_test_and_clear(&work->flags, K_WORK_QUEUED_BIT))
    {
      (void)pending_remove(queue, work);
    }
}

/* kernel/work.c queue_submit_locked(): no draining or plugging here. */

static inline int queue_submit_locked(struct k_work_q *queue,
                                      struct k_work *work)
{
  if (queue == NULL)
    {
      return -EINVAL;
    }

  if (!flag_test(&queue->flags, K_WORK_QUEUE_STARTED_BIT))
    {
      return -ENODEV;
    }

  pending_append(queue, work);
  k_sem_give(&queue->notify);
  return 1;
}

/* kernel/work.c submit_to_queue_locked() */

static int submit_to_queue_locked(struct k_work *work,
                                  struct k_work_q **queuep)
{
  int ret = 0;

  if (flag_test(&work->flags, K_WORK_CANCELING_BIT))
    {
      /* Disallowed */

      ret = -EBUSY;
    }
  else if (!flag_test(&work->flags, K_WORK_QUEUED_BIT))
    {
      /* Not currently queued */

      ret = 1;

      /* If no queue specified resubmit to last queue. */

      if (*queuep == NULL)
        {
          *queuep = work->queue;
        }

      /* If the work is currently running we have to use the queue it's
       * running on to prevent handler re-entrancy.
       */

      if (flag_test(&work->flags, K_WORK_RUNNING_BIT))
        {
          __ASSERT_NO_MSG(work->queue != NULL);
          *queuep = work->queue;
          ret = 2;
        }

      int rc = queue_submit_locked(*queuep, work);

      if (rc < 0)
        {
          ret = rc;
        }
      else
        {
          flag_set(&work->flags, K_WORK_QUEUED_BIT);
          work->queue = *queuep;
        }
    }
  else
    {
      /* Already queued, do nothing. */
    }

  if (ret <= 0)
    {
      *queuep = NULL;
    }

  return ret;
}

void k_work_init(struct k_work *work, k_work_handler_t handler)
{
  __ASSERT_NO_MSG(work != NULL);
  __ASSERT_NO_MSG(handler != NULL);

  *work = (struct k_work){ .handler = handler };
}

int k_work_busy_get(const struct k_work *work)
{
  k_spinlock_key_t key = k_spin_lock(&work_lock);
  int ret = work_busy_get_locked(work);

  k_spin_unlock(&work_lock, key);
  return ret;
}

int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work)
{
  __ASSERT_NO_MSG(work != NULL);
  __ASSERT_NO_MSG(work->handler != NULL);

  k_spinlock_key_t key = k_spin_lock(&work_lock);
  int ret = submit_to_queue_locked(work, &queue);

  k_spin_unlock(&work_lock, key);
  return ret;
}

int k_work_submit(struct k_work *work)
{
  return k_work_submit_to_queue(&k_sys_work_q, work);
}

/* kernel/work.c cancel_async_locked() */

static int cancel_async_locked(struct k_work *work)
{
  /* If we haven't already started canceling, do it now. */

  if (!flag_test(&work->flags, K_WORK_CANCELING_BIT))
    {
      /* Remove it from the queue, if it's queued. */

      queue_remove_locked(work->queue, work);
    }

  /* If it's still busy after it's been dequeued, then flag it as
   * canceling.
   */

  int ret = work_busy_get_locked(work);

  if (ret != 0)
    {
      flag_set(&work->flags, K_WORK_CANCELING_BIT);
      ret = work_busy_get_locked(work);
    }

  return ret;
}

bool k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync)
{
  __ASSERT_NO_MSG(work != NULL);
  __ASSERT_NO_MSG(sync != NULL);
  __ASSERT_NO_MSG(!flag_test(&work->flags, K_WORK_DELAYABLE_BIT));

  k_spinlock_key_t key = k_spin_lock(&work_lock);
  bool pending = (work_busy_get_locked(work) != 0U);
  bool need_wait = false;

  if (pending)
    {
      (void)cancel_async_locked(work);

      /* kernel/work.c cancel_sync_locked(): wait if it is still running. */

      need_wait = flag_test(&work->flags, K_WORK_CANCELING_BIT);
      if (need_wait)
        {
          __ASSERT_NO_MSG(work->canceller == NULL);
          (void)k_sem_init(&sync->sem, 0, 1);
          work->canceller = sync;
        }
    }

  k_spin_unlock(&work_lock, key);

  if (need_wait)
    {
      (void)k_sem_take(&sync->sem, K_FOREVER);
    }

  return pending;
}

/* kernel/work.c work_timeout(): the timer of a delayable item. */

static void work_timeout(struct _timeout *to)
{
  struct k_work_delayable *dw =
    CONTAINER_OF(to, struct k_work_delayable, timeout);
  struct k_work *wp = &dw->work;
  k_spinlock_key_t key = k_spin_lock(&work_lock);
  struct k_work_q *queue = NULL;

  /* K_WORK_DELAYED_BIT is the wake-ownership flag: whichever side (this
   * handler or unschedule_locked()) clears it first owns the outcome.
   */

  if (flag_test_and_clear(&wp->flags, K_WORK_DELAYED_BIT))
    {
      queue = dw->queue;
      (void)submit_to_queue_locked(wp, &queue);
    }

  k_spin_unlock(&work_lock, key);
}

void k_work_init_delayable(struct k_work_delayable *dwork,
                           k_work_handler_t handler)
{
  __ASSERT_NO_MSG(dwork != NULL);
  __ASSERT_NO_MSG(handler != NULL);

  *dwork = (struct k_work_delayable)
    {
      .work =
        {
          .handler = handler,
          .flags = K_WORK_DELAYABLE,
        },
    };
}

/* kernel/work.c schedule_for_queue_locked() */

static int schedule_for_queue_locked(struct k_work_q **queuep,
                                     struct k_work_delayable *dwork,
                                     k_timeout_t delay)
{
  int ret = 1;
  struct k_work *work = &dwork->work;

  if (K_TIMEOUT_EQ(delay, K_NO_WAIT))
    {
      return submit_to_queue_locked(work, queuep);
    }

  flag_set(&work->flags, K_WORK_DELAYED_BIT);
  dwork->queue = *queuep;

  /* Add timeout */

  z_add_timeout(&dwork->timeout, work_timeout, delay);

  return ret;
}

/* kernel/work.c unschedule_locked(): with interrupts masked the timer
 * cannot be running at the same time, so stopping it is enough.
 */

static inline bool unschedule_locked(struct k_work_delayable *dwork)
{
  bool ret = false;
  struct k_work *work = &dwork->work;

  if (flag_test_and_clear(&work->flags, K_WORK_DELAYED_BIT))
    {
      z_abort_timeout(&dwork->timeout);
      ret = true;
    }

  return ret;
}

int k_work_schedule_for_queue(struct k_work_q *queue,
                              struct k_work_delayable *dwork,
                              k_timeout_t delay)
{
  __ASSERT_NO_MSG(queue != NULL);
  __ASSERT_NO_MSG(dwork != NULL);

  struct k_work *work = &dwork->work;
  int ret = 0;
  k_spinlock_key_t key = k_spin_lock(&work_lock);

  /* Schedule the work item if it's idle or running. */

  if ((work_busy_get_locked(work) & ~K_WORK_RUNNING) == 0U)
    {
      ret = schedule_for_queue_locked(&queue, dwork, delay);
    }

  k_spin_unlock(&work_lock, key);
  return ret;
}

int k_work_reschedule_for_queue(struct k_work_q *queue,
                                struct k_work_delayable *dwork,
                                k_timeout_t delay)
{
  __ASSERT_NO_MSG(queue != NULL);
  __ASSERT_NO_MSG(dwork != NULL);

  int ret;
  k_spinlock_key_t key = k_spin_lock(&work_lock);

  /* Remove any active scheduling. */

  (void)unschedule_locked(dwork);

  /* Schedule the work item with the new parameters. */

  ret = schedule_for_queue_locked(&queue, dwork, delay);

  k_spin_unlock(&work_lock, key);
  return ret;
}

void z_work_queue_init(struct k_work_q *queue)
{
  *queue = (struct k_work_q){ .thread = 0 };
  (void)k_sem_init(&queue->notify, 0, K_SEM_MAX_LIMIT);
}

/* One pass of kernel/work.c work_queue_main(). */

bool z_work_queue_process(struct k_work_q *queue, bool *yield)
{
  struct k_work *work;
  k_work_handler_t handler;
  k_spinlock_key_t key = k_spin_lock(&work_lock);

  *yield = false;

  /* Check for and prepare any new work. */

  work = pending_get(queue);
  if (work == NULL)
    {
      k_spin_unlock(&work_lock, key);
      return false;
    }

  /* Mark that there's some work active that's not on the pending list. */

  flag_set(&work->flags, K_WORK_RUNNING_BIT);
  flag_clear(&work->flags, K_WORK_QUEUED_BIT);
  handler = work->handler;

  k_spin_unlock(&work_lock, key);

  __ASSERT_NO_MSG(handler != NULL);
  handler(work);

  /* Mark the work item as no longer running and deal with any cancellation
   * issued while it was running.
   */

  key = k_spin_lock(&work_lock);

  flag_clear(&work->flags, K_WORK_RUNNING_BIT);
  if (flag_test(&work->flags, K_WORK_CANCELING_BIT))
    {
      finalize_cancel_locked(work);
    }

  *yield = !flag_test(&queue->flags, K_WORK_QUEUE_NO_YIELD_BIT) &&
           queue->head != NULL;
  k_spin_unlock(&work_lock, key);
  return true;
}
