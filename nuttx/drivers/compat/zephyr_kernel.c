/* SPDX-License-Identifier: Apache-2.0 */
/* The OS part of <zephyr/kernel.h> on NuttX: mutexes, semaphores, k_poll()
 * on semaphores, the threads of the work queues, the timer of a delayable
 * work item, thread names, and the start of what Zephyr starts at boot
 * (zephyr_compat_initialize()).  The work queue logic itself is
 * zephyr_work.c.  Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 *
 *   Zephyr                    here
 *   k_mutex                   rmutex_t (recursive, priority inheritance)
 *   k_sem                     sem_t without priority inheritance (NuttX's
 *                             default for a semaphore), with the limit
 *                             checked in k_sem_give()
 *   k_poll (semaphores)       a semaphore of the call, given by k_sem_give()
 *                             of any semaphore it waits for
 *   work queue thread         a kernel thread on the queue's static stack
 *                             (kthread_create_with_stack()) running
 *                             zephyr_work.c's z_work_queue_process()
 *   _timeout of an item       a wdog (the callback runs in the timer
 *                             interrupt, as Zephyr's timeout does in the
 *                             system clock interrupt)
 *   k_thread_name_set()       the TCB's name
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sched.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/work.h>

#include "priorities.h"

/* Mutexes ----------------------------------------------------------------*/

int k_mutex_init(struct k_mutex *mutex)
{
  return nxrmutex_init(&mutex->rmutex);
}

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
  int ret;

  if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
    {
      ret = nxrmutex_trylock(&mutex->rmutex);
      return ret == 0 ? 0 : -EBUSY;
    }

  if (K_TIMEOUT_EQ(timeout, K_FOREVER))
    {
      do
        {
          ret = nxrmutex_lock(&mutex->rmutex);
        }
      while (ret == -EINTR);

      return ret;
    }

  ret = nxrmutex_ticklock(&mutex->rmutex, (uint32_t)timeout.ticks);
  return ret == -ETIMEDOUT ? -EAGAIN : ret;
}

int k_mutex_unlock(struct k_mutex *mutex)
{
  return nxrmutex_unlock(&mutex->rmutex);
}

/* Semaphores -------------------------------------------------------------*/

int k_sem_init(struct k_sem *sem, unsigned int initial_count,
               unsigned int limit)
{
  if (limit == 0 || initial_count > limit)
    {
      return -EINVAL;
    }

  sem->limit = limit;
  sem->poll_events = NULL;
  return nxsem_init(&sem->sem, 0, initial_count);
}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
  int ret;

  if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
    {
      return nxsem_trywait(&sem->sem) == 0 ? 0 : -EBUSY;
    }

  if (K_TIMEOUT_EQ(timeout, K_FOREVER))
    {
      return nxsem_wait_uninterruptible(&sem->sem);
    }

  ret = nxsem_tickwait_uninterruptible(&sem->sem,
                                       (uint32_t)timeout.ticks);
  return ret == -ETIMEDOUT ? -EAGAIN : ret;
}

/* Under the compatibility layer's spin lock (no switch until the end), so
 * that a poller made ready cannot run, return and reuse its event while the
 * list is still being walked.
 */

void k_sem_give(struct k_sem *sem)
{
  static struct k_spinlock lock;
  k_spinlock_key_t key = k_spin_lock(&lock);
  struct k_poll_event *event;
  int value;

  if (nxsem_get_value(&sem->sem, &value) < 0 || value < (int)sem->limit)
    {
      nxsem_post(&sem->sem);
    }

  for (event = sem->poll_events; event != NULL; event = event->next)
    {
      event->state |= K_POLL_STATE_SEM_AVAILABLE;
      nxsem_post((sem_t *)event->waiter);
    }

  k_spin_unlock(&lock, key);
}

void k_sem_reset(struct k_sem *sem)
{
  irqstate_t flags = up_irq_save();

  while (nxsem_trywait(&sem->sem) == 0)
    {
    }

  up_irq_restore(flags);
}

unsigned int k_sem_count_get(struct k_sem *sem)
{
  int value = 0;

  (void)nxsem_get_value(&sem->sem, &value);
  return value > 0 ? (unsigned int)value : 0;
}

/* Polling ----------------------------------------------------------------*/

void k_poll_event_init(struct k_poll_event *event, uint32_t type, int mode,
                       void *obj)
{
  *event = (struct k_poll_event)
    {
      .type = type,
      .mode = (uint32_t)mode,
      .state = K_POLL_STATE_NOT_READY,
      .sem = obj,
    };
}

/* Marks the events whose semaphore has a count; returns how many. */

static int poll_ready(struct k_poll_event *events, int num_events)
{
  int ready = 0;

  for (int i = 0; i < num_events; i++)
    {
      if (events[i].type == K_POLL_TYPE_SEM_AVAILABLE &&
          k_sem_count_get(events[i].sem) > 0)
        {
          events[i].state |= K_POLL_STATE_SEM_AVAILABLE;
        }

      ready += events[i].state != K_POLL_STATE_NOT_READY;
    }

  return ready;
}

static void poll_unlink(struct k_poll_event *event)
{
  struct k_poll_event **link = &event->sem->poll_events;

  while (*link != NULL)
    {
      if (*link == event)
        {
          *link = event->next;
          break;
        }

      link = &(*link)->next;
    }

  event->next = NULL;
  event->waiter = NULL;
}

int k_poll(struct k_poll_event *events, int num_events, k_timeout_t timeout)
{
  sem_t waiter;
  irqstate_t flags;
  int ready;
  int i;

  nxsem_init(&waiter, 0, 0);
  flags = up_irq_save();
  for (i = 0; i < num_events; i++)
    {
      events[i].state = K_POLL_STATE_NOT_READY;
    }

  ready = poll_ready(events, num_events);
  if (ready == 0 && !K_TIMEOUT_EQ(timeout, K_NO_WAIT))
    {
      for (i = 0; i < num_events; i++)
        {
          if (events[i].type == K_POLL_TYPE_SEM_AVAILABLE)
            {
              events[i].waiter = &waiter;
              events[i].next = events[i].sem->poll_events;
              events[i].sem->poll_events = &events[i];
            }
        }

      up_irq_restore(flags);
      if (K_TIMEOUT_EQ(timeout, K_FOREVER))
        {
          (void)nxsem_wait_uninterruptible(&waiter);
        }
      else
        {
          (void)nxsem_tickwait_uninterruptible(&waiter,
                                               (uint32_t)timeout.ticks);
        }

      flags = up_irq_save();
      for (i = 0; i < num_events; i++)
        {
          if (events[i].type == K_POLL_TYPE_SEM_AVAILABLE)
            {
              poll_unlink(&events[i]);
            }
        }

      ready = poll_ready(events, num_events);
    }

  up_irq_restore(flags);
  nxsem_destroy(&waiter);
  return ready > 0 ? 0 : -EAGAIN;
}

/* Threads ----------------------------------------------------------------*/

int k_thread_name_set(k_tid_t thread, const char *name)
{
#if CONFIG_TASK_NAME_SIZE > 0
  FAR struct tcb_s *tcb = nxsched_get_tcb(thread);

  if (tcb == NULL || name == NULL)
    {
      return -EINVAL;
    }

  strlcpy(tcb->name, name, sizeof(tcb->name));
  return 0;
#else
  return -ENOSYS;
#endif
}

/* The queue a new thread serves, handed over by k_work_queue_start(),
 * which waits until the thread has taken it.
 */

static mutex_t g_queue_start_lock = NXMUTEX_INITIALIZER;
static sem_t g_queue_taken = SEM_INITIALIZER(0);
static struct k_work_q *g_queue_starting;

/* The thread of a work queue: kernel/work.c work_queue_main(). */

static int work_queue_main(int argc, FAR char *argv[])
{
  struct k_work_q *queue = g_queue_starting;
  bool yield;

  (void)argc;
  (void)argv;
  nxsem_post(&g_queue_taken);

  for (; ; )
    {
      if (z_work_queue_process(queue, &yield))
        {
          if (yield)
            {
              sched_yield();
            }

          continue;
        }

      /* Nothing pending: sleep until the next submission. */

      (void)k_sem_take(&queue->notify, K_FOREVER);
    }

  return 0;
}

void k_work_queue_start(struct k_work_q *queue, k_thread_stack_t *stack,
                        size_t stack_size, int prio,
                        const struct k_work_queue_config *cfg)
{
  int pid;

  z_work_queue_init(queue);
  queue->flags = BIT(K_WORK_QUEUE_STARTED_BIT);
  if (cfg != NULL && cfg->no_yield)
    {
      queue->flags |= BIT(K_WORK_QUEUE_NO_YIELD_BIT);
    }

  nxmutex_lock(&g_queue_start_lock);
  g_queue_starting = queue;
  pid = kthread_create_with_stack(cfg != NULL && cfg->name != NULL ?
                                  cfg->name : "workq", prio, stack,
                                  (int)stack_size, work_queue_main, NULL);
  DEBUGASSERT(pid > 0);
  queue->thread = pid > 0 ? pid : 0;
  if (pid > 0)
    {
      nxsem_wait_uninterruptible(&g_queue_taken);
    }

  nxmutex_unlock(&g_queue_start_lock);
}

/* Timeouts of delayable items --------------------------------------------*/

static void timeout_expired(wdparm_t arg)
{
  struct _timeout *to = (struct _timeout *)arg;

  to->fn(to);
}

void z_add_timeout(struct _timeout *to, _timeout_func_t fn,
                   k_timeout_t timeout)
{
  to->fn = fn;
  (void)wd_start(&to->wdog, (clock_t)timeout.ticks, timeout_expired,
                 (wdparm_t)to);
}

void z_abort_timeout(struct _timeout *to)
{
  (void)wd_cancel(&to->wdog);
}

/* What Zephyr starts at boot -------------------------------------------*/

struct k_work_q k_sys_work_q;

static K_THREAD_STACK_DEFINE(sys_work_q_stack, NANO_STACK_SYSWORKQ);

int zephyr_compat_initialize(void)
{
  static const struct k_work_queue_config cfg =
    {
      .name = "sysworkq",
      .no_yield = false,
    };

  /* kernel/system_work_q.c k_sys_work_q_init() (POST_KERNEL). */

  k_work_queue_start(&k_sys_work_q, sys_work_q_stack,
                     K_THREAD_STACK_SIZEOF(sys_work_q_stack),
                     NANO_PRIO_SYSWORKQ, &cfg);

  /* subsys/rtio/rtio_workq.c static_init() (POST_KERNEL). */

  rtio_workq_initialize();

  /* drivers/gpio/gpio_rpi_pico.c (POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY
   * 40).
   */

  return device_init(DEVICE_DT_GET(DT_NODELABEL(gpio0)));
}
