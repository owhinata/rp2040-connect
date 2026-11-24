/* SPDX-License-Identifier: Apache-2.0 */
/* The OS part of <zephyr/kernel.h> on NuttX: semaphores, k_poll()
 * on semaphores, the start of a thread, the timer of a delayable work
 * item, thread names, sleeps.  The work queue logic, the work queues'
 * threads and the start of what Zephyr starts at boot are OS-independent
 * (common/compat/zephyr_work.c, zephyr_init.c).  The types are in
 * zephyr_os.h.  Part of the NuttX implementation of the small Zephyr
 * compatibility layer of the port (common/compat/port/nuttx).  The
 * mutexes are zephyr_mutex.c.
 *
 *   Zephyr                    here
 *   k_sem                     sem_t without priority inheritance (NuttX's
 *                             default for a semaphore), with the limit
 *                             checked in k_sem_give()
 *   k_poll (semaphores)       a semaphore of the call, given by k_sem_give()
 *                             of any semaphore it waits for
 *   _timeout of an item       a wdog (the callback runs in the timer
 *                             interrupt, as Zephyr's timeout does in the
 *                             system clock interrupt)
 *   k_thread_name_set()       the TCB's name
 *   K_THREAD_DEFINE() thread  a kernel thread on its static stack
 *                             (kthread_create_with_stack()), started by
 *                             z_thread_static_start() (the init task; the
 *                             layer's own threads, the work queues' among
 *                             them, the same way)
 *   k_sleep()                 nxsched_ticksleep()
 *   k_msleep(), k_usleep()    nxsched_msleep(), nxsched_usleep() (rounded
 *                             up to ticks; the same wake-up as k_sleep())
 *   k_busy_wait()             up_udelay() (polls the TIMER)
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sched.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>

#include <zephyr/kernel.h>

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

/* The threads of K_THREAD_DEFINE() and of the layer (the work queues',
 * common/compat/zephyr_init.c, and the others): the start data is handed
 * over to the new thread, and z_thread_static_start() waits until the
 * thread has taken it (a kernel thread's arguments are strings).
 */

static mutex_t g_thread_start_lock = NXMUTEX_INITIALIZER;
static sem_t g_thread_taken = SEM_INITIALIZER(0);
static const struct z_thread_static *g_thread_starting;

static int static_thread_main(int argc, FAR char *argv[])
{
  const struct z_thread_static *thread = g_thread_starting;

  (void)argc;
  (void)argv;
  nxsem_post(&g_thread_taken);
  thread->entry(thread->p1, thread->p2, thread->p3);
  return 0;
}

int z_thread_static_start(const struct z_thread_static *thread)
{
  int pid;

  nxmutex_lock(&g_thread_start_lock);
  g_thread_starting = thread;
  pid = kthread_create_with_stack(thread->name, thread->prio, thread->stack,
                                  (int)thread->stack_size,
                                  static_thread_main, NULL);
  if (pid > 0)
    {
      *thread->tid = pid;
      nxsem_wait_uninterruptible(&g_thread_taken);
    }

  nxmutex_unlock(&g_thread_start_lock);
  return pid > 0 ? 0 : pid;
}

/* Time -------------------------------------------------------------------*/

int32_t k_sleep(k_timeout_t timeout)
{
  if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
    {
      sched_yield();
      return 0;
    }

  if (K_TIMEOUT_EQ(timeout, K_FOREVER))
    {
      for (; ; )
        {
          nxsched_ticksleep(UINT32_MAX / 2);
        }
    }

  /* clock_delay2abstick(): the current tick + ticks + 1. */

  nxsched_ticksleep((unsigned int)timeout.ticks);
  return 0;
}

/* nxsched_msleep(), nxsched_usleep(): the time rounded up to ticks
 * (MSEC2TICK(), USEC2TICK()), then nxsched_ticksleep(), which yields for 0
 * ticks and otherwise wakes at the current tick + ticks + 1, as k_sleep().
 */

int32_t k_msleep(int32_t ms)
{
  nxsched_msleep(ms > 0 ? (unsigned int)ms : 0);
  return 0;
}

int32_t k_usleep(int32_t us)
{
  nxsched_usleep(us > 0 ? (useconds_t)us : 0);
  return 0;
}

void k_busy_wait(uint32_t usec_to_wait)
{
  up_udelay(usec_to_wait);
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
