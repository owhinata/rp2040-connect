/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's kernel API (<zephyr/kernel.h>, Zephyr v4.5.0) that
 * the ported FIFO stream (lsm6dso_stream.c, lsm6dso_fifo_irq.c), the RTIO
 * part of the compatibility layer and the IMU service use, with Zephyr's
 * names, types and meanings:
 *
 *   k_timeout_t, K_NO_WAIT, K_FOREVER, K_MSEC(), K_TIMEOUT_EQ()
 *   k_spinlock (spinlock.h), k_mutex, k_sem (K_SEM_DEFINE), k_poll() on
 *   semaphores (K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY)
 *   k_work, k_work_delayable, k_work_q, k_work_sync: k_work_init(),
 *   k_work_submit() (the system work queue), k_work_submit_to_queue(),
 *   k_work_cancel_sync(), k_work_init_delayable(),
 *   k_work_schedule_for_queue(), k_work_reschedule_for_queue(),
 *   k_work_delayable_from_work(), k_work_queue_start(),
 *   k_work_queue_thread_get()
 *   K_THREAD_STACK_DEFINE(), K_THREAD_STACK_SIZEOF(), K_PRIO_PREEMPT(),
 *   k_thread_name_set(), K_THREAD_DEFINE() (started by the init task)
 *   K_MUTEX_DEFINE(), k_uptime_get(), k_sleep(), snprintk()
 *   (<zephyr/sys/printk.h>)
 *
 * The work items follow kernel/work.c (flags RUNNING, CANCELING, QUEUED,
 * DELAYED; the return values of submit and schedule): zephyr_work.c, which
 * is OS-independent.  The OS parts (semaphores, the wait in k_poll(), the
 * queue threads, the timer of a delayable item) are in zephyr_kernel.c on
 * NuttX (sem_t, kthreads, wdog) and in a fake for host tests.
 *
 * Timeouts are kernel ticks, as in Zephyr: the NuttX tick is 100 us
 * (CONFIG_USEC_PER_TICK), the same as the Zephyr build's
 * CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000.  K_MSEC() rounds up to ticks
 * (k_ms_to_ticks_ceil64), and a relative wait ends at the current tick +
 * ticks + 1 in both kernels (Zephyr's z_add_timeout(), NuttX's
 * clock_delay2abstick()).
 *
 * Priorities: K_PRIO_PREEMPT(p) is the NuttX priority 118 - p
 * (NANO_PRIO_FROM_ZEPHYR, priorities.h).
 *
 * Not here: threads created at run time (k_thread_create()), k_wakeup(),
 * the cycle counter (k_cycle_get_*()),
 * k_queue, k_msgq, k_timer, k_poll on other object types, user mode, SMP.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_KERNEL_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_KERNEL_H

/* NuttX's headers first: <zephyr/sys/atomic.h> takes over NuttX's atomic
 * names.
 */

#ifdef __NuttX__
#  include <nuttx/config.h>
#  include <sys/types.h>
#  include <nuttx/irq.h>
#  include <nuttx/mutex.h>
#  include <nuttx/semaphore.h>
#  include <nuttx/wdog.h>
#  include <stdio.h>            /* for <zephyr/sys/printk.h> */
#endif

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "priorities.h"

/* The Zephyr build has CONFIG_THREAD_NAME=y (lsm6dso_stream.c names its
 * work queue thread under it).
 */

#ifndef CONFIG_THREAD_NAME
#  define CONFIG_THREAD_NAME 1
#endif

/* Timeouts ---------------------------------------------------------------*/

#ifdef __NuttX__
#  define Z_USEC_PER_TICK CONFIG_USEC_PER_TICK
#else
#  define Z_USEC_PER_TICK 100
#endif

typedef struct
{
  int64_t ticks;
} k_timeout_t;

#define K_TICKS_FOREVER   ((int64_t)-1)
#define K_NO_WAIT         ((k_timeout_t){ .ticks = 0 })
#define K_FOREVER         ((k_timeout_t){ .ticks = K_TICKS_FOREVER })
#define K_TICKS(t)        ((k_timeout_t){ .ticks = (t) })
#define K_MSEC(ms) \
  ((k_timeout_t){ .ticks = ((int64_t)MAX((ms), 0) * 1000 + \
                            Z_USEC_PER_TICK - 1) / Z_USEC_PER_TICK })
#define K_TIMEOUT_EQ(a, b) ((a).ticks == (b).ticks)

/* The OS-specific objects ------------------------------------------------*/

struct k_poll_event;

#ifdef __NuttX__

/* k_mutex: recursive for its owner, with priority inheritance
 * (CONFIG_PRIORITY_INHERITANCE).
 */

struct k_mutex
{
  rmutex_t rmutex;
};

#  define Z_MUTEX_INITIALIZER(obj) { .rmutex = NXRMUTEX_INITIALIZER }

/* k_sem: a count with an upper limit; the threads in k_poll() on it. */

struct k_sem
{
  sem_t sem;
  unsigned int limit;
  struct k_poll_event *poll_events;
};

#  define Z_SEM_INITIALIZER(obj, initial, lim) \
  { .sem = SEM_INITIALIZER(initial), .limit = (lim), .poll_events = NULL }
#  define K_SEM_MAX_LIMIT SEM_VALUE_MAX

/* The timer of a delayable work item (Zephyr's struct _timeout). */

struct _timeout;
typedef void (*_timeout_func_t)(struct _timeout *t);

struct _timeout
{
  struct wdog_s wdog;
  _timeout_func_t fn;
};

typedef pid_t k_tid_t;
#else
/* Host test: struct k_mutex, struct k_sem, Z_SEM_INITIALIZER(),
 * K_SEM_MAX_LIMIT, struct _timeout, _timeout_func_t, k_tid_t.
 */
#  include "zephyr_os_host.h"
#endif

/* Mutexes ----------------------------------------------------------------*/

/* A mutex defined and initialized at build time (Zephyr: global, not
 * static).
 */

#define K_MUTEX_DEFINE(name) \
  struct k_mutex name = Z_MUTEX_INITIALIZER(name)

int k_mutex_init(struct k_mutex *mutex);

/* K_FOREVER waits for ever (a signal does not end it); K_NO_WAIT returns
 * -EBUSY if the mutex is held; a timeout returns -EAGAIN when it ends.
 */

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout);
int k_mutex_unlock(struct k_mutex *mutex);

/* Semaphores -------------------------------------------------------------*/

#define K_SEM_DEFINE(name, initial_count, count_limit) \
  struct k_sem name = Z_SEM_INITIALIZER(name, initial_count, count_limit)

int k_sem_init(struct k_sem *sem, unsigned int initial_count,
               unsigned int limit);

/* 0, -EBUSY (K_NO_WAIT and not available) or -EAGAIN (timed out). */

int k_sem_take(struct k_sem *sem, k_timeout_t timeout);

/* Adds one up to the limit (at the limit it does nothing) and wakes the
 * threads in k_poll() on it.  Any context, interrupt handlers included.
 */

void k_sem_give(struct k_sem *sem);

/* Sets the count to 0. */

void k_sem_reset(struct k_sem *sem);
unsigned int k_sem_count_get(struct k_sem *sem);

/* Polling ----------------------------------------------------------------*/

/* Only semaphores, and only in K_POLL_MODE_NOTIFY_ONLY: k_poll() returns
 * when one of them has a count (it does not take it).
 */

enum
{
  K_POLL_TYPE_IGNORE = 0,
  K_POLL_TYPE_SEM_AVAILABLE = 2,
};

enum k_poll_modes
{
  K_POLL_MODE_NOTIFY_ONLY = 0,
};

#define K_POLL_STATE_NOT_READY      0
#define K_POLL_STATE_SEM_AVAILABLE  2

struct k_poll_event
{
  struct k_poll_event *next;   /* in the semaphore's list while waiting */
  void *waiter;                /* the waiting k_poll() call */
  uint32_t type;
  uint32_t state;
  uint32_t mode;
  struct k_sem *sem;
};

void k_poll_event_init(struct k_poll_event *event, uint32_t type, int mode,
                       void *obj);

/* 0 when at least one event is ready (its state says which), -EAGAIN when
 * the timeout ended first.
 */

int k_poll(struct k_poll_event *events, int num_events, k_timeout_t timeout);

/* Threads ----------------------------------------------------------------*/

/* A thread's stack, as in Zephyr a static array (NuttX's
 * kthread_create_with_stack() runs a kernel thread on it).
 */

typedef uint8_t k_thread_stack_t;

#define K_THREAD_STACK_DEFINE(sym, size) \
  k_thread_stack_t __aligned(8) sym[size]
#define K_THREAD_STACK_SIZEOF(sym) sizeof(sym)

#define K_PRIO_PREEMPT(x) NANO_PRIO_FROM_ZEPHYR(x)

/* Names the thread (the name "ps" shows).  0 or -EINVAL. */

int k_thread_name_set(k_tid_t thread, const char *name);

/* K_THREAD_DEFINE(name, stack_size, entry, p1, p2, p3, prio, options,
 * delay): a thread with a static stack that runs entry(p1, p2, p3) at
 * K_PRIO_PREEMPT(prio), named by the token name (Zephyr's "kernel ps" shows
 * e.g. "imu_stream_tid"), and the k_tid_t variable name.  Zephyr starts
 * every such thread before main() (z_init_static_threads()); NuttX has no
 * list of them, so the init task starts each one at the same point with
 * K_THREAD_DEFINE_START(name) (<zephyr_compat.h>), which calls the
 * function z_thread_start_<name>() that this macro defines.  The thread is
 * a NuttX kernel thread on the static stack (kthread_create_with_stack(),
 * as the work queue threads); it ends when entry returns.  Only options 0
 * and delay 0 (start at once) are supported.
 */

struct z_thread_static
{
  const char *name;
  k_thread_stack_t *stack;
  size_t stack_size;
  void (*entry)(void *p1, void *p2, void *p3);
  void *p1;
  void *p2;
  void *p3;
  int prio;                     /* NuttX priority */
  k_tid_t *tid;
};

/* Starts the thread; 0 or a negative errno. */

int z_thread_static_start(const struct z_thread_static *thread);

#define K_THREAD_DEFINE(_name, _stack_size, _entry, _p1, _p2, _p3, _prio, \
                        _options, _delay) \
  BUILD_ASSERT((_options) == 0 && (_delay) == 0, \
               "K_THREAD_DEFINE: only options 0 and delay 0"); \
  static K_THREAD_STACK_DEFINE(_k_thread_stack_##_name, _stack_size); \
  k_tid_t _name; \
  static const struct z_thread_static _k_thread_data_##_name = \
    { \
      .name = #_name, \
      .stack = _k_thread_stack_##_name, \
      .stack_size = (_stack_size), \
      .entry = (_entry), \
      .p1 = (_p1), \
      .p2 = (_p2), \
      .p3 = (_p3), \
      .prio = K_PRIO_PREEMPT(_prio), \
      .tid = &_name, \
    }; \
  int z_thread_start_##_name(void); \
  int z_thread_start_##_name(void) \
  { \
    return z_thread_static_start(&_k_thread_data_##_name); \
  }

/* Time -------------------------------------------------------------------*/

/* k_uptime_get(): the time since boot in milliseconds, from the kernel's
 * tick count, rounded down (k_ticks_to_ms_floor64(k_uptime_ticks())).
 * The NuttX tick is the TIMER count over CONFIG_USEC_PER_TICK (the board's
 * tickless timer, nano_oneshot.c), so this is the TIMER read once,
 * Z_UPTIME_MS(); it is 64 bits wide where clock_systime_ticks() is 32.
 */

int64_t k_uptime_get(void);

/* Ticks to milliseconds the way k_uptime_get() does it: floor(floor(us /
 * tick) * tick / 1000), which equals floor(us / 1000) when 1000 is a
 * multiple of the tick length (100 us).
 */

#define Z_UPTIME_MS(us) ((int64_t)((uint64_t)(us) / 1000U))
BUILD_ASSERT(1000 % Z_USEC_PER_TICK == 0, "Z_UPTIME_MS needs 1000 % tick == 0");

/* k_sleep(): K_NO_WAIT yields; otherwise sleeps until the current tick +
 * timeout.ticks + 1 (Zephyr's kernel/sleep.c and z_add_timeout(), NuttX's
 * nxsched_ticksleep() and clock_delay2abstick()).  Returns 0 (nothing in
 * the port wakes a sleeping thread early, k_wakeup() is not provided);
 * K_FOREVER sleeps for ever.
 */

int32_t k_sleep(k_timeout_t timeout);

/* Work items -------------------------------------------------------------*/

struct k_work;
struct k_work_q;
struct k_work_sync;

typedef void (*k_work_handler_t)(struct k_work *work);

/* kernel/work.c's flags. */

#define K_WORK_RUNNING_BIT    0
#define K_WORK_CANCELING_BIT  1
#define K_WORK_QUEUED_BIT     2
#define K_WORK_DELAYED_BIT    3
#define K_WORK_DELAYABLE_BIT  8

#define K_WORK_RUNNING    BIT(K_WORK_RUNNING_BIT)
#define K_WORK_CANCELING  BIT(K_WORK_CANCELING_BIT)
#define K_WORK_QUEUED     BIT(K_WORK_QUEUED_BIT)
#define K_WORK_DELAYED    BIT(K_WORK_DELAYED_BIT)
#define K_WORK_MASK       (K_WORK_DELAYED | K_WORK_QUEUED | \
                           K_WORK_RUNNING | K_WORK_CANCELING)
#define K_WORK_DELAYABLE  BIT(K_WORK_DELAYABLE_BIT)

#define K_WORK_QUEUE_STARTED_BIT  0
#define K_WORK_QUEUE_NO_YIELD_BIT 3

struct k_work
{
  struct k_work *next;          /* in the queue's pending list */
  k_work_handler_t handler;
  struct k_work_q *queue;       /* the queue it was last submitted to */
  uint32_t flags;
  struct k_work_sync *canceller;
};

struct k_work_delayable
{
  struct k_work work;
  struct _timeout timeout;
  struct k_work_q *queue;
};

/* One cancellation in k_work_cancel_sync() (Zephyr allows several threads
 * to wait for the same item; the port has one).
 */

struct k_work_sync
{
  struct k_sem sem;
};

struct k_work_q
{
  struct k_work *head;          /* pending items, in submission order */
  struct k_work *tail;
  struct k_sem notify;          /* wakes the thread */
  k_tid_t thread;
  uint32_t flags;
};

struct k_work_queue_config
{
  const char *name;
  bool no_yield;
};

/* The system work queue ("sysworkq"): Zephyr's runs at the cooperative
 * priority -1 with a 1024-byte stack (CONFIG_SYSTEM_WORKQUEUE_PRIORITY,
 * _STACK_SIZE); here NANO_PRIO_SYSWORKQ (119) and NANO_STACK_SYSWORKQ.
 * Started by zephyr_compat_initialize().
 */

extern struct k_work_q k_sys_work_q;

void k_work_init(struct k_work *work, k_work_handler_t handler);

/* 0 already queued, 1 queued, 2 queued again while running (on the queue
 * that runs it), -EBUSY canceling, -EINVAL no queue, -ENODEV not started.
 * Any context.
 */

int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work);
int k_work_submit(struct k_work *work);

/* The flags of k_work_busy_get(). */

int k_work_busy_get(const struct k_work *work);

/* Removes a queued item and waits for a running one to return.  True if
 * the item was busy.  Threads only, not the item's own queue.
 */

bool k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync);

void k_work_init_delayable(struct k_work_delayable *dwork,
                           k_work_handler_t handler);

/* Does nothing (0) if the item is already delayed or queued; otherwise
 * queues it now (K_NO_WAIT, the k_work_submit_to_queue() values) or after
 * the delay (1).  A running item can be scheduled.
 */

int k_work_schedule_for_queue(struct k_work_q *queue,
                              struct k_work_delayable *dwork,
                              k_timeout_t delay);

/* Stops a pending delay first, then as k_work_schedule_for_queue(): with
 * K_NO_WAIT the item is queued now even if a longer delay was pending.
 */

int k_work_reschedule_for_queue(struct k_work_q *queue,
                                struct k_work_delayable *dwork,
                                k_timeout_t delay);

static inline struct k_work_delayable *
k_work_delayable_from_work(struct k_work *work)
{
  return CONTAINER_OF(work, struct k_work_delayable, work);
}

/* Starts the queue's thread (stack size and priority as given, named by
 * cfg->name if any).
 */

void k_work_queue_start(struct k_work_q *queue, k_thread_stack_t *stack,
                        size_t stack_size, int prio,
                        const struct k_work_queue_config *cfg);

static inline k_tid_t k_work_queue_thread_get(struct k_work_q *queue)
{
  return queue->thread;
}

/* Internal to the compatibility layer -----------------------------------*/

/* zephyr_work.c: initializes a queue, and takes the next pending item off
 * it and runs it in the caller (the queue's thread); false if there was
 * none.  *yield is set when other items are pending after it (Zephyr then
 * yields unless the queue was configured with no_yield).
 */

void z_work_queue_init(struct k_work_q *queue);
bool z_work_queue_process(struct k_work_q *queue, bool *yield);

/* The OS part: start and stop the timer of a delayable item.  The timer
 * function runs in an interrupt handler (NuttX: the wdog).
 */

void z_add_timeout(struct _timeout *to, _timeout_func_t fn,
                   k_timeout_t timeout);
void z_abort_timeout(struct _timeout *to);

#include "zephyr_compat.h"

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_KERNEL_H */
