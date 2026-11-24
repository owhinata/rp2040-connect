/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's kernel API (<zephyr/kernel.h>, Zephyr v4.5.0) that
 * the LSM6DSO's FIFO stream (lsm6dso_os_impl.h, lsm6dso_os.c), the RTIO
 * part of the compatibility layer and the IMU service use, with Zephyr's
 * names, types and meanings:
 *
 *   k_timeout_t, K_NO_WAIT, K_FOREVER, K_MSEC(), K_SECONDS(), K_TIMEOUT_EQ()
 *   k_spinlock (spinlock.h), k_mutex, k_sem (K_SEM_DEFINE), k_poll() on
 *   semaphores (K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY)
 *   k_work, k_work_delayable, k_work_q, k_work_sync: k_work_init(),
 *   k_work_submit() (the system work queue), k_work_submit_to_queue(),
 *   k_work_cancel_sync(), k_work_init_delayable(),
 *   k_work_schedule_for_queue(), k_work_reschedule_for_queue(),
 *   k_work_delayable_from_work(), k_work_queue_start(),
 *   k_work_queue_thread_get()
 *   K_THREAD_STACK_DEFINE(), K_THREAD_STACK_SIZEOF(), K_PRIO_PREEMPT(),
 *   k_thread_name_set(), K_THREAD_DEFINE() (started by the init task),
 *   k_current_get()
 *   K_MUTEX_DEFINE(), k_uptime_get(), k_sleep(), k_msleep(), k_usleep(),
 *   k_busy_wait(), snprintk() (<zephyr/sys/printk.h>)
 *   k_cycle_get_32(), k_cycle_get_64(), k_cyc_to_us_floor64(),
 *   k_cyc_to_us_ceil64(), k_cyc_to_ns_floor64(),
 *   sys_clock_hw_cycles_per_sec()
 *   K_HEAP_DEFINE(), k_heap_alloc(), k_heap_free() (zephyr_heap.h, with
 *   either OS part)
 *
 * The work items follow kernel/work.c (flags RUNNING, CANCELING, QUEUED,
 * DELAYED; the return values of submit and schedule): zephyr_work.c, which
 * is OS-independent, as is zephyr_init.c (the work queues' threads on
 * z_thread_static_start(), the system work queue, k_uptime_get() and
 * zephyr_compat_initialize()).  The OS parts (semaphores, the wait in
 * k_poll(), the start of a thread, the timer of a delayable item) are each
 * OS's: on NuttX zephyr_kernel.c and zephyr_mutex.c (sem_t, kthreads,
 * wdog), in host tests a fake (common/tests/zephyr_fakes), and for an OS
 * that offers only basic primitives (FreeRTOS, ThreadX) the generic ones of
 * common/compat/kernel_generic on the primitives of its zephyr_kernel_os.h.
 *
 * Timeouts are kernel ticks, as in Zephyr (100 us in the Zephyr build,
 * CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000).  K_MSEC() rounds up to ticks
 * (k_ms_to_ticks_ceil64), and a relative wait ends at the current tick +
 * ticks + 1 (Zephyr's z_add_timeout()).
 *
 * Not here: threads created at run time (k_thread_create()), k_wakeup(),
 * k_queue, k_msgq, k_timer, k_poll on other object types, user mode, SMP.
 *
 * The OS part: "zephyr_os.h", which this header includes first, from the
 * OS's directory of the layer (common/compat/port/nuttx on NuttX,
 * common/tests/zephyr_fakes in host tests, common/compat/port/generic for
 * an OS on basic primitives), provides:
 *
 *   - the OS's headers that must come before the layer's (the layer's
 *     <zephyr/sys/atomic.h> takes over atomic names, see
 *     "zephyr_os_atomic.h"),
 *   - Z_USEC_PER_TICK: the length of the tick in microseconds (1000 must
 *     be a multiple of it),
 *   - struct k_mutex and Z_MUTEX_INITIALIZER(obj); a k_mutex left all
 *     zero must work as an unlocked mutex (Zephyr's does: the MQTT
 *     client's before mqtt_client_init()),
 *   - struct k_sem, Z_SEM_INITIALIZER(obj, initial, limit) and
 *     K_SEM_MAX_LIMIT,
 *   - struct _timeout and _timeout_func_t (the timer of a delayable work
 *     item),
 *   - k_tid_t, and k_current_get() (declared or inline),
 *   - optionally K_THREAD_STACK_RESERVED: the bytes K_THREAD_STACK_DEFINE()
 *     adds after each stack for the OS (the thread's control block); 0
 *     by default,
 *   - k_yield() (declared or inline): gives the turn to the next ready
 *     thread of the same priority, or goes on if there is none (threads
 *     only),
 *   - K_PRIO_PREEMPT(x): the OS's priority of Zephyr's preemptible
 *     priority x, keeping their order,
 *   - k_cycle_get_32() and k_cycle_get_64() (declared or inline): a
 *     counter of 1 MHz (sys_clock_hw_cycles_per_sec() and the conversions
 *     below assume it),
 *   - Z_OS_ASSERT(test): the OS's own assertion, for what the layer cannot
 *     return (a work queue's thread that did not start: Zephyr's
 *     k_work_queue_start() cannot fail); it may compile to nothing,
 *   - optionally Z_SEM_TAKE_FOREVER(sem): k_sem_take(sem, K_FOREVER) the
 *     OS's shorter way (the work queues' threads sleep with it between
 *     submissions); the default below is k_sem_take() itself,
 *   - for the shell engine (common/compat/shell_engine) only:
 *     k_is_in_isr(), k_is_pre_kernel() and k_thread_custom_data_get(),
 *     k_thread_custom_data_set() (declared or inline; the custom data is
 *     NULL until set and before the kernel runs threads).  The NuttX build
 *     does not build the engine and has none of them.
 *
 * The OS implements the functions declared here that zephyr_work.c and
 * zephyr_init.c do not: k_mutex_*(), k_sem_*(), k_poll_event_init(),
 * k_poll(), k_thread_name_set(), z_thread_static_start() (every thread of
 * the layer starts with it), k_sleep(), k_msleep(), k_usleep(),
 * k_busy_wait(), z_add_timeout() and z_abort_timeout().
 *
 * Part of the small Zephyr compatibility layer of the port, its
 * OS-independent part (common/compat, see common/compat/README.md).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_KERNEL_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_KERNEL_H

/* The OS part first (its headers come before the layer's). */

#include "zephyr_os.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/* The Zephyr build has CONFIG_THREAD_NAME=y (the LSM6DSO's lsm6dso_os.c names
 * its work queue thread under it).
 */

#ifndef CONFIG_THREAD_NAME
#  define CONFIG_THREAD_NAME 1
#endif

/* Timeouts ---------------------------------------------------------------*/

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
#define K_SECONDS(s)      K_MSEC((s) * 1000)
#define K_TIMEOUT_EQ(a, b) ((a).ticks == (b).ticks)

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

/* The wait of a work queue's thread for its next submission (zephyr_init.c):
 * k_sem_take(sem, K_FOREVER) unless "zephyr_os.h" has a shorter way to it.
 */

#ifndef Z_SEM_TAKE_FOREVER
#  define Z_SEM_TAKE_FOREVER(sem) k_sem_take((sem), K_FOREVER)
#endif

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

/* A thread's stack, as in Zephyr a static array (the OS runs the thread
 * on it, e.g. NuttX's kthread_create_with_stack()).  As in Zephyr, the
 * array has K_THREAD_STACK_RESERVED bytes more than size, which are the
 * OS's ("zephyr_os.h" may define it: the generic kernel part keeps the
 * thread's control block there; 0 otherwise), and K_THREAD_STACK_SIZEOF()
 * is size, what the thread has for its stack.
 */

typedef uint8_t k_thread_stack_t;

#ifndef K_THREAD_STACK_RESERVED
#  define K_THREAD_STACK_RESERVED 0
#endif

#define K_THREAD_STACK_DEFINE(sym, size) \
  k_thread_stack_t __aligned(8) sym[(size) + K_THREAD_STACK_RESERVED]
#define K_THREAD_STACK_SIZEOF(sym) (sizeof(sym) - K_THREAD_STACK_RESERVED)

/* K_PRIO_PREEMPT() and k_current_get(): "zephyr_os.h".  The calling
 * thread's k_tid_t is what K_THREAD_DEFINE() stores in its variable.
 */

/* Names the thread (the name "ps" shows).  0 or -EINVAL. */

int k_thread_name_set(k_tid_t thread, const char *name);

/* K_THREAD_DEFINE(name, stack_size, entry, p1, p2, p3, prio, options,
 * delay): a thread with a static stack that runs entry(p1, p2, p3) at
 * K_PRIO_PREEMPT(prio), named by the token name (Zephyr's "kernel ps" shows
 * e.g. "imu_stream_tid"), and the k_tid_t variable name.  Zephyr starts
 * every such thread before main() (z_init_static_threads()); the port has
 * no list of them, so the application's table (app_threads_start(),
 * common/imu/app_init.c) starts each one at the same point with
 * K_THREAD_DEFINE_START(name) (<zephyr_compat.h>), which calls the
 * function z_thread_start_<name>() that this macro defines.  The OS runs
 * the thread on the static stack (z_thread_static_start(); on NuttX a
 * kernel thread, kthread_create_with_stack()); it ends when entry returns.
 * Only options 0 and delay 0 (start at once) are supported.  The layer
 * starts its own threads (the work queues', "RTIO WQ 0", "net_mgmt") with
 * z_thread_static_start() too.
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
  int prio;                     /* the OS's, K_PRIO_PREEMPT() */
  k_tid_t *tid;
};

/* The OS part: starts a thread named thread->name on thread->stack
 * (stack_size bytes: the storage of K_THREAD_STACK_DEFINE() and its
 * K_THREAD_STACK_SIZEOF(); the OS may use the K_THREAD_STACK_RESERVED
 * bytes after them) at the OS's priority thread->prio, which runs
 * thread->entry(p1, p2, p3) and ends when entry returns; sets
 * *thread->tid before the new thread can use it.  *thread must stay valid
 * while the thread runs (the OS may read it in the new thread).  The new
 * thread may run before this returns.  0 or a negative errno (*thread->tid
 * then unchanged or 0).
 */

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
 * tick count, rounded down (k_ticks_to_ms_floor64(k_uptime_ticks())),
 * Z_UPTIME_MS() of the microseconds since boot: here of the cycle counter,
 * the TIMER read once (zephyr_init.c).  On NuttX the tick is the TIMER
 * count over CONFIG_USEC_PER_TICK (the board's tickless timer,
 * rp2040_oneshot.c), so this is the tick count in ms; it is 64 bits wide
 * where clock_systime_ticks() is 32.  The FreeRTOS and ThreadX ticks are not used: they are 32
 * bits wide and stand still while interrupts are masked for longer than a
 * tick, until the application's tick catch-up runs.
 */

int64_t k_uptime_get(void);

/* Ticks to milliseconds the way k_uptime_get() does it: floor(floor(us /
 * tick) * tick / 1000), which equals floor(us / 1000) when 1000 is a
 * multiple of the tick length (Z_USEC_PER_TICK, 100 us on NuttX).
 */

#define Z_UPTIME_MS(us) ((int64_t)((uint64_t)(us) / 1000U))
BUILD_ASSERT(1000 % Z_USEC_PER_TICK == 0, "Z_UPTIME_MS needs 1000 % tick == 0");

/* k_sleep(): K_NO_WAIT yields; otherwise sleeps until the current tick +
 * timeout.ticks + 1 (Zephyr's kernel/sleep.c and z_add_timeout(); on NuttX
 * nxsched_ticksleep() and clock_delay2abstick()).  Returns 0 (nothing in
 * the port wakes a sleeping thread early, k_wakeup() is not provided);
 * K_FOREVER sleeps for ever.
 */

int32_t k_sleep(k_timeout_t timeout);

/* k_msleep(), k_usleep(): the time rounded up to ticks, then as k_sleep():
 * the thread wakes at the current tick + ticks + 1 (Zephyr's
 * z_tick_sleep() and z_add_timeout(); on NuttX nxsched_msleep() and
 * nxsched_usleep(), which round up the same way and wait until
 * clock_delay2abstick()).  0 or less yields, as k_sleep(K_NO_WAIT).  A
 * signal does not end the sleep.  Return 0 (Zephyr: the time left, which
 * only k_wakeup() makes non-zero).  zephyr_kernel.c on NuttX; a host test
 * that calls them defines them.
 */

int32_t k_msleep(int32_t ms);
int32_t k_usleep(int32_t us);

/* k_busy_wait(): spins for at least usec_to_wait microseconds without
 * leaving the CPU (on NuttX up_udelay(), which polls the TIMER: the board's
 * rp2040_oneshot.c).  zephyr_kernel.c on NuttX; a host test that calls it
 * defines it.
 */

void k_busy_wait(uint32_t usec_to_wait);

/* The cycle counter: the Zephyr build's is the 125 MHz SysTick count
 * (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC), which falls behind under load (see
 * nuttx/README.md); here it is the board's monotonic clock, the RP2040
 * TIMER (1 MHz from the crystal; NuttX's rp2040_timer_us(); k_cycle_get_32()
 * and k_cycle_get_64() are in "zephyr_os.h"), as for the sensor clock
 * (<zephyr/drivers/sensor_clock.h>).  A cycle is a microsecond, so the
 * conversions to microseconds are exact (Zephyr's round down or up a
 * count of 8 ns cycles).  k_cycle_get_32() is the low 32 bits, wrapping
 * after about 71.6 minutes (Zephyr's after about 34 s).  The IMU service
 * times its queue lock waits and its publishes with these, the LSM6DSO
 * driver its FIFO reads (k_cycle_get_32(), k_cyc_to_ns_floor64()).
 */

#define sys_clock_hw_cycles_per_sec() 1000000

static inline uint64_t k_cyc_to_us_floor64(uint64_t cycles)
{
  return cycles;
}

static inline uint64_t k_cyc_to_us_ceil64(uint64_t cycles)
{
  return cycles;
}

static inline uint64_t k_cyc_to_ns_floor64(uint64_t cycles)
{
  return cycles * 1000U;
}

/* Work items-------------------------------------------------------------*/

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
  struct z_thread_static thread_start; /* the thread's, zephyr_init.c */
};

struct k_work_queue_config
{
  const char *name;
  bool no_yield;
};

/* The system work queue ("sysworkq"): Zephyr's runs at the cooperative
 * priority -1 with a 1024-byte stack (CONFIG_SYSTEM_WORKQUEUE_PRIORITY,
 * _STACK_SIZE); on NuttX APP_PRIO_SYSWORKQ (119) and APP_STACK_SYSWORKQ.
 * Started by zephyr_compat_initialize() (zephyr_init.c).
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
 * cfg->name if any, else "workq") with z_thread_static_start(); it runs
 * z_work_queue_process() and sleeps on the queue's semaphore when nothing
 * is pending (zephyr_init.c).
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
 * function runs in an interrupt handler (NuttX: the wdog), so it cannot
 * run while k_spin_lock() is held (unschedule_locked() of zephyr_work.c
 * relies on it).
 */

void z_add_timeout(struct _timeout *to, _timeout_func_t fn,
                   k_timeout_t timeout);
void z_abort_timeout(struct _timeout *to);

/* k_heap: OS-independent, on k_spin_lock() (zephyr_heap.h, zephyr_heap.c
 * of common/compat), the same whichever OS part this header has.
 */

#include "zephyr_heap.h"

#include "zephyr_compat.h"

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_KERNEL_H */
