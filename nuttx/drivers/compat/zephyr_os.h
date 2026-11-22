/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of <zephyr/kernel.h> of the Zephyr compatibility layer
 * (common/compat/zephyr/kernel.h, which includes this header first and
 * lists what it must provide): NuttX's headers that come before the
 * layer's, the tick, the OS objects (k_mutex, k_sem, the timer of a
 * delayable work item, k_tid_t), the priorities, the calling thread,
 * k_yield(), the cycle counter, the OS's assertion and the work queues'
 * wait.  The functions on them are zephyr_kernel.c and zephyr_mutex.c.
 * Part of the NuttX implementation of the layer (nuttx/drivers/compat, see
 * nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_OS_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_OS_H

/* NuttX's headers first: <zephyr/sys/atomic.h> takes over NuttX's atomic
 * names.
 */

#include <nuttx/config.h>
#include <sys/types.h>
#include <assert.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>
#include <stdio.h>            /* for <zephyr/sys/printk.h> */
#include <arch/board/rp2040_timer_us.h> /* the cycle counter */

#include <stdbool.h>
#include <stdint.h>

#include "priorities.h"

/* Timeouts are kernel ticks, as in Zephyr: the NuttX tick is 100 us
 * (CONFIG_USEC_PER_TICK), the same as the Zephyr build's
 * CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000.  A relative wait ends at the
 * current tick + ticks + 1 in both kernels (Zephyr's z_add_timeout(),
 * NuttX's clock_delay2abstick()).
 */

#define Z_USEC_PER_TICK CONFIG_USEC_PER_TICK

/* The OS-specific objects ------------------------------------------------*/

struct k_poll_event;

/* k_mutex: recursive for its owner, with priority inheritance
 * (CONFIG_PRIORITY_INHERITANCE).  READY is set by Z_MUTEX_INITIALIZER()
 * and k_mutex_init(); a mutex left all zero (never initialized) is
 * initialized by its first k_mutex_lock() or k_mutex_unlock()
 * (zephyr_mutex.c).  Zephyr's k_mutex is unlocked when all zero
 * (lock_count 0); NuttX's is held by thread 0 (mholder 0) and would make
 * its first locker wait for ever.
 */

struct k_mutex
{
  rmutex_t rmutex;
  bool ready;
};

#define Z_MUTEX_INITIALIZER(obj) \
  { .rmutex = NXRMUTEX_INITIALIZER, .ready = true }

/* k_sem: a count with an upper limit; the threads in k_poll() on it. */

struct k_sem
{
  sem_t sem;
  unsigned int limit;
  struct k_poll_event *poll_events;
};

#define Z_SEM_INITIALIZER(obj, initial, lim) \
  { .sem = SEM_INITIALIZER(initial), .limit = (lim), .poll_events = NULL }
#define K_SEM_MAX_LIMIT SEM_VALUE_MAX

/* k_sem_take(sem, K_FOREVER) (zephyr_kernel.c) without the call: the work
 * queues' threads sleep with it between submissions (<zephyr/kernel.h>).
 */

#define Z_SEM_TAKE_FOREVER(s) nxsem_wait_uninterruptible(&(s)->sem)

/* The timer of a delayable work item (Zephyr's struct _timeout): a wdog,
 * whose callback runs in the timer interrupt.
 */

struct _timeout;
typedef void (*_timeout_func_t)(struct _timeout *t);

struct _timeout
{
  struct wdog_s wdog;
  _timeout_func_t fn;
};

/* A thread: its NuttX thread ID, which is what K_THREAD_DEFINE() stores
 * in its k_tid_t (Zephyr: the thread's struct k_thread).
 */

typedef pid_t k_tid_t;

/* Priorities: K_PRIO_PREEMPT(p) is the NuttX priority 118 - p
 * (APP_PRIO_FROM_ZEPHYR, priorities.h).
 */

#define K_PRIO_PREEMPT(x) APP_PRIO_FROM_ZEPHYR(x)

/* The calling thread. */

/* <unistd.h>'s; that header is not included here: its getopt() names
 * (optarg, optind, ...) are macros.
 */

pid_t gettid(void);

static inline k_tid_t k_current_get(void)
{
  return gettid();
}

/* k_yield(): sched_yield() (<sched.h>'s, declared here as gettid()). */

int sched_yield(void);

static inline void k_yield(void)
{
  sched_yield();
}

/* The OS's assertion: DEBUGASSERT() (nothing without
 * CONFIG_DEBUG_ASSERTIONS, as in the imu configuration).
 */

#define Z_OS_ASSERT(test) DEBUGASSERT(test)

/* The cycle counter: the board's monotonic clock, the RP2040 TIMER
 * (rp2040_timer_us(), 1 MHz from the crystal; see <zephyr/kernel.h>).
 */

static inline uint64_t k_cycle_get_64(void)
{
  return rp2040_timer_us();
}

static inline uint32_t k_cycle_get_32(void)
{
  return (uint32_t)rp2040_timer_us();
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_OS_H */
