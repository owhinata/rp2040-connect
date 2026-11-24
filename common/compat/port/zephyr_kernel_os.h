/* SPDX-License-Identifier: MIT */
/* What the generic kernel part of the Zephyr compatibility layer needs from
 * the OS: the primitives on which it implements Zephyr's k_sem (with
 * Zephyr's wake rules), k_poll() on semaphores, the sorted timeout list of
 * the delayable work items, the sleeps, the start of a thread, k_spinlock
 * and k_mutex (zephyr_kernel.c, zephyr_mutex.c of common/compat/kernel_generic,
 * zephyr_os.h, zephyr_os_spinlock.h of common/compat/port/generic: the
 * kernel part for an OS that offers only these primitives).  An OS that
 * offers them writes nothing else of the kernel part but a tick hook that
 * calls zephyr_compat_tick() (zephyr_compat_os.h; with tick_catchup.h for a
 * periodic tick that interrupts masked for long can lose) and, only when
 * its headers define atomic names, its own zephyr_os_atomic.h (the default
 * here is empty); an OS with richer objects of its own can implement
 * the Zephyr-level functions directly instead (the NuttX build,
 * common/compat/port/nuttx).  The primitives of the FreeRTOS and ThreadX
 * builds are common/compat/port/{freertos,threadx}/zephyr_kernel_os_impl.h.
 *
 * The generic part includes this header (through zephyr_os.h, first in
 * <zephyr/kernel.h>, and zephyr_os_spinlock.h); it includes the OS's part,
 * "zephyr_kernel_os_impl.h", then declares the functions that part must
 * define, so a missing or different one does not compile.  The OS's part
 * defines them as static inline functions: they are on the paths of
 * k_sem_give(), k_sem_take(), k_poll(), k_spin_lock() and the tick, which
 * must compile as if they called the OS directly (no call through a
 * pointer, no extra call).  The OS's part reads no header of the layer
 * (it comes before <zephyr/kernel.h> has defined anything); it may include
 * <errno.h>.
 *
 * zephyr_kernel_os_impl.h: types and constants
 *
 *   z_os_thread_t        a thread's handle (Zephyr's k_tid_t); NULL is no
 *                        thread
 *   z_os_prio_t          an integer priority of a running thread, a larger
 *                        value more urgent (the order of the semaphores'
 *                        and poll events' waiters)
 *   z_os_ticks_t         an unsigned count of kernel ticks
 *   Z_OS_TICKS_FOREVER   the largest z_os_ticks_t: a wait without end;
 *                        every smaller value is a finite wait
 *   Z_OS_TICK_HZ         ticks per second (Zephyr's timeouts are these
 *                        ticks; 1000000 must be a multiple of it)
 *   z_os_woken_t         an integer, 0 at first, that z_os_wake_from_isr()
 *                        sets when the interrupt handler should switch at
 *                        its end
 *   Z_OS_THREAD_NAME_SIZE  the bytes of a thread's name array, with its
 *                        terminating NUL (z_os_thread_name())
 *   Z_OS_THREAD_RESERVED the bytes the OS keeps of each thread after its
 *                        stack, for its control block of the thread (a
 *                        constant expression, 0 if it needs none; a
 *                        multiple of 8): K_THREAD_STACK_DEFINE() reserves
 *                        them (Zephyr's K_THREAD_STACK_RESERVED, zephyr_os.h),
 *                        so every thread brings its own and none is
 *                        allocated (z_os_thread_start())
 *   struct k_mutex, Z_MUTEX_INITIALIZER(obj)
 *                        the OS's own recursive mutex with priority
 *                        inheritance; a k_mutex left all zero (never
 *                        initialized, or zeroed while unlocked after use,
 *                        mqtt_client_init()) must work as an unlocked
 *                        mutex, as Zephyr's does: the OS holds no
 *                        reference to an unlocked mutex
 *   Z_OS_ASSERT(test)    the OS's own assertion (it may compile to
 *                        nothing)
 *
 * The context and lock words below: a "thread" is a thread of the OS with
 * the scheduler running; "ISR" is an interrupt handler (including the
 * tick's hook, from which zephyr_compat_tick() runs).  "Locked" means
 * between z_os_task_lock() and z_os_task_unlock() in a thread, or
 * z_os_isr_lock() and z_os_isr_unlock() in an ISR (k_spin_lock()).
 */

#ifndef COMMON_COMPAT_PORT_ZEPHYR_KERNEL_OS_H
#define COMMON_COMPAT_PORT_ZEPHYR_KERNEL_OS_H

#include "zephyr_kernel_os_impl.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef Z_OS_TICKS_FOREVER
#error "zephyr_kernel_os_impl.h must define Z_OS_TICKS_FOREVER"
#endif
#ifndef Z_OS_TICK_HZ
#error "zephyr_kernel_os_impl.h must define Z_OS_TICK_HZ"
#endif
#ifndef Z_OS_THREAD_NAME_SIZE
#error "zephyr_kernel_os_impl.h must define Z_OS_THREAD_NAME_SIZE"
#endif
#ifndef Z_OS_THREAD_RESERVED
#error "zephyr_kernel_os_impl.h must define Z_OS_THREAD_RESERVED"
#endif
#ifndef Z_MUTEX_INITIALIZER
#error "zephyr_kernel_os_impl.h must define struct k_mutex and Z_MUTEX_INITIALIZER(obj)"
#endif
#ifndef Z_OS_ASSERT
#error "zephyr_kernel_os_impl.h must define Z_OS_ASSERT(test)"
#endif

/* Context and lock ---------------------------------------------------------*/

/* True in an ISR.  Any context. */
static bool z_os_in_isr(void);

/* True before the OS runs threads (before its scheduler starts), when
 * nothing may wait and there is no calling thread.  Any context.
 */
static bool z_os_pre_kernel(void);

/* k_spin_lock() in a thread: no interrupt handler and no other thread runs
 * until the matching z_os_task_unlock().  It nests (a counter), and the OS
 * calls of this header made inside (z_os_wake()) keep it: they must not
 * enable interrupts or switch.  A switch they make due happens at the
 * outermost unlock, as Zephyr switches at k_spin_unlock().
 */
static void z_os_task_lock(void);
static void z_os_task_unlock(void);

/* k_spin_lock() in an ISR: masks the interrupts that may call the layer and
 * returns the previous state for z_os_isr_unlock().  Nests by the state.
 * The state is never UINT32_MAX (the key of a thread's lock).
 */
static uint32_t z_os_isr_lock(void);
static void z_os_isr_unlock(uint32_t state);

/* Threads ------------------------------------------------------------------*/

/* The calling thread.  Threads only. */
static z_os_thread_t z_os_thread_self(void);

/* The calling thread's priority (as it runs now, with any inherited
 * priority).  Threads only.
 */
static z_os_prio_t z_os_thread_prio_self(void);

/* Starts fn(arg) in a new thread named name, on the static stack of
 * stack_size bytes, at the OS priority prio (APP_PRIO_FROM_ZEPHYR() of
 * priorities.h and the OS's own threads' priorities, priorities_os.h).
 * stack is the storage of K_THREAD_STACK_DEFINE() (8-byte aligned, never
 * freed) and stack_size its K_THREAD_STACK_SIZEOF() (a multiple of 8): the
 * Z_OS_THREAD_RESERVED bytes at stack + stack_size are the OS's for the
 * thread's control block, so the start allocates nothing.  *tid is set
 * before the new thread can run, as Zephyr starts its static threads with
 * the scheduler locked.  From a thread or before the scheduler starts.  0,
 * -EINVAL (bad priority or stack) or another negative errno of the OS.
 * When fn returns, the thread calls z_os_thread_exit() (fn does it).  It may
 * be called again on the stack of a thread that has ended (the OS removes
 * its records of that thread first).  Time slices among threads of the
 * same Zephyr priority >= 0 (Zephyr's CONFIG_TIMESLICE_SIZE 20 ms,
 * CONFIG_TIMESLICE_PRIORITY 0) are the OS part's to provide.
 */
static int z_os_thread_start(void (*fn)(void *arg), const char *name, void *stack,
			     size_t stack_size, int prio, void *arg, z_os_thread_t *tid);

/* Ends the calling thread.  Does not return. */
static void z_os_thread_exit(void);

/* The writable name array of thread tid (Z_OS_THREAD_NAME_SIZE bytes), what
 * the OS shows as its name ("ps").  Any context.
 */
static char *z_os_thread_name(z_os_thread_t tid);

/* Gives the turn to the next ready thread of the same priority, or goes on
 * if there is none (Zephyr's k_yield()).  Threads only, not locked.
 */
static void z_os_yield(void);

/* The calling thread's one pointer of its own (Zephyr's custom data of a
 * thread), NULL until it sets one; get returns NULL before the OS runs
 * threads.  Threads only (get also before the scheduler starts).
 */
static void *z_os_thread_local_get(void);
static void z_os_thread_local_set(void *value);

/* Waits and wakes --------------------------------------------------------*/

/* Each thread the layer starts (z_os_thread_start()) has one wake flag of
 * its own, which nothing else of the OS or the application uses; a thread
 * that waits through the layer must be one of them.  The flag is latched: a
 * z_os_wake() made after z_os_wait_prepare() and before the thread's
 * z_os_wait() makes that z_os_wait() return at once (k_sem_take() and
 * k_poll() unlock before they wait, and the give may come in between).
 * Several wakes before a wait count as one.
 */

/* Clears the calling thread's flag (a wake left by an earlier wait that
 * ended by its timeout as the wake came).  Threads only, locked.
 */
static void z_os_wait_prepare(void);

/* Waits until the calling thread's flag is set or ticks have passed (the
 * wait ends at the current tick + ticks, so between ticks - 1 and ticks
 * after the call; Z_OS_TICKS_FOREVER: no timeout; 0 returns at once), and
 * clears it.  With a finite timeout it returns only for one of these two;
 * without one it may also return early (the callers check again).
 * Threads only, not locked.
 */
static void z_os_wait(z_os_ticks_t ticks);

/* Sets the flag of tid, which may be waiting in z_os_wait() or not yet.
 * Threads only, locked (the switch to tid, if it is more urgent, waits for
 * z_os_task_unlock()).
 */
static void z_os_wake(z_os_thread_t tid);

/* z_os_wake() from an ISR, locked: sets *woken (left unchanged otherwise)
 * when tid is more urgent than the interrupted thread.
 */
static void z_os_wake_from_isr(z_os_thread_t tid, z_os_woken_t *woken);

/* At the end of an ISR's use of the layer, unlocked: switches when woken is
 * set, at the end of the interrupt (or does nothing when the OS switches
 * at the end of every interrupt by itself).
 */
static void z_os_yield_from_isr(z_os_woken_t woken);

/* Time ---------------------------------------------------------------------*/

/* Sleeps until the current tick + ticks (1 <= ticks < Z_OS_TICKS_FOREVER).
 * Threads only, not locked.
 */
static void z_os_sleep(z_os_ticks_t ticks);

/* Stops the calling thread with no end (nothing wakes it).  May return
 * (the caller calls it again).  Threads only, not locked.
 */
static void z_os_suspend(void);

/* Spins for at least us microseconds.  Any context. */
static void z_os_busy_wait_us(uint32_t us);

/* Microseconds since the start of the board, monotonic, 64 bits (Zephyr's
 * cycle counter, k_cycle_get_64(), at 1 MHz).  Any context.
 */
static uint64_t z_os_clock_us(void);

/* Mutexes --------------------------------------------------------------------*/

/* The OS's own mutex, so that priority inheritance is the OS's.  Threads
 * only, not locked.  An all-zero struct k_mutex is unlocked.  When a
 * waiter's timeout ends its wait, the owner's priority inherited from it
 * is dropped (as Zephyr's).
 */

/* k_mutex_init(): 0 or -EINVAL. */
static int z_os_mutex_init(struct k_mutex *mutex);

/* Takes mutex, recursively for its holder, waiting at most ticks
 * (z_os_wait() for the meaning; 0: no wait).  True when taken.
 */
static bool z_os_mutex_lock(struct k_mutex *mutex, z_os_ticks_t ticks);

/* Releases one level: 0, -EINVAL when nobody holds it, -EPERM when another
 * thread does (Zephyr's kernel/mutex.c).
 */
static int z_os_mutex_unlock(struct k_mutex *mutex);

#endif /* COMMON_COMPAT_PORT_ZEPHYR_KERNEL_OS_H */
