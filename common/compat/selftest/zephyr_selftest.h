/* SPDX-License-Identifier: MIT */
/* The self-test of the Zephyr compatibility layer (zephyr_selftest.c): the
 * acceptance test of an OS's part of the layer.  It checks Zephyr's meanings:
 * mutexes (all zero, recursive, timeouts, priority inheritance), semaphores
 * (limit, timeouts, the order of the waiters, reset), sleeps and their
 * rounding, the clock, k_sem_give() and k_work_submit() from interrupt
 * handlers, k_poll(), delayable work items, the spin lock, atomics,
 * K_THREAD_DEFINE() threads, thread names, time slices of 20 ms, and the
 * tick's catch-up after interrupts were masked for 50 ms (where the port
 * can mask them).  The FreeRTOS build runs it as the "selftest" command of
 * the board's console (built with -DNANO_SELFTEST=ON) and as a host test on
 * the FreeRTOS POSIX port (freertos/tests/compat).
 *
 * Call it from a thread at Zephyr priority 14 (NANO_PRIO_SHELL, the shell's
 * thread) after zephyr_compat_initialize(): the checks of the order in which
 * threads run assume it.  It prints one line per test and the measured times,
 * after each test (zephyr_selftest_port_quiet() then waits until printing no
 * longer disturbs), and runs for about a second; two threads spin for 140 ms
 * (every lower priority waits then).  The K_THREAD_DEFINE() threads start at
 * the first run only (and then wait for ever); later runs skip that test.
 *
 * What a build provides:
 *
 * - "zephyr_selftest_os.h" on the include path, read after <zephyr/kernel.h>
 *   (the OS's part; FreeRTOS: common/compat/kernel_generic/zephyr_selftest_os.h
 *   on the primitives, with freertos/drivers/compat/zephyr_selftest_os_impl.h):
 *
 *     zephyr_selftest_os_prio_t        the type of an OS priority
 *     ZEPHYR_SELFTEST_OS_NAME_MAX      the longest thread name, in characters
 *                                      (k_thread_name_set() cuts to it)
 *     uint32_t zephyr_selftest_os_tick(void)
 *                                      the OS's tick count
 *     void zephyr_selftest_os_tick_sync(void)
 *                                      returns just after a tick
 *     bool zephyr_selftest_os_thread_create(void (*fn)(void *),
 *             const char *name, size_t stack_size, void *arg, int prio,
 *             k_tid_t *tid)
 *                                      starts fn(arg) in a new thread at the
 *                                      OS priority prio (K_PRIO_PREEMPT());
 *                                      one above the caller runs at once;
 *                                      true if it started
 *     void zephyr_selftest_os_thread_exit(void)
 *                                      called by such a thread at its end
 *     zephyr_selftest_os_prio_t zephyr_selftest_os_prio_get(k_tid_t tid)
 *     zephyr_selftest_os_prio_t zephyr_selftest_os_prio_self(void)
 *                                      the current priority (raised by
 *                                      priority inheritance) of a thread, of
 *                                      the caller
 *     const char *zephyr_selftest_os_name_get(k_tid_t tid)
 *                                      the name the OS shows for a thread
 *     k_tid_t zephyr_selftest_os_current(void)
 *                                      the calling thread, as the OS has it
 *                                      (to compare with k_current_get())
 *     bool zephyr_selftest_os_in_isr(void)
 *                                      true in an interrupt handler, and in
 *                                      a timeout of z_add_timeout()
 *     void zephyr_selftest_os_sched_lock(void),
 *     void zephyr_selftest_os_sched_unlock(void)
 *                                      no other thread runs in between; the
 *                                      threads started meanwhile run at the
 *                                      unlock
 *
 * - the board's or host's part, the four zephyr_selftest_port_*() functions
 *   below (FreeRTOS: freertos/drivers/compat/rp2040/zephyr_selftest_port.c on
 *   the board, freertos/tests/compat/main.c on the host).
 */

#ifndef COMMON_COMPAT_SELFTEST_ZEPHYR_SELFTEST_H
#define COMMON_COMPAT_SELFTEST_ZEPHYR_SELFTEST_H

#include <stdbool.h>
#include <stdint.h>

/* Runs the tests; returns the number of tests that failed. */
int zephyr_selftest_run(void);

/* The platform's part ---------------------------------------------------------*/

/* Calls fn(arg) once from a hardware interrupt handler about us microseconds
 * later (the board: an alarm of the TIMER).  0, or -ENOTSUP where there is
 * none (the FreeRTOS host test: only the tick is an interrupt there).
 */
int zephyr_selftest_port_alarm(uint32_t us, void (*fn)(void *arg), void *arg);

/* Writes out what the console has been given and returns when the writing
 * no longer disturbs the measurements (the board: the USB transfers are done,
 * so no more USB interrupts and timer-task wake-ups follow).  Called after
 * each test.
 */
void zephyr_selftest_port_quiet(void);

/* True if the tick follows real time (the board), so the times measured with
 * k_cycle_get_64() are checked too; the FreeRTOS POSIX port's tick is a
 * thread's usleep() and only the tick counts are checked there.
 */
bool zephyr_selftest_port_realtime(void);

/* Masks every interrupt for us microseconds (spinning on the clock), as a
 * flash erase does, and returns 0 (the FreeRTOS board build: the tick hook
 * then counts the ticks lost meanwhile, freertos/app/usb-console/src/tick.c);
 * -ENOTSUP where the tick cannot catch up (the FreeRTOS host test: the POSIX
 * port's tick is a thread's usleep()).
 */
int zephyr_selftest_port_mask_irqs(uint32_t us);

#endif /* COMMON_COMPAT_SELFTEST_ZEPHYR_SELFTEST_H */
