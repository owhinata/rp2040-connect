/* SPDX-License-Identifier: MIT */
/* The host test of the Zephyr compatibility layer's kernel
 * (zephyr_kernel_test.c), on an OS's part of the layer.  It checks Zephyr's
 * meanings:
 * mutexes (all zero, recursive, timeouts, priority inheritance), semaphores
 * (limit, timeouts, the order of the waiters, reset), sleeps and their
 * rounding, the clock, k_sem_give() and k_work_submit() from interrupt
 * handlers, k_poll(), delayable work items, the spin lock, atomics,
 * K_THREAD_DEFINE() threads, thread names, time slices of 20 ms, and the
 * tick's catch-up after interrupts were masked for 50 ms (where the port
 * can mask them).  The FreeRTOS build runs it on the FreeRTOS POSIX port
 * (freertos/tests/compat, freertos/script/host_tests.sh); another OS's host
 * test can run it on that OS's part through the boundary below.
 *
 * Call it from a thread at Zephyr priority 14 (APP_PRIO_SHELL, the shell's
 * thread) after zephyr_compat_initialize(): the checks of the order in which
 * threads run assume it.  It prints one line per test and the measured times,
 * after each test (zephyr_kernel_test_port_quiet() then waits until printing no
 * longer disturbs), and runs for about a second; two threads spin for 140 ms
 * (every lower priority waits then).  The K_THREAD_DEFINE() threads start at
 * the first run only (and then wait for ever); later runs skip that test.
 *
 * What a host test provides:
 *
 * - "zephyr_kernel_test_os.h" on the include path, read after <zephyr/kernel.h>
 *   (the OS's part; for an OS on the generic kernel part's primitives,
 *   kernel_generic/zephyr_kernel_test_os.h of this directory with the OS's
 *   zephyr_kernel_test_os_impl.h; FreeRTOS:
 *   freertos/tests/compat/zephyr_kernel_test_os_impl.h):
 *
 *     zephyr_kernel_test_os_prio_t        the type of an OS priority
 *     ZEPHYR_KERNEL_TEST_OS_NAME_MAX      the longest thread name, in characters
 *                                      (k_thread_name_set() cuts to it)
 *     uint32_t zephyr_kernel_test_os_tick(void)
 *                                      the OS's tick count
 *     void zephyr_kernel_test_os_tick_sync(void)
 *                                      returns just after a tick
 *     bool zephyr_kernel_test_os_thread_create(void (*fn)(void *),
 *             const char *name, size_t stack_size, void *arg, int prio,
 *             k_tid_t *tid)
 *                                      starts fn(arg) in a new thread at the
 *                                      OS priority prio (K_PRIO_PREEMPT());
 *                                      one above the caller runs at once;
 *                                      true if it started
 *     void zephyr_kernel_test_os_thread_exit(void)
 *                                      called by such a thread at its end
 *     zephyr_kernel_test_os_prio_t zephyr_kernel_test_os_prio_get(k_tid_t tid)
 *     zephyr_kernel_test_os_prio_t zephyr_kernel_test_os_prio_self(void)
 *                                      the current priority (raised by
 *                                      priority inheritance) of a thread, of
 *                                      the caller
 *     const char *zephyr_kernel_test_os_name_get(k_tid_t tid)
 *                                      the name the OS shows for a thread
 *     k_tid_t zephyr_kernel_test_os_current(void)
 *                                      the calling thread, as the OS has it
 *                                      (to compare with k_current_get())
 *     bool zephyr_kernel_test_os_in_isr(void)
 *                                      true in an interrupt handler, and in
 *                                      a timeout of z_add_timeout()
 *     void zephyr_kernel_test_os_sched_lock(void),
 *     void zephyr_kernel_test_os_sched_unlock(void)
 *                                      no other thread runs in between; the
 *                                      threads started meanwhile run at the
 *                                      unlock
 *
 * - the platform's part, the four zephyr_kernel_test_port_*() functions
 *   below (FreeRTOS: freertos/tests/compat/main.c).  The checks of real time,
 *   of interrupt handlers and of the tick's catch-up need a platform whose
 *   tick follows real time and that can mask interrupts (a board); the
 *   FreeRTOS POSIX port has neither, so its part returns false and -ENOTSUP
 *   and those checks are skipped there.
 */

#ifndef COMMON_TESTS_ZEPHYR_KERNEL_ZEPHYR_KERNEL_TEST_H
#define COMMON_TESTS_ZEPHYR_KERNEL_ZEPHYR_KERNEL_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* Runs the tests; returns the number of tests that failed. */
int zephyr_kernel_test_run(void);

/* The platform's part ---------------------------------------------------------*/

/* Calls fn(arg) once from a hardware interrupt handler about us microseconds
 * later (the board: an alarm of the TIMER).  0, or -ENOTSUP where there is
 * none (the FreeRTOS host test: only the tick is an interrupt there).
 */
int zephyr_kernel_test_port_alarm(uint32_t us, void (*fn)(void *arg), void *arg);

/* Writes out what the console has been given and returns when the writing
 * no longer disturbs the measurements (the board: the USB transfers are done,
 * so no more USB interrupts and timer-task wake-ups follow).  Called after
 * each test.
 */
void zephyr_kernel_test_port_quiet(void);

/* True if the tick follows real time (the board), so the times measured with
 * k_cycle_get_64() are checked too; the FreeRTOS POSIX port's tick is a
 * thread's usleep() and only the tick counts are checked there.
 */
bool zephyr_kernel_test_port_realtime(void);

/* Masks every interrupt for us microseconds (spinning on the clock), as a
 * flash erase does, and returns 0 (on a board with the FreeRTOS build's tick
 * hook, which counts the ticks lost meanwhile, freertos/app/usb-console/src/tick.c);
 * -ENOTSUP where the tick cannot catch up (the FreeRTOS host test: the POSIX
 * port's tick is a thread's usleep()).
 */
int zephyr_kernel_test_port_mask_irqs(uint32_t us);

#endif /* COMMON_TESTS_ZEPHYR_KERNEL_ZEPHYR_KERNEL_TEST_H */
