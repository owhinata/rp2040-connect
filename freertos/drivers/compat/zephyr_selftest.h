/* SPDX-License-Identifier: MIT */
/* The self-test of the FreeRTOS part of the Zephyr compatibility layer
 * (zephyr_selftest.c): the "selftest" command of the board's console (built
 * with -DNANO_SELFTEST=ON) and the host test on the FreeRTOS POSIX port
 * (freertos/tests/compat) run the same checks of Zephyr's meanings: mutexes
 * (all zero, recursive, timeouts, priority inheritance), semaphores (limit,
 * timeouts, the order of the waiters, reset), sleeps and their rounding,
 * the clock, k_sem_give() and k_work_submit() from interrupt handlers,
 * k_poll(), delayable work items, the spin lock, atomics, K_THREAD_DEFINE()
 * threads, thread names, time slices of 20 ms, and Zephyr's rule that a
 * thread preempted by a higher one keeps its turn (zephyr_os_sched.h).
 *
 * Call it from a task at NANO_PRIO_SHELL (Zephyr 14, the shell's thread)
 * after zephyr_compat_initialize(): the checks of the order in which tasks
 * run assume it.  It prints one line per test and the measured times, after
 * each test (zephyr_selftest_port_quiet() then waits until printing no longer
 * disturbs), and runs for about a second; two tasks spin for 140 ms twice
 * (every lower priority waits then).  The K_THREAD_DEFINE() threads start at the first run
 * only (and then wait for ever); later runs skip that test.
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_SELFTEST_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_SELFTEST_H

#include <stdbool.h>
#include <stdint.h>

/* Runs the tests; returns the number of tests that failed. */
int zephyr_selftest_run(void);

/* The platform's part (rp2040/zephyr_selftest_port.c on the board, the host
 * test's own on the host) ---------------------------------------------------*/

/* Calls fn(arg) once from a hardware interrupt handler about us microseconds
 * later (the board: an alarm of the TIMER).  0, or -ENOTSUP where there is
 * none (the host: only the tick is an interrupt there).
 */
int zephyr_selftest_port_alarm(uint32_t us, void (*fn)(void *arg), void *arg);

/* Writes out what the console has been given and returns when the writing
 * no longer disturbs the measurements (the board: the USB transfers are done,
 * so no more USB interrupts and timer-task wake-ups follow).  Called after
 * each test.
 */
void zephyr_selftest_port_quiet(void);

/* True if the tick follows real time (the board), so the times measured with
 * the TIMER (k_cycle_get_64()) are checked too; the POSIX port's tick is a
 * thread's usleep() and only the tick counts are checked there.
 */
bool zephyr_selftest_port_realtime(void);

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_SELFTEST_H */
