/* SPDX-License-Identifier: MIT */
/* The direct checks of the board: the tick against the TIMER, and the
 * thread's lock of the layer (z_os_task_lock(), zephyr_kernel_os_impl.h) on
 * ThreadX's Cortex-M0 port, through the layer's primitives only (the shared
 * test checks the lock through k_spin_lock()):
 *
 *   tick_rate    the ThreadX ticks counted while the TIMER runs 1 s
 *                (busy, so every tick interrupt runs): 1000 +- 1
 *   lock_wake    a more urgent thread woken (z_os_wake()) inside the lock
 *                does not run until z_os_task_unlock(), and has run when it
 *                returns
 *   lock_nested  the same with the lock taken twice: not at the inner
 *                unlock, at the outer one
 *   lock_alarm   a TIMER alarm due inside the lock (its interrupt is
 *                masked) runs at the unlock, not before, and the more
 *                urgent thread it wakes (z_os_wake_from_isr()) runs at the
 *                end of that interrupt, before the unlock returns
 *
 * Each lock is held for LOCK_HOLD_US, spinning on the TIMER (a tick is
 * lost meanwhile and caught up, src/tick.c).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/timer.h"
#include "pico/time.h"

#include <zephyr/kernel.h>

#include "board_kernel.h"
#include "priorities.h"
#include "zephyr_kernel_os.h"

#define LOCK_HOLD_US 2000
#define ALARM_US     300
/* Zephyr 10: above the caller (14). */
#define WAKER_PRIO   APP_PRIO_FROM_ZEPHYR(10)
#define WAKER_STACK  1024

char board_tick_line[96];

static K_THREAD_STACK_DEFINE(waker_stack, WAKER_STACK);
static z_os_thread_t waker_tid;
static volatile bool waker_waiting;
static volatile bool waker_ran;
static volatile uint64_t waker_us;

static volatile bool alarm_ran;
static volatile bool alarm_in_isr;
static volatile uint64_t alarm_us;

void board_result(const char *name, bool ok, int *passed, int *failed)
{
	printf("%s: %s\n", name, ok ? "ok" : "FAILED");
	if (ok) {
		(*passed)++;
	} else {
		(*failed)++;
	}
}

void board_tick_test(int *passed, int *failed)
{
	uint32_t t0;
	uint32_t ticks;
	uint64_t u0;
	uint64_t us;

	z_os_sleep(1);
	t0 = (uint32_t)tx_time_get();
	u0 = time_us_64();
	busy_wait_us(1000000);
	ticks = (uint32_t)tx_time_get() - t0;
	us = time_us_64() - u0;
	snprintf(board_tick_line, sizeof(board_tick_line),
		 "tick_rate: %lu ThreadX ticks in %llu us of the TIMER", (unsigned long)ticks,
		 (unsigned long long)us);
	printf("%s\n", board_tick_line);
	board_result("tick_rate", ticks >= 999 && ticks <= 1001, passed, failed);
}

/* The waker: waits for its flag (more urgent than the caller, it gets there
 * before z_os_thread_start() returns), then records when it ran and ends
 * (z_os_thread_start() reuses the stack of the ended thread).
 */
static void waker_main(void *arg)
{
	(void)arg;
	z_os_wait_prepare();
	waker_waiting = true;
	z_os_wait(Z_OS_TICKS_FOREVER);
	waker_us = time_us_64();
	waker_ran = true;
	z_os_thread_exit();
}

static bool waker_start(void)
{
	waker_waiting = false;
	waker_ran = false;
	if (z_os_thread_start(waker_main, "lock_waker", waker_stack,
			      K_THREAD_STACK_SIZEOF(waker_stack), WAKER_PRIO, NULL,
			      &waker_tid) != 0) {
		printf("  could not start the waker\n");
		return false;
	}
	if (!waker_waiting) {
		printf("  the waker did not run first\n");
		return false;
	}
	return true;
}

/* Lets the waker end before its stack is used again (woken here if a
 * failed check left it waiting).
 */
static void waker_join(void)
{
	if (waker_waiting && !waker_ran) {
		z_os_task_lock();
		z_os_wake(waker_tid);
		z_os_task_unlock();
	}
	for (int i = 0; i < 10 && !waker_ran; i++) {
		z_os_sleep(1);
	}
}

static void test_lock_wake(int *passed, int *failed)
{
	bool inside;
	bool after;
	bool ok = waker_start();

	z_os_task_lock();
	z_os_wake(waker_tid);
	z_os_busy_wait_us(LOCK_HOLD_US);
	inside = waker_ran;
	z_os_task_unlock();
	after = waker_ran;
	printf("  inside the lock: %s, after the unlock: %s\n", inside ? "ran" : "not run",
	       after ? "ran" : "not run");
	waker_join();
	board_result("lock_wake", ok && !inside && after, passed, failed);
}

static void test_lock_nested(int *passed, int *failed)
{
	bool inside;
	bool between;
	bool after;
	bool ok = waker_start();

	z_os_task_lock();
	z_os_task_lock();
	z_os_wake(waker_tid);
	z_os_busy_wait_us(LOCK_HOLD_US);
	inside = waker_ran;
	z_os_task_unlock();
	z_os_busy_wait_us(LOCK_HOLD_US);
	between = waker_ran;
	z_os_task_unlock();
	after = waker_ran;
	printf("  inside: %s, after the inner unlock: %s, after the outer: %s\n",
	       inside ? "ran" : "not run", between ? "ran" : "not run", after ? "ran" : "not run");
	waker_join();
	board_result("lock_nested", ok && !inside && !between && after, passed, failed);
}

/* In the alarm's interrupt (the Pico SDK's default alarm pool). */
static int64_t alarm_fired(alarm_id_t id, void *user_data)
{
	z_os_woken_t woken = 0;
	uint32_t key;

	(void)id;
	(void)user_data;
	alarm_us = time_us_64();
	alarm_in_isr = z_os_in_isr();
	alarm_ran = true;
	key = z_os_isr_lock();
	z_os_wake_from_isr(waker_tid, &woken);
	z_os_isr_unlock(key);
	z_os_yield_from_isr(woken);
	return 0; /* once */
}

static void test_lock_alarm(int *passed, int *failed)
{
	bool armed;
	bool isr_inside;
	bool waker_inside;
	bool isr_after;
	bool waker_after;
	uint64_t unlock_us;
	bool ok = waker_start();

	alarm_ran = false;
	alarm_in_isr = false;
	z_os_task_lock();
	armed = add_alarm_in_us(ALARM_US, alarm_fired, NULL, true) > 0;
	z_os_busy_wait_us(LOCK_HOLD_US);
	isr_inside = alarm_ran;
	waker_inside = waker_ran;
	unlock_us = time_us_64();
	z_os_task_unlock();
	isr_after = alarm_ran;
	waker_after = waker_ran;
	if (!armed) {
		printf("  could not arm the alarm\n");
	}
	printf("  inside the lock: alarm %s, waker %s; after the unlock: alarm %s, waker %s\n",
	       isr_inside ? "ran" : "not run", waker_inside ? "ran" : "not run",
	       isr_after ? "ran" : "not run", waker_after ? "ran" : "not run");
	if (isr_after && waker_after) {
		printf("  the alarm ran %lld us after the unlock began,"
		       " the waker %lld us after the alarm\n",
		       (long long)(alarm_us - unlock_us), (long long)(waker_us - alarm_us));
	}
	waker_join();
	board_result("lock_alarm",
		     ok && armed && !isr_inside && !waker_inside && isr_after && waker_after &&
			     alarm_in_isr && alarm_us >= unlock_us && waker_us >= alarm_us,
		     passed, failed);
}

void board_lock_test(int *passed, int *failed)
{
	test_lock_wake(passed, failed);
	test_lock_nested(passed, failed);
	test_lock_alarm(passed, failed);
}
