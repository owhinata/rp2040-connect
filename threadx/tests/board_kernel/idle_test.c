/* SPDX-License-Identifier: MIT */
/* The checks of the idle wait's CPU load (threadx/app/usb-console/src/
 * idle.c, built here with IDLE_BLINK=0: the LED is this firmware's) and of
 * the threads' run time (run_time.c, ThreadX's execution profile kit):
 *
 *   idle     the runner sleeps 1 s with nothing else ready: cpu_load_get()
 *            below 5%, and tx_low_power_enter() runs about once per tick
 *            (900 to 3000 times; the scheduler's wait sleeps in WFI and
 *            each interrupt ends one wait); then the runner spins 1 s:
 *            above 95%
 *   profile  two threads spin 300 ms and 100 ms of the TIMER inside a 1 s
 *            window while the runner sleeps: their run times within 1% of
 *            that; the threads' run time over the window within 2% of the
 *            window's cpu_load_get() (handlers in the wait count as load
 *            there, as idle in the kit); the threads' and the kit's idle
 *            time add up to the window within 1%
 *
 * The count of tx_low_power_enter() comes from the link
 * (--wrap=tx_low_power_enter, CMakeLists.txt): the call in ThreadX's
 * scheduler reaches __wrap_tx_low_power_enter() below.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/time.h"

#include "tx_api.h"

#include <zephyr/kernel.h>

#include "board_kernel.h"
#include "cpu_load.h"
#include "priorities.h"
#include "run_time.h"
#include "zephyr_kernel_os.h"
#include "zephyr_kernel_test.h"

#define WINDOW_MS   1000
#define SPIN_A_US   300000u
#define SPIN_B_US   100000u
#define SPIN_STACK  1024
#define SPIN_A_PRIO APP_PRIO_FROM_ZEPHYR(9)
#define SPIN_B_PRIO APP_PRIO_FROM_ZEPHYR(10)

/* The idle bounds, in tenths of a percent. */
#define IDLE_LOAD_MAX 50
#define BUSY_LOAD_MIN 950

void __wrap_tx_low_power_enter(void);
void __real_tx_low_power_enter(void);

/* In the scheduler's wait (PendSV, interrupts masked). */
static volatile uint32_t enter_count;

void __wrap_tx_low_power_enter(void)
{
	enter_count++;
	__real_tx_low_power_enter();
}

static void test_idle(int *passed, int *failed)
{
	uint32_t n0;
	uint32_t n1;
	uint64_t t0;
	uint64_t t1;
	int idle;
	int busy;

	zephyr_kernel_test_port_quiet();
	(void)cpu_load_get(true);
	n0 = enter_count;
	t0 = time_us_64();
	z_os_sleep(WINDOW_MS);
	idle = cpu_load_get(true);
	n1 = enter_count;
	t1 = time_us_64();
	busy_wait_us(WINDOW_MS * 1000u);
	busy = cpu_load_get(true);

	printf("  sleeping: cpu_load %d.%d%%, tx_low_power_enter %lu times in %llu us\n", idle / 10,
	       idle % 10, (unsigned long)(n1 - n0), (unsigned long long)(t1 - t0));
	printf("  spinning: cpu_load %d.%d%%\n", busy / 10, busy % 10);
	board_result("idle",
		     idle < IDLE_LOAD_MAX && busy > BUSY_LOAD_MIN && n1 - n0 >= 900 &&
			     n1 - n0 <= 3000,
		     passed, failed);
}

struct spinner {
	uint32_t us;
	volatile bool done;
};

static K_THREAD_STACK_DEFINE(spin_a_stack, SPIN_STACK);
static K_THREAD_STACK_DEFINE(spin_b_stack, SPIN_STACK);
static z_os_thread_t spin_a_tid;
static z_os_thread_t spin_b_tid;

/* Waits for the next tick or so, so that both are started before either
 * spins, then spins its time of the TIMER and ends.
 */
static void spin_main(void *arg)
{
	struct spinner *s = arg;
	uint64_t t0;

	z_os_sleep(10);
	t0 = time_us_64();
	while (time_us_64() - t0 < s->us) {
	}
	s->done = true;
}

static uint32_t permille(uint64_t part, uint64_t whole)
{
	return whole == 0 ? 0 : (uint32_t)(part * 1000u / whole);
}

static uint64_t diff(uint64_t a, uint64_t b)
{
	return a > b ? a - b : b - a;
}

static void test_profile(int *passed, int *failed)
{
	static struct spinner a = {.us = SPIN_A_US};
	static struct spinner b = {.us = SPIN_B_US};
	EXECUTION_TIME threads0;
	EXECUTION_TIME threads1;
	uint64_t w0;
	uint64_t w1;
	uint64_t idle0;
	uint64_t idle1;
	uint64_t a_us = 0;
	uint64_t b_us = 0;
	uint64_t window;
	uint64_t threads;
	uint64_t idle;
	int load;
	bool started;

	zephyr_kernel_test_port_quiet();
	a.done = false;
	b.done = false;
	(void)cpu_load_get(true);
	(void)_tx_execution_thread_total_time_get(&threads0);
	idle0 = run_time_idle_us();
	w0 = run_time_total_us();

	started = z_os_thread_start(spin_main, "spin_a", spin_a_stack,
				    K_THREAD_STACK_SIZEOF(spin_a_stack), SPIN_A_PRIO, &a,
				    &spin_a_tid) == 0 &&
		  z_os_thread_start(spin_main, "spin_b", spin_b_stack,
				    K_THREAD_STACK_SIZEOF(spin_b_stack), SPIN_B_PRIO, &b,
				    &spin_b_tid) == 0;
	z_os_sleep(WINDOW_MS);

	load = cpu_load_get(true);
	w1 = run_time_total_us();
	idle1 = run_time_idle_us();
	(void)_tx_execution_thread_total_time_get(&threads1);
	if (started) {
		a_us = run_time_thread_us(spin_a_tid);
		b_us = run_time_thread_us(spin_b_tid);
	}

	window = w1 - w0;
	threads = threads1 - threads0;
	idle = idle1 - idle0;
	printf("  spin_a %llu us, spin_b %llu us (spun %lu and %lu)\n", (unsigned long long)a_us,
	       (unsigned long long)b_us, (unsigned long)SPIN_A_US, (unsigned long)SPIN_B_US);
	printf("  window %llu us: threads %llu us (%lu permille), idle %llu us,"
	       " cpu_load %d permille\n",
	       (unsigned long long)window, (unsigned long long)threads,
	       (unsigned long)permille(threads, window), (unsigned long long)idle, load);
	board_result("profile",
		     started && a.done && b.done && diff(a_us, SPIN_A_US) <= SPIN_A_US / 100 &&
			     diff(b_us, SPIN_B_US) <= SPIN_B_US / 100 &&
			     diff(permille(threads, window), (uint32_t)load) <= 20 &&
			     diff(threads + idle, window) <= window / 100,
		     passed, failed);
}

void board_idle_test(int *passed, int *failed)
{
	test_idle(passed, failed);
	test_profile(passed, failed);
}
