/* SPDX-License-Identifier: MIT */
/* The checks of the Pico SDK's locks and sleeps on ThreadX
 * (threadx/drivers/pico_sdk_interop):
 *
 *   sdk_pre_kernel     sleep_ms(2) and an SDK mutex in main() before ThreadX
 *                      is set up (the SDK's own waits): 2.0 to 2.2 ms, and
 *                      the owner is 0 (core 0's id)
 *   sdk_mutex          a thread holds an SDK mutex for 200 ms, sleeping in
 *                      ThreadX; a more urgent one blocks in
 *                      mutex_enter_blocking() and gets it when it is let go
 *                      (190 to 230 ms later); meanwhile the least urgent one,
 *                      spinning, gets the processor (it would not if the
 *                      waiter spun, and the holder would never let go)
 *   sdk_mutex_timeout  with the mutex held by that thread: mutex_try_enter()
 *                      fails and names the holder's TX_THREAD as the owner,
 *                      mutex_enter_timeout_ms(50) fails after 50 to 52 ms,
 *                      mutex_enter_timeout_ms(1000) gets it when it is let
 *                      go, then mutex_try_enter() succeeds
 *   sdk_sleep          sleep_ms(100) of a thread lasts 100 to 102 ms of the
 *                      TIMER, and a less urgent spinning thread runs during
 *                      it
 *   sdk_printf         two threads printf() at once (Zephyr 6 every 1 ms,
 *                      Zephyr 9 without a pause, a few lines each; the SDK's
 *                      stdout and stdio_usb mutexes): both end within 3 s,
 *                      and no printf() takes 500 ms or more (the SDK gives
 *                      up a mutex after PICO_STDIO_DEADLOCK_TIMEOUT_MS, 1 s)
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/mutex.h"
#include "pico/time.h"

#include "tx_api.h"

#include <zephyr/kernel.h>

#include "board_kernel.h"
#include "priorities.h"
#include "zephyr_kernel_os.h"
#include "zephyr_kernel_test.h"

#define STACK_SIZE    1536
#define URGENT_PRIO   APP_PRIO_FROM_ZEPHYR(5)
#define HOLDER_PRIO   APP_PRIO_FROM_ZEPHYR(8)
#define SPINNER_PRIO  APP_PRIO_FROM_ZEPHYR(11)
#define HOLD_MS       200
#define SPIN_US       400000u
#define SLEEP_MS      100
#define PRINT_LINES   5
#define PRINT_HI_PRIO APP_PRIO_FROM_ZEPHYR(6)
#define PRINT_LO_PRIO APP_PRIO_FROM_ZEPHYR(9)
#define JOIN_MS       3000

static K_THREAD_STACK_DEFINE(stack_a, STACK_SIZE);
static K_THREAD_STACK_DEFINE(stack_b, STACK_SIZE);
static K_THREAD_STACK_DEFINE(stack_c, STACK_SIZE);
static z_os_thread_t tid_a;
static z_os_thread_t tid_b;
static z_os_thread_t tid_c;

static mutex_t mtx;

/* Before the kernel ----------------------------------------------------------*/

static uint64_t pre_sleep_us;
static uint32_t pre_owner = 1;

void board_sdk_pre_kernel(void)
{
	uint64_t t0 = time_us_64();

	sleep_ms(2);
	pre_sleep_us = time_us_64() - t0;

	mutex_init(&mtx);
	mutex_enter_blocking(&mtx);
	pre_owner = (uint32_t)mtx.owner;
	mutex_exit(&mtx);
}

/* The threads ------------------------------------------------------------------*/

static bool start(void (*fn)(void *), const char *name, k_thread_stack_t *stack, size_t size,
		  int prio, void *arg, z_os_thread_t *tid)
{
	if (z_os_thread_start(fn, name, stack, size, prio, arg, tid) != 0) {
		printf("  could not start %s\n", name);
		return false;
	}
	return true;
}

/* Up to JOIN_MS for each flag. */
static bool join(volatile bool *a, volatile bool *b, volatile bool *c)
{
	for (int ms = 0; ms < JOIN_MS; ms++) {
		if ((a == NULL || *a) && (b == NULL || *b) && (c == NULL || *c)) {
			return true;
		}
		z_os_sleep(1);
	}
	return false;
}

static volatile bool holder_done;
static volatile uint64_t holder_release_us;

static void holder_main(void *arg)
{
	(void)arg;
	mutex_enter_blocking(&mtx);
	z_os_sleep(HOLD_MS);
	holder_release_us = time_us_64();
	mutex_exit(&mtx);
	holder_done = true;
}

static bool holder_start(void)
{
	holder_done = false;
	return start(holder_main, "sdk_holder", stack_a, K_THREAD_STACK_SIZEOF(stack_a),
		     HOLDER_PRIO, NULL, &tid_a);
}

/* Counts its loops while *watch is set, until SPIN_US of the TIMER since it
 * began.
 */
struct spinner {
	volatile bool *watch;
	uint32_t during;
	uint32_t loops;
	volatile bool done;
};

static void spinner_main(void *arg)
{
	struct spinner *s = arg;
	uint64_t t0 = time_us_64();

	while (time_us_64() - t0 < SPIN_US) {
		s->loops++;
		if (*s->watch) {
			s->during++;
		}
	}
	s->done = true;
}

static bool spinner_start(struct spinner *s, volatile bool *watch)
{
	s->watch = watch;
	s->during = 0;
	s->loops = 0;
	s->done = false;
	return start(spinner_main, "sdk_spinner", stack_c, K_THREAD_STACK_SIZEOF(stack_c),
		     SPINNER_PRIO, s, &tid_c);
}

/* sdk_mutex ---------------------------------------------------------------------*/

static volatile bool waiter_waiting;
static volatile bool waiter_done;
static volatile uint64_t waiter_t0;
static volatile uint64_t waiter_t1;

static void waiter_main(void *arg)
{
	(void)arg;
	waiter_t0 = time_us_64();
	waiter_waiting = true;
	mutex_enter_blocking(&mtx);
	waiter_waiting = false;
	waiter_t1 = time_us_64();
	mutex_exit(&mtx);
	waiter_done = true;
}

static void test_mutex(int *passed, int *failed)
{
	static struct spinner s;
	uint64_t waited;
	bool ok;

	waiter_done = false;
	waiter_waiting = false;
	ok = holder_start() &&
	     start(waiter_main, "sdk_waiter", stack_b, K_THREAD_STACK_SIZEOF(stack_b), URGENT_PRIO,
		   NULL, &tid_b) &&
	     spinner_start(&s, &waiter_waiting);
	ok = join(&holder_done, &waiter_done, &s.done) && ok;

	waited = waiter_t1 - waiter_t0;
	printf("  the waiter blocked %llu us; the spinner looped %lu times meanwhile, %lu in all\n",
	       (unsigned long long)waited, (unsigned long)s.during, (unsigned long)s.loops);
	board_result("sdk_mutex",
		     ok && waited >= (HOLD_MS - 10) * 1000u && waited <= (HOLD_MS + 30) * 1000u &&
			     waiter_t1 >= holder_release_us && s.during > 0,
		     passed, failed);
}

static void test_mutex_timeout(int *passed, int *failed)
{
	uint32_t owner = 0;
	bool tried;
	bool short_wait;
	bool long_wait = false;
	bool tried_after = false;
	uint64_t t0;
	uint64_t short_us;
	uint64_t long_us = 0;
	bool ok = holder_start();

	tried = mutex_try_enter(&mtx, &owner);
	t0 = time_us_64();
	short_wait = mutex_enter_timeout_ms(&mtx, 50);
	short_us = time_us_64() - t0;
	if (!short_wait) {
		long_wait = mutex_enter_timeout_ms(&mtx, 1000);
		long_us = time_us_64() - t0;
		if (long_wait) {
			mutex_exit(&mtx);
			tried_after = mutex_try_enter(&mtx, NULL);
			if (tried_after) {
				mutex_exit(&mtx);
			}
		}
	}
	ok = join(&holder_done, NULL, NULL) && ok;

	printf("  try_enter %s (owner %s the holder); timeout_ms(50) %s after %llu us;"
	       " timeout_ms(1000) %s after %llu us; try_enter %s\n",
	       tried ? "entered" : "failed", owner == (uint32_t)(uintptr_t)tid_a ? "is" : "is not",
	       short_wait ? "entered" : "timed out", (unsigned long long)short_us,
	       long_wait ? "entered" : "timed out", (unsigned long long)long_us,
	       tried_after ? "entered" : "failed");
	board_result("sdk_mutex_timeout",
		     ok && !tried && owner == (uint32_t)(uintptr_t)tid_a && !short_wait &&
			     short_us >= 50000u && short_us <= 52000u && long_wait &&
			     long_us <= (HOLD_MS + 5) * 1000u && tried_after,
		     passed, failed);
}

/* sdk_sleep ---------------------------------------------------------------------*/

static volatile bool sleeper_sleeping;
static volatile bool sleeper_done;
static volatile uint64_t sleeper_us;

static void sleeper_main(void *arg)
{
	uint64_t t0;

	(void)arg;
	sleeper_sleeping = true;
	t0 = time_us_64();
	sleep_ms(SLEEP_MS);
	sleeper_us = time_us_64() - t0;
	sleeper_sleeping = false;
	sleeper_done = true;
}

static void test_sleep(int *passed, int *failed)
{
	static struct spinner s;
	bool ok;

	sleeper_done = false;
	sleeper_sleeping = false;
	ok = start(sleeper_main, "sdk_sleeper", stack_b, K_THREAD_STACK_SIZEOF(stack_b),
		   URGENT_PRIO, NULL, &tid_b) &&
	     spinner_start(&s, &sleeper_sleeping);
	ok = join(&sleeper_done, &s.done, NULL) && ok;

	printf("  sleep_ms(%d) took %llu us; the spinner looped %lu times meanwhile\n", SLEEP_MS,
	       (unsigned long long)sleeper_us, (unsigned long)s.during);
	board_result("sdk_sleep",
		     ok && sleeper_us >= SLEEP_MS * 1000u && sleeper_us <= (SLEEP_MS + 2) * 1000u &&
			     s.during > 0,
		     passed, failed);
}

/* sdk_printf --------------------------------------------------------------------*/

struct printer {
	const char *name;
	uint32_t gap_ms;
	uint32_t max_us;
	volatile bool done;
};

static void printer_main(void *arg)
{
	struct printer *p = arg;

	for (int i = 0; i < PRINT_LINES; i++) {
		uint64_t t0 = time_us_64();
		uint32_t us;

		printf("  %s %d\n", p->name, i);
		us = (uint32_t)(time_us_64() - t0);
		if (us > p->max_us) {
			p->max_us = us;
		}
		if (p->gap_ms != 0) {
			z_os_sleep(p->gap_ms);
		}
	}
	p->done = true;
}

static void test_printf(int *passed, int *failed)
{
	static struct printer hi = {.name = "printf_hi", .gap_ms = 1};
	static struct printer lo = {.name = "printf_lo", .gap_ms = 0};
	bool ok;

	zephyr_kernel_test_port_quiet();
	hi.done = false;
	hi.max_us = 0;
	lo.done = false;
	lo.max_us = 0;
	/* hi first: it prints a line and sleeps, then lo prints without a
	 * pause and hi comes back every tick.
	 */
	ok = start(printer_main, "printf_hi", stack_a, K_THREAD_STACK_SIZEOF(stack_a),
		   PRINT_HI_PRIO, &hi, &tid_a) &&
	     start(printer_main, "printf_lo", stack_b, K_THREAD_STACK_SIZEOF(stack_b),
		   PRINT_LO_PRIO, &lo, &tid_b);
	ok = join(&hi.done, &lo.done, NULL) && ok;

	printf("  the longest printf(): %lu us (hi), %lu us (lo)\n", (unsigned long)hi.max_us,
	       (unsigned long)lo.max_us);
	board_result("sdk_printf", ok && hi.max_us < 500000u && lo.max_us < 500000u, passed,
		     failed);
}

void board_sdk_test(int *passed, int *failed)
{
	printf("  before the kernel: sleep_ms(2) took %llu us, the mutex's owner was %lu\n",
	       (unsigned long long)pre_sleep_us, (unsigned long)pre_owner);
	board_result("sdk_pre_kernel",
		     pre_sleep_us >= 2000u && pre_sleep_us <= 2200u && pre_owner == 0u, passed,
		     failed);
	test_mutex(passed, failed);
	test_mutex_timeout(passed, failed);
	test_sleep(passed, failed);
	test_printf(passed, failed);
}
