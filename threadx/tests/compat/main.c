/* SPDX-License-Identifier: MIT */
/* The host test of the ThreadX part of the Zephyr compatibility layer: the
 * layer's kernel test (common/tests/zephyr_kernel/zephyr_kernel_test.c, on
 * its kernel_generic/zephyr_kernel_test_os.h) on ThreadX's Linux port.  The
 * layer's tick, zephyr_compat_tick(), is a ThreadX timer of one tick here,
 * which with TX_TIMER_PROCESS_IN_ISR (tx_user.h) runs in the port's tick
 * interrupt, once per tick after ThreadX has counted it (the board calls it
 * from its tick interrupt instead).  The test runs twice (the second run
 * reuses the objects of the first) in a thread of the layer at the shell's
 * priority; the exit status is 0 if no test failed.  Built and run by
 * threadx/script/host_tests.sh.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "tx_api.h"

#include <zephyr/kernel.h>

#include "priorities.h"
#include "zephyr_compat.h"
#include "zephyr_compat_os.h"
#include "zephyr_kernel_test.h"

#define RUNNER_STACK 4096
#define RUNS         2

static TX_TIMER tick_timer;
static K_THREAD_STACK_DEFINE(runner_stack, RUNNER_STACK);
static k_tid_t runner_tid;

static void tick_expired(ULONG input)
{
	(void)input;
	zephyr_compat_tick();
}

int zephyr_kernel_test_port_alarm(uint32_t us, void (*fn)(void *arg), void *arg)
{
	(void)us;
	(void)fn;
	(void)arg;
	return -ENOTSUP;
}

/* Nothing here disturbs the measurements (no interrupts but the tick). */
void zephyr_kernel_test_port_quiet(void)
{
	fflush(stdout);
}

/* The port's tick is a Linux thread's timed wait between ticks. */
bool zephyr_kernel_test_port_realtime(void)
{
	return false;
}

/* The port's interrupts cannot be held back for a time and caught up. */
int zephyr_kernel_test_port_mask_irqs(uint32_t us)
{
	(void)us;
	return -ENOTSUP;
}

static void runner(void *arg)
{
	int failed = 0;

	(void)arg;
	for (int i = 0; i < RUNS; i++) {
		failed += zephyr_kernel_test_run();
	}
	fflush(stdout);
	_exit(failed == 0 ? 0 : 1);
}

void tx_application_define(void *first_unused_memory)
{
	(void)first_unused_memory;
	if (zephyr_compat_initialize() != 0) {
		_exit(2);
	}
	if (tx_timer_create(&tick_timer, (CHAR *)"zephyr_tick", tick_expired, 0, 1, 1,
			    TX_AUTO_ACTIVATE) != TX_SUCCESS) {
		_exit(2);
	}
	if (z_os_thread_start(runner, "runner", runner_stack, K_THREAD_STACK_SIZEOF(runner_stack),
			      APP_PRIO_SHELL, NULL, &runner_tid) != 0) {
		_exit(2);
	}
}

int main(void)
{
	/* printf() must not allocate in a thread (the port suspends threads
	 * with a signal wherever they are, the C library's locks too).
	 */
	static char out[1 << 16];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	tx_kernel_enter();
	return 3;
}
