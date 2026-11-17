/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS part of the Zephyr compatibility layer: the
 * board's self-test (freertos/drivers/compat/zephyr_selftest.c) on the
 * FreeRTOS POSIX port, with the board's tick hook
 * (freertos/app/usb-console/src/tick.c, built here as app_tick_hook()).  The
 * self-test runs twice (the second run reuses the objects of the first) in
 * a task at the shell's priority; the exit status is 0 if no test failed.
 * Built and run by freertos/script/host_tests.sh.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include "priorities.h"
#include "zephyr_compat.h"
#include "zephyr_selftest.h"

#define RUNNER_STACK 4096
#define RUNS         2

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook() (time slices and zephyr_compat_tick()),
 * counted as an interrupt handler.
 */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

int zephyr_selftest_port_alarm(uint32_t us, void (*fn)(void *arg), void *arg)
{
	(void)us;
	(void)fn;
	(void)arg;
	return -ENOTSUP;
}

/* Nothing here disturbs the measurements (no interrupts but the tick). */
void zephyr_selftest_port_quiet(void)
{
	fflush(stdout);
}

bool zephyr_selftest_port_realtime(void)
{
	return false;
}

static void runner(void *arg)
{
	int failed = 0;

	(void)arg;
	for (int i = 0; i < RUNS; i++) {
		failed += zephyr_selftest_run();
	}
	fflush(stdout);
	_exit(failed == 0 ? 0 : 1);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 16];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_SHELL,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
