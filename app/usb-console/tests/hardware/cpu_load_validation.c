/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/cpu_load.h>

static atomic_t timer_calls;
static void timer_work(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	atomic_inc(&timer_calls);
	/* 500 us every 2 ms creates work inside the timer ISR. */
	k_busy_wait(500);
}
K_TIMER_DEFINE(cpu_load_test_timer, timer_work, NULL);

static int cmd_cpu_load_validate(const struct shell *sh, size_t argc, char **argv)
{
	int idle, thread, isr;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	/* cpu_load_get(true) resets the global window; run only when IMU is idle. */
	(void)cpu_load_get(true);
	k_sleep(K_SECONDS(1));
	idle = cpu_load_get(true);
	k_busy_wait(500000);
	thread = cpu_load_get(true);
	atomic_set(&timer_calls, 0);
	k_timer_start(&cpu_load_test_timer, K_MSEC(2), K_MSEC(2));
	k_sleep(K_SECONDS(2));
	k_timer_stop(&cpu_load_test_timer);
	isr = cpu_load_get(true);
	shell_print(sh, "cpu_load_test idle=%d.%d%% thread=%d.%d%% isr=%d.%d%% timer_calls=%d",
		    idle / 10, idle % 10, thread / 10, thread % 10, isr / 10, isr % 10,
		    (int)atomic_get(&timer_calls));
	if (idle < 0 || thread < 800 || isr < 120 || isr <= idle + 80 ||
	    atomic_get(&timer_calls) < 700) {
		shell_error(sh, "CPU load validation failed");
		return -EIO;
	}
	shell_print(sh, "CPU load validation passed");
	return 0;
}
SHELL_CMD_REGISTER(cpu_load_validate, NULL, "Validate idle, thread and ISR CPU load",
		   cmd_cpu_load_validate);
