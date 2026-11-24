/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the commands of the USB console (commands_os.h of
 * common/pico_sdk/app/usb-console/src, which describes each item;
 * commands.c registers the commands): the kernel's name and version for
 * "version", and "ps" on FreeRTOS's task list and run time counters, with
 * the counters' clock.
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include "app.h"
#include "commands_os.h"
#include "ps_interval.h"

const char *commands_os_kernel_name(void)
{
	return "FreeRTOS kernel";
}

const char *commands_os_kernel_version(void)
{
	return tskKERNEL_VERSION_NUMBER;
}

#define PS_TASKS_MAX 24

/* The run time of the tasks, in microseconds of the TIMER: the counter
 * FreeRTOSConfig.h names (portGET_RUN_TIME_COUNTER_VALUE()), which the
 * kernel reads at each switch.  The other hooks are in fault_os.c
 * (fatal errors), idle_os.h and tick.c.
 */
uint64_t app_run_time_us(void)
{
	return time_us_64();
}

_Static_assert(PS_TASKS_MAX <= PS_INTERVAL_TASKS_MAX, "ps keeps every task");

/* The tasks: state, priority, the least free stack so far and the share of
 * the CPU time since the previous "ps" (since the start for the first), from
 * any shell: the run time counters (TIMER, 1 us) of this list less those of
 * the previous one (common/shell/ps_interval.h; nothing runs between two
 * lists).  The share is of all the time of the interval, the idle task's
 * included, so the rows add up to about 100%; interrupt handlers count in
 * the task they interrupted.  Then the CPU load over the same interval as
 * the IMU service's "cpu_active" measures it (idle windows, interrupt
 * handlers count as load: idle.c).
 */
static K_MUTEX_DEFINE(ps_lock);

int commands_os_ps(const struct shell *sh, size_t argc, char **argv)
{
	static TaskStatus_t tasks[PS_TASKS_MAX];
	static struct ps_interval interval;
	static unsigned int permille[PS_TASKS_MAX];
	/* Indexed by eTaskState: running, ready, blocked, suspended, deleted, invalid. */
	static const char states[] = "XRBSD?";
	configRUN_TIME_COUNTER_TYPE total;
	UBaseType_t count;
	uint64_t elapsed;
	uint64_t window_us;
	int permille_window;

	(void)argc;
	(void)argv;

	k_mutex_lock(&ps_lock, K_FOREVER);
	if (uxTaskGetNumberOfTasks() > PS_TASKS_MAX) {
		k_mutex_unlock(&ps_lock);
		shell_error(sh, "More than %d tasks", PS_TASKS_MAX);
		return -ENOMEM;
	}
	count = uxTaskGetSystemState(tasks, PS_TASKS_MAX, &total);
	permille_window = idle_ps_window(&window_us);
	elapsed = ps_interval_begin(&interval, total);
	for (UBaseType_t i = 0; i < count; i++) {
		const TaskStatus_t *t = &tasks[i];
		uint64_t run = ps_interval_task(&interval, (uintptr_t)t->xHandle, t->xTaskNumber,
						t->ulRunTimeCounter);

		permille[i] = ps_interval_permille(run, elapsed);
	}
	ps_interval_end(&interval);

	shell_print(sh, "%-20s %-5s %4s %10s %6s", "NAME", "STATE", "PRIO", "STACK FREE", "CPU");
	for (UBaseType_t i = 0; i < count; i++) {
		const TaskStatus_t *t = &tasks[i];

		shell_print(sh, "%-20s %-5c %4lu %10lu %4u.%u%%", t->pcTaskName,
			    states[t->eCurrentState <= eInvalid ? t->eCurrentState : eInvalid],
			    (unsigned long)t->uxCurrentPriority,
			    (unsigned long)(t->usStackHighWaterMark * sizeof(StackType_t)),
			    permille[i] / 10, permille[i] % 10);
	}
	/* The interrupt stack (the main stack, irq_stack.c): not a task. */
	shell_print(sh, "%-20s %-5c %4c %10lu %6c", "(interrupts)", '-', '-',
		    (unsigned long)(irq_stack_size() - MIN(irq_stack_used(), irq_stack_size())),
		    '-');
	shell_print(sh, "STATE: X running, R ready, B blocked, S suspended, D deleted");
	shell_print(sh, "cpu_active: %d.%d%% in the %llu.%03llu s since the previous \"ps\"",
		    permille_window / 10, permille_window % 10, window_us / 1000000u,
		    window_us / 1000u % 1000u);
	k_mutex_unlock(&ps_lock);
	return 0;
}
