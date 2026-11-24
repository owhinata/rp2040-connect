/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the commands of the USB console (commands_os.h of
 * common/pico_sdk/app/usb-console/src, which describes each item;
 * commands.c registers the commands): the kernel's name and version for
 * "version", and "ps" on ThreadX's list of created threads and the
 * execution profile kit's run times (run_time.h).
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "tx_api.h"
#include "tx_thread.h"

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "app.h"
#include "commands_os.h"
#include "ps_interval.h"
#include "run_time.h"

const char *commands_os_kernel_name(void)
{
	return "ThreadX kernel";
}

/* "6.5.2.202603b", from tx_api.h (the tag v6.5.2.202603b_rel).  Two shells
 * may write it at once, with the same text.
 */
const char *commands_os_kernel_version(void)
{
	static char version[24];

	if (version[0] == '\0') {
		snprintf(version, sizeof(version), "%d.%d.%d.%d%c", THREADX_MAJOR_VERSION,
			 THREADX_MINOR_VERSION, THREADX_PATCH_VERSION, THREADX_BUILD_VERSION,
			 THREADX_HOTFIX_VERSION);
	}
	return version;
}

/* The threads (PS_TASKS_MAX - 1 at most) and the row of the scheduler's
 * wait.
 */
#define PS_TASKS_MAX 24

_Static_assert(PS_TASKS_MAX <= PS_INTERVAL_TASKS_MAX, "ps keeps every task");

struct ps_row {
	const TX_THREAD *thread;
	const char *name;
	UINT priority;
	char state;
	uint64_t run_us;
	unsigned int permille;
};

/* The row of the scheduler's wait: an id that is no thread's. */
static const char ps_idle_id;

/* The states as the FreeRTOS build shows them: X the caller, R ready, B
 * waiting with a timeout, S suspended (tx_thread_suspend()) or waiting
 * without one (FreeRTOS keeps a task that blocks for ever in its list of
 * suspended tasks).
 */
static char ps_state(const TX_THREAD *t)
{
	if (t == _tx_thread_current_ptr) {
		return 'X';
	}
	switch (t->tx_thread_state) {
	case TX_READY:
		return 'R';
	case TX_SUSPENDED:
		return 'S';
	default:
		return t->tx_thread_timer.tx_timer_internal_list_head != TX_NULL ? 'B' : 'S';
	}
}

/* The bytes at the bottom of the stack that still hold ThreadX's fill
 * (TX_STACK_FILL, 0xef bytes, written by tx_thread_create()): the least
 * free stack so far, as FreeRTOS's high-water mark.
 */
static size_t ps_stack_free(const TX_THREAD *t)
{
	const ULONG *start = t->tx_thread_stack_start;
	const ULONG *end = (const ULONG *)((const uint8_t *)t->tx_thread_stack_end + 1);
	const ULONG *p = start;

	while (p < end && *p == TX_STACK_FILL) {
		p++;
	}
	return (size_t)((uintptr_t)p - (uintptr_t)start);
}

/* The threads: state, priority (ThreadX's, 0 the most urgent: Zephyr's
 * priority + 16, src/priorities_os.h; raised by a mutex's priority
 * inheritance), the least free stack so far and the share of the CPU time
 * since the previous "ps" (since the start for the first), from any shell:
 * the run times of this list less those of the previous one
 * (common/shell/ps_interval.h; nothing runs between two lists).  Threads
 * that have ended (TX_COMPLETED, TX_TERMINATED: the start-up thread "main"
 * once it is done), which ThreadX keeps in its list, are left out, as the
 * other builds' lists do not show them.  The row "(idle)" is the time the
 * scheduler waited with no thread ready (no thread of ThreadX: the kit's
 * idle time), so the rows add up to about 100% as with FreeRTOS's idle
 * task; interrupt handlers count in the thread they interrupted, or in
 * "(idle)".  Then the CPU load over the same interval as the IMU service's
 * "cpu_active" measures it (idle windows, interrupt handlers count as
 * load: idle.c).
 *
 * The list is read with interrupts masked (no ThreadX service is called
 * there), the stacks after: the threads' stacks and control blocks are
 * static, and a thread started again on its stack keeps its place.
 */
static K_MUTEX_DEFINE(ps_lock);

int commands_os_ps(const struct shell *sh, size_t argc, char **argv)
{
	TX_INTERRUPT_SAVE_AREA
	static struct ps_row rows[PS_TASKS_MAX - 1];
	static struct ps_interval interval;
	const TX_THREAD *t;
	uint64_t total;
	uint64_t idle_us;
	uint64_t elapsed;
	uint64_t window_us;
	unsigned int idle_permille;
	unsigned int count = 0;
	bool full = false;
	ULONG created;
	int permille_window;

	(void)argc;
	(void)argv;

	k_mutex_lock(&ps_lock, K_FOREVER);
	TX_DISABLE
	created = _tx_thread_created_count;
	t = _tx_thread_created_ptr;
	for (ULONG i = 0; i < created; i++, t = t->tx_thread_created_next) {
		struct ps_row *row;

		if (t->tx_thread_state == TX_COMPLETED || t->tx_thread_state == TX_TERMINATED) {
			continue;
		}
		if (count == ARRAY_SIZE(rows)) {
			full = true;
			break;
		}
		row = &rows[count++];
		row->thread = t;
		row->name = t->tx_thread_name;
		row->priority = t->tx_thread_priority;
		row->state = ps_state(t);
		row->run_us = run_time_thread_us((TX_THREAD *)t);
	}
	idle_us = run_time_idle_us();
	total = run_time_total_us();
	TX_RESTORE

	if (full) {
		k_mutex_unlock(&ps_lock);
		shell_error(sh, "More than %d tasks", (int)ARRAY_SIZE(rows));
		return -ENOMEM;
	}

	permille_window = idle_ps_window(&window_us);
	elapsed = ps_interval_begin(&interval, total);
	for (unsigned int i = 0; i < count; i++) {
		uint64_t run =
			ps_interval_task(&interval, (uintptr_t)rows[i].thread, 0, rows[i].run_us);

		rows[i].permille = ps_interval_permille(run, elapsed);
	}
	idle_permille = ps_interval_permille(
		ps_interval_task(&interval, (uintptr_t)&ps_idle_id, 0, idle_us), elapsed);
	ps_interval_end(&interval);

	shell_print(sh, "%-20s %-5s %4s %10s %6s", "NAME", "STATE", "PRIO", "STACK FREE", "CPU");
	for (unsigned int i = 0; i < count; i++) {
		const struct ps_row *row = &rows[i];

		shell_print(sh, "%-20s %-5c %4lu %10lu %4u.%u%%", row->name, row->state,
			    (unsigned long)row->priority, (unsigned long)ps_stack_free(row->thread),
			    row->permille / 10, row->permille % 10);
	}
	/* The scheduler's wait (in PendSV, on the main stack): not a thread. */
	shell_print(sh, "%-20s %-5c %4c %10c %4u.%u%%", "(idle)", '-', '-', '-', idle_permille / 10,
		    idle_permille % 10);
	/* The interrupt stack (the main stack, irq_stack.c): not a thread. */
	shell_print(sh, "%-20s %-5c %4c %10lu %6c", "(interrupts)", '-', '-',
		    (unsigned long)(irq_stack_size() - MIN(irq_stack_used(), irq_stack_size())),
		    '-');
	shell_print(sh, "STATE: X running, R ready, B blocked, S suspended");
	shell_print(sh, "cpu_active: %d.%d%% in the %llu.%03llu s since the previous \"ps\"",
		    permille_window / 10, permille_window % 10, window_us / 1000000u,
		    window_us / 1000u % 1000u);
	k_mutex_unlock(&ps_lock);
	return 0;
}
