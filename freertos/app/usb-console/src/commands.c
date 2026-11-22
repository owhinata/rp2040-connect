/* SPDX-License-Identifier: MIT */
/* The commands of this build that only FreeRTOS and the Pico SDK can
 * answer ("version", "free", "ps" and, as a build option, "selftest"),
 * written as Zephyr shell commands (common/compat/shell_engine/
 * shell_engine.h), and the table of all the commands of the shell
 * (shell_engine_cmds[]).
 */
#include <errno.h>
#include <malloc.h>
#include <stdint.h>
#include "pico/version.h"
#include "FreeRTOS.h"
#include "task.h"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include "app.h"
#include "shell_engine.h"
#include "ps_interval.h"
#ifdef SELFTEST
#include "i2c_dw_selftest.h"
#include "lsm6dso_selftest.h"
#include "zephyr_selftest.h"
#endif

#define PS_TASKS_MAX 24

_Static_assert(PS_TASKS_MAX <= PS_INTERVAL_TASKS_MAX, "ps keeps every task");

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_print(sh, "FreeRTOS kernel %s, Pico SDK %s, GCC %s", tskKERNEL_VERSION_NUMBER,
		    PICO_SDK_VERSION_STRING, __VERSION__);
	return 0;
}

SHELL_CMD_REGISTER(version, NULL, "Show the versions of the kernel, SDK and compiler", cmd_version);

/* The FreeRTOS heap (heap_4: the kernel objects that are not static) and
 * newlib's (malloc(): the shell's state of a command line, for example).
 */
static int cmd_free(const struct shell *sh, size_t argc, char **argv)
{
	struct mallinfo m = mallinfo();

	(void)argc;
	(void)argv;

	shell_print(sh, "heap: %u bytes, free %u, least free %u",
		    (unsigned int)configTOTAL_HEAP_SIZE, (unsigned int)xPortGetFreeHeapSize(),
		    (unsigned int)xPortGetMinimumEverFreeHeapSize());
	shell_print(sh, "malloc: %u bytes from the system, in use %u, free %u",
		    (unsigned int)m.arena, (unsigned int)m.uordblks, (unsigned int)m.fordblks);
	return 0;
}

SHELL_CMD_REGISTER(free, NULL, "Show the use of the FreeRTOS heap and of malloc()", cmd_free);

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

static int cmd_ps(const struct shell *sh, size_t argc, char **argv)
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

SHELL_CMD_REGISTER(ps, NULL, "List the tasks and show the CPU load", cmd_ps);

#ifdef SELFTEST
/* The Zephyr compatibility layer's self-test (a build option, SELFTEST;
 * common/compat/selftest/zephyr_selftest.h), then the check of I2C0 on the
 * board's LSM6DSOX (freertos/drivers/i2c_dw/i2c_dw_selftest.h) and that of
 * the LSM6DSOX itself with its FIFO stream
 * (freertos/drivers/lsm6dso/lsm6dso_selftest.h), which leaves the sensor at
 * rest for the I2C0 check of the next run.  Runs in this task, at the
 * shell's priority, as the test expects.
 */
static int cmd_selftest(const struct shell *sh, size_t argc, char **argv)
{
	int failed;

	(void)sh;
	(void)argc;
	(void)argv;

	failed = zephyr_selftest_run();
	failed += i2c_dw_selftest_run();
	failed += lsm6dso_selftest_run();
	return failed == 0 ? 0 : -EIO;
}

SHELL_CMD_REGISTER(selftest, NULL, "Test the Zephyr compatibility layer (build option)",
		   cmd_selftest);
#endif

/* The commands of other sources: "help" of the shell engine
 * (common/compat/shell_engine/shell_engine.c), the generic commands of
 * common/shell/kernel_shell.c ("bootsel", "date", "devmem", "fault", "id",
 * "reboot", "sleep", "uptime"), common/shell/dmesg_shell.c,
 * common/shell/i2c_dma_shell.c, common/shell/ifconfig_shell.c,
 * common/imu/imu_stream.c, common/imu/kv_store.c (with "kv wipe" of
 * common/compat/kv_store_wipe.c), common/shell/nina_shell.c ("nina"),
 * common/compat/zephyr_wifi_shell.c ("wifi", with "wifi cred" of the copy
 * of Zephyr's wifi_credentials_shell.c), common/imu/time_sync.c ("ntp"),
 * common/compat/zephyr_zperf_shell.c and
 * common/drivers/sensor_shell/sensor_shell.c ("sensor").
 */
SHELL_ENGINE_CMD_DECLARE(bootsel);
SHELL_ENGINE_CMD_DECLARE(date);
SHELL_ENGINE_CMD_DECLARE(devmem);
SHELL_ENGINE_CMD_DECLARE(dmesg);
SHELL_ENGINE_CMD_DECLARE(fault);
SHELL_ENGINE_CMD_DECLARE(help);
SHELL_ENGINE_CMD_DECLARE(i2c_dma);
SHELL_ENGINE_CMD_DECLARE(id);
SHELL_ENGINE_CMD_DECLARE(ifconfig);
SHELL_ENGINE_CMD_DECLARE(imu);
SHELL_ENGINE_CMD_DECLARE(kv);
SHELL_ENGINE_CMD_DECLARE(nina);
SHELL_ENGINE_CMD_DECLARE(ntp);
SHELL_ENGINE_CMD_DECLARE(reboot);
SHELL_ENGINE_CMD_DECLARE(sensor);
SHELL_ENGINE_CMD_DECLARE(sleep);
SHELL_ENGINE_CMD_DECLARE(uptime);
SHELL_ENGINE_CMD_DECLARE(wifi);
SHELL_ENGINE_CMD_DECLARE(zperf);

/* All the commands, sorted by name (Zephyr's "help" lists them so). */
/* One command per line. */
/* clang-format off */
const struct shell_engine_cmd shell_engine_cmds[] = {
	SHELL_ENGINE_CMD(bootsel),
	SHELL_ENGINE_CMD(date),
	SHELL_ENGINE_CMD(devmem),
	SHELL_ENGINE_CMD(dmesg),
	SHELL_ENGINE_CMD(fault),
	SHELL_ENGINE_CMD(free),
	SHELL_ENGINE_CMD(help),
	SHELL_ENGINE_CMD(i2c_dma),
	SHELL_ENGINE_CMD(id),
	SHELL_ENGINE_CMD(ifconfig),
	SHELL_ENGINE_CMD(imu),
	SHELL_ENGINE_CMD(kv),
	SHELL_ENGINE_CMD(nina),
	SHELL_ENGINE_CMD(ntp),
	SHELL_ENGINE_CMD(ps),
	SHELL_ENGINE_CMD(reboot),
#ifdef SELFTEST
	SHELL_ENGINE_CMD(selftest),
#endif
	SHELL_ENGINE_CMD(sensor),
	SHELL_ENGINE_CMD(sleep),
	SHELL_ENGINE_CMD(uptime),
	SHELL_ENGINE_CMD(version),
	SHELL_ENGINE_CMD(wifi),
	SHELL_ENGINE_CMD(zperf),
};
/* clang-format on */

const size_t shell_engine_cmd_count = sizeof(shell_engine_cmds) / sizeof(shell_engine_cmds[0]);
