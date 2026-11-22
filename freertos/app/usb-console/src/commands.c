/* SPDX-License-Identifier: MIT */
/* The commands of the USB console of this build, written as Zephyr shell
 * commands (common/compat/shell_engine/nano_shell.h), and the table of all the
 * commands of the shell (nano_shell_cmds[]).
 */
#include <errno.h>
#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"
#include "pico/version.h"
#include "hardware/watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/clock.h>
#include "app.h"
#include "fault.h"
#include "nano_shell.h"
#include "os_clock.h"
#include "ps_interval.h"
#ifdef NANO_SELFTEST
#include "i2c_dw_selftest.h"
#include "lsm6dso_selftest.h"
#include "zephyr_selftest.h"
#endif

#define PS_TASKS_MAX     24
/* Time for the USB code to send a command's last message before a restart. */
#define RESTART_DELAY_MS 100

_Static_assert(PS_TASKS_MAX <= PS_INTERVAL_TASKS_MAX, "ps keeps every task");

static int cmd_help(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;

	nano_shell_help();
	return 0;
}

SHELL_CMD_ARG_REGISTER(help, NULL, "Prints the help message.", cmd_help, 1, 0);

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_print(sh, "FreeRTOS kernel %s, Pico SDK %s, GCC %s", tskKERNEL_VERSION_NUMBER,
		    PICO_SDK_VERSION_STRING, __VERSION__);
	return 0;
}

SHELL_CMD_REGISTER(version, NULL, "Show the versions of the kernel, SDK and compiler", cmd_version);

static int cmd_id(const struct shell *sh, size_t argc, char **argv)
{
	char id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

	(void)argc;
	(void)argv;

	pico_get_unique_board_id_string(id, sizeof(id));
	shell_print(sh, "%s", id);
	return 0;
}

SHELL_CMD_REGISTER(id, NULL, "Show the board's unique ID (of the flash)", cmd_id);

static int cmd_uptime(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t us = time_us_64();

	(void)argc;
	(void)argv;

	shell_print(sh, "%llu.%06llu s (TIMER), %lu ticks of 1/%d s", us / 1000000u, us % 1000000u,
		    (unsigned long)os_tick_count(), configTICK_RATE_HZ);
	return 0;
}

SHELL_CMD_REGISTER(uptime, NULL, "Show the time since the start (TIMER) and the ticks", cmd_uptime);

/* The wall clock (UTC, set by the time synchronization), in the format of the
 * NuttX build's "date".  It counts from 1970 until the time is known.
 */
static int cmd_date(const struct shell *sh, size_t argc, char **argv)
{
	struct timespec ts;
	struct tm tm;
	char text[32];

	(void)argc;
	(void)argv;

	if (sys_clock_gettime(SYS_CLOCK_REALTIME, &ts) != 0 || gmtime_r(&ts.tv_sec, &tm) == NULL ||
	    strftime(text, sizeof(text), "%a, %b %d %H:%M:%S %Y", &tm) == 0) {
		shell_error(sh, "Could not read the time");
		return -EIO;
	}
	shell_print(sh, "%s", text);
	return 0;
}

SHELL_CMD_ARG_REGISTER(date, NULL, "Show the date and time (UTC)", cmd_date, 1, 0);

/* Zephyr's "kernel sleep <ms>": k_msleep(ms), with Zephyr's rounding
 * (os_sleep_us()), and its messages (subsys/shell/modules/kernel_service).
 */
static int cmd_sleep(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	unsigned long ms;

	(void)argc;
	errno = 0;
	ms = strtoul(argv[1], &end, 10);
	if (errno != 0 || end == argv[1] || *end != '\0') {
		shell_error(sh, "Unable to parse input (err %d)", -EINVAL);
		return -EINVAL;
	}
	os_sleep_us((uint64_t)ms * 1000u);
	return 0;
}

SHELL_CMD_ARG_REGISTER(sleep, NULL, "Sleep for <ms> milliseconds (Zephyr's kernel sleep)",
		       cmd_sleep, 2, 0);

/* Zephyr's "devmem" (subsys/shell/modules/devmem_service.c): the same
 * arguments (address and value in hexadecimal, width in bits) and the same
 * output.  Reads or writes any address: one that does not exist ends in a
 * HardFault ("fault" after the restart).
 */
static int cmd_devmem(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t addr;
	unsigned long width = 32;
	uint32_t value;

	addr = (uintptr_t)strtoul(argv[1], NULL, 16);
	if (argc >= 3) {
		width = strtoul(argv[2], NULL, 10);
	}
	shell_print(sh, "Using data width %lu", width);
	if (width != 8 && width != 16 && width != 32) {
		shell_error(sh, "Incorrect data width");
		return -EINVAL;
	}
	if (argc <= 3) {
		if (width == 8) {
			value = *(volatile uint8_t *)addr;
		} else if (width == 16) {
			value = *(volatile uint16_t *)addr;
		} else {
			value = *(volatile uint32_t *)addr;
		}
		shell_print(sh, "Read value 0x%lx", (unsigned long)value);
		return 0;
	}
	value = (uint32_t)strtoul(argv[3], NULL, 16);
	shell_print(sh, "Writing value 0x%lx", (unsigned long)value);
	if (width == 8) {
		*(volatile uint8_t *)addr = (uint8_t)value;
	} else if (width == 16) {
		*(volatile uint16_t *)addr = (uint16_t)value;
	} else {
		*(volatile uint32_t *)addr = value;
	}
	return 0;
}

SHELL_CMD_ARG_REGISTER(devmem, NULL,
		       "Read or write memory: devmem <address> [<width> [<value>]]\n"
		       "(address and value in hexadecimal, width 8, 16 or 32 bits)",
		       cmd_devmem, 2, 2);

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

void fault_print(const struct fault_record *r)
{
	char line[128];

	for (unsigned int i = 0; fault_record_line(r, i, line, sizeof(line)) > 0; i++) {
		printf("%s\n", line);
	}
}

/* The fatal error that ended the run before this start (fault.c). */
static int cmd_fault(const struct shell *sh, size_t argc, char **argv)
{
	const struct fault_record *r = fault_last();

	(void)argc;
	(void)argv;

	if (r == NULL) {
		shell_print(sh, "No fatal error before this start");
		return 0;
	}
	fault_print(r);
	return 0;
}

SHELL_CMD_REGISTER(fault, NULL, "Show the fatal error that ended the last run", cmd_fault);

/* Zephyr's "kernel reboot": restarts at once, whatever the arguments. */
static int cmd_reboot(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_print(sh, "Rebooting");
	watchdog_reboot(0, 0, RESTART_DELAY_MS);
	vTaskDelay(portMAX_DELAY);
	return 0;
}

SHELL_CMD_REGISTER(reboot, NULL, "Restart the board", cmd_reboot);

/* Restarts into the boot ROM's USB loader (the RPI-RP2 drive), as holding the
 * board's REC pad low at reset does.
 */
static int cmd_bootsel(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_print(sh, "Entering BOOTSEL (the RPI-RP2 drive)");
	vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
	reset_usb_boot(0, 0);
}

SHELL_CMD_REGISTER(bootsel, NULL, "Restart into the USB bootloader (BOOTSEL)", cmd_bootsel);

#ifdef NANO_SELFTEST
/* The Zephyr compatibility layer's self-test (a build option, NANO_SELFTEST;
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

/* The commands of other sources: common/shell/dmesg_shell.c,
 * common/shell/i2c_dma_shell.c, common/imu/imu_stream.c,
 * common/imu/kv_store.c (with "kv wipe" of common/compat/kv_store_wipe.c),
 * common/shell/nina_shell.c ("nina"),
 * common/compat/zephyr_wifi_shell.c
 * ("wifi", with "wifi cred" of the copy of Zephyr's
 * wifi_credentials_shell.c), freertos/drivers/wifi/nano_ifconfig.c,
 * common/imu/time_sync.c ("ntp"), common/compat/zephyr_zperf_shell.c and
 * common/drivers/sensor_shell/sensor_shell.c ("sensor").
 */
NANO_SHELL_CMD_DECLARE(dmesg);
NANO_SHELL_CMD_DECLARE(i2c_dma);
NANO_SHELL_CMD_DECLARE(ifconfig);
NANO_SHELL_CMD_DECLARE(imu);
NANO_SHELL_CMD_DECLARE(kv);
NANO_SHELL_CMD_DECLARE(nina);
NANO_SHELL_CMD_DECLARE(ntp);
NANO_SHELL_CMD_DECLARE(sensor);
NANO_SHELL_CMD_DECLARE(wifi);
NANO_SHELL_CMD_DECLARE(zperf);

/* All the commands, sorted by name (Zephyr's "help" lists them so). */
/* One command per line. */
/* clang-format off */
const struct nano_shell_cmd nano_shell_cmds[] = {
	NANO_SHELL_CMD(bootsel),
	NANO_SHELL_CMD(date),
	NANO_SHELL_CMD(devmem),
	NANO_SHELL_CMD(dmesg),
	NANO_SHELL_CMD(fault),
	NANO_SHELL_CMD(free),
	NANO_SHELL_CMD(help),
	NANO_SHELL_CMD(i2c_dma),
	NANO_SHELL_CMD(id),
	NANO_SHELL_CMD(ifconfig),
	NANO_SHELL_CMD(imu),
	NANO_SHELL_CMD(kv),
	NANO_SHELL_CMD(nina),
	NANO_SHELL_CMD(ntp),
	NANO_SHELL_CMD(ps),
	NANO_SHELL_CMD(reboot),
#ifdef NANO_SELFTEST
	NANO_SHELL_CMD(selftest),
#endif
	NANO_SHELL_CMD(sensor),
	NANO_SHELL_CMD(sleep),
	NANO_SHELL_CMD(uptime),
	NANO_SHELL_CMD(version),
	NANO_SHELL_CMD(wifi),
	NANO_SHELL_CMD(zperf),
};
/* clang-format on */

const size_t nano_shell_cmd_count = sizeof(nano_shell_cmds) / sizeof(nano_shell_cmds[0]);
