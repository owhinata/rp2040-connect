/* SPDX-License-Identifier: MIT */
/* The commands of the USB console of this build, written as Zephyr shell
 * commands (freertos/drivers/shell/nano_shell.h), and the table of all the
 * commands of the shell (nano_shell_cmds[]).
 */
#include <errno.h>
#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"
#include "pico/version.h"
#include "hardware/watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include <zephyr/shell/shell.h>
#include "app.h"
#include "fault.h"
#include "nano_shell.h"
#include "os_clock.h"
#ifdef NANO_SELFTEST
#include "zephyr_selftest.h"
#endif

#define PS_TASKS_MAX     16
/* Time for the USB code to send a command's last message before a restart. */
#define RESTART_DELAY_MS 100

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
 * the CPU time since the start.
 */
static int cmd_ps(const struct shell *sh, size_t argc, char **argv)
{
	static TaskStatus_t tasks[PS_TASKS_MAX];
	/* Indexed by eTaskState: running, ready, blocked, suspended, deleted, invalid. */
	static const char states[] = "XRBSD?";
	configRUN_TIME_COUNTER_TYPE total;
	UBaseType_t count;
	uint64_t window_us;
	int permille_window;

	(void)argc;
	(void)argv;

	if (uxTaskGetNumberOfTasks() > PS_TASKS_MAX) {
		shell_error(sh, "More than %d tasks", PS_TASKS_MAX);
		return -ENOMEM;
	}
	count = uxTaskGetSystemState(tasks, PS_TASKS_MAX, &total);
	shell_print(sh, "%-20s %-5s %4s %10s %6s", "NAME", "STATE", "PRIO", "STACK FREE", "CPU");
	for (UBaseType_t i = 0; i < count; i++) {
		const TaskStatus_t *t = &tasks[i];
		unsigned int permille = 0;

		if (total > 0) {
			permille = (unsigned int)(t->ulRunTimeCounter * 1000u / total);
		}
		shell_print(sh, "%-20s %-5c %4lu %10lu %4u.%u%%", t->pcTaskName,
			    states[t->eCurrentState <= eInvalid ? t->eCurrentState : eInvalid],
			    (unsigned long)t->uxCurrentPriority,
			    (unsigned long)(t->usStackHighWaterMark * sizeof(StackType_t)),
			    permille / 10, permille % 10);
	}
	shell_print(sh, "STATE: X running, R ready, B blocked, S suspended, D deleted");

	/* The load as the IMU service's "cpu_active" measures it: idle windows,
	 * interrupt handlers count as load (idle.c).
	 */
	permille_window = idle_ps_window(&window_us);
	shell_print(sh, "cpu_active: %d.%d%% in the %llu.%03llu s since the previous \"ps\"",
		    permille_window / 10, permille_window % 10, window_us / 1000000u,
		    window_us / 1000u % 1000u);
	return 0;
}

SHELL_CMD_REGISTER(ps, NULL, "List the tasks and show the CPU load", cmd_ps);

/* The use of the interrupt stack since the start, in the format of the
 * "IRQ 00" line of the Zephyr build's "kernel thread stacks" and the NuttX
 * build's "irqstack".  Arguments are ignored.
 */
static int cmd_irqstack(const struct shell *sh, size_t argc, char **argv)
{
	size_t size = irq_stack_size();
	size_t used = irq_stack_used();
	size_t unused = used < size ? size - used : 0;

	(void)argc;
	(void)argv;

	shell_print(sh, "%p %-32s (real size %4zu):\tunused %4zu\tusage %4zu / %4zu (%2zu %%)",
		    (void *)irq_stack_base(), "IRQ 00", size, unused, used, size,
		    used * 100 / size);
	return 0;
}

SHELL_CMD_REGISTER(irqstack, NULL, "Show the use of the interrupt stack", cmd_irqstack);

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
 * freertos/drivers/compat/zephyr_selftest.h).  Runs in this task, at the
 * shell's priority, as the test expects.
 */
static int cmd_selftest(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;

	return zephyr_selftest_run() == 0 ? 0 : -EIO;
}

SHELL_CMD_REGISTER(selftest, NULL, "Test the Zephyr compatibility layer (build option)",
		   cmd_selftest);
#endif

/* The commands of other sources: freertos/drivers/log/log_ram.c,
 * common/imu/kv_store.c (with "kv wipe" of common/compat/kv_store_wipe.c)
 * and common/drivers/nina/nina_shell.c.
 */
NANO_SHELL_CMD_DECLARE(dmesg);
NANO_SHELL_CMD_DECLARE(kv);
NANO_SHELL_CMD_DECLARE(nina);

/* All the commands, sorted by name (Zephyr's "help" lists them so). */
/* One command per line. */
/* clang-format off */
const struct nano_shell_cmd nano_shell_cmds[] = {
	NANO_SHELL_CMD(bootsel),
	NANO_SHELL_CMD(devmem),
	NANO_SHELL_CMD(dmesg),
	NANO_SHELL_CMD(fault),
	NANO_SHELL_CMD(free),
	NANO_SHELL_CMD(help),
	NANO_SHELL_CMD(id),
	NANO_SHELL_CMD(irqstack),
	NANO_SHELL_CMD(kv),
	NANO_SHELL_CMD(nina),
	NANO_SHELL_CMD(ps),
	NANO_SHELL_CMD(reboot),
#ifdef NANO_SELFTEST
	NANO_SHELL_CMD(selftest),
#endif
	NANO_SHELL_CMD(sleep),
	NANO_SHELL_CMD(uptime),
	NANO_SHELL_CMD(version),
};
/* clang-format on */

const size_t nano_shell_cmd_count = sizeof(nano_shell_cmds) / sizeof(nano_shell_cmds[0]);
