/* SPDX-License-Identifier: MIT */
/* The commands of the USB console. */
#include <errno.h>
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
#include "app.h"
#include "fault.h"
#include "os_clock.h"
#ifdef NANO_SELFTEST
#include "zephyr_selftest.h"
#endif

#define PS_TASKS_MAX     16
/* Time for the USB code to send a command's last message before a restart. */
#define RESTART_DELAY_MS 100

static int cmd_help(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	cli_help(&console_cli);
	return 0;
}

static int cmd_version(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	printf("FreeRTOS kernel %s, Pico SDK %s, GCC %s\n", tskKERNEL_VERSION_NUMBER,
	       PICO_SDK_VERSION_STRING, __VERSION__);
	return 0;
}

static int cmd_id(int argc, char **argv)
{
	char id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

	(void)argc;
	(void)argv;

	pico_get_unique_board_id_string(id, sizeof(id));
	printf("%s\n", id);
	return 0;
}

static int cmd_uptime(int argc, char **argv)
{
	uint64_t us = time_us_64();

	(void)argc;
	(void)argv;

	printf("%llu.%06llu s (TIMER), %lu ticks of 1/%d s\n", us / 1000000u, us % 1000000u,
	       (unsigned long)os_tick_count(), configTICK_RATE_HZ);
	return 0;
}

/* Zephyr's "kernel sleep <ms>": k_msleep(ms), with Zephyr's rounding
 * (os_sleep_us()).
 */
static int cmd_sleep(int argc, char **argv)
{
	char *end;
	unsigned long ms;

	if (argc != 2) {
		printf("Usage: sleep <ms>\n");
		return -EINVAL;
	}
	errno = 0;
	ms = strtoul(argv[1], &end, 10);
	if (errno != 0 || end == argv[1] || *end != '\0') {
		printf("Unable to parse input (err %d)\n", -EINVAL);
		return -EINVAL;
	}
	os_sleep_us((uint64_t)ms * 1000u);
	return 0;
}

/* Zephyr's "devmem" (subsys/shell/modules/devmem_service.c): the same
 * arguments (address and value in hexadecimal, width in bits) and the same
 * output.  Reads or writes any address: one that does not exist ends in a
 * HardFault ("fault" after the restart).
 */
static int cmd_devmem(int argc, char **argv)
{
	uintptr_t addr;
	unsigned long width = 32;
	uint32_t value;

	if (argc < 2 || argc > 4) {
		printf("Usage: devmem <address> [<width> [<value>]]\n");
		return -EINVAL;
	}
	addr = (uintptr_t)strtoul(argv[1], NULL, 16);
	if (argc >= 3) {
		width = strtoul(argv[2], NULL, 10);
	}
	printf("Using data width %lu\n", width);
	if (width != 8 && width != 16 && width != 32) {
		printf("Incorrect data width\n");
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
		printf("Read value 0x%lx\n", (unsigned long)value);
		return 0;
	}
	value = (uint32_t)strtoul(argv[3], NULL, 16);
	printf("Writing value 0x%lx\n", (unsigned long)value);
	if (width == 8) {
		*(volatile uint8_t *)addr = (uint8_t)value;
	} else if (width == 16) {
		*(volatile uint16_t *)addr = (uint16_t)value;
	} else {
		*(volatile uint32_t *)addr = value;
	}
	return 0;
}

static int cmd_free(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	printf("heap: %u bytes, free %u, least free %u\n", (unsigned int)configTOTAL_HEAP_SIZE,
	       (unsigned int)xPortGetFreeHeapSize(),
	       (unsigned int)xPortGetMinimumEverFreeHeapSize());
	return 0;
}

/* The tasks: state, priority, the least free stack so far and the share of
 * the CPU time since the start.
 */
static int cmd_ps(int argc, char **argv)
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
		return -ENOMEM;
	}
	count = uxTaskGetSystemState(tasks, PS_TASKS_MAX, &total);
	printf("%-20s %-5s %4s %10s %6s\n", "NAME", "STATE", "PRIO", "STACK FREE", "CPU");
	for (UBaseType_t i = 0; i < count; i++) {
		const TaskStatus_t *t = &tasks[i];
		unsigned int permille = 0;

		if (total > 0) {
			permille = (unsigned int)(t->ulRunTimeCounter * 1000u / total);
		}
		printf("%-20s %-5c %4lu %10lu %4u.%u%%\n", t->pcTaskName,
		       states[t->eCurrentState <= eInvalid ? t->eCurrentState : eInvalid],
		       (unsigned long)t->uxCurrentPriority,
		       (unsigned long)(t->usStackHighWaterMark * sizeof(StackType_t)),
		       permille / 10, permille % 10);
	}
	printf("STATE: X running, R ready, B blocked, S suspended, D deleted\n");

	/* The load as the IMU service's "cpu_active" measures it: idle windows,
	 * interrupt handlers count as load (idle.c).
	 */
	permille_window = idle_ps_window(&window_us);
	printf("cpu_active: %d.%d%% in the %llu.%03llu s since the previous \"ps\"\n",
	       permille_window / 10, permille_window % 10, window_us / 1000000u,
	       window_us / 1000u % 1000u);
	return 0;
}

/* The use of the interrupt stack since the start, in the format of the
 * "IRQ 00" line of the Zephyr build's "kernel thread stacks" and the NuttX
 * build's "irqstack".  Arguments are ignored.
 */
static int cmd_irqstack(int argc, char **argv)
{
	size_t size = irq_stack_size();
	size_t used = irq_stack_used();
	size_t unused = used < size ? size - used : 0;

	(void)argc;
	(void)argv;

	printf("%p %-32s (real size %4zu):\tunused %4zu\tusage %4zu / %4zu (%2zu %%)\n",
	       (void *)irq_stack_base(), "IRQ 00", size, unused, used, size, used * 100 / size);
	return 0;
}

void fault_print(const struct fault_record *r)
{
	char line[128];

	for (unsigned int i = 0; fault_record_line(r, i, line, sizeof(line)) > 0; i++) {
		printf("%s\n", line);
	}
}

/* The fatal error that ended the run before this start (fault.c). */
static int cmd_fault(int argc, char **argv)
{
	const struct fault_record *r = fault_last();

	(void)argc;
	(void)argv;

	if (r == NULL) {
		printf("No fatal error before this start\n");
		return 0;
	}
	fault_print(r);
	return 0;
}

static int cmd_reboot(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	printf("Rebooting\n");
	stdio_flush();
	watchdog_reboot(0, 0, RESTART_DELAY_MS);
	vTaskDelay(portMAX_DELAY);
	return 0;
}

/* Restarts into the boot ROM's USB loader (the RPI-RP2 drive), as holding the
 * board's REC pad low at reset does.
 */
static int cmd_bootsel(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	printf("Entering BOOTSEL (the RPI-RP2 drive)\n");
	stdio_flush();
	vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
	reset_usb_boot(0, 0);
}

#ifdef NANO_SELFTEST
/* The Zephyr compatibility layer's self-test (a build option, NANO_SELFTEST;
 * freertos/drivers/compat/zephyr_selftest.h).  Runs in this task, at the
 * shell's priority, as the test expects.
 */
static int cmd_selftest(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	return zephyr_selftest_run() == 0 ? 0 : -EIO;
}
#endif

const struct cli_command console_commands[] = {
	{"help", "", "List the commands", cmd_help},
	{"version", "", "Show the versions of the kernel, SDK and compiler", cmd_version},
	{"id", "", "Show the board's unique ID (of the flash)", cmd_id},
	{"uptime", "", "Show the time since the start (TIMER) and the ticks", cmd_uptime},
	{"sleep", "<ms>", "Sleep for ms milliseconds (Zephyr's kernel sleep)", cmd_sleep},
	{"devmem", "<addr> [<bits> [<val>]]", "Read or write memory (hexadecimal)", cmd_devmem},
	{"ps", "", "List the tasks and show the CPU load", cmd_ps},
	{"irqstack", "", "Show the use of the interrupt stack", cmd_irqstack},
	{"fault", "", "Show the fatal error that ended the last run", cmd_fault},
	{"free", "", "Show the use of the FreeRTOS heap", cmd_free},
	{"reboot", "", "Restart the board", cmd_reboot},
	{"bootsel", "", "Restart into the USB bootloader (BOOTSEL)", cmd_bootsel},
#ifdef NANO_SELFTEST
	{"selftest", "", "Test the Zephyr compatibility layer (build option)", cmd_selftest},
#endif
};

const size_t console_command_count = sizeof(console_commands) / sizeof(console_commands[0]);
