/* SPDX-License-Identifier: MIT */
/* The commands of the USB console. */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"
#include "pico/version.h"
#include "hardware/watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"

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
	       (unsigned long)xTaskGetTickCount(), configTICK_RATE_HZ);
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

	(void)argc;
	(void)argv;

	if (uxTaskGetNumberOfTasks() > PS_TASKS_MAX) {
		return -ENOMEM;
	}
	count = uxTaskGetSystemState(tasks, PS_TASKS_MAX, &total);
	printf("%-16s %-5s %4s %10s %6s\n", "NAME", "STATE", "PRIO", "STACK FREE", "CPU");
	for (UBaseType_t i = 0; i < count; i++) {
		const TaskStatus_t *t = &tasks[i];
		unsigned int permille = 0;

		if (total > 0) {
			permille = (unsigned int)(t->ulRunTimeCounter * 1000u / total);
		}
		printf("%-16s %-5c %4lu %10lu %4u.%u%%\n", t->pcTaskName,
		       states[t->eCurrentState <= eInvalid ? t->eCurrentState : eInvalid],
		       (unsigned long)t->uxCurrentPriority,
		       (unsigned long)(t->usStackHighWaterMark * sizeof(StackType_t)),
		       permille / 10, permille % 10);
	}
	printf("STATE: X running, R ready, B blocked, S suspended, D deleted\n");
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

const struct cli_command console_commands[] = {
	{"help", "", "List the commands", cmd_help},
	{"version", "", "Show the versions of the kernel, SDK and compiler", cmd_version},
	{"id", "", "Show the board's unique ID (of the flash)", cmd_id},
	{"uptime", "", "Show the time since the start", cmd_uptime},
	{"ps", "", "List the tasks", cmd_ps},
	{"free", "", "Show the use of the FreeRTOS heap", cmd_free},
	{"reboot", "", "Restart the board", cmd_reboot},
	{"bootsel", "", "Restart into the USB bootloader (BOOTSEL)", cmd_bootsel},
};

const size_t console_command_count = sizeof(console_commands) / sizeof(console_commands[0]);
