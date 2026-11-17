/* SPDX-License-Identifier: MIT */
/* The USB console on the Pico SDK and the FreeRTOS kernel: a command line on
 * the USB CDC ACM port and a task that blinks the LED.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "fault.h"
#include "priorities.h"
#include "zephyr_compat.h"

/* Priorities and stacks (bytes): src/priorities_os.h and
 * common/compat/priorities.h.  The console is the Zephyr build's shell
 * thread (NANO_PRIO_SHELL, 14 -> 6, 3072 bytes): the commands run in it.
 */
#define CONSOLE_PRIORITY     NANO_PRIO_SHELL
#define CONSOLE_STACK_SIZE   (NANO_STACK_SHELL / sizeof(StackType_t))
#define HEARTBEAT_PRIORITY   NANO_PRIO_HEARTBEAT
#define HEARTBEAT_STACK_SIZE (NANO_STACK_HEARTBEAT / sizeof(StackType_t))

#define HEARTBEAT_PERIOD_MS 500
/* The console task wakes at least this often, to notice the terminal opening
 * the port.
 */
#define CONSOLE_POLL_MS     50
#define CONSOLE_PROMPT      "freertos:~$ "

struct cli console_cli;

static StackType_t console_stack[CONSOLE_STACK_SIZE];
static StaticTask_t console_tcb;
static TaskHandle_t console_task;

static StackType_t heartbeat_stack[HEARTBEAT_STACK_SIZE];
static StaticTask_t heartbeat_tcb;

/* The run time of the tasks ("ps"), named in FreeRTOSConfig.h.  The other
 * hooks are in fault.c (fatal errors), idle.c and tick.c.
 */
uint64_t app_run_time_us(void)
{
	return time_us_64();
}

static void console_write(const char *text, size_t len)
{
	stdio_put_string(text, (int)len, false, false);
}

/* Called by the Pico SDK's USB code when characters have arrived: from its
 * interrupt, or from a task that is writing to the console.
 */
static void console_chars_available(void *param)
{
	(void)param;

	if (xPortIsInsideInterrupt()) {
		BaseType_t woken = pdFALSE;

		vTaskNotifyGiveFromISR(console_task, &woken);
		portYIELD_FROM_ISR(woken);
	} else {
		xTaskNotifyGive(console_task);
	}
}

/* The record of the fatal error that ended the last run, once, at the first
 * connection of the terminal after the start ("fault" shows it again).
 */
static void console_banner(void)
{
	static bool shown;

	if (shown || fault_last() == NULL) {
		return;
	}
	shown = true;
	printf("*** The last run ended with a fatal error (\"fault\" shows it again):\n");
	fault_print(fault_last());
}

static void console_main(void *arg)
{
	bool connected = false;
	int c;

	(void)arg;

	cli_init(&console_cli, console_commands, console_command_count, CONSOLE_PROMPT,
		 console_write);
	stdio_set_chars_available_callback(console_chars_available, NULL);

	for (;;) {
		/* The terminal has opened the port (DTR): show the prompt. */
		if (stdio_usb_connected() != connected) {
			connected = !connected;
			if (connected) {
				console_write("\r\n", 2);
				console_banner();
				cli_prompt(&console_cli);
			}
		}
		while ((c = stdio_getchar_timeout_us(0)) >= 0) {
			cli_input(&console_cli, (char)c);
		}
		/* Until characters arrive; the timeout is for noticing the terminal. */
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONSOLE_POLL_MS));
	}
}

static void heartbeat_main(void *arg)
{
	TickType_t next = xTaskGetTickCount();

	(void)arg;

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	for (;;) {
		gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
		xTaskDelayUntil(&next, pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
	}
}

int main(void)
{
	/* First: the paint for the interrupt stack's high-water mark, and the
	 * record the last run may have left.
	 */
	irq_stack_paint();
	fault_boot();

	stdio_init_all();

	/* heap_4 sets up its heap at the first allocation and reports no free
	 * space until then ("free"). Nothing else here allocates from it.
	 */
	vPortFree(pvPortMalloc(1));

	/* What Zephyr starts before main(): the system work queue ("sysworkq")
	 * and the RTIO work queue ("RTIO WQ 0") of the compatibility layer
	 * (freertos/drivers/compat).
	 */
	if (zephyr_compat_initialize() != 0) {
		panic("zephyr_compat_initialize");
	}

	console_task = xTaskCreateStatic(console_main, "console", CONSOLE_STACK_SIZE, NULL,
					 CONSOLE_PRIORITY, console_stack, &console_tcb);
	xTaskCreateStatic(heartbeat_main, "heartbeat", HEARTBEAT_STACK_SIZE, NULL,
			  HEARTBEAT_PRIORITY, heartbeat_stack, &heartbeat_tcb);

	vTaskStartScheduler();

	for (;;) {
	}
}
