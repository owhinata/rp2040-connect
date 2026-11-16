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

/* Priorities (a higher number is a higher priority) and stacks (words). */
#define CONSOLE_PRIORITY     (tskIDLE_PRIORITY + 2)
#define CONSOLE_STACK_SIZE   1024
#define HEARTBEAT_PRIORITY   (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_STACK_SIZE 256

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

/* Hooks named in FreeRTOSConfig.h. */

uint64_t app_run_time_us(void)
{
	return time_us_64();
}

void app_assert_failed(const char *file, int line)
{
	panic("FreeRTOS assertion failed: %s:%d", file, line);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
	(void)task;
	panic("Stack overflow in task %s", name);
}

void vApplicationMallocFailedHook(void)
{
	panic("FreeRTOS heap exhausted");
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
	stdio_init_all();

	/* heap_4 sets up its heap at the first allocation and reports no free
	 * space until then ("free"). Nothing else here allocates from it.
	 */
	vPortFree(pvPortMalloc(1));

	console_task = xTaskCreateStatic(console_main, "console", CONSOLE_STACK_SIZE, NULL,
					 CONSOLE_PRIORITY, console_stack, &console_tcb);
	xTaskCreateStatic(heartbeat_main, "heartbeat", HEARTBEAT_STACK_SIZE, NULL,
			  HEARTBEAT_PRIORITY, heartbeat_stack, &heartbeat_tcb);

	vTaskStartScheduler();

	for (;;) {
	}
}
