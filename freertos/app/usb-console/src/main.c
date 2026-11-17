/* SPDX-License-Identifier: MIT */
/* The USB console on the Pico SDK and the FreeRTOS kernel: a shell on the
 * USB CDC ACM port, the key-value store and a task that blinks the LED.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "app_log.h"
#include "cli.h"
#include "fault.h"
#include "kv_store.h"
#include "log_ram.h"
#include "nano_console.h"
#include "nano_flash_kv_os.h"
#include "nano_shell.h"
#include "nano_stdio.h"
#include "priorities.h"
#include "zephyr_compat.h"

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

/* Priorities and stacks (bytes): src/priorities_os.h and
 * common/compat/priorities.h.  The console is the Zephyr build's shell
 * thread (NANO_PRIO_SHELL, 14 -> 6, 3072 bytes): the commands run in it.
 */
#define CONSOLE_PRIORITY     NANO_PRIO_SHELL
#define CONSOLE_STACK_SIZE   (NANO_STACK_SHELL / sizeof(StackType_t))
#define MAIN_PRIORITY        NANO_PRIO_MAIN
#define MAIN_STACK_SIZE      (NANO_STACK_MAIN / sizeof(StackType_t))
#define HEARTBEAT_PRIORITY   NANO_PRIO_HEARTBEAT
#define HEARTBEAT_STACK_SIZE (NANO_STACK_HEARTBEAT / sizeof(StackType_t))

#define HEARTBEAT_PERIOD_MS 500
/* The console task wakes at least this often, to notice the terminal opening
 * the port.
 */
#define CONSOLE_POLL_MS     50
#define CONSOLE_PROMPT      "freertos:~$ "
/* Characters taken from the USB code at a time (its receive buffer has 64). */
#define CONSOLE_READ_SIZE   64

/* The USB console: the terminal of the USB session and the default console
 * of the tasks that have none (freertos/drivers/shell/nano_console.h).
 */
static struct nano_console usb_console;
static struct cli usb_cli;

static StackType_t console_stack[CONSOLE_STACK_SIZE];
static StaticTask_t console_tcb;
static TaskHandle_t console_task;

static StackType_t main_stack[MAIN_STACK_SIZE];
static StaticTask_t main_tcb;

static StackType_t heartbeat_stack[HEARTBEAT_STACK_SIZE];
static StaticTask_t heartbeat_tcb;

/* The run time of the tasks ("ps"), named in FreeRTOSConfig.h.  The other
 * hooks are in fault.c (fatal errors), idle.c and tick.c.
 */
uint64_t app_run_time_us(void)
{
	return time_us_64();
}

/* The USB console's terminal: the Pico SDK's USB CDC ACM driver, which
 * drops the text while no terminal has the port open (DTR) and, when the
 * terminal stops reading, waits up to 500 ms for room in its 64-byte
 * buffer and then drops the rest until room comes again
 * (PICO_STDIO_USB_STDOUT_TIMEOUT_US).
 */
static void usb_out(struct nano_console *con, const char *buf, size_t len)
{
	(void)con;
	stdio_usb.out_chars(buf, (int)len);
}

static void cli_write(void *ctx, const char *text, size_t len)
{
	nano_console_write(ctx, text, len);
}

static void cli_execute(void *ctx, char *line)
{
	(void)ctx;
	(void)nano_shell_execute(line);
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
 * connection of the terminal after the start ("fault" shows it again, and
 * the RAM log has it, "dmesg").
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
	char buf[CONSOLE_READ_SIZE];
	bool connected = false;
	int n;

	(void)arg;

	nano_console_bind(NULL, &usb_console);
	cli_init(&usb_cli, CONSOLE_PROMPT, cli_write, cli_execute, &usb_console);
	stdio_usb.set_chars_available_callback(console_chars_available, NULL);

	for (;;) {
		/* The terminal has opened the port (DTR): show the prompt. */
		if (stdio_usb_connected() != connected) {
			connected = !connected;
			if (connected) {
				cli_write(&usb_console, "\r\n", 2);
				console_banner();
				cli_prompt(&usb_cli);
			}
		}
		while ((n = stdio_usb.in_chars(buf, sizeof(buf))) > 0) {
			for (int i = 0; i < n; i++) {
				cli_input(&usb_cli, buf[i]);
			}
		}
		/* Until characters arrive; the timeout is for noticing the terminal. */
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONSOLE_POLL_MS));
	}
}

/* The Zephyr build's main() (zephyr/app/usb-console/src/main.c), as the task
 * "main" with its priority (NANO_PRIO_MAIN: Zephyr's 0, above the console) and
 * stack, ending when done as Zephyr's main thread does.  It runs before the
 * console's task, which has a lower priority, as the Zephyr build's main()
 * runs before its shell thread.
 * kv_store_init() is settings_subsys_init(): it checks what the key-value
 * area holds, mounts NVS on it (the "fs_nvs" lines of "dmesg") and reads the
 * settings' handlers.  Data that is neither Zephyr's NVS nor erased is not
 * touched: the check logs it and every call of the store fails with -EILSEQ
 * until "kv wipe confirm" (common/compat/zephyr_flash_kv.c).  In the Zephyr
 * build the Wi-Fi credentials' SYS_INIT() has run settings_subsys_init()
 * before main(); here nothing uses the settings before this task.
 */
static void main_main(void *arg)
{
	int ret;

	(void)arg;

	LOG_INF("USB console started");
	ret = kv_store_init();
	if (ret) {
		LOG_ERR("Could not initialize settings: %d", ret);
	}
	vTaskDelete(NULL);
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
	/* First: the paint for the interrupt stack's high-water mark, the RAM
	 * log kept across the restart, and the record the last run may have
	 * left (fault_boot() also puts it into the RAM log).
	 */
	irq_stack_paint();
	log_ram_initialize();
	fault_boot();

	/* The USB CDC ACM port, and the stdio of the tasks on the consoles
	 * (freertos/drivers/shell/nano_stdio.h).
	 */
	stdio_init_all();
	nano_console_init(&usb_console, "usb", usb_out, NULL);
	nano_console_set_default(&usb_console);
	nano_stdio_init(&stdio_usb);

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

	/* The bounds of the key-value area from the linker script; until they
	 * are found right, the store refuses to erase or program the flash
	 * (freertos/drivers/kv).
	 */
	(void)nano_flash_kv_initialize();

	xTaskCreateStatic(main_main, "main", MAIN_STACK_SIZE, NULL, MAIN_PRIORITY, main_stack,
			  &main_tcb);
	console_task = xTaskCreateStatic(console_main, "console", CONSOLE_STACK_SIZE, NULL,
					 CONSOLE_PRIORITY, console_stack, &console_tcb);
	xTaskCreateStatic(heartbeat_main, "heartbeat", HEARTBEAT_STACK_SIZE, NULL,
			  HEARTBEAT_PRIORITY, heartbeat_stack, &heartbeat_tcb);

	vTaskStartScheduler();

	for (;;) {
	}
}
