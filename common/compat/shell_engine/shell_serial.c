/* SPDX-License-Identifier: MIT */
/* The serial session of the shell engine (shell_serial.h), on the console
 * transport (shell_serial_os.h) and the compatibility layer's kernel API.
 */

#include <stdbool.h>
#include <stddef.h>

#include <zephyr/kernel.h>

#include "cli.h"
#include "priorities.h"
#include "shell_console.h"
#include "shell_engine.h"
#include "shell_serial.h"
#include "shell_serial_os.h"

/* Characters taken from the terminal's driver at a time (the Pico SDK's USB
 * CDC ACM driver's receive buffer has 64).
 */
#define READ_SIZE 64

static struct shell_console console;
static struct cli serial_cli;
static const char *serial_prompt;
static void (*connect_hook)(void);

static K_THREAD_STACK_DEFINE(session_stack, APP_STACK_SHELL);
static k_tid_t session_tid;
/* Given when characters have arrived. */
static K_SEM_DEFINE(rx_sem, 0, 1);

void shell_serial_rx_ready(void)
{
	k_sem_give(&rx_sem);
}

static void serial_out(struct shell_console *con, const char *buf, size_t len)
{
	ARG_UNUSED(con);
	shell_serial_os_write(buf, len);
}

static void cli_write(void *ctx, const char *text, size_t len)
{
	shell_console_write(ctx, text, len);
}

static void cli_execute(void *ctx, char *line)
{
	ARG_UNUSED(ctx);
	(void)shell_engine_execute(line);
}

static void session_main(void *p1, void *p2, void *p3)
{
	char buf[READ_SIZE];
	bool connected = false;
	int n;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	shell_console_bind(&console);
	cli_init(&serial_cli, serial_prompt, cli_write, cli_execute, &console);
	shell_serial_os_rx_init();

	for (;;) {
		/* A terminal has opened the line: show the prompt. */
		if (shell_serial_os_connected() != connected) {
			connected = !connected;
			if (connected) {
				cli_write(&console, "\r\n", 2);
				if (connect_hook != NULL) {
					connect_hook();
				}
				cli_prompt(&serial_cli);
			}
		}
		while ((n = shell_serial_os_read(buf, sizeof(buf))) > 0) {
			for (int i = 0; i < n; i++) {
				cli_input(&serial_cli, buf[i]);
			}
		}
		/* Until characters arrive; the timeout is for noticing the terminal. */
		(void)k_sem_take(&rx_sem, K_MSEC(SHELL_SERIAL_POLL_MS));
	}
}

void shell_serial_init(const char *name)
{
	shell_console_init(&console, name, serial_out, NULL);
	shell_console_set_default(&console);
	shell_serial_os_stdio_init();
}

int shell_serial_start(const char *prompt, void (*on_connect)(void))
{
	/* The thread's start data, which it may read while it runs. */
	static const struct z_thread_static session = {
		.name = "console",
		.stack = session_stack,
		.stack_size = K_THREAD_STACK_SIZEOF(session_stack),
		.entry = session_main,
		.prio = APP_PRIO_SHELL,
		.tid = &session_tid,
	};

	serial_prompt = prompt;
	connect_hook = on_connect;
	return z_thread_static_start(&session);
}
