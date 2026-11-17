/* SPDX-License-Identifier: MIT */
/* The consoles and the output of each task (nano_console.h). */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nano_console.h"
#include "zephyr_os_port.h"

/* The longest a writer waits for another one on the same console
 * (PICO_STDIO_DEADLOCK_TIMEOUT_MS of the Pico SDK's stdio).
 */
#define LOCK_TIMEOUT_MS 1000

/* The text of nano_console_vprintf() that is formatted on the stack. */
#define FORMAT_BUF_SIZE 128

static struct nano_console *default_console;

void nano_console_init(struct nano_console *con, const char *name, nano_console_out_t out,
		       void *ctx)
{
	memset(con, 0, sizeof(*con));
	con->name = name;
	con->out = out;
	con->ctx = ctx;
	con->lock = xSemaphoreCreateMutexStatic(&con->lock_mem);
	con->sh.console = con;
	con->getopt.opterr = 1;
	con->getopt.optind = 1;
	con->getopt.place = "";
#if CONFIG_GETOPT_LONG
	con->getopt.nonopt_start = -1;
	con->getopt.nonopt_end = -1;
#endif
}

void nano_console_set_default(struct nano_console *con)
{
	default_console = con;
}

void nano_console_bind(TaskHandle_t task, struct nano_console *con)
{
	vTaskSetThreadLocalStoragePointer(task, NANO_CONSOLE_TLS_INDEX, con);
}

struct nano_console *nano_console_bound(void)
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
		return NULL;
	}
	return pvTaskGetThreadLocalStoragePointer(NULL, NANO_CONSOLE_TLS_INDEX);
}

struct nano_console *nano_console_current(void)
{
	struct nano_console *con;

	if (z_os_in_isr()) {
		return NULL;
	}
	con = nano_console_bound();
	return con != NULL ? con : default_console;
}

/* "\n" -> "\r\n", in as few calls of out as the line ends allow. */
static void write_crlf(struct nano_console *con, const char *buf, size_t len)
{
	static const char crlf[] = "\r\n";
	size_t start = 0;

	for (size_t i = 0; i < len; i++) {
		bool after_cr = i > 0 ? buf[i - 1] == '\r' : con->last_cr;

		if (buf[i] == '\n' && !after_cr) {
			if (i > start) {
				con->out(con, buf + start, i - start);
			}
			con->out(con, crlf, 2);
			start = i + 1;
		}
	}
	if (start < len) {
		con->out(con, buf + start, len - start);
	}
	if (len > 0) {
		con->last_cr = buf[len - 1] == '\r';
	}
}

void nano_console_write(struct nano_console *con, const char *buf, size_t len)
{
	bool locked;

	if (con == NULL || len == 0 || z_os_in_isr()) {
		return;
	}
	locked = xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
	if (locked && xSemaphoreTake(con->lock, pdMS_TO_TICKS(LOCK_TIMEOUT_MS)) != pdTRUE) {
		return;
	}
	write_crlf(con, buf, len);
	if (locked) {
		xSemaphoreGive(con->lock);
	}
}

void nano_console_vprintf(struct nano_console *con, const char *fmt, va_list args)
{
	char buf[FORMAT_BUF_SIZE];
	char *text = buf;
	va_list copy;
	int n;

	if (con == NULL) {
		return;
	}
	va_copy(copy, args);
	n = vsnprintf(buf, sizeof(buf), fmt, copy);
	va_end(copy);
	if (n < 0) {
		return;
	}
	if ((size_t)n >= sizeof(buf)) {
		text = malloc((size_t)n + 1);
		if (text != NULL) {
			vsnprintf(text, (size_t)n + 1, fmt, args);
		} else {
			text = buf;
			n = sizeof(buf) - 1;
		}
	}
	nano_console_write(con, text, (size_t)n);
	if (text != buf) {
		free(text);
	}
}
