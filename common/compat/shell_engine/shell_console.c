/* SPDX-License-Identifier: MIT */
/* The consoles and the output of each thread (shell_console.h). */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "shell_console.h"

/* The longest a writer waits for another one on the same console
 * (PICO_STDIO_DEADLOCK_TIMEOUT_MS of the Pico SDK's stdio).
 */
#define LOCK_TIMEOUT_MS 1000

/* The text of shell_console_vprintf() that is formatted on the stack. */
#define FORMAT_BUF_SIZE 128

static struct shell_console *default_console;

void shell_console_init(struct shell_console *con, const char *name, shell_console_out_t out,
			void *ctx)
{
	memset(con, 0, sizeof(*con));
	con->name = name;
	con->out = out;
	con->ctx = ctx;
	(void)k_mutex_init(&con->lock);
	con->other_lock_timeout_ms = LOCK_TIMEOUT_MS;
	con->sh.console = con;
	con->getopt.opterr = 1;
	con->getopt.optind = 1;
	con->getopt.place = "";
#if CONFIG_GETOPT_LONG
	con->getopt.nonopt_start = -1;
	con->getopt.nonopt_end = -1;
#endif
}

void shell_console_set_default(struct shell_console *con)
{
	default_console = con;
}

void shell_console_bind(struct shell_console *con)
{
	k_thread_custom_data_set(con);
}

struct shell_console *shell_console_bound(void)
{
	return k_thread_custom_data_get();
}

struct shell_console *shell_console_current(void)
{
	struct shell_console *con;

	if (k_is_in_isr()) {
		return NULL;
	}
	con = shell_console_bound();
	return con != NULL ? con : default_console;
}

/* "\n" -> "\r\n", in as few calls of out as the line ends allow. */
static void write_crlf(struct shell_console *con, const char *buf, size_t len)
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

void shell_console_write(struct shell_console *con, const char *buf, size_t len)
{
	bool locked;

	if (con == NULL || len == 0 || k_is_in_isr()) {
		return;
	}
	locked = !k_is_pre_kernel();
	if (locked) {
		uint32_t ms =
			shell_console_bound() == con ? LOCK_TIMEOUT_MS : con->other_lock_timeout_ms;

		if (k_mutex_lock(&con->lock, K_MSEC(ms)) != 0) {
			return;
		}
	}
	write_crlf(con, buf, len);
	if (locked) {
		(void)k_mutex_unlock(&con->lock);
	}
}

void shell_console_set_other_lock_timeout(struct shell_console *con, uint32_t ms)
{
	con->other_lock_timeout_ms = ms;
}

bool shell_console_lock(struct shell_console *con, uint32_t timeout_ms)
{
	return k_mutex_lock(&con->lock, K_MSEC(timeout_ms)) == 0;
}

void shell_console_unlock(struct shell_console *con)
{
	(void)k_mutex_unlock(&con->lock);
}

void shell_console_vprintf(struct shell_console *con, const char *fmt, va_list args)
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
	shell_console_write(con, text, (size_t)n);
	if (text != buf) {
		free(text);
	}
}
