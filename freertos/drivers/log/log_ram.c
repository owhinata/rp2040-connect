/* SPDX-License-Identifier: MIT */
/* The RAM log of the FreeRTOS build (common/log/log_ram.h, app_log.h): a
 * port of the Zephyr build's zephyr/app/usb-console/src/log_ram.c, as the
 * NuttX build's nuttx/drivers/log/log_ram.c.
 *
 * - The ring (common/log/log_ring.[ch], 8192 bytes as the NuttX build's)
 *   is in RAM that the start-up code leaves alone (__uninitialized_ram,
 *   Zephyr: __noinit), so it survives the watchdog's restart after a
 *   fatal error, "reboot" and the RESET button; log_ram_initialize() empties
 *   it only when its header is not intact (after power-on).  `dropped`
 *   (bytes pushed out since the start) starts at 0 at every start, as in
 *   Zephyr.
 * - The lock is "interrupts disabled" (save_and_disable_interrupts(), no
 *   FreeRTOS call inside; Zephyr: a spinlock, the same on one core), so the
 *   LOG_* macros may be used in interrupt handlers.
 * - LOG_* lines are formatted by app_log_vprintf() at once, in the caller's
 *   context and on its stack (an interrupt handler's too, as on NuttX):
 *   "[HH:MM:SS.mmm,uuu] <lvl> module: message\n" with the time of the call
 *   from the TIMER (nano_mono_us()).  Zephyr's deferred mode takes the time
 *   at the call too and formats later in its logging thread.
 * - "dmesg" prints it (Zephyr's command, the same text).
 *
 * Nothing else of the OS writes into it (the Zephyr build also stores
 * printk(), the NuttX build syslog(); the FreeRTOS kernel and the Pico SDK
 * have no log).
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico.h"
#include "hardware/sync.h"

#include <zephyr/shell/shell.h>

#include "app_log.h"
#include "log_ram.h"
#include "log_ring.h"
#include "os_clock.h"

#if LOG_RAM_BUFFER_SIZE != LOG_RAM_SIZE
#error "log_ram.h and log_ring.h disagree on the size"
#endif

static struct retained_log __uninitialized_ram(retained);

/* Bytes pushed out of the buffer since the start.  Under the lock. */
static uint64_t dropped;

static void store(const char *data, size_t length)
{
	uint32_t flags = save_and_disable_interrupts();

	log_ring_store(&retained, &dropped, data, length);
	restore_interrupts(flags);
}

void log_ram_initialize(void)
{
	uint32_t flags = save_and_disable_interrupts();

	log_ring_validate(&retained);
	restore_interrupts(flags);
}

size_t log_ram_piece(uint64_t *position, bool *first, char *piece, size_t max)
{
	uint32_t flags = save_and_disable_interrupts();
	size_t n = log_ring_piece(&retained, dropped, position, first, piece, max);

	restore_interrupts(flags);
	return n;
}

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	char line[APP_LOG_LINE_MAX];
	size_t room = sizeof(line) - 1; /* one byte for '\n' */
	size_t length;
	int n;

	n = log_ring_prefix(line, room + 1, nano_mono_us(), level, module, func);
	length = n < 0 ? 0 : ((size_t)n < room ? (size_t)n : room);
	if (length < room) {
		n = vsnprintf(line + length, room - length + 1, fmt, ap);
		if (n > 0) {
			length += (size_t)n < room - length ? (size_t)n : room - length;
		}
	}
	line[length++] = '\n';
	store(line, length);
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	app_log_vprintf(level, module, func, fmt, ap);
	va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	/* Zephyr: k_uptime_get_32(), milliseconds since the start. */
	uint32_t now_ms = (uint32_t)(nano_mono_us() / 1000u);
	uint32_t flags = save_and_disable_interrupts();
	bool pass = log_ring_ratelimit(last_ms, skipped, now_ms, rate_ms, skipped_out);

	restore_interrupts(flags);
	return pass;
}

/* cmd_dmesg() of the Zephyr build: the log in pieces of up to 96 bytes, so
 * that no second copy of it is needed; the position counts bytes from the
 * start (log_ram_piece()), which keeps it valid when new messages push old
 * ones out while printing; printing stops after twice the ring's size in
 * case logging keeps up with it.  Arguments are ignored.
 */
static int cmd_dmesg(const struct shell *sh, size_t argc, char **argv)
{
	char piece[97];
	uint64_t position = 0;
	bool first = true;
	bool any = false;

	(void)argc;
	(void)argv;
	for (size_t printed = 0; printed < 2u * LOG_RAM_BUFFER_SIZE;) {
		size_t n = log_ram_piece(&position, &first, piece, sizeof(piece) - 1);

		if (n == 0) {
			break;
		}
		piece[n] = '\0';
		shell_fprintf(sh, SHELL_NORMAL, "%s", piece);
		position += n;
		printed += n;
		any = true;
	}
	if (!any) {
		shell_print(sh, "No logs.");
	}
	return 0;
}

SHELL_CMD_REGISTER(dmesg, NULL, "Show logs stored in RAM", cmd_dmesg);
