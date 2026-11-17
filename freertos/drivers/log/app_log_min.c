/* SPDX-License-Identifier: MIT */
/* The smallest receiver of the LOG_* macros (common/log/app_log.h) for the
 * FreeRTOS build, until the RAM log comes (F03: the ring kept across a
 * restart and "dmesg", as nuttx/drivers/log/log_ram.c, which replaces this
 * file).
 *
 * Each message becomes one line in Zephyr's format,
 * "[HH:MM:SS.mmm,uuu] <lvl> module: message\n" (log_ring_prefix(), the time
 * from the TIMER), formatted at once in the caller's context and on its
 * stack, and stored in a ring of common/log/log_ring.h in ordinary RAM
 * (emptied at every start).  Nothing reads it yet.  Usable in tasks and in
 * interrupt handlers: the ring is changed with interrupts disabled
 * (save_and_disable_interrupts(), no FreeRTOS call inside).
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware/sync.h"

#include "app_log.h"
#include "log_ring.h"
#include "os_clock.h"

static struct retained_log ring;
static uint64_t dropped;

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	char line[APP_LOG_LINE_MAX];
	size_t room = sizeof(line) - 1; /* one byte for '\n' */
	size_t length;
	uint32_t flags;
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

	flags = save_and_disable_interrupts();
	log_ring_store(&ring, &dropped, line, length);
	restore_interrupts(flags);
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
