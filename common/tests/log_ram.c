/* SPDX-License-Identifier: MIT */
/* common/log/log_ram.c on the host's part of its boundary
 * (tests/log_ram_os_host): the lines of the LOG_* macros (prefix, levels,
 * function name, the cut at APP_LOG_LINE_MAX), the OS's raw bytes, the
 * check at the start (a restart keeps the ring, a spoiled header empties
 * it), the rate limit on the uptime in milliseconds, and that every access
 * to the ring is under the lock.  The ring itself is tested in
 * tests/log_ring.c.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_log.h"
#include "log_ram.h"
#include "log_ring.h"

LOG_MODULE_REGISTER(test_mod, LOG_LEVEL_DBG);

int log_ram_test_masked;
int log_ram_test_saves;
int log_ram_test_attached;
uint64_t log_ram_test_now_us;
extern struct retained_log *const log_ram_test_ring;

/* What "dmesg" prints. */
static size_t dmesg(char *out, size_t size)
{
	char piece[97];
	uint64_t position = 0;
	bool first = true;
	size_t total = 0;

	for (;;) {
		size_t n = log_ram_piece(&position, &first, piece, sizeof(piece) - 1);

		assert(log_ram_test_masked == 0);
		if (n == 0) {
			break;
		}
		assert(total + n < size);
		memcpy(out + total, piece, n);
		total += n;
		position += n;
	}
	out[total] = '\0';
	return total;
}

static char text[2 * LOG_RAM_SIZE];

static void test_initialize(void)
{
	int saves = log_ram_test_saves;

	/* All zero, as after power-on: not intact, emptied. */
	log_ram_initialize();
	assert(log_ram_test_attached == 1);
	assert(log_ram_test_saves == saves + 1 && log_ram_test_masked == 0);
	assert(log_ram_test_ring->magic == LOG_RAM_MAGIC);
	assert(dmesg(text, sizeof(text)) == 0);
}

static void test_lines(void)
{
	int saves = log_ram_test_saves;

	log_ram_test_now_us = 3723004005ULL;
	LOG_WRN("hello %d", 42);
	assert(log_ram_test_saves == saves + 1 && log_ram_test_masked == 0);
	log_ram_test_now_us = 798223;
	LOG_ERR("e");
	LOG_INF("i");
	LOG_DBG("d");
	app_log_printf(LOG_LEVEL_INF, "other", NULL, "%s", "");
	log_ram_store("raw\r\n", 5);
	assert(log_ram_test_masked == 0);
	dmesg(text, sizeof(text));
	assert(strcmp(text, "[01:02:03.004,005] <wrn> test_mod: hello 42\n"
			    "[00:00:00.798,223] <err> test_mod: e\n"
			    "[00:00:00.798,223] <inf> test_mod: i\n"
			    "[00:00:00.798,223] <dbg> test_mod.test_lines: d\n"
			    "[00:00:00.798,223] <inf> other: \n"
			    "raw\r\n") == 0);
}

static void test_cut(void)
{
	char message[300];
	char *line;

	memset(message, 'x', sizeof(message) - 1);
	message[sizeof(message) - 1] = '\0';
	log_ram_test_now_us = 1;
	LOG_INF("%s", message);
	dmesg(text, sizeof(text));
	line = strstr(text, "[00:00:00.000,001] <inf> test_mod: xxx");
	assert(line != NULL);
	/* APP_LOG_LINE_MAX bytes with the '\n', the last of the log. */
	assert(strlen(line) == APP_LOG_LINE_MAX);
	assert(line[APP_LOG_LINE_MAX - 1] == '\n' && line[APP_LOG_LINE_MAX - 2] == 'x');

	/* A module name longer than the line: the prefix is cut too. */
	memset(message, 'm', sizeof(message) - 1);
	app_log_printf(LOG_LEVEL_ERR, message, NULL, "lost");
	dmesg(text, sizeof(text));
	line = strstr(text, "[00:00:00.000,001] <err> mmm");
	assert(line != NULL && strlen(line) == APP_LOG_LINE_MAX);
	assert(line[APP_LOG_LINE_MAX - 1] == '\n' && line[APP_LOG_LINE_MAX - 2] == 'm');
}

static void test_restart(void)
{
	size_t before = dmesg(text, sizeof(text));

	/* A restart that keeps the RAM: the ring stays. */
	log_ram_initialize();
	assert(log_ram_test_attached == 2);
	assert(dmesg(text, sizeof(text)) == before);

	/* A spoiled header (power-on): emptied. */
	log_ram_test_ring->magic_inverse ^= 1;
	log_ram_initialize();
	assert(dmesg(text, sizeof(text)) == 0);
}

static void test_ratelimit(void)
{
	for (int i = 0; i < 3; i++) {
		/* 1.0 s, 1.5 s, 6.0 s: the second is held back. */
		static const uint64_t at[] = {1000000, 1500000, 6000000};

		log_ram_test_now_us = at[i];
		LOG_WRN_RATELIMIT("r%d", i);
		assert(log_ram_test_masked == 0);
	}
	dmesg(text, sizeof(text));
	assert(strcmp(text, "[00:00:01.000,000] <wrn> test_mod: r0\n"
			    "[00:00:06.000,000] <wrn> test_mod: Skipped 1 messages\n"
			    "[00:00:06.000,000] <wrn> test_mod: r2\n") == 0);

	/* The uptime is the clock in milliseconds, wrapping at 32 bits. */
	uint32_t last = 0, skipped = 0, out = 99;

	log_ram_test_now_us = (0x100000000ULL + 7) * 1000U + 999U;
	assert(app_log_ratelimit_pass(&last, &skipped, 5000, &out) && out == 0);
	assert(last == 7);
}

int main(void)
{
	test_initialize();
	test_lines();
	test_cut();
	test_restart();
	test_ratelimit();
	puts("log_ram ok");
	return 0;
}
