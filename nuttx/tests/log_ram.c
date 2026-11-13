/* SPDX-License-Identifier: Apache-2.0 */
/* drivers/log/log_ring.c: the retained ring, the dmesg pieces, the line
 * prefix (against Zephyr's timestamp arithmetic in log_output.c) and the
 * rate limit.
 */
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "log_ring.h"

static struct retained_log ring;
static uint64_t dropped;

/* cmd_dmesg() of the Zephyr build, collecting the output. */
static size_t dmesg(char *out, size_t size, int overwrite_after_first)
{
	char piece[97];
	uint64_t position = 0;
	bool first = true;
	size_t total = 0;

	for (size_t printed = 0; printed < 2U * LOG_RAM_SIZE;) {
		size_t n = log_ring_piece(&ring, dropped, &position, &first, piece,
					  sizeof(piece) - 1);
		if (!n) {
			break;
		}
		assert(total + n <= size);
		memcpy(out + total, piece, n);
		total += n;
		position += n;
		printed += n;
		if (overwrite_after_first) {
			/* New messages keep coming while printing. */
			log_ring_store(&ring, &dropped, "abcdefghijklmnopqrstuvwxyz0123456789"
				       "abcdefghijklmnopqrstuvwxyz0123456789abcdefghijklmnopqrstuvwxyz",
				       98);
		}
	}
	return total;
}

/* Zephyr's store_output() byte loop. */
static void reference_store(struct retained_log *log, uint64_t *drop, const char *data,
			    size_t length)
{
	for (size_t i = 0; i < length; i++) {
		if (log->length == LOG_RAM_SIZE) {
			log->start = (log->start + 1) % LOG_RAM_SIZE;
			log->length--;
			(*drop)++;
		}
		log->buffer[(log->start + log->length) % LOG_RAM_SIZE] = data[i];
		log->length++;
	}
}

static void test_store_matches_zephyr(void)
{
	static struct retained_log a, b;
	static char data[3 * LOG_RAM_SIZE];
	uint64_t da = 0, db = 0;

	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));
	srand(7);
	for (int round = 0; round < 20000; round++) {
		size_t n = (size_t)rand() % (round % 50 == 0 ? sizeof(data) : 200);

		for (size_t i = 0; i < n; i++) {
			data[i] = (char)rand();
		}
		log_ring_store(&a, &da, data, n);
		reference_store(&b, &db, data, n);
		assert(a.start == b.start && a.length == b.length && da == db);
		assert(memcmp(a.buffer, b.buffer, LOG_RAM_SIZE) == 0);
	}
}

static void test_validate(void)
{
	memset(&ring, 0xa5, sizeof(ring));
	log_ring_validate(&ring);
	assert(ring.magic == LOG_RAM_MAGIC && ring.magic_inverse == ~LOG_RAM_MAGIC);
	assert(ring.start == 0 && ring.length == 0);

	/* Intact: kept. */
	ring.start = 4095;
	ring.length = 4096;
	log_ring_validate(&ring);
	assert(ring.start == 4095 && ring.length == 4096);

	/* Each broken field empties it. */
	ring.start = 4096;
	log_ring_validate(&ring);
	assert(ring.start == 0 && ring.length == 0);
	ring.length = 4097;
	log_ring_validate(&ring);
	assert(ring.length == 0);
	ring.length = 10;
	ring.magic_inverse = 0;
	log_ring_validate(&ring);
	assert(ring.length == 0 && ring.magic_inverse == ~LOG_RAM_MAGIC);
	ring.length = 10;
	ring.magic = 0;
	log_ring_validate(&ring);
	assert(ring.length == 0 && ring.magic == LOG_RAM_MAGIC);
}

static void test_store_and_dmesg(void)
{
	static char data[10000], out[3 * LOG_RAM_SIZE];
	size_t n;

	memset(&ring, 0, sizeof(ring));
	log_ring_validate(&ring);
	dropped = 0;

	/* Empty: nothing printed ("No logs." is the command's). */
	assert(dmesg(out, sizeof(out), 0) == 0);

	for (size_t i = 0; i < sizeof(data); i++) {
		data[i] = (char)('A' + i % 26);
	}
	log_ring_store(&ring, &dropped, data, 100);
	n = dmesg(out, sizeof(out), 0);
	assert(n == 100 && memcmp(out, data, 100) == 0);

	/* Over 4096 bytes: the oldest go, dropped counts them. */
	log_ring_store(&ring, &dropped, data + 100, 5000);
	assert(ring.length == LOG_RAM_SIZE && dropped == 5100 - LOG_RAM_SIZE);
	n = dmesg(out, sizeof(out), 0);
	assert(n == LOG_RAM_SIZE && memcmp(out, data + 5100 - LOG_RAM_SIZE, n) == 0);

	/* Logging while printing: skips what was overwritten, stops at 2*4096. */
	n = dmesg(out, sizeof(out), 1);
	assert(n >= 2U * LOG_RAM_SIZE - 96 && n <= 2U * LOG_RAM_SIZE + 96);
}

/* Zephyr's timestamp_print() with freq 1 MHz and 64-bit timestamps. */
static void zephyr_prefix(char *buf, size_t size, uint64_t timestamp, const char *lvl,
			  const char *module)
{
	const uint32_t freq = 1000000;
	uint64_t total_seconds = timestamp / freq;
	uint32_t remainder = timestamp % freq;
	uint32_t ms = (remainder * 1000U) / freq;
	uint32_t us = (1000 * (remainder * 1000U - (ms * freq))) / freq;
	uint64_t hours = total_seconds / 3600U;
	uint32_t seconds = (uint32_t)(total_seconds - hours * 3600U);
	uint32_t mins = seconds / 60U;

	seconds -= mins * 60U;
	snprintf(buf, size, "[%02" PRIu64 ":%02u:%02u.%03u,%03u] <%s> %s: ", hours, mins,
		 seconds, ms, us, lvl, module);
}

static void test_prefix(void)
{
	char a[128], b[128];
	static const char *const lvl[] = {"", "err", "wrn", "inf", "dbg"};

	log_ring_prefix(a, sizeof(a), 798223, 3, "usb_console", NULL);
	assert(strcmp(a, "[00:00:00.798,223] <inf> usb_console: ") == 0);
	log_ring_prefix(a, sizeof(a), 2, 1, "LSM6DSO", NULL);
	assert(strcmp(a, "[00:00:00.000,002] <err> LSM6DSO: ") == 0);
	log_ring_prefix(a, sizeof(a), 3723004005ULL, 2, "m", NULL);
	assert(strcmp(a, "[01:02:03.004,005] <wrn> m: ") == 0);
	log_ring_prefix(a, sizeof(a), 360000000001ULL, 4, "m", "f");
	assert(strcmp(a, "[100:00:00.000,001] <dbg> m.f: ") == 0);

	/* Cut like snprintf, returns the full length. */
	assert(log_ring_prefix(a, 10, 798223, 3, "usb_console", NULL) == 38);
	assert(strncmp(a, "[00:00:00", 9) == 0 && strlen(a) == 9);
	assert(log_ring_prefix(a, 0, 1, 3, "m", NULL) == 28);

	srand(1);
	for (int i = 0; i < 200000; i++) {
		uint64_t t = ((uint64_t)rand() << 31 ^ (uint64_t)rand()) >> (rand() % 40);
		int level = 1 + rand() % 4;

		log_ring_prefix(a, sizeof(a), t, level, "mod", NULL);
		zephyr_prefix(b, sizeof(b), t, lvl[level], "mod");
		assert(strcmp(a, b) == 0);
	}
}

static void test_ratelimit(void)
{
	uint32_t last = 0, skipped = 0, out = 99;

	assert(log_ring_ratelimit(&last, &skipped, 1000, 5000, &out) && out == 0);
	assert(!log_ring_ratelimit(&last, &skipped, 1001, 5000, &out));
	assert(!log_ring_ratelimit(&last, &skipped, 5999, 5000, &out));
	assert(skipped == 2);
	assert(log_ring_ratelimit(&last, &skipped, 6000, 5000, &out) && out == 2);
	assert(skipped == 0 && last == 6000);
	/* Wraps at 32 bits like k_uptime_get_32(). */
	last = 0xfffffff0u;
	assert(!log_ring_ratelimit(&last, &skipped, 0xfffffff0u + 4999u, 5000, &out));
	assert(log_ring_ratelimit(&last, &skipped, 0xfffffff0u + 5000u, 5000, &out) && out == 1);
}

int main(void)
{
	test_validate();
	test_store_matches_zephyr();
	test_store_and_dmesg();
	test_prefix();
	test_ratelimit();
	puts("log_ram ok");
	return 0;
}
