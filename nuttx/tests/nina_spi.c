/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of nuttx/drivers/nina/nina_spi.c with a fake board
 * (<nano_nina.h>: SPI, CS, READY, RESET, IRQ) and fake kernel services
 * (nina_os.h: lock, sleeps, clock).
 *
 * The first half repeats the Zephyr build's tests/nina_spi.c case by case
 * (framing of wide parameters and padding, bulk transfers, the
 * three-parameter answers of 0x75 and 0x7a, the length checks, a failing
 * bulk transfer).  The rest covers what the Zephyr test does not: narrow
 * parameters, the version request, the START search, each -EPROTO,
 * -EMSGSIZE and -EINVAL, 0x27, nina_ip_config() (0x21), the READY
 * timeouts (2000, 20 and 10000 ms of the fake clock, polled every
 * 100 us), the reset sequence of the first request, the order of the CS
 * edges and transfers, and the lock.
 */

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "../drivers/nina/nina_spi.c"

/* Fake board -------------------------------------------------------------*/

static uint8_t request[2048], response[2048];
static size_t request_len, response_len, response_offset;
static unsigned int phase, calls, bulk_calls, single_calls;
static int cs_active, fail_bulk, bus_ready = 1, fill_byte = -1;
static unsigned int fail_single_at;      /* fail the Nth 1-byte transfer */
static int ready_never_from = -1;        /* READY stays off from this phase */
static int ready_stuck_phase = -1;       /* READY stays on with CS in it */
static int lock_depth, lock_calls, lock_result;
static long long now_us;
static unsigned int polls;
static char events[4096];

static void event(const char *e)
{
	assert(strlen(events) + strlen(e) + 2 < sizeof(events));
	strcat(events, e);
	strcat(events, " ");
}

bool nano_nina_bus_ready(void)
{
	return bus_ready;
}

int nano_nina_pin_configure(enum nano_nina_pin pin, enum nano_nina_mode mode)
{
	static const char *const pins[] = {"CS", "READY", "RESET", "IRQ"};
	static const char *const modes[] = {"inactive", "active", "high", "input"};
	char e[32];

	assert(lock_depth > 0);
	snprintf(e, sizeof(e), "%s=%s", pins[pin], modes[mode]);
	event(e);
	if (pin == NANO_NINA_CS) {
		cs_active = mode == NANO_NINA_OUTPUT_ACTIVE;
	}
	return 0;
}

int nano_nina_pin_set(enum nano_nina_pin pin, int value)
{
	assert(lock_depth > 0);
	if (pin == NANO_NINA_CS) {
		/* The test's requests never assert CS twice in a row. */
		assert(cs_active != value);
		cs_active = value;
		if (value) {
			phase++;
		}
		event(value ? "CS1" : "CS0");
	} else {
		char e[16];

		snprintf(e, sizeof(e), "%s=%d", pin == NANO_NINA_RESET ? "RESET" : "IRQ", value);
		event(e);
	}
	return 0;
}

int nano_nina_pin_get(enum nano_nina_pin pin)
{
	assert(pin == NANO_NINA_READY && lock_depth > 0);
	polls++;
	if (ready_never_from >= 0 && (int)phase >= ready_never_from && !cs_active) {
		return 0;
	}
	if (cs_active && (int)phase == ready_stuck_phase) {
		return 1;
	}
	/* The module is ready (line low) while CS is off and answers CS by
	 * releasing the line.
	 */
	return !cs_active;
}

int nano_nina_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
	assert(lock_depth > 0 && cs_active && tx != NULL && len > 0);
	calls++;
	if (len > 1) {
		bulk_calls++;
		if (fail_bulk) {
			return -EIO;
		}
	} else {
		single_calls++;
		if (fail_single_at && single_calls == fail_single_at) {
			return -EIO;
		}
	}
	for (size_t i = 0; i < len; i++) {
		if (phase == 1) {
			assert(request_len < sizeof(request));
			request[request_len++] = tx[i];
			if (rx) {
				rx[i] = 0xff;
			}
		} else {
			assert(phase == 2 && tx[i] == 0xff && rx != NULL);
			if (response_offset < response_len) {
				rx[i] = response[response_offset++];
			} else {
				assert(fill_byte >= 0);
				rx[i] = (uint8_t)fill_byte;
			}
		}
	}
	return 0;
}

/* Fake kernel services ---------------------------------------------------*/

int nina_os_lock(void)
{
	lock_calls++;
	if (lock_result) {
		return lock_result;
	}
	lock_depth++;
	return 0;
}

void nina_os_unlock(void)
{
	assert(lock_depth > 0);
	lock_depth--;
}

void nina_os_sleep_us(uint32_t us)
{
	assert(us == 100);
	now_us += us;
}

void nina_os_sleep_ms(uint32_t ms)
{
	char e[16];

	snprintf(e, sizeof(e), "sleep%u", (unsigned int)ms);
	event(e);
	now_us += (long long)ms * 1000;
}

int64_t nina_os_uptime_ms(void)
{
	return now_us / 1000;
}

/* Helpers ----------------------------------------------------------------*/

static void reset_fake(const uint8_t *reply, size_t len)
{
	memcpy(response, reply, len);
	response_len = len;
	response_offset = request_len = phase = calls = bulk_calls = single_calls = 0;
	cs_active = fail_bulk = 0;
	fail_single_at = 0;
	fill_byte = -1;
	ready_never_from = ready_stuck_phase = -1;
	lock_calls = lock_result = 0;
	bus_ready = 1;
	now_us = 0;
	polls = 0;
	events[0] = '\0';
	initialized = true;
}

/* nina_request() with no parameters and an 8-bit answer. */
static int simple(uint8_t cmd, uint8_t *out, size_t cap, size_t *n)
{
	return nina_request(cmd, NULL, 0, false, false, out, cap, n);
}

static void zephyr_cases(void)
{
	uint8_t payload[512], output[512], id = 7;
	struct nina_param params[] = {{&id, 1}, {payload, sizeof(payload)}};
	size_t n;
	const uint8_t send_reply[] = {0xe0, 0xc4, 1, 2, 0x00, 0x02, 0xee};

	for (size_t i = 0; i < sizeof(payload); i++) {
		payload[i] = (uint8_t)i;
	}
	reset_fake(send_reply, sizeof(send_reply));
	assert(nina_request(0x44, params, 2, true, false, output, sizeof(output), &n) == 0);
	assert(n == 2 && output[0] == 0 && output[1] == 2);
	assert(request_len == 524 && (request_len % 4) == 0);
	assert(memcmp(request, "\xe0\x44\x02\x00\x01\x07\x02\x00", 8) == 0);
	assert(memcmp(request + 8, payload, sizeof(payload)) == 0);
	assert(request[520] == 0xee && request[521] == 0xff &&
	       request[522] == 0xff && request[523] == 0xff);
	assert(bulk_calls >= 1 && calls < 30 && !cs_active);

	uint8_t large[1460];
	for (size_t i = 0; i < sizeof(large); i++) {
		large[i] = (uint8_t)(i * 7);
	}
	params[1].data = large;
	params[1].len = sizeof(large);
	reset_fake(send_reply, sizeof(send_reply));
	assert(nina_request(0x44, params, 2, true, false, output, sizeof(output), &n) == 0);
	assert(request_len == 1472 && request[6] == 0x05 && request[7] == 0xb4);
	assert(memcmp(request + 8, large, sizeof(large)) == 0);
	assert(request[1468] == 0xee && request[1471] == 0xff && !cs_active);
	params[1].data = payload;
	params[1].len = sizeof(payload);

	uint8_t read_reply[519] = {0xe0, 0xc5, 1, 2, 0};
	memcpy(read_reply + 5, payload, sizeof(payload));
	read_reply[517] = 0xee;
	reset_fake(read_reply, 518);
	assert(nina_request(0x45, params, 1, true, true, output, sizeof(output), &n) == 0);
	assert(n == sizeof(payload) && memcmp(output, payload, n) == 0);
	assert(bulk_calls >= 4 && calls < 30 && !cs_active);

	const uint8_t accept_reply[] = {0xe0, 0xf5, 3, 1, 3, 4, 192, 168, 1, 2,
				       2, 0x34, 0x12, 0xee};
	reset_fake(accept_reply, sizeof(accept_reply));
	assert(nina_request(0x75, params, 1, false, false, output, sizeof(output), &n) == 0);
	assert(n == 7 && output[0] == 3 && output[1] == 192 && output[6] == 0x12);

	reset_fake(read_reply, 518);
	assert(nina_request(0x45, params, 1, true, true, output, 511, &n) == -EMSGSIZE);
	assert(!cs_active);

	const uint8_t malformed_accept[] = {0xe0, 0xf5, 3, 2, 3, 4, 0xee};
	reset_fake(malformed_accept, sizeof(malformed_accept));
	assert(nina_request(0x75, params, 1, false, false, output, sizeof(output), &n) ==
	       -EMSGSIZE);
	assert(!cs_active);

	/* recvfrom: address, port and data, each with a 16-bit length. */
	const uint8_t recvfrom_reply[] = {0xe0, 0xfa, 3, 0, 4, 10, 42, 0, 1, 0, 2, 0x12, 0x34,
					  0, 3, 'a', 'b', 'c', 0xee};
	uint8_t size[2] = {16, 0};
	struct nina_param recvfrom_params[] = {{&id, 1}, {size, 2}};
	reset_fake(recvfrom_reply, sizeof(recvfrom_reply));
	assert(nina_request(0x7a, recvfrom_params, 2, false, true, output, 6 + 16, &n) == 0);
	assert(n == 9 && output[0] == 10 && output[3] == 1 && output[4] == 0x12 &&
	       output[5] == 0x34 && !memcmp(output + 6, "abc", 3));
	assert(memcmp(request, "\xe0\x7a\x02\x01\x07\x02\x10\x00\xee", 9) == 0);
	/* An empty datagram is still three parameters. */
	const uint8_t empty_reply[] = {0xe0, 0xfa, 3, 0, 4, 10, 42, 0, 1, 0, 2, 0x12, 0x34,
				       0, 0, 0xee};
	reset_fake(empty_reply, sizeof(empty_reply));
	assert(nina_request(0x7a, recvfrom_params, 2, false, true, output, 6 + 16, &n) == 0);
	assert(n == 6);
	/* A datagram larger than the caller's buffer and a malformed address
	 * are rejected without leaving chip select asserted. */
	reset_fake(recvfrom_reply, sizeof(recvfrom_reply));
	assert(nina_request(0x7a, recvfrom_params, 2, false, true, output, 6 + 2, &n) ==
	       -EMSGSIZE);
	assert(!cs_active);
	const uint8_t bad_address[] = {0xe0, 0xfa, 3, 0, 3, 10, 42, 0, 0xee};
	reset_fake(bad_address, sizeof(bad_address));
	assert(nina_request(0x7a, recvfrom_params, 2, false, true, output, 6 + 16, &n) ==
	       -EMSGSIZE);
	assert(!cs_active);

	reset_fake(send_reply, sizeof(send_reply));
	fail_bulk = 1;
	assert(nina_request(0x44, params, 2, true, false, output, sizeof(output), &n) == -EIO);
	assert(!cs_active);
}

static void framing_cases(void)
{
	uint8_t out[64], b = 0x5a, two[2] = {1, 2};
	size_t n = 99;
	const uint8_t version_reply[] = {0xe0, 0xb7, 1, 5, '3', '.', '0', '.', '2', 0xee};

	/* "nina version": no parameters, 4 bytes, no padding. */
	reset_fake(version_reply, sizeof(version_reply));
	assert(simple(0x37, out, 15, &n) == 0);
	assert(n == 5 && memcmp(out, "3.0.2", 5) == 0);
	assert(request_len == 4 && memcmp(request, "\xe0\x37\x00\xee", 4) == 0);
	/* Every byte of a 1-byte answer parameter is read one by one except
	 * the data (one bulk read), and CS is up for exactly the two phases.
	 */
	assert(bulk_calls == 1 && single_calls == 4 + 5 && phase == 2 && !cs_active);
	assert(strcmp(events, "CS1 CS0 CS1 CS0 ") == 0);
	assert(lock_calls == 1 && lock_depth == 0);

	/* Narrow parameters: 8-bit lengths, padded with 0xff to 4 bytes. */
	const uint8_t one_reply[] = {0xe0, 0x90, 1, 1, 1, 0xee};
	struct nina_param p1[] = {{&b, 1}};
	reset_fake(one_reply, sizeof(one_reply));
	assert(nina_request(0x10, p1, 1, false, false, out, sizeof(out), &n) == 0);
	assert(n == 1 && out[0] == 1);
	assert(request_len == 8 &&
	       memcmp(request, "\xe0\x10\x01\x01\x5a\xee\xff\xff", 8) == 0);
	struct nina_param p2[] = {{&b, 1}, {two, 2}};
	reset_fake(one_reply, sizeof(one_reply));
	assert(nina_request(0x10, p2, 2, false, false, out, sizeof(out), &n) == 0);
	assert(request_len == 12 &&
	       memcmp(request, "\xe0\x10\x02\x01\x5a\x02\x01\x02\xee\xff\xff\xff", 12) == 0);
	/* An empty parameter is just its length. */
	struct nina_param p0[] = {{NULL, 0}};
	reset_fake(one_reply, sizeof(one_reply));
	assert(nina_request(0x10, p0, 1, false, false, out, sizeof(out), &n) == 0);
	assert(request_len == 8 && memcmp(request, "\xe0\x10\x01\x00\xee\xff\xff\xff", 8) == 0);
	/* Wide parameters: 16-bit lengths, high byte first; 255+1 is allowed. */
	static uint8_t big[256];
	struct nina_param pw[] = {{big, sizeof(big)}};
	reset_fake(one_reply, sizeof(one_reply));
	assert(nina_request(0x10, pw, 1, true, false, out, sizeof(out), &n) == 0);
	assert(request_len == 264 && request[3] == 1 && request[4] == 0 && request[261] == 0xee);
	/* A request that is already a multiple of 4 gets no padding. */
	uint8_t three[3] = {7, 8, 9};
	struct nina_param p3[] = {{three, 3}};
	reset_fake(one_reply, sizeof(one_reply));
	assert(nina_request(0x10, p3, 1, false, false, out, sizeof(out), &n) == 0);
	assert(request_len == 8 && memcmp(request, "\xe0\x10\x01\x03\x07\x08\x09\xee", 8) == 0);
}

static void response_cases(void)
{
	uint8_t out[64];
	size_t n = 1234;

	/* The answer may start after other bytes: up to 1000 are read. */
	uint8_t late[1000 + 9];
	memset(late, 0xff, 999);
	memcpy(late + 999, "\xe0\xb7\x01\x02\x41\x42\xee", 7);
	reset_fake(late, 999 + 7);
	assert(simple(0x37, out, 15, &n) == 0 && n == 2 && out[0] == 'A');
	/* START never comes: 1000 bytes, then -EPROTO, CS released, n untouched. */
	reset_fake(NULL, 0);
	fill_byte = 0x00;
	n = 1234;
	assert(simple(0x37, out, 15, &n) == -EPROTO);
	assert(single_calls == 4 + 1000 && !cs_active && n == 1234);
	/* A transfer error during the START search stops it with that error. */
	reset_fake(NULL, 0);
	fill_byte = 0x00;
	fail_single_at = 4 + 10;
	assert(simple(0x37, out, 15, &n) == -EIO && single_calls == 14 && !cs_active);
	/* Wrong reply code, wrong parameter count, missing END. */
	const uint8_t wrong_cmd[] = {0xe0, 0xb8, 1, 1, 'x', 0xee};
	reset_fake(wrong_cmd, sizeof(wrong_cmd));
	assert(simple(0x37, out, 15, &n) == -EPROTO && !cs_active);
	const uint8_t wrong_count[] = {0xe0, 0xb7, 2, 1, 'x', 1, 'y', 0xee};
	reset_fake(wrong_count, sizeof(wrong_count));
	assert(simple(0x37, out, 15, &n) == -EPROTO && !cs_active);
	const uint8_t no_end[] = {0xe0, 0xb7, 1, 1, 'x', 0xef};
	reset_fake(no_end, sizeof(no_end));
	assert(simple(0x37, out, 15, &n) == -EPROTO && !cs_active && n == 1234);
	/* Longer than the caller's buffer: -EMSGSIZE before the data. */
	const uint8_t too_long[] = {0xe0, 0xb7, 1, 16, 0xee};
	reset_fake(too_long, sizeof(too_long));
	assert(simple(0x37, out, 15, &n) == -EMSGSIZE && !cs_active && response_offset == 4);
	/* Exactly the buffer's size is fine. */
	uint8_t exact[] = {0xe0, 0xb7, 1, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0xee};
	reset_fake(exact, sizeof(exact));
	assert(simple(0x37, out, 15, &n) == 0 && n == 15 && out[14] == 15);
	/* An answer parameter longer than 128 bytes is read in 128-byte pieces. */
	static uint8_t long_reply[4 + 2 + 300 + 1] = {0xe0, 0xc5, 1, 0x01, 0x2c};
	static uint8_t big_out[300];
	for (int i = 0; i < 300; i++) {
		long_reply[5 + i] = (uint8_t)(i * 3);
	}
	long_reply[305] = 0xee;
	reset_fake(long_reply, 306);
	assert(nina_request(0x45, NULL, 0, false, true, big_out, sizeof(big_out), &n) == 0);
	assert(n == 300 && big_out[299] == (uint8_t)(299 * 3) && bulk_calls == 3);
	/* An empty response parameter. */
	const uint8_t empty[] = {0xe0, 0xb7, 1, 0, 0xee};
	reset_fake(empty, sizeof(empty));
	assert(simple(0x37, out, 15, &n) == 0 && n == 0 && bulk_calls == 0);

	/* 0x27 (scan results): the count comes from the answer, and each
	 * entry is stored as its length and its bytes (up to 32).
	 */
	const uint8_t scan[] = {0xe0, 0xa7, 3, 2, 'a', 'b', 0, 3, 'x', 'y', 'z', 0xee};
	reset_fake(scan, sizeof(scan));
	assert(simple(0x27, out, sizeof(out), &n) == 0);
	assert(n == 8 && memcmp(out, "\x02" "ab" "\x00" "\x03" "xyz", 8) == 0);
	const uint8_t scan_none[] = {0xe0, 0xa7, 0, 0xee};
	reset_fake(scan_none, sizeof(scan_none));
	assert(simple(0x27, out, sizeof(out), &n) == 0 && n == 0);
	uint8_t scan33[4 + 33 + 1] = {0xe0, 0xa7, 1, 33};
	scan33[37] = 0xee;
	reset_fake(scan33, sizeof(scan33));
	assert(simple(0x27, out, sizeof(out), &n) == -EMSGSIZE && !cs_active);
	/* No room for the length byte: total == cap. */
	reset_fake(scan, sizeof(scan));
	assert(simple(0x27, out, 3, &n) == -EMSGSIZE && !cs_active);
	/* Even for an empty last entry. */
	const uint8_t scan_empty_last[] = {0xe0, 0xa7, 2, 2, 'a', 'b', 0, 0xee};
	reset_fake(scan_empty_last, sizeof(scan_empty_last));
	assert(simple(0x27, out, 4, &n) == 0 && n == 4);
	reset_fake(scan_empty_last, sizeof(scan_empty_last));
	assert(simple(0x27, out, 3, &n) == -EMSGSIZE && !cs_active);
	/* Room for the length byte but not the data. */
	reset_fake(scan, sizeof(scan));
	assert(simple(0x27, out, 2, &n) == -EMSGSIZE && !cs_active);

	/* 0x75 answer lengths must be 1, 4, 2; 0x7a's first two 4 and 2. */
	const uint8_t accept_bad_port[] = {0xe0, 0xf5, 3, 1, 3, 4, 1, 2, 3, 4, 1, 9, 0xee};
	reset_fake(accept_bad_port, sizeof(accept_bad_port));
	assert(nina_request(0x75, NULL, 0, false, false, out, sizeof(out), &n) == -EMSGSIZE);
	const uint8_t accept_two[] = {0xe0, 0xf5, 2, 1, 3, 4, 1, 2, 3, 4, 0xee};
	reset_fake(accept_two, sizeof(accept_two));
	assert(nina_request(0x75, NULL, 0, false, false, out, sizeof(out), &n) == -EPROTO);
	const uint8_t recv_bad_port[] = {0xe0, 0xfa, 3, 0, 4, 1, 2, 3, 4, 0, 3, 1, 2, 3, 0, 0, 0xee};
	reset_fake(recv_bad_port, sizeof(recv_bad_port));
	assert(nina_request(0x7a, NULL, 0, false, true, out, sizeof(out), &n) == -EMSGSIZE);
	/* Any other command answering with 3 parameters is a protocol error. */
	const uint8_t three_params[] = {0xe0, 0xc5, 3, 0, 1, 1, 0, 1, 2, 0, 1, 3, 0xee};
	reset_fake(three_params, sizeof(three_params));
	assert(nina_request(0x45, NULL, 0, false, true, out, sizeof(out), &n) == -EPROTO);
}

static void argument_cases(void)
{
	uint8_t out[16], b = 1;
	size_t n;
	struct nina_param p[7] = {{&b, 1}, {&b, 1}, {&b, 1}, {&b, 1}, {&b, 1}, {&b, 1}, {&b, 1}};
	struct nina_param null_data[] = {{NULL, 1}};
	static uint8_t big[65536];
	struct nina_param narrow256[] = {{big, 256}};
	struct nina_param wide65536[] = {{big, 65536}};

	reset_fake(NULL, 0);
	assert(nina_request(0x10, p, 7, false, false, out, sizeof(out), &n) == -EINVAL);
	assert(nina_request(0x10, NULL, 0, false, false, NULL, sizeof(out), &n) == -EINVAL);
	assert(nina_request(0x10, NULL, 0, false, false, out, sizeof(out), NULL) == -EINVAL);
	assert(nina_request(0x10, null_data, 1, false, false, out, sizeof(out), &n) == -EINVAL);
	assert(nina_request(0x10, narrow256, 1, false, false, out, sizeof(out), &n) == -EINVAL);
	assert(nina_request(0x10, wide65536, 1, true, false, out, sizeof(out), &n) == -EINVAL);
	/* Checked before the lock, the reset and any line. */
	assert(lock_calls == 0 && phase == 0 && polls == 0 && events[0] == '\0');
	/* Six parameters are allowed. */
	const uint8_t reply[] = {0xe0, 0x90, 1, 0, 0xee};
	reset_fake(reply, sizeof(reply));
	assert(nina_request(0x10, p, 6, false, false, out, sizeof(out), &n) == 0);
	assert(request_len == 3 + 12 + 1 && request[2] == 6);
}

static void ready_cases(void)
{
	uint8_t out[16];
	size_t n = 77;
	const uint8_t reply[] = {0xe0, 0xb7, 1, 1, 'x', 0xee};

	/* The module never gets ready: 2000 ms of polling every 100 us, CS
	 * never asserted.
	 */
	reset_fake(reply, sizeof(reply));
	ready_never_from = 0;
	assert(simple(0x37, out, 15, &n) == -ETIMEDOUT);
	assert(polls == 20000 && now_us == 2000000 && phase == 0 && events[0] == '\0');
	assert(lock_depth == 0 && n == 77);
	/* It does not answer CS by releasing READY: 20 ms, nothing sent, CS
	 * released.
	 */
	reset_fake(reply, sizeof(reply));
	ready_stuck_phase = 1;
	assert(simple(0x37, out, 15, &n) == -ETIMEDOUT);
	assert(polls == 1 + 200 && now_us == 20000 && calls == 0 && !cs_active);
	assert(strcmp(events, "CS1 CS0 ") == 0);
	/* It never gets ready to answer: 10000 ms after the request. */
	reset_fake(reply, sizeof(reply));
	ready_never_from = 1;
	assert(simple(0x37, out, 15, &n) == -ETIMEDOUT);
	assert(now_us == 10000000 && request_len == 4 && !cs_active);
	assert(strcmp(events, "CS1 CS0 ") == 0);
	/* It does not release READY for the answer: 20 ms, nothing read. */
	reset_fake(reply, sizeof(reply));
	ready_stuck_phase = 2;
	assert(simple(0x37, out, 15, &n) == -ETIMEDOUT);
	assert(now_us == 20000 && response_offset == 0 && !cs_active);
	assert(strcmp(events, "CS1 CS0 CS1 CS0 ") == 0);
	/* A ready module costs one poll per wait and no sleep. */
	reset_fake(reply, sizeof(reply));
	assert(simple(0x37, out, 15, &n) == 0 && polls == 4 && now_us == 0);
}

static void init_cases(void)
{
	uint8_t out[16];
	size_t n;
	const uint8_t reply[] = {0xe0, 0xb7, 1, 1, 'x', 0xee};

	/* The first request resets the module: the lines as nina_wifi.c's
	 * init_nina() sets them, 10 ms in reset, 750 ms to boot, then IRQ
	 * low and an input.
	 */
	reset_fake(reply, sizeof(reply));
	initialized = false;
	assert(simple(0x37, out, 15, &n) == 0 && n == 1);
	assert(strcmp(events,
		      "CS=inactive READY=input IRQ=high RESET=active sleep10 RESET=0 "
		      "sleep750 IRQ=0 IRQ=input CS1 CS0 CS1 CS0 ") == 0);
	assert(now_us == 760000 && initialized);
	/* Only once. */
	reset_fake(reply, sizeof(reply));
	assert(simple(0x37, out, 15, &n) == 0 && strcmp(events, "CS1 CS0 CS1 CS0 ") == 0);
	/* No bus: -ENODEV, nothing touched, and the next request tries again. */
	reset_fake(reply, sizeof(reply));
	initialized = false;
	bus_ready = 0;
	assert(simple(0x37, out, 15, &n) == -ENODEV && events[0] == '\0' && !initialized);
	assert(lock_depth == 0);
	/* A failing lock: its error, nothing touched. */
	reset_fake(reply, sizeof(reply));
	lock_result = -ECANCELED;
	assert(simple(0x37, out, 15, &n) == -ECANCELED && events[0] == '\0' && polls == 0);
}

static void ip_config_cases(void)
{
	uint8_t data[12];
	const uint8_t reply[] = {0xe0, 0xa1, 3, 4, 10, 42, 0, 7, 4, 255, 255, 255, 0,
				 4, 10, 42, 0, 1, 0xee};

	reset_fake(reply, sizeof(reply));
	assert(nina_ip_config(data) == 0);
	assert(memcmp(data, "\x0a\x2a\x00\x07\xff\xff\xff\x00\x0a\x2a\x00\x01", 12) == 0);
	assert(request_len == 8 && memcmp(request, "\xe0\x21\x01\x01\xff\xee\xff\xff", 8) == 0);
	assert(lock_calls == 1 && lock_depth == 0 && !cs_active);
	/* The request goes out in one bulk write. */
	assert(strcmp(events, "CS1 CS0 CS1 CS0 ") == 0);
	/* Wrong count, wrong length, wrong code, missing END. */
	const uint8_t one[] = {0xe0, 0xa1, 1, 4, 1, 2, 3, 4, 0xee};
	reset_fake(one, sizeof(one));
	assert(nina_ip_config(data) == -EPROTO && !cs_active);
	const uint8_t len3[] = {0xe0, 0xa1, 3, 3, 1, 2, 3, 0xee};
	reset_fake(len3, sizeof(len3));
	assert(nina_ip_config(data) == -EMSGSIZE && !cs_active);
	const uint8_t code[] = {0xe0, 0xa2, 3, 0xee};
	reset_fake(code, sizeof(code));
	assert(nina_ip_config(data) == -EPROTO && !cs_active);
	uint8_t no_end[sizeof(reply)];
	memcpy(no_end, reply, sizeof(reply));
	no_end[sizeof(reply) - 1] = 0;
	reset_fake(no_end, sizeof(no_end));
	assert(nina_ip_config(data) == -EPROTO && !cs_active);
	/* command()'s START search gives up after 1000 bytes too. */
	reset_fake(NULL, 0);
	fill_byte = 0;
	assert(nina_ip_config(data) == -EPROTO && single_calls == 1000);
	/* READY timeouts as in nina_request(), 2000 ms for the answer. */
	reset_fake(reply, sizeof(reply));
	ready_never_from = 0;
	assert(nina_ip_config(data) == -ETIMEDOUT && now_us == 2000000 && phase == 0);
	reset_fake(reply, sizeof(reply));
	ready_never_from = 1;
	assert(nina_ip_config(data) == -ETIMEDOUT && now_us == 2000000 && !cs_active);
	reset_fake(reply, sizeof(reply));
	ready_stuck_phase = 1;
	assert(nina_ip_config(data) == -ETIMEDOUT && now_us == 20000 && calls == 0 && !cs_active);
}

int main(void)
{
	zephyr_cases();
	framing_cases();
	response_cases();
	argument_cases();
	ready_cases();
	init_cases();
	ip_config_cases();
	puts("NINA SPI framing, answers, timeouts and reset tests passed");
	return 0;
}
