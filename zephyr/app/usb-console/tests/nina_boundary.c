/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of this build's side of the shared NINA core: the SPI link
 * (common/drivers/nina/nina_spi.c, which the test includes) on the board's
 * boundary drivers/nina/nina_bus.c and the kernel's drivers/nina/nina_os.c,
 * built on fakes of the Zephyr calls (tests/nina_fakes), whose
 * registration of the callbacks fails or succeeds as the test says.
 *
 * - Framing and bulk transfers of nina_request() (the tests of the former
 *   drivers/nina/nina_wifi.c), through spi_transceive() and spi_write().
 * - The lines of the devicetree and the flags: the reset sequence, CS on
 *   GPIO9 around each half, a bus that is not ready.
 * - The waits for READY: sampled every 100 us when nina_os_ready_start()
 *   fails (the callback cannot be added), or on the edge interrupt of
 *   GPIO10 when it succeeds (the callback added once per reset of the
 *   core, for GPIO10 only; the semaphore reset before the edge is enabled;
 *   the edge towards the level waited for; the edge disabled again before
 *   any SPI transfer; waits of at most 10 ms; the same 2000 ms deadline).
 * - The socket events on the IRQ line (GPIO2): a start that fails (the
 *   callback cannot be added; the edge cannot be enabled, the callback
 *   removed again), the start (the callback for GPIO2 only, its rising
 *   edge), the count of the
 *   edges, a wait that ends at an edge (k_sem_reset() waking the waiter,
 *   one count left by k_sem_give()), at once when the count moved, at the
 *   end of its time without an edge, and an edge between the caller's
 *   read of the count and its k_sem_take() (the count left ends it at
 *   once); a count left from an earlier edge does not end a later wait.
 * - With CONFIG_NINA_PASSTHROUGH (as the build has it), the NINA
 *   passthrough mode (drivers/nina/nina_passthrough.h): while it is
 *   active the bus is never ready, so a request is -ENODEV before any line
 *   or SPI call, the module is not reset, and the link works again once
 *   the mode is gone.
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "nina_spi.c"
#include "nina_test.h"

#define PIN_CS    9
#define PIN_READY 10
#define PIN_RESET 3
#define PIN_IRQ   2

struct device fake_device;
static uint8_t request[2048], response[2048];
static size_t request_len, response_len, response_offset;
static unsigned int phase, calls, bulk_calls;
static int cs_active, fail_bulk;
static bool bus_ready = true;
#ifdef CONFIG_NINA_PASSTHROUGH
static bool passthrough;

bool nina_passthrough_active(void)
{
	return passthrough;
}
#endif

/* The clock (microseconds) and the module's READY: "busy" is how many more
 * samples (or, with the interrupt, waits) READY stays away from the level
 * the core waits for; -1 never. */
static int64_t now_us;
static int busy;
static unsigned int sleeps, takes, resets, adds, irq_enables;
static int64_t longest_take_us;
static gpio_flags_t irq_mode = GPIO_INT_DISABLE;
static struct gpio_callback *callback;
static struct k_sem *ready_sem_seen;
static int ready_add_result; /* gpio_add_callback_dt() for READY */
/* The IRQ line: its callback and edge, the semaphore its interrupt uses,
 * the planned edge (microseconds; -1 none), an edge to come right before
 * the next k_sem_take(). */
static gpio_flags_t event_mode = GPIO_INT_DISABLE;
static struct gpio_callback *event_callback;
static struct k_sem *event_sem_seen;
static bool in_event_isr, edge_before_take;
static int64_t edge_at_us = -1;
static unsigned int event_takes, event_removes;
static int event_add_result, event_irq_result; /* the start's two calls */

static void fire_edge(void)
{
	in_event_isr = true;
	event_callback->handler(&fake_device, event_callback, BIT(PIN_IRQ));
	in_event_isr = false;
}

/* What the board's calls did, in order. */
static char trace[64][24];
static unsigned int trace_len;
static bool tracing;

static void note(const char *fmt, int a, long b)
{
	if (tracing) {
		assert(trace_len < 64);
		snprintf(trace[trace_len++], sizeof(trace[0]), fmt, a, b);
	}
}

bool device_is_ready(const struct device *dev)
{
	assert(dev == &fake_device);
	return bus_ready;
}

bool gpio_is_ready_dt(const struct gpio_dt_spec *spec)
{
	assert(spec->port == &fake_device);
	return true;
}

int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, gpio_flags_t flags)
{
	note("cfg %d %lx", spec->pin, (long)flags);
	return 0;
}

int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value)
{
	note("set %d %ld", spec->pin, value);
	if (spec->pin == PIN_CS) {
		cs_active = value;
		if (value) {
			phase++;
		}
	} else {
		assert(spec->pin == PIN_RESET || spec->pin == PIN_IRQ);
	}
	return 0;
}

/* READY (active low on the board, logical here): the module is ready while
 * CS is inactive and busy while CS is active, unless "busy" says
 * otherwise. */
int gpio_pin_get_dt(const struct gpio_dt_spec *spec)
{
	int level = !cs_active;

	assert(spec->pin == PIN_READY);
	if (busy == 0) {
		return level;
	}
	if (!ready_irq && busy > 0) {
		busy--;
	}
	return !level;
}

int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *spec, gpio_flags_t flags)
{
	if (spec->pin == PIN_IRQ) {
		assert(flags == GPIO_INT_EDGE_TO_ACTIVE && event_callback);
		if (event_irq_result) {
			return event_irq_result;
		}
		event_mode = flags;
		return 0;
	}
	assert(spec->pin == PIN_READY && ready_irq);
	assert(flags == GPIO_INT_DISABLE || flags == GPIO_INT_EDGE_TO_ACTIVE ||
	       flags == GPIO_INT_EDGE_TO_INACTIVE);
	if (flags != GPIO_INT_DISABLE) {
		/* The semaphore was reset just before. */
		assert(ready_sem_seen && ready_sem_seen->count == 0);
		irq_enables++;
	}
	irq_mode = flags;
	return 0;
}

int gpio_add_callback_dt(const struct gpio_dt_spec *spec, struct gpio_callback *cb)
{
	if (spec->pin == PIN_IRQ) {
		assert(cb->pin_mask == BIT(PIN_IRQ) && cb->handler && !event_callback);
		if (event_add_result) {
			return event_add_result;
		}
		event_callback = cb;
		return 0;
	}
	assert(spec->pin == PIN_READY && cb->pin_mask == BIT(PIN_READY) && cb->handler);
	adds++;
	if (ready_add_result) {
		return ready_add_result;
	}
	callback = cb;
	return 0;
}

int gpio_remove_callback_dt(const struct gpio_dt_spec *spec, struct gpio_callback *cb)
{
	assert(spec->pin == PIN_IRQ && cb == event_callback);
	event_callback = NULL;
	event_removes++;
	return 0;
}

static void check_transfer(void)
{
	/* No edge interrupt while CS toggles READY. */
	assert(irq_mode == GPIO_INT_DISABLE);
	assert(cs_active);
}

int spi_transceive(const struct device *dev, const struct spi_config *cfg,
		   const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	const uint8_t *src = tx->buffers[0].buf;
	uint8_t *dst = rx->buffers[0].buf;
	size_t len = tx->buffers[0].len;

	assert(dev == &fake_device && cfg->frequency == 8000000 && cfg->operation == 8);
	check_transfer();
	assert(tx->count == 1 && rx->count == 1);
	assert(len == rx->buffers[0].len);
	calls++;
	if (len > 1) {
		bulk_calls++;
		if (fail_bulk) {
			return -EIO;
		}
	}
	for (size_t i = 0; i < len; i++) {
		if (phase == 1) {
			assert(request_len < sizeof(request));
			request[request_len++] = src[i];
			dst[i] = 0xff;
		} else {
			assert(phase == 2 && src[i] == 0xff);
			assert(response_offset < response_len);
			dst[i] = response[response_offset++];
		}
	}
	return 0;
}

int spi_write(const struct device *dev, const struct spi_config *cfg, const struct spi_buf_set *tx)
{
	const struct spi_buf *buf = &tx->buffers[0];

	assert(dev == &fake_device && cfg->frequency == 8000000);
	check_transfer();
	assert(phase == 1 && tx->count == 1);
	calls++;
	bulk_calls++;
	if (fail_bulk) {
		return -EIO;
	}
	assert(buf->len <= sizeof(request) - request_len);
	memcpy(request + request_len, buf->buf, buf->len);
	request_len += buf->len;
	return 0;
}

int k_mutex_lock(struct k_mutex *lock, k_timeout_t timeout)
{
	(void)lock;
	assert(timeout.us == -1);
	return 0;
}

int k_mutex_unlock(struct k_mutex *lock)
{
	(void)lock;
	return 0;
}

int32_t k_sleep(k_timeout_t timeout)
{
	note("sleep %d %ld", 0, (long)timeout.us);
	/* With the edge interrupt of READY, only the reset sequence sleeps. */
	assert((!ready_irq && timeout.us == 100) || timeout.us == 10000 || timeout.us == 750000);
	sleeps++;
	now_us += timeout.us;
	return 0;
}

int64_t k_uptime_get(void)
{
	return now_us / 1000;
}

void k_sem_reset(struct k_sem *sem)
{
	if (in_event_isr) {
		assert(!event_sem_seen || event_sem_seen == sem);
		event_sem_seen = sem;
		sem->count = 0;
		return;
	}
	ready_sem_seen = sem;
	sem->count = 0;
	resets++;
}

void k_sem_give(struct k_sem *sem)
{
	if (sem->count < sem->limit) {
		sem->count++;
	}
}

/* The wait: the edge comes when "busy" runs out, if the interrupt is
 * enabled for it (the GPIO driver's handler runs the callback). */
int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	/* The events' wait: a count left is taken at once; else the planned
	 * edge within the timeout runs the interrupt, whose k_sem_reset()
	 * wakes this waiter with -EAGAIN (its k_sem_give() leaves a count);
	 * else the time runs out. */
	if (event_sem_seen && sem == event_sem_seen) {
		assert(timeout.us >= 0);
		event_takes++;
		if (edge_before_take) {
			edge_before_take = false;
			fire_edge();
		}
		if (sem->count) {
			sem->count--;
			return 0;
		}
		if (edge_at_us >= 0 && edge_at_us <= now_us + timeout.us) {
			now_us = edge_at_us > now_us ? edge_at_us : now_us;
			edge_at_us = -1;
			fire_edge();
			return -EAGAIN;
		}
		now_us += timeout.us;
		return -EAGAIN;
	}
	assert(sem == ready_sem_seen && ready_irq);
	assert(timeout.us > 0 && timeout.us <= 10000);
	assert(irq_mode == (cs_active ? GPIO_INT_EDGE_TO_INACTIVE : GPIO_INT_EDGE_TO_ACTIVE));
	takes++;
	if (timeout.us > longest_take_us) {
		longest_take_us = timeout.us;
	}
	if (!sem->count && busy > 0 && --busy == 0) {
		callback->handler(&fake_device, callback, BIT(PIN_READY));
	}
	if (sem->count) {
		sem->count--;
		now_us += 1;
		return 0;
	}
	now_us += timeout.us;
	return -EAGAIN;
}

static void reset_fake(const uint8_t *reply, size_t len)
{
	memcpy(response, reply, len);
	response_len = len;
	response_offset = request_len = phase = calls = bulk_calls = 0;
	cs_active = fail_bulk = 0;
	busy = 0;
	initialized = true;
}

static void test_framing(void)
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
	assert(request[520] == 0xee && request[521] == 0xff && request[522] == 0xff &&
	       request[523] == 0xff);
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

	const uint8_t accept_reply[] = {0xe0, 0xf5, 3, 1, 3,    4,    192,
					168,  1,    2, 2, 0x34, 0x12, 0xee};
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
	const uint8_t recvfrom_reply[] = {0xe0, 0xfa, 3,    0, 4, 10,  42,  0,   1,   0,
					  2,    0x12, 0x34, 0, 3, 'a', 'b', 'c', 0xee};
	uint8_t size[2] = {16, 0};
	struct nina_param recvfrom_params[] = {{&id, 1}, {size, 2}};
	reset_fake(recvfrom_reply, sizeof(recvfrom_reply));
	assert(nina_request(0x7a, recvfrom_params, 2, false, true, output, 6 + 16, &n) == 0);
	assert(n == 9 && output[0] == 10 && output[3] == 1 && output[4] == 0x12 &&
	       output[5] == 0x34 && !memcmp(output + 6, "abc", 3));
	assert(memcmp(request, "\xe0\x7a\x02\x01\x07\x02\x10\x00\xee", 9) == 0);
	/* An empty datagram is still three parameters. */
	const uint8_t empty_reply[] = {0xe0, 0xfa, 3, 0,    4,    10, 42, 0,
				       1,    0,    2, 0x12, 0x34, 0,  0,  0xee};
	reset_fake(empty_reply, sizeof(empty_reply));
	assert(nina_request(0x7a, recvfrom_params, 2, false, true, output, 6 + 16, &n) == 0);
	assert(n == 6);
	/* A datagram larger than the caller's buffer and a malformed address
	 * are rejected without leaving chip select asserted. */
	reset_fake(recvfrom_reply, sizeof(recvfrom_reply));
	assert(nina_request(0x7a, recvfrom_params, 2, false, true, output, 6 + 2, &n) == -EMSGSIZE);
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

/* The first request resets the module with the devicetree's lines, then
 * starts the waits for READY: READY_RESULT is what adding its callback
 * gives, and chooses how the waits wait.  A bus that is not ready is
 * -ENODEV before any line is touched. */
static void test_reset(int ready_result)
{
	const uint8_t reply[] = {0xe0, 0xb7, 1, 5, '3', '.', '0', '.', '2', 0xee};
	static const char *const expected[] = {
		"cfg 9 11", "cfg 10 14",      "cfg 2 13", "cfg 3 12", "sleep 0 10000",
		"set 3 0",  "sleep 0 750000", "set 2 0",  "cfg 2 14", "set 9 1",
		"set 9 0",  "set 9 1",        "set 9 0",
	};
	uint8_t version[16];
	size_t n;

	reset_fake(reply, sizeof(reply));
	initialized = false;
	ready_irq = ready_result != 0; /* the opposite: set by the start */
	ready_add_result = ready_result;
	adds = 0;
	bus_ready = false;
	trace_len = 0;
	tracing = true;
	assert(nina_request(0x37, NULL, 0, false, false, version, sizeof(version), &n) == -ENODEV);
	assert(trace_len == 0 && !initialized && adds == 0);

	bus_ready = true;
	assert(nina_request(0x37, NULL, 0, false, false, version, sizeof(version), &n) == 0);
	tracing = false;
	assert(n == 5 && !memcmp(version, "3.0.2", 5) && initialized);
	assert(adds == 1 && ready_irq == (ready_result == 0));
	assert(trace_len == sizeof(expected) / sizeof(expected[0]));
	for (unsigned int i = 0; i < trace_len; i++) {
		assert(!strcmp(trace[i], expected[i]));
	}
}

#ifdef CONFIG_NINA_PASSTHROUGH
/* The passthrough mode owns GPIO8, GPIO9 and the module's lines: no bus. */
static void test_passthrough(void)
{
	const uint8_t reply[] = {0xe0, 0xb7, 1, 5, '3', '.', '0', '.', '2', 0xee};
	unsigned int sleeps_before;
	uint8_t version[16];
	size_t n;

	reset_fake(reply, sizeof(reply));
	initialized = false;
	passthrough = true;
	sleeps_before = sleeps;
	trace_len = 0;
	tracing = true;
	assert(!nina_bus_ready());
	assert(nina_request(0x37, NULL, 0, false, false, version, sizeof(version), &n) == -ENODEV);
	assert(nina_request(0x37, NULL, 0, false, false, version, sizeof(version), &n) == -ENODEV);
	assert(trace_len == 0 && !initialized && calls == 0 && sleeps == sleeps_before);

	passthrough = false;
	assert(nina_bus_ready());
	assert(nina_request(0x37, NULL, 0, false, false, version, sizeof(version), &n) == 0);
	tracing = false;
	assert(n == 5 && !memcmp(version, "3.0.2", 5) && initialized);
}
#endif

/* The module answers late: READY stays busy for a while before each half,
 * then for good (the 2000 ms deadline of the first wait). */
static void test_ready_waits(void)
{
	const uint8_t reply[] = {0xe0, 0xb7, 1, 1, 'x', 0xee};
	uint8_t out[8];
	size_t n;
	int64_t start;

	reset_fake(reply, sizeof(reply));
	busy = 5;
	sleeps = takes = 0;
	start = now_us;
	assert(nina_request(0x37, NULL, 0, false, false, out, sizeof(out), &n) == 0);
	assert(n == 1 && out[0] == 'x' && busy == 0);
	if (ready_irq) {
		/* Five waits of the first wait_ready(), the fifth ends on the edge. */
		assert(takes == 5 && sleeps == 0 && longest_take_us == 10000);
		assert(now_us - start == 4 * 10000 + 1);
	} else {
		assert(sleeps == 5 && takes == 0 && now_us - start == 5 * 100);
	}

	/* Never ready: -ETIMEDOUT after 2000 ms, CS never asserted. */
	reset_fake(reply, sizeof(reply));
	now_us = (now_us / 1000 + 1) * 1000;
	busy = -1;
	sleeps = takes = calls = 0;
	start = now_us;
	assert(nina_request(0x37, NULL, 0, false, false, out, sizeof(out), &n) == -ETIMEDOUT);
	assert(phase == 0 && calls == 0);
	if (ready_irq) {
		assert(takes == 200 && sleeps == 0);
		assert(now_us - start >= 2000 * 1000 && now_us - start <= 2010 * 1000);
		assert(irq_mode == GPIO_INT_DISABLE);
	} else {
		assert(sleeps == 20000 && takes == 0);
		assert(now_us - start >= 2000 * 1000 && now_us - start <= 2001 * 1000);
	}
	busy = 0;

	if (!ready_irq) {
		return;
	}
	/* A wake-up left from before (an edge after the last wait) is dropped
	 * by the next arm; the callback was added once. */
	callback->handler(&fake_device, callback, BIT(PIN_READY));
	assert(ready_sem_seen->count == 1);
	reset_fake(reply, sizeof(reply));
	busy = 2;
	takes = 0;
	assert(nina_request(0x37, NULL, 0, false, false, out, sizeof(out), &n) == 0);
	assert(takes == 2 && adds == 1 && resets == irq_enables && resets > 0);
	assert(irq_mode == GPIO_INT_DISABLE);
}

static void test_events(void)
{
	uint32_t seq;
	int64_t start;

	/* A start that fails: the callback cannot be added, or the edge cannot
	 * be enabled (the callback is removed again).  Its error. */
	event_add_result = -ENOTSUP;
	assert(nina_os_event_start() == -ENOTSUP);
	assert(!event_callback && event_mode == GPIO_INT_DISABLE && event_removes == 0);
	event_add_result = 0;
	event_irq_result = -EIO;
	assert(nina_os_event_start() == -EIO);
	assert(!event_callback && event_mode == GPIO_INT_DISABLE && event_removes == 1);
	event_irq_result = 0;

	/* The start: GPIO2's callback, then its rising edge. */
	assert(nina_os_event_start() == 0);
	assert(event_callback && event_mode == GPIO_INT_EDGE_TO_ACTIVE && event_removes == 1);
	/* The first edge names the semaphore (the fake learns it). */
	seq = nina_os_event_seq();
	fire_edge();
	assert(nina_os_event_seq() == seq + 1 && event_sem_seen && event_sem_seen->count == 1);

	/* The count moved since SEQ: at once, no take. */
	event_takes = 0;
	start = now_us;
	nina_os_event_wait(seq, 100);
	assert(event_takes == 0 && now_us == start);

	/* A wait for the next edge, which comes 7 ms later: the count left by
	 * the first edge is taken, then the wait ends at the edge. */
	seq = nina_os_event_seq();
	now_us = (now_us / 1000 + 1) * 1000;
	start = now_us;
	edge_at_us = start + 7000;
	event_takes = 0;
	nina_os_event_wait(seq, 100);
	assert(nina_os_event_seq() == seq + 1 && now_us == start + 7000 && event_takes == 2);

	/* No edge: the time (100 ms), the count left taken first. */
	seq = nina_os_event_seq();
	start = now_us;
	event_takes = 0;
	nina_os_event_wait(seq, 100);
	assert(nina_os_event_seq() == seq && event_takes == 2);
	assert(now_us - start >= 100 * 1000 && now_us - start <= 101 * 1000);
	assert(event_sem_seen->count == 0);

	/* An edge after the caller read the count, before its k_sem_take():
	 * the count it leaves ends the wait at once. */
	seq = nina_os_event_seq();
	start = now_us;
	edge_before_take = true;
	event_takes = 0;
	nina_os_event_wait(seq, 100);
	assert(nina_os_event_seq() == seq + 1 && now_us == start && event_takes == 1);

	/* 0 ms: no wait. */
	seq = nina_os_event_seq();
	event_takes = 0;
	nina_os_event_wait(seq, 0);
	assert(event_takes == 0);
}

int main(void)
{
	/* nina_os_ready_start() fails: READY is sampled. */
	test_framing();
	test_reset(-ENOTSUP);
#ifdef CONFIG_NINA_PASSTHROUGH
	test_passthrough();
#endif
	test_ready_waits();
	/* It succeeds: the edge interrupt of READY. */
	test_reset(0);
	test_framing();
	test_ready_waits();
	test_events();
	puts("NINA SPI link on the Zephyr boundary: tests passed");
	return 0;
}
