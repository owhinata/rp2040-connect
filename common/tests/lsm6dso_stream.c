/* SPDX-License-Identifier: Apache-2.0 */
/* The FIFO stream's state machine (common/drivers/lsm6dso/lsm6dso_stream.c) on the host's part of
 * its boundaries (tests/lsm6dso_os_host: the OS's work item, INT1 line and clock, and a request
 * of its user that this test resolves), with a fake core: the FIFO calls of lsm6dso_core.h,
 * defined here, over a fake FIFO.
 *
 * - The start at the first run: no INT1 line (-ENODEV, nothing started), the FIFO with the
 *   watermark and both sensors, INT1 enabled and routed; a level already active at the start
 *   runs the work again; a failure to enable INT1 or to read its level stops what was started.
 * - The runs: an edge of INT1 runs the work; a batch at the watermark is delivered with its
 *   header (words, watermark flag, periods, gains, conversions, first time stamps), and the
 *   request goes on; below the watermark nothing is delivered and the work polls again in 20 ms;
 *   an overrun ends the request with -EOVERFLOW; a request ended by its user (canceled: the
 *   stream stops and ends it with -ECANCELED; gone: the stream stops).
 * - The graceful stop: a stop that arrives during a full FIFO read delivers that batch, then the
 *   final partial batch, then bypass mode and -ECANCELED; a pause failure, a buffer failure and
 *   an overrun end it with their error; an empty FIFO ends it without a batch; a stop before the
 *   first run starts nothing.  Another request is refused (-EBUSY), a stop of another request too
 *   (-ENOENT).
 * - The watermark: its bounds, and no change while a request is served.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lsm6dso_cal.h"
#include "lsm6dso_fifo_packet.h"
#include "lsm6dso_stream.h"

struct lsm6dso_stream_req_host {
	bool canceled;
};

static struct lsm6dso core;
static struct lsm6dso_stream stream;
static struct lsm6dso_stream_req_host request;
static struct lsm6dso_stream_req_host *live_req;
static const lsm6dso_stream_req_t req7 = {7};
static unsigned int fifo_words, injected_tail, pause_calls, stop_calls, start_calls, completions;
static unsigned int delivered_words;
static uint16_t start_watermark;
static bool batching_paused, inject_stop, fail_pause, fail_alloc, overrun, int1_routed;
static int terminal_error, route_error;
static union {
	uint64_t align;
	uint8_t bytes[4096];
} receive_buffer;

/* The core's FIFO. */

int lsm6dso_fifo_start(struct lsm6dso *c, uint16_t watermark, bool accel, bool gyro)
{
	assert(c == &core && accel && gyro);
	start_calls++;
	start_watermark = watermark;
	return 0;
}

int lsm6dso_fifo_stop(struct lsm6dso *c)
{
	(void)c;
	stop_calls++;
	return 0;
}

void lsm6dso_fifo_record_stop_error(struct lsm6dso *c, int error)
{
	(void)c;
	(void)error;
}

int lsm6dso_fifo_route_int1(struct lsm6dso *c, bool enabled)
{
	(void)c;
	if (route_error != 0) {
		return route_error;
	}
	int1_routed = enabled;
	return 0;
}

int lsm6dso_fifo_pause_batch(struct lsm6dso *c)
{
	(void)c;
	pause_calls++;
	if (fail_pause) {
		return -EIO;
	}
	batching_paused = true;
	return 0;
}

int lsm6dso_fifo_status(struct lsm6dso *c, struct lsm6dso_fifo_status *status)
{
	(void)c;
	*status = (struct lsm6dso_fifo_status){
		.words = fifo_words, .watermark = fifo_words >= 32, .overrun = overrun};
	return 0;
}

int lsm6dso_fifo_read(struct lsm6dso *c, uint8_t *out, size_t capacity,
		      struct lsm6dso_fifo_status *status)
{
	(void)c;
	assert(capacity >= fifo_words);
	status->words = fifo_words;
	status->watermark = fifo_words >= 32;
	status->overrun = false;
	status->overrun_latched = false;
	for (unsigned int i = 0; i < fifo_words; i++) {
		memset(out + i * 7, 0, 7);
		out[i * 7] = i % 2 ? 0x08 : 0x10;
	}
	fifo_words = injected_tail;
	injected_tail = 0;
	if (inject_stop) {
		inject_stop = false;
		assert(lsm6dso_stream_request_stop(&stream, &req7) == 0);
	}
	return 0;
}

void lsm6dso_track_gyro_bias(struct lsm6dso *c, const uint8_t *words, size_t len)
{
	(void)c;
	(void)words;
	(void)len;
}

void lsm6dso_copy_cal(struct lsm6dso *c, struct lsm6dso_cal *accel, struct lsm6dso_cal *gyro)
{
	(void)c;
	lsm6dso_cal_nominal(accel, LSM6DSO_CAL_ACCEL, 244);
	lsm6dso_cal_nominal(gyro, LSM6DSO_CAL_GYRO, 70000);
}

/* The user's request: number 7. */

lsm6dso_stream_req_live_t *lsm6dso_stream_req_host_get(int id)
{
	return id == 7 ? live_req : NULL;
}

bool lsm6dso_stream_req_host_canceled(lsm6dso_stream_req_live_t *live)
{
	return live->canceled;
}

int lsm6dso_stream_req_host_buffer(lsm6dso_stream_req_live_t *live, uint32_t min_len,
				   uint32_t max_len, uint8_t **buf, uint32_t *len)
{
	(void)live;
	assert(max_len == sizeof(struct lsm6dso_fifo_packet) + 512 * 7);
	if (fail_alloc) {
		return -ENOMEM;
	}
	assert(min_len <= sizeof(receive_buffer.bytes));
	*buf = receive_buffer.bytes;
	*len = sizeof(receive_buffer.bytes);
	return 0;
}

void lsm6dso_stream_req_host_done(lsm6dso_stream_req_live_t *live)
{
	struct lsm6dso_fifo_packet *p = (void *)receive_buffer.bytes;

	assert(live == live_req && p->marker == LSM6DSO_FIFO_PACKET_MARKER);
	completions++;
	delivered_words += p->words;
	/* RTIO resubmits a multishot SQE before reporting the completion. */
	assert(lsm6dso_stream_submit(&stream, &req7) == 0);
}

void lsm6dso_stream_req_host_error(lsm6dso_stream_req_live_t *live, int error)
{
	assert(live == live_req);
	terminal_error = error;
	live_req = NULL;
}

/* A new stream with request 7 submitted, its INT1 line ready. */
static void start_stream(bool active)
{
	memset(&stream, 0, sizeof(stream));
	lsm6dso_stream_init(&stream, &core);
	assert(stream.watermark == 32 && !stream.requested);
	stream.int1.ready = true;
	stream.active = active;
	request = (struct lsm6dso_stream_req_host){0};
	live_req = &request;
	terminal_error = 0;
	assert(lsm6dso_stream_submit(&stream, &req7) == 0);
	assert(stream.requested && stream.work.submitted == 1);
}

static void run(void)
{
	stream.work.pending = false;
	lsm6dso_stream_run(&stream.work);
}

static void test_start(void)
{
	const struct lsm6dso_fifo_packet *p = (const void *)receive_buffer.bytes;
	unsigned int stops;

	/* No INT1 line: nothing starts; the stop that follows any failure. */
	start_stream(false);
	stream.int1.ready = false;
	stops = stop_calls;
	run();
	assert(terminal_error == -ENODEV && live_req == NULL && !stream.requested);
	assert(start_calls == 0 && stream.int1.enables == 0 && stop_calls == stops + 1);

	/* The start: the FIFO, INT1 enabled and routed; nothing to read yet. */
	start_stream(false);
	run();
	assert(start_calls == 1 && start_watermark == 32);
	assert(stream.active && stream.irq_active && stream.int1.enabled && int1_routed);
	assert(stream.work.submitted == 1 && stream.work.scheduled == 1 &&
	       stream.work.delay_ms == 20);

	/* An edge runs the work; a batch at the watermark. */
	assert(lsm6dso_stream_int1(&stream.int1) == 0 && stream.work.submitted == 2);
	fifo_words = 32;
	completions = delivered_words = 0;
	run();
	assert(completions == 1 && delivered_words == 32 && live_req == &request);
	assert(p->words == 32 && p->watermark == 1);
	assert(p->accel_period_ns == 1000000000U / 208 && p->gyro_period_ns == 1000000000U / 208);
	assert(p->accel_gain_ug == 244 && p->gyro_gain_udps == 70000);
	assert(p->accel_first_timestamp_ns == 1000000000ULL - 15ULL * (1000000000U / 208));
	assert(p->gyro_first_timestamp_ns == p->accel_first_timestamp_ns);
	assert(p->accel_cal.m[0][0] != 0 && p->gyro_cal.m[0][0] != 0);
	/* The resubmission ran the work again; the poll that followed found it pending. */
	assert(stream.work.submitted == 3 && stream.work.scheduled == 1);

	/* Below the watermark: no batch, the next poll. */
	fifo_words = 5;
	run();
	assert(completions == 1 && stream.work.scheduled == 2 && stream.work.delay_ms == 20);
	fifo_words = 0;

	/* Another request, and a stop of another request. */
	assert(lsm6dso_stream_submit(&stream, &(lsm6dso_stream_req_t){9}) == -EBUSY);
	assert(lsm6dso_stream_request_stop(&stream, &(lsm6dso_stream_req_t){9}) == -ENOENT);

	/* An overrun ends the request. */
	overrun = true;
	stops = stop_calls;
	run();
	assert(terminal_error == -EOVERFLOW && live_req == NULL && !stream.requested);
	assert(!stream.active && !stream.irq_active && !stream.int1.enabled && !int1_routed);
	assert(stop_calls == stops + 1);
	overrun = false;

	/* A level already active at the start runs the work again. */
	start_stream(false);
	stream.int1.level = 1;
	run();
	assert(stream.active && stream.work.submitted == 2 && stream.work.scheduled == 0);

	/* The user cancels the request: stop, -ECANCELED. */
	request.canceled = true;
	run();
	assert(terminal_error == -ECANCELED && live_req == NULL && !stream.active &&
	       stream.int1.disables == 1);

	/* The request is gone: stop, without an error to report. */
	start_stream(false);
	run();
	live_req = NULL;
	terminal_error = 1;
	run();
	assert(terminal_error == 1 && !stream.requested && !stream.active && !stream.int1.enabled);

	/* INT1 cannot be enabled, or its level read: what was started is stopped. */
	start_stream(false);
	stream.int1.enable_error = -EIO;
	run();
	assert(terminal_error == -EIO && !stream.active && !stream.irq_active && !int1_routed);
	start_stream(false);
	stream.int1.level = -EINVAL;
	run();
	assert(terminal_error == -EINVAL && !stream.active && !stream.int1.enabled &&
	       stream.int1.disables == 1);

	/* INT1 cannot be routed. */
	start_stream(false);
	route_error = -EIO;
	run();
	assert(terminal_error == -EIO && !stream.active && !stream.irq_active &&
	       !stream.int1.enabled);
	route_error = 0;
}

static void test_stop(void)
{
	start_stream(true);
	completions = delivered_words = 0;
	stop_calls = 0;
	assert(lsm6dso_stream_request_stop(&stream, &(lsm6dso_stream_req_t){8}) == -ENOENT);
	/* The stop request arrives during a full FIFO read. That batch must be
	 * delivered, followed by the final partial batch, before bypass mode.
	 */
	fifo_words = 32;
	injected_tail = 5;
	inject_stop = true;
	run();
	assert(completions == 1 && delivered_words == 32 && stop_calls == 0);
	run();
	assert(batching_paused && pause_calls == 1 && fifo_words == 0 && completions == 2 &&
	       delivered_words == 37 && stop_calls == 0);
	run();
	assert(stop_calls == 1 && terminal_error == -ECANCELED && live_req == NULL);

	/* A pause failure must report an error rather than claiming a clean drain. */
	start_stream(true);
	fail_pause = true;
	assert(lsm6dso_stream_request_stop(&stream, &req7) == 0);
	run();
	assert(terminal_error == -EIO && live_req == NULL);

	/* A full buffer or FIFO overrun must not produce a successful final batch. */
	fail_pause = false;
	fail_alloc = true;
	fifo_words = 5;
	start_stream(true);
	assert(lsm6dso_stream_request_stop(&stream, &req7) == 0);
	run();
	assert(terminal_error == -ENOMEM && live_req == NULL && completions == 2);

	fail_alloc = false;
	overrun = true;
	start_stream(true);
	assert(lsm6dso_stream_request_stop(&stream, &req7) == 0);
	run();
	assert(terminal_error == -EOVERFLOW && live_req == NULL && completions == 2);

	/* An empty FIFO stops without allocating or delivering an empty batch. */
	overrun = false;
	fifo_words = 0;
	start_stream(true);
	assert(lsm6dso_stream_request_stop(&stream, &req7) == 0);
	run();
	assert(terminal_error == -ECANCELED && live_req == NULL && completions == 2);

	/* A stop before the work runs does not start FIFO acquisition. */
	start_calls = 0;
	start_stream(false);
	assert(lsm6dso_stream_request_stop(&stream, &req7) == 0);
	run();
	assert(terminal_error == -ECANCELED && live_req == NULL && completions == 2 &&
	       start_calls == 0);
}

static void test_watermark(void)
{
	/* Configuration has bounds and must not change a live stream. */
	assert(lsm6dso_stream_set_watermark(&stream, 0) == -EINVAL);
	assert(lsm6dso_stream_set_watermark(&stream, 257) == -EINVAL);
	assert(lsm6dso_stream_set_watermark(&stream, 128) == 0);
	assert(lsm6dso_stream_get_watermark(&stream) == 128);
	stream.requested = true;
	assert(lsm6dso_stream_set_watermark(&stream, 64) == -EBUSY);
	assert(lsm6dso_stream_get_watermark(&stream) == 128);
	stream.requested = false;
	assert(lsm6dso_stream_set_watermark(&stream, 256) == 0);
}

int main(void)
{
	core.accel_freq = 208;
	core.gyro_freq = 208;
	core.acc_gain = 244;
	core.gyro_gain = 70000;
	test_start();
	test_stop();
	test_watermark();
	return 0;
}
