/* SPDX-License-Identifier: Apache-2.0 */
/* The FIFO stream (lsm6dso_stream.h) on the core's FIFO (lsm6dso.c), the
 * OS's work item and INT1 line (lsm6dso_os.h) and its user's request
 * (lsm6dso_stream_req.h).
 */
#include "lsm6dso_os.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lsm6dso_fifo_packet.h"
#include "lsm6dso_stream.h"

#define STREAM_CANCEL_POLL_MS 20

#define STREAM_OF(ptr, member)                                                                     \
	((struct lsm6dso_stream *)((char *)(ptr) - offsetof(struct lsm6dso_stream, member)))

static uint32_t period_ns(uint16_t freq)
{
	/* The driver's nominal 12 Hz entry corresponds to the 12.5 Hz ODR. */
	return freq == 12 ? 80000000U : freq ? 1000000000U / freq : 0;
}

/* Forgets the request, which the stream no longer serves. */
static void stream_release(struct lsm6dso_stream *stream)
{
	lsm6dso_os_spinlock_key_t key = lsm6dso_os_spin_lock(&stream->lock);

	stream->requested = false;
	lsm6dso_os_spin_unlock(&stream->lock, key);
}

static int stream_stop(struct lsm6dso_stream *stream)
{
	struct lsm6dso *core = stream->core;
	int ret = 0, err;

	if (stream->active) {
		ret = lsm6dso_fifo_route_int1(core, false);
	}
	if (stream->irq_active) {
		err = lsm6dso_os_int1_disable(&stream->int1);
		if (ret == 0) {
			ret = err;
		}
		stream->irq_active = false;
	}
	err = lsm6dso_fifo_stop(core);
	if (ret == 0) {
		ret = err;
	}
	lsm6dso_fifo_record_stop_error(core, ret);
	stream->active = false;
	stream->batch_paused = false;
	return ret;
}

static int stream_int1_start(struct lsm6dso_stream *stream)
{
	int ret;

	ret = lsm6dso_os_int1_enable(&stream->int1);
	if (ret != 0) {
		return ret;
	}
	/* Do not lose a level that was already asserted before edge detection. */
	ret = lsm6dso_os_int1_get(&stream->int1);
	if (ret < 0) {
		(void)lsm6dso_os_int1_disable(&stream->int1);
		return ret;
	}
	if (ret > 0) {
		lsm6dso_os_work_submit(&stream->work);
	}
	return 0;
}

static int stream_start(struct lsm6dso_stream *stream)
{
	struct lsm6dso *core = stream->core;
	int ret;

	if (!lsm6dso_os_int1_ready(&stream->int1)) {
		return -ENODEV;
	}
	/* The core reads the FIFO with lsm6dso_os_bus_read(). */
	if (!lsm6dso_os_bus_read_supported(core->ctx)) {
		ret = -ENOTSUP;
	} else {
		ret = lsm6dso_fifo_start(core, stream->watermark, core->accel_freq != 0,
					 core->gyro_freq != 0);
	}
	if (ret != 0) {
		(void)lsm6dso_fifo_stop(core);
		return ret;
	}
	stream->active = true;
	ret = stream_int1_start(stream);
	if (ret == 0) {
		stream->irq_active = true;
		ret = lsm6dso_fifo_route_int1(core, true);
	}
	if (ret != 0) {
		(void)stream_stop(stream);
	}
	return ret;
}

static int stream_read(struct lsm6dso_stream *stream, lsm6dso_stream_req_live_t *live, bool drain)
{
	struct lsm6dso *core = stream->core;
	struct lsm6dso_fifo_status status;
	struct lsm6dso_fifo_packet *packet;
	struct lsm6dso_fifo_sample sample;
	struct lsm6dso_cal accel_cal, gyro_cal;
	uint8_t *buf;
	uint32_t len;
	uint64_t now;
	uint16_t accel_count = 0, gyro_count = 0;
	size_t cursor = 0;
	int ret;

	ret = lsm6dso_fifo_status(core, &status);
	if (ret != 0) {
		return ret;
	}
	/* The number of lost samples is unknown. Terminate with an explicit
	 * discontinuity instead of returning an apparently continuous batch.
	 */
	if (status.overrun || status.overrun_latched) {
		return -EOVERFLOW;
	}
	if (status.words == 0 || (!drain && !status.watermark)) {
		return 0;
	}
	ret = lsm6dso_stream_req_buffer(
		live, sizeof(*packet) + (drain ? 1 : stream->watermark) * LSM6DSO_FIFO_WORD_SIZE,
		sizeof(*packet) + LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE, &buf, &len);
	if (ret != 0) {
		return ret;
	}
	packet = (void *)buf;
	ret = lsm6dso_fifo_read(core, packet->data,
				(len - sizeof(*packet)) / LSM6DSO_FIFO_WORD_SIZE, &status);
	if (ret != 0) {
		return ret;
	}
	if (status.overrun || status.overrun_latched) {
		return -EOVERFLOW;
	}
	if (status.words == 0) {
		return 0;
	}
	while (lsm6dso_fifo_next(packet->data, status.words * LSM6DSO_FIFO_WORD_SIZE, &cursor,
				 &sample) > 0) {
		accel_count += sample.kind == LSM6DSO_FIFO_ACCEL;
		gyro_count += sample.kind == LSM6DSO_FIFO_GYRO;
	}
	/* Before the copy below, so that this batch already uses a bias it
	 * has just updated.
	 */
	lsm6dso_track_gyro_bias(core, packet->data, status.words * LSM6DSO_FIFO_WORD_SIZE);
	ret = lsm6dso_os_clock_ns(&now);
	if (ret != 0) {
		return ret;
	}
	packet->marker = LSM6DSO_FIFO_PACKET_MARKER;
	packet->words = status.words;
	packet->watermark = status.watermark;
	packet->accel_period_ns = period_ns(core->accel_freq);
	packet->gyro_period_ns = period_ns(core->gyro_freq);
	packet->accel_gain_ug = core->acc_gain;
	packet->gyro_gain_udps = core->gyro_gain;
	lsm6dso_copy_cal(core, &accel_cal, &gyro_cal);
	memcpy(&packet->accel_cal, &accel_cal, sizeof(accel_cal));
	memcpy(&packet->gyro_cal, &gyro_cal, sizeof(gyro_cal));
	/* Host-read-time based estimate: FIFO and transfer latency are unknown. */
	uint64_t accel_age =
		accel_count ? (uint64_t)(accel_count - 1) * packet->accel_period_ns : 0;
	uint64_t gyro_age = gyro_count ? (uint64_t)(gyro_count - 1) * packet->gyro_period_ns : 0;
	packet->accel_first_timestamp_ns = now > accel_age ? now - accel_age : 0;
	packet->gyro_first_timestamp_ns = now > gyro_age ? now - gyro_age : 0;
	return 1;
}

void lsm6dso_stream_run(lsm6dso_os_work_t *work)
{
	struct lsm6dso_stream *stream = STREAM_OF(work, work);
	lsm6dso_stream_req_live_t *live;
	lsm6dso_os_spinlock_key_t key;
	bool stop_requested;
	int ret;

	key = lsm6dso_os_spin_lock(&stream->lock);
	live = stream->requested ? lsm6dso_stream_req_get(&stream->req) : NULL;
	stop_requested = stream->stop_requested;
	lsm6dso_os_spin_unlock(&stream->lock, key);
	if (!live) {
		/* The user may have ended the request while it completed a
		 * batch; then it is gone without coming back.
		 */
		if (stream->active) {
			(void)stream_stop(stream);
		}
		stream_release(stream);
		return;
	}
	if (lsm6dso_stream_req_canceled(live)) {
		ret = stream_stop(stream);
		stream_release(stream);
		lsm6dso_stream_req_error(live, ret ? ret : -ECANCELED);
		return;
	}
	if (stop_requested) {
		if (!stream->active) {
			ret = 0;
		} else if (!stream->batch_paused) {
			ret = lsm6dso_fifo_pause_batch(stream->core);
			if (ret == 0) {
				stream->batch_paused = true;
			}
		} else {
			ret = 0;
		}
		if (ret == 0 && stream->active) {
			ret = stream_read(stream, live, true);
		}
		if (ret <= 0) {
			int stop_ret = stream_stop(stream);

			stream_release(stream);
			lsm6dso_stream_req_error(live, ret < 0    ? ret
						       : stop_ret ? stop_ret
								  : -ECANCELED);
			return;
		}
		/* A final partial batch is delivered like the others. The
		 * request's next run returns here to check that the FIFO is empty.
		 */
		lsm6dso_stream_req_done(live);
		return;
	}
	ret = stream->active ? 0 : stream_start(stream);
	if (ret == 0) {
		ret = stream_read(stream, live, false);
	}
	if (ret == 0) {
		lsm6dso_os_work_schedule(&stream->work, STREAM_CANCEL_POLL_MS);
		return;
	}
	if (ret < 0) {
		(void)stream_stop(stream);
		stream_release(stream);
		lsm6dso_stream_req_error(live, ret);
		return;
	}
	/* RTIO resubmits a multishot request as it completes a batch. */
	lsm6dso_stream_req_done(live);
	/* Also check if the user ended the request meanwhile. */
	lsm6dso_os_work_schedule(&stream->work, STREAM_CANCEL_POLL_MS);
}

int lsm6dso_stream_int1(lsm6dso_os_int1_t *int1)
{
	struct lsm6dso_stream *stream = STREAM_OF(int1, int1);

	lsm6dso_os_work_submit(&stream->work);
	return 0;
}

int lsm6dso_stream_submit(struct lsm6dso_stream *stream, const lsm6dso_stream_req_t *req)
{
	lsm6dso_os_spinlock_key_t key;
	bool busy;

	key = lsm6dso_os_spin_lock(&stream->lock);
	busy = stream->requested && !lsm6dso_stream_req_same(&stream->req, req);
	if (!busy) {
		if (!stream->requested) {
			stream->stop_requested = false;
			stream->batch_paused = false;
		}
		stream->requested = true;
		stream->req = *req;
	}
	lsm6dso_os_spin_unlock(&stream->lock, key);
	if (busy) {
		return -EBUSY;
	}
	lsm6dso_os_work_submit(&stream->work);
	return 0;
}

int lsm6dso_stream_request_stop(struct lsm6dso_stream *stream, const lsm6dso_stream_req_t *req)
{
	lsm6dso_os_spinlock_key_t key;
	bool found;

	key = lsm6dso_os_spin_lock(&stream->lock);
	found = stream->requested && lsm6dso_stream_req_same(&stream->req, req);
	if (found) {
		stream->stop_requested = true;
	}
	lsm6dso_os_spin_unlock(&stream->lock, key);
	if (!found) {
		return -ENOENT;
	}
	lsm6dso_os_work_submit(&stream->work);
	return 0;
}

int lsm6dso_stream_set_watermark(struct lsm6dso_stream *stream, uint16_t watermark)
{
	lsm6dso_os_spinlock_key_t key;

	if (watermark == 0 || watermark > LSM6DSO_FIFO_MAX_WATERMARK) {
		return -EINVAL;
	}
	key = lsm6dso_os_spin_lock(&stream->lock);
	if (stream->requested) {
		lsm6dso_os_spin_unlock(&stream->lock, key);
		return -EBUSY;
	}
	stream->watermark = watermark;
	lsm6dso_os_spin_unlock(&stream->lock, key);
	return 0;
}

uint16_t lsm6dso_stream_get_watermark(struct lsm6dso_stream *stream)
{
	lsm6dso_os_spinlock_key_t key = lsm6dso_os_spin_lock(&stream->lock);
	uint16_t watermark = stream->watermark;

	lsm6dso_os_spin_unlock(&stream->lock, key);
	return watermark;
}

void lsm6dso_stream_init(struct lsm6dso_stream *stream, struct lsm6dso *core)
{
	stream->core = core;
	lsm6dso_os_work_init(&stream->work);
	stream->watermark = LSM6DSO_FIFO_DEFAULT_WATERMARK;
}
