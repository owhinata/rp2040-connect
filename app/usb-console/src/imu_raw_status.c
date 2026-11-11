/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_raw_status.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

/* Hand-rolled so that 64-bit output does not depend on the libc printf
 * configuration of the target. */
struct writer {
	char *out;
	size_t capacity, length;
	bool overflow;
};

static void put_str(struct writer *w, const char *s)
{
	size_t n = strlen(s);

	if (w->overflow || n >= w->capacity - w->length) {
		w->overflow = true;
		return;
	}
	memcpy(w->out + w->length, s, n);
	w->length += n;
}

static void put_u64(struct writer *w, uint64_t value)
{
	char digits[21];
	size_t i = sizeof(digits);

	digits[--i] = '\0';
	do {
		digits[--i] = (char)('0' + value % 10);
		value /= 10;
	} while (value);
	put_str(w, digits + i);
}

static void put_int(struct writer *w, int value)
{
	if (value < 0) {
		put_str(w, "-");
		put_u64(w, (uint64_t)(-(int64_t)value));
	} else {
		put_u64(w, (uint64_t)value);
	}
}

static void key(struct writer *w, const char *name)
{
	put_str(w, w->length > 1 ? ",\"" : "\"");
	put_str(w, name);
	put_str(w, "\":");
}

static void field_u64(struct writer *w, const char *name, uint64_t value)
{
	key(w, name);
	put_u64(w, value);
}

static void field_str(struct writer *w, const char *name, const char *value)
{
	key(w, name);
	put_str(w, "\"");
	put_str(w, value);
	put_str(w, "\"");
}

static void field_session(struct writer *w, const uint8_t session[16])
{
	static const char hex[] = "0123456789abcdef";
	char text[33];

	for (size_t i = 0; i < 16; i++) {
		text[i * 2] = hex[session[i] >> 4];
		text[i * 2 + 1] = hex[session[i] & 15];
	}
	text[32] = '\0';
	field_str(w, "session", text);
}

static int finish(struct writer *w)
{
	put_str(w, "}");
	if (w->overflow) {
		return -ENOSPC;
	}
	w->out[w->length] = '\0';
	return (int)w->length;
}

int imu_raw_status_encode(char *out, size_t capacity, const struct imu_raw_status *s)
{
	struct writer w = {.out = out, .capacity = capacity};

	if (!out || !capacity || !s || s->state > IMU_RAW_STATE_STOPPED) {
		return -EINVAL;
	}
	put_str(&w, "{");
	field_u64(&w, "v", 1);
	field_session(&w, s->session_id);
	field_str(&w, "state", s->state == IMU_RAW_STATE_MEASURING ? "measuring" :
		  s->state == IMU_RAW_STATE_STOPPED ? "stopped" : "idle");
	field_u64(&w, "epoch", s->epoch_id);
	field_u64(&w, "uptime_ms", s->uptime_ms);
	field_u64(&w, "accel", s->accel_acquired);
	field_u64(&w, "gyro", s->gyro_acquired);
	field_u64(&w, "next_batch", s->next_batch);
	field_u64(&w, "timestamp_words", s->timestamp_words);
	field_u64(&w, "fifo_overruns", s->fifo_overrun_observations);
	field_u64(&w, "i2c_errors", s->i2c_errors);
	key(&w, "raw_error");
	put_int(&w, s->raw_error);
	key(&w, "error");
	put_int(&w, s->measure_error);
	field_u64(&w, "dropped_batches", s->dropped_batches);
	field_u64(&w, "dropped_accel", s->dropped_accel);
	field_u64(&w, "dropped_gyro", s->dropped_gyro);
	field_u64(&w, "published", s->published);
	field_u64(&w, "connects", s->connects);
	field_u64(&w, "failures", s->failures);
	field_u64(&w, "queue_high", s->queue_high_watermark);
	if (s->tick_valid) {
		if (s->tick_mono_us < 0) {
			return -EINVAL;
		}
		field_u64(&w, "tick", s->tick);
		field_u64(&w, "tick_mono_us", (uint64_t)s->tick_mono_us);
		field_u64(&w, "tick_window_us", s->tick_window_us);
	}
	if (s->tick_valid && s->unix_valid) {
		if (s->unix_us < 0 || s->unix_estimate_us < 0) {
			return -EINVAL;
		}
		field_u64(&w, "unix_us", (uint64_t)s->unix_us);
		field_u64(&w, "unix_estimate_us", (uint64_t)s->unix_estimate_us);
		field_u64(&w, "sync_delay_us", s->sync_delay_us);
		field_str(&w, "sync", "ntp");
	} else {
		field_str(&w, "sync", "unsynchronized");
	}
	return finish(&w);
}

int imu_raw_info_encode(char *out, size_t capacity, const struct imu_raw_info *info,
			const char *device_id)
{
	struct writer w = {.out = out, .capacity = capacity};

	if (!out || !capacity || !info || !device_id) {
		return -EINVAL;
	}
	for (const char *c = device_id; *c; c++) {
		if (!((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') ||
		      (*c >= '0' && *c <= '9') || *c == '-' || *c == '_')) {
			return -EINVAL;
		}
	}
	put_str(&w, "{");
	field_u64(&w, "v", 1);
	field_session(&w, info->session_id);
	field_u64(&w, "epoch", info->epoch_id);
	field_str(&w, "device", device_id);
	field_str(&w, "sensor", "LSM6DSO");
	field_str(&w, "payload", "IMU1");
	field_u64(&w, "payload_version", 2);
	field_str(&w, "raw", "int16le");
	field_str(&w, "axes", "xyz");
	field_u64(&w, "accel_odr_hz", info->accel_odr_hz);
	field_u64(&w, "gyro_odr_hz", info->gyro_odr_hz);
	field_u64(&w, "accel_ug_per_lsb", info->accel_ug_per_lsb);
	field_u64(&w, "gyro_udps_per_lsb", info->gyro_udps_per_lsb);
	field_u64(&w, "fifo_watermark", info->fifo_watermark);
	field_u64(&w, "timestamp_decimation", info->timestamp_decimation);
	if (info->timestamp_decimation) {
		field_str(&w, "time", "sensor_tick");
		field_u64(&w, "tick_hz_nominal", 40000);
		key(&w, "freq_fine");
		put_int(&w, info->freq_fine);
	} else {
		field_str(&w, "time", "unavailable");
	}
	return finish(&w);
}

int imu_raw_sync_encode(char *out, size_t capacity, uint64_t id, uint64_t t2_us, uint64_t t3_us)
{
	struct writer w = {.out = out, .capacity = capacity};

	if (!out || !capacity) {
		return -EINVAL;
	}
	put_str(&w, "{");
	field_u64(&w, "v", 1);
	field_u64(&w, "id", id);
	field_u64(&w, "t2_us", t2_us);
	field_u64(&w, "t3_us", t3_us);
	return finish(&w);
}
