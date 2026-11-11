/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/cpu_load.h>

#include "imu_measure_packet.h"
#include "imu_cpu_profile.h"
#if defined(CONFIG_IMU_RAW_BATCH_DIAGNOSTIC)
#include "imu_raw_batch.h"
#endif
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
#include "imu_raw_mqtt.h"
#include "imu_tick_clock.h"
#include "time_sync.h"
#endif
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
#include "../drivers/lsm6dso/lsm6dso_fifo.h"
#include "../drivers/lsm6dso/lsm6dso_fifo_decoder.h"
#define TIMESTAMP_TRACE_WORDS 48
#endif
#include "../drivers/lsm6dso/lsm6dso_fifo_stats.h"
#if defined(CONFIG_IMU_MQTT_PROBE)
#include "../tests/hardware/imu_mqtt_probe.h"
#endif

#define IMU_MEASURE_CLEANUP_MS 2000
#define IMU_MEASURE_MAX_SECONDS 3600
#define IMU_MEASURE_CQE_BATCH 8
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
#define IMU_RAW_FLUSH_INTERVAL_MS 100
#define IMU_RAW_REPORT_INTERVAL_MS 500
#define IMU_RAW_TIME_POINT_INTERVAL_MS 1000
/* After the driver ends a stream with an error it has already stopped the
 * FIFO. Give it this long to release the stream, and give up when this many
 * restarts in a row deliver no data. */
#define IMU_RECOVERY_RELEASE_MS 200
#define IMU_RECOVERY_MAX_CONSECUTIVE 3
#endif
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
#define IMU_MEASURE_STACK_SIZE 3072
#else
#define IMU_MEASURE_STACK_SIZE 2048
#endif

SENSOR_DT_STREAM_IODEV(imu_measure_iodev, DT_NODELABEL(lsm6dsox),
	{ SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE });
RTIO_DEFINE_WITH_MEMPOOL(imu_measure_rtio, 4, 16, 128, 64, 4);
K_SEM_DEFINE(imu_measure_request, 0, 1);
K_SEM_DEFINE(imu_measure_stop_signal, 0, 1);
K_MUTEX_DEFINE(imu_measure_result_lock);

struct imu_measure_result {
	struct imu_measure_counts counts;
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
	uint8_t trace[TIMESTAMP_TRACE_WORDS][LSM6DSO_FIFO_WORD_SIZE];
	uint32_t trace_count;
	uint64_t timestamp_words;
	uint64_t unexpected_tags[32];
	uint8_t first_unexpected[LSM6DSO_FIFO_WORD_SIZE];
	uint64_t first_unexpected_index;
	bool first_unexpected_seen;
	uint32_t unexpected_at_start;
	uint32_t unexpected_at_end;
	uint32_t unexpected_all_ff;
	uint16_t first_unexpected_offset, first_unexpected_packet_words;
	uint8_t first_context[3][LSM6DSO_FIFO_WORD_SIZE];
	bool first_has_before, first_has_after;
#endif
#if defined(CONFIG_IMU_RAW_BATCH_DIAGNOSTIC)
	uint64_t raw_batches, raw_accel, raw_gyro, raw_timestamps;
	uint64_t raw_dropped_batches, raw_dropped_accel, raw_dropped_gyro;
	int raw_error;
	uint32_t raw_discontinuities, raw_epoch;
	struct imu_raw_time raw_time;
	uint32_t raw_time_point_failures, raw_time_point_window_us;
	uint32_t raw_unix_unavailable, raw_tick_holds;
	int64_t raw_us_per_million_ticks;
#endif
	uint32_t recoveries;
	int last_recovery_error, stream_rc;
	struct lsm6dso_fifo_stats fifo;
	uint64_t elapsed_ms;
	uint64_t cpu_busy_cycles;
	uint64_t decode_elapsed_cycles;
	uint64_t cpu_total_cycles;
	uint32_t dropped_cqes;
	uint32_t bufferless_cqes;
	uint32_t invalid_packets;
	uint64_t stop_requested_ms;
	int error;
	int stop_error;
	int cpu_active_permille;
	bool cpu_active_valid;
	bool valid;
	bool fifo_valid;
	bool cpu_valid;
	bool stopped_early;
	bool cleanup_timeout;
	bool terminal_seen;
	bool graceful_requested;
};

static struct imu_measure_result last_result;
static atomic_t measure_running;
static atomic_t measure_stop;
static atomic_t recovery_required;
static uint32_t requested_seconds;
static const struct shell *measure_shell;
static rtio_sqe_handle_t active_handle = RTIO_SQE_HANDLE_INVALID;
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
static uint8_t measurement_session[16];
#endif
#if defined(CONFIG_IMU_FAULT_INJECTION)
static atomic_t inject_stall_ms;
#endif

/* Errors after which the driver has stopped the FIFO and a restart can
 * continue the session in a new epoch. Only the RAW transport can express
 * the discontinuity, so other builds keep ending the measurement. */
static bool stream_error_recoverable(int rc)
{
	return IS_ENABLED(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL) &&
	       (rc == -EOVERFLOW || rc == -ENOMEM || rc == -ENOBUFS || rc == -EIO);
}

#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
static void fifo_stats_add(struct lsm6dso_fifo_stats *total, const struct lsm6dso_fifo_stats *run)
{
	total->words_read += run->words_read;
	total->transfer_time_ns += run->transfer_time_ns;
	total->max_transfer_time_ns = MAX(total->max_transfer_time_ns, run->max_transfer_time_ns);
	total->transfers += run->transfers;
	total->i2c_errors += run->i2c_errors;
	total->buffer_too_small += run->buffer_too_small;
	total->overrun_observations += run->overrun_observations;
}
#endif

#if defined(CONFIG_IMU_RAW_BATCH_DIAGNOSTIC)
/* Diagnostic only: the fixed session ID must never be used on the wire. */
static struct imu_raw_batch raw_batch;
#if !defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
static int raw_diagnostic_sink(void *context, const uint8_t *payload, size_t length)
{
	ARG_UNUSED(context);
	ARG_UNUSED(payload);
	ARG_UNUSED(length);
	return 0;
}
#endif
#endif

#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
/* Hands the acquisition counters to the transport. Returns true once the
 * driver statistics belong to this run (the FIFO starts asynchronously). */
/* Latest reading of the sensor tick counter together with the board clock.
 * Owned by the measurement thread. */
static struct {
	bool valid, unix_valid;
	uint64_t tick;
	int64_t mono_us, unix_us, unix_estimate_us;
	uint32_t window_us, sync_delay_us;
	uint32_t failures;
} raw_time_point;

/* Sensor tick to board clock. Kept across measurements for its rate and so
 * that batch times never go back. Owned by the measurement thread. */
static struct imu_tick_clock raw_tick_clock;
static bool raw_tick_clock_ready;
static uint32_t raw_unix_unavailable;

/* Server time of a sample for the batch header. */
static bool raw_tick_to_unix(void *context, uint64_t tick, uint64_t *unix_us)
{
	int64_t mono_us, time_us;

	ARG_UNUSED(context);
	if (!imu_tick_clock_convert(&raw_tick_clock, tick, &mono_us) ||
	    !time_sync_unix_us(mono_us, &time_us, NULL) || time_us <= 0) {
		raw_unix_unavailable++;
		return false;
	}
	*unix_us = (uint64_t)time_us;
	return true;
}

static void raw_time_point_take(const struct device *dev)
{
	const struct imu_raw_time *fifo_time = &raw_batch.time;
	int64_t before, after;
	uint64_t wraps;
	uint32_t tick;

	if (lsm6dso_timestamp_read(dev, time_sync_mono_us, &tick, &before, &after)) {
		raw_time_point.failures++;
		return;
	}
	/* Extend like the FIFO timestamps. The register is read later than
	 * the newest FIFO word, so it may already be past a wrap. */
	wraps = fifo_time->wraps;
	if (fifo_time->raw_seen && tick < fifo_time->last_raw &&
	    fifo_time->last_raw - tick > UINT32_MAX / 2U) {
		wraps++;
	}
	raw_time_point.tick = (wraps << 32) | tick;
	raw_time_point.mono_us = before + (after - before) / 2;
	raw_time_point.window_us = (uint32_t)MIN(after - before, (int64_t)UINT32_MAX);
	imu_tick_clock_add(&raw_tick_clock, raw_time_point.tick, raw_time_point.mono_us);
	/* Converted now: a point is reported more than once, and the smoothed
	 * clock only answers for times after its latest correction. */
	raw_time_point.unix_valid =
		time_sync_unix_us(raw_time_point.mono_us, &raw_time_point.unix_us,
				  &raw_time_point.sync_delay_us) &&
		time_sync_estimate_us(raw_time_point.mono_us, &raw_time_point.unix_estimate_us, NULL);
	raw_time_point.valid = true;
}

static bool raw_report(const struct device *dev, int raw_error, int error,
		       uint32_t generation, const struct lsm6dso_fifo_stats *earlier,
		       enum imu_raw_state state, bool send_info)
{
	struct imu_raw_status status = {
		.state = state, .epoch_id = raw_batch.meta.epoch_id,
		.uptime_ms = (uint64_t)k_uptime_get(),
		.accel_acquired = raw_batch.next_accel, .gyro_acquired = raw_batch.next_gyro,
		.next_batch = raw_batch.next_batch, .timestamp_words = raw_batch.timestamp_words,
		.raw_error = raw_error, .measure_error = error,
	};
	struct lsm6dso_fifo_stats fifo;
	bool current;

	lsm6dso_fifo_get_stats(dev, &fifo);
	current = fifo.generation == generation;
	memcpy(status.session_id, measurement_session, sizeof(status.session_id));
	if (raw_time_point.valid) {
		status.tick_valid = true;
		status.tick = raw_time_point.tick;
		status.tick_mono_us = raw_time_point.mono_us;
		status.tick_window_us = raw_time_point.window_us;
		status.unix_valid = raw_time_point.unix_valid;
		status.unix_us = raw_time_point.unix_us;
		status.unix_estimate_us = raw_time_point.unix_estimate_us;
		status.sync_delay_us = raw_time_point.sync_delay_us;
	}
	status.fifo_overrun_observations = earlier->overrun_observations;
	status.i2c_errors = earlier->i2c_errors;
	if (current) {
		status.fifo_overrun_observations += fifo.overrun_observations;
		status.i2c_errors += fifo.i2c_errors;
	}
	if (current && send_info) {
		struct imu_raw_info info = {
			.epoch_id = raw_batch.meta.epoch_id,
			.accel_odr_hz = fifo.accel_hz, .gyro_odr_hz = fifo.gyro_hz,
			.accel_ug_per_lsb = fifo.accel_ug_per_lsb,
			.gyro_udps_per_lsb = fifo.gyro_udps_per_lsb,
			.fifo_watermark = lsm6dso_fifo_get_watermark(dev),
			.timestamp_decimation = (uint8_t)lsm6dso_fifo_get_timestamp_decimation(dev),
			.freq_fine = fifo.freq_fine,
		};

		memcpy(info.session_id, measurement_session, sizeof(info.session_id));
		imu_raw_mqtt_set_info(&info);
	}
	imu_raw_mqtt_report(&status);
	return current;
}
#endif

static void consume_cqes(struct imu_measure_result *result, bool include)
{
	struct rtio_cqe *cqe;

	for (unsigned int i = 0; i < IMU_MEASURE_CQE_BATCH &&
		(cqe = rtio_cqe_consume(&imu_measure_rtio)) != NULL; i++) {
		uint8_t *buffer = NULL;
		uint32_t length = 0;
		int rc = cqe->result;

		int buf_rc = rtio_cqe_get_mempool_buffer(&imu_measure_rtio, cqe, &buffer, &length);
		rtio_cqe_release(&imu_measure_rtio, cqe);
		if (include && rc < 0) {
			result->terminal_seen = true;
			result->stream_rc = rc;
			/* A recoverable error during the run is left to the measurement
			 * loop, which either restarts the stream or records the error. */
			if ((rc != -ECANCELED || !result->graceful_requested) && result->error == 0 &&
			    (result->graceful_requested || !stream_error_recoverable(rc))) {
				result->error = rc;
			}
		} else if (include && rc == 0) {
			if (buf_rc != 0 || !buffer || !length) {
				result->bufferless_cqes++;
				if (result->error == 0) {
					result->error = -ENODATA;
				}
			} else {
				uint32_t before = k_cycle_get_32();

				rc = imu_measure_count_packet(buffer, length, &result->counts);
#if defined(CONFIG_IMU_RAW_BATCH_DIAGNOSTIC)
				if (!rc) {
					const struct lsm6dso_fifo_packet *packet = (const void *)buffer;
					size_t cursor = 0;

					rc = imu_raw_batch_consume(&raw_batch, packet->data,
						packet->words * LSM6DSO_FIFO_WORD_SIZE, &cursor);
					if (rc == -EPROTO && IS_ENABLED(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)) {
						/* The rest of this packet cannot be trusted. Mark
						 * the gap instead of ending the measurement. */
						result->raw_discontinuities++;
						rc = imu_raw_batch_new_epoch(&raw_batch);
					}
					if (rc) {
						result->raw_error = rc;
					}
				}
#endif
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
				if (!rc) {
					const struct lsm6dso_fifo_packet *packet = (const void *)buffer;

					for (size_t w = 0; w < packet->words; w++) {
						const uint8_t *word = packet->data +
							w * LSM6DSO_FIFO_WORD_SIZE;

						result->timestamp_words += (word[0] >> 3) == 4;
						uint8_t tag = word[0] >> 3;

						if (tag != LSM6DSO_FIFO_ACCEL &&
						    tag != LSM6DSO_FIFO_GYRO && tag != 4) {
							static const uint8_t all_ff[] =
								{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

							result->unexpected_tags[tag]++;
							result->unexpected_at_start += w == 0;
							result->unexpected_at_end += w + 1 == packet->words;
							result->unexpected_all_ff +=
								memcmp(word, all_ff, sizeof(all_ff)) == 0;
							if (!result->first_unexpected_seen) {
								result->first_unexpected_seen = true;
								memcpy(result->first_unexpected, word,
								       LSM6DSO_FIFO_WORD_SIZE);
								result->first_unexpected_index =
									result->counts.accel_samples +
									result->counts.gyro_samples +
									result->counts.other_words -
									packet->words + w;
								result->first_unexpected_offset = w;
								result->first_unexpected_packet_words = packet->words;
								result->first_has_before = w != 0;
								result->first_has_after = w + 1 < packet->words;
								memcpy(result->first_context[1], word,
								       LSM6DSO_FIFO_WORD_SIZE);
								if (w != 0) {
									memcpy(result->first_context[0],
									       word - LSM6DSO_FIFO_WORD_SIZE,
									       LSM6DSO_FIFO_WORD_SIZE);
								}
								if (w + 1 < packet->words) {
									memcpy(result->first_context[2],
									       word + LSM6DSO_FIFO_WORD_SIZE,
									       LSM6DSO_FIFO_WORD_SIZE);
								}
							}
						}
						if (result->trace_count < TIMESTAMP_TRACE_WORDS) {
							memcpy(result->trace[result->trace_count++], word,
							       LSM6DSO_FIFO_WORD_SIZE);
						}
					}
				}
#endif
#if defined(CONFIG_IMU_MQTT_PROBE)
				if (!rc) {
					rc = imu_mqtt_probe_submit(buffer, length);
				}
#endif
				result->decode_elapsed_cycles += (uint32_t)(k_cycle_get_32() - before);
				if (rc != 0) {
					result->invalid_packets++;
					if (result->error == 0) {
						result->error = rc;
					}
				}
			}
		}
		if (buffer) {
			rtio_release_buffer(&imu_measure_rtio, buffer, length);
		}
	}
}

static void print_result(const struct shell *shell, const struct imu_measure_result *result)
{
	if (!result->valid) {
		shell_print(shell, "No measurement result");
		return;
	}
	shell_print(shell, "elapsed_ms=%" PRIu64 " stop_requested_ms=%" PRIu64
		    " batches=%" PRIu64 " stopped_early=%u", result->elapsed_ms,
		    result->stop_requested_ms, result->counts.batches, result->stopped_early);
	shell_print(shell, "decode_elapsed_ns=%" PRIu64 " (includes preemption)",
		    k_cyc_to_ns_floor64(result->decode_elapsed_cycles));
	shell_print(shell, "accel_samples=%" PRIu64 " gyro_samples=%" PRIu64
		    " other_words=%" PRIu64, result->counts.accel_samples,
		    result->counts.gyro_samples, result->counts.other_words);
#if defined(CONFIG_IMU_RAW_BATCH_DIAGNOSTIC)
	shell_print(shell, "raw_batches=%" PRIu64 " raw_accel=%" PRIu64 " raw_gyro=%" PRIu64
		    " raw_timestamps=%" PRIu64 " raw_error=%d",
		    result->raw_batches, result->raw_accel, result->raw_gyro,
		    result->raw_timestamps, result->raw_error);
	shell_print(shell, "raw_dropped_batches=%" PRIu64 " accel=%" PRIu64 " gyro=%" PRIu64,
		    result->raw_dropped_batches, result->raw_dropped_accel, result->raw_dropped_gyro);
	shell_print(shell, "raw_epoch=%" PRIu32 " raw_discontinuities=%" PRIu32
		    " recoveries=%" PRIu32 " last_recovery_error=%d",
		    result->raw_epoch, result->raw_discontinuities, result->recoveries,
		    result->last_recovery_error);
	shell_print(shell, "raw_time anchors=%" PRIu64 " irregular_periods=%" PRIu64
		    " counter_mismatches=%" PRIu64 " counter_skips=%" PRIu64,
		    result->raw_time.anchors, result->raw_time.irregular_periods,
		    result->raw_time.counter_mismatches, result->raw_time.counter_skips);
	shell_print(shell, "raw_time_point failures=%" PRIu32 " last_window_us=%" PRIu32
		    " unix_unavailable=%" PRIu32 " monotonic_holds=%" PRIu32
		    " us_per_1e6_ticks=%" PRId64,
		    result->raw_time_point_failures, result->raw_time_point_window_us,
		    result->raw_unix_unavailable, result->raw_tick_holds,
		    result->raw_us_per_million_ticks);
	shell_print(shell, "raw_time slot_ticks=%" PRIu32 " anchor_ticks=%" PRIu32 "..%" PRIu32
		    " anchor_slots=%" PRIu32 "..%" PRIu32 " last_tick=%" PRIu64,
		    result->raw_time.slot_ticks, result->raw_time.min_anchor_ticks,
		    result->raw_time.max_anchor_ticks, result->raw_time.min_anchor_slots,
		    result->raw_time.max_anchor_slots, result->raw_time.anchor_tick);
#endif
#if defined(CONFIG_NINA_SEND_PROFILE)
	if (result->counts.other_words) {
		const uint8_t *w = result->counts.first_other;
		shell_print(shell, "first_other=%02x %02x %02x %02x %02x %02x %02x",
			    w[0], w[1], w[2], w[3], w[4], w[5], w[6]);
		shell_print(shell, "first_other_index=%" PRIu32 " packet_words=%" PRIu32
			    " other_batches=%" PRIu32 " valid_after_other_batches=%" PRIu32,
			    result->counts.first_other_index, result->counts.first_other_packet_words,
			    result->counts.batches_with_other, result->counts.batches_valid_after_other);
	}
	for (size_t i = 0; i < ARRAY_SIZE(result->counts.other_tags); ++i) {
		if (result->counts.other_tags[i]) {
			shell_print(shell, "other_tag[%zu]=%" PRIu32, i,
				    result->counts.other_tags[i]);
		}
	}
#endif
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
	shell_print(shell, "timestamp_words=%" PRIu64 " trace_words=%" PRIu32,
		    result->timestamp_words, result->trace_count);
	for (size_t tag = 0; tag < ARRAY_SIZE(result->unexpected_tags); tag++) {
		if (result->unexpected_tags[tag]) {
			shell_print(shell, "unexpected_tag[%zu]=%" PRIu64,
				    tag, result->unexpected_tags[tag]);
		}
	}
	if (result->counts.other_words > result->timestamp_words) {
		const uint8_t *w = result->first_unexpected;

		shell_print(shell, "first_unexpected[%" PRIu64 "]=%02x %02x %02x %02x %02x %02x %02x",
			    result->first_unexpected_index, w[0], w[1], w[2], w[3],
			    w[4], w[5], w[6]);
		shell_print(shell, "unexpected_all_ff=%" PRIu32 " first_word=%" PRIu32
			    " last_word=%" PRIu32 " first_offset=%u packet_words=%u",
			    result->unexpected_all_ff, result->unexpected_at_start,
			    result->unexpected_at_end, result->first_unexpected_offset,
			    result->first_unexpected_packet_words);
		for (size_t i = 0; i < 3; i++) {
			if ((i == 0 && !result->first_has_before) ||
			    (i == 2 && !result->first_has_after)) {
				continue;
			}
			w = result->first_context[i];
			shell_print(shell, "unexpected_context[%zu]=%02x %02x %02x %02x %02x %02x %02x",
				    i, w[0], w[1], w[2], w[3], w[4], w[5], w[6]);
		}
	}
	for (uint32_t i = 0; i < result->trace_count; i++) {
		const uint8_t *w = result->trace[i];

		shell_print(shell, "fifo[%" PRIu32 "]=%02x %02x %02x %02x %02x %02x %02x",
			    i, w[0], w[1], w[2], w[3], w[4], w[5], w[6]);
	}
#endif
	if (result->elapsed_ms) {
		uint64_t accel_mhz = result->counts.accel_samples * 1000000ULL / result->elapsed_ms;
		uint64_t gyro_mhz = result->counts.gyro_samples * 1000000ULL / result->elapsed_ms;

		shell_print(shell, "observed_hz accel=%" PRIu64 ".%03" PRIu64
			    " gyro=%" PRIu64 ".%03" PRIu64,
			    accel_mhz / 1000, accel_mhz % 1000, gyro_mhz / 1000,
			    gyro_mhz % 1000);
	}
	if (result->fifo_valid) {
		uint64_t received_words = result->counts.accel_samples + result->counts.gyro_samples +
					  result->counts.other_words;
		bool complete = result->terminal_seen && !result->cleanup_timeout && !result->error &&
			!result->stop_error && !result->dropped_cqes && !result->bufferless_cqes &&
			!result->invalid_packets &&
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
			result->counts.other_words == result->timestamp_words &&
#else
			!result->counts.other_words &&
#endif
			!result->fifo.fault &&
			!result->fifo.stop_error &&
			!result->fifo.i2c_errors && !result->fifo.buffer_too_small &&
			!result->fifo.overrun_observations;
		enum imu_measure_accounting accounting = imu_measure_check_accounting(
			&result->counts, result->fifo.words_read, result->fifo.transfers, complete);
		const char *accounting_text = accounting == IMU_MEASURE_ACCOUNTING_MATCH ? "match" :
			accounting == IMU_MEASURE_ACCOUNTING_MISMATCH ? "mismatch" : "incomplete";

		shell_print(shell, "configured_hz accel=%u gyro=%u fifo_words=%" PRIu64
			    " transfers=%" PRIu32, result->fifo.accel_hz, result->fifo.gyro_hz,
			    result->fifo.words_read, result->fifo.transfers);
		shell_print(shell, "transfer_ns=%" PRIu64 " max_transfer_ns=%" PRIu64,
			    result->fifo.transfer_time_ns, result->fifo.max_transfer_time_ns);
		shell_print(shell, "i2c_errors=%" PRIu32 " buffer_too_small=%" PRIu32
			    " overrun_observations=%" PRIu32 " driver_stop_error=%d",
			    result->fifo.i2c_errors, result->fifo.buffer_too_small,
			    result->fifo.overrun_observations, result->fifo.stop_error);
		shell_print(shell, "accounting=%s received_words=%" PRIu64 " fifo_words=%" PRIu64
			    " received_batches=%" PRIu64 " fifo_transfers=%" PRIu32,
			    accounting_text, received_words, result->fifo.words_read,
			    result->counts.batches, result->fifo.transfers);
	} else {
		shell_print(shell, "accounting=unavailable");
	}
	if (result->cpu_valid && result->cpu_total_cycles) {
		uint64_t pct = result->cpu_busy_cycles * 10000ULL / result->cpu_total_cycles;

		shell_print(shell, "cpu_busy=%" PRIu64 ".%02" PRIu64 "%% (scheduler non-idle; idle IRQ excluded)",
			    pct / 100, pct % 100);
	} else {
		shell_print(shell, "cpu_busy=unavailable");
	}
	if (result->cpu_active_valid && result->cpu_active_permille >= 0) {
		shell_print(shell, "cpu_active=%d.%d%% (idle hooks; includes interrupts)",
			    result->cpu_active_permille / 10, result->cpu_active_permille % 10);
	} else {
		shell_print(shell, "cpu_active=unavailable");
	}
	shell_print(shell, "stream_error=%d stop_error=%d dropped_completions=%" PRIu32,
		    result->error, result->stop_error, result->dropped_cqes);
	shell_print(shell, "bufferless_completions=%" PRIu32 " invalid_packets=%" PRIu32
		    " terminal_seen=%u cleanup_timeout=%u", result->bufferless_cqes,
		    result->invalid_packets, result->terminal_seen, result->cleanup_timeout);
}

static void measure_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	while (true) {
		struct imu_measure_result result = { .valid = true };
		struct lsm6dso_fifo_stats cleanup_fifo = {0};
		struct lsm6dso_fifo_stats initial_fifo;
		k_thread_runtime_stats_t before, after;
		uint32_t dropped_before;
		int64_t start, deadline, end, requested_at, cleanup_deadline;
		uint32_t expected_generation;
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		struct lsm6dso_fifo_stats earlier_fifo = {0};
		uint64_t batches_at_restart = 0;
		unsigned int restarts_without_data = 0;
#endif
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		int64_t next_raw_flush;
		uint64_t observed_raw_batch;
		bool raw_pending_timed;
		int64_t next_raw_report, next_time_point;
		bool raw_info_sent;
#endif
		int rc;

		k_sem_take(&imu_measure_request, K_FOREVER);
		consume_cqes(&result, false);
#if defined(CONFIG_IMU_RAW_BATCH_DIAGNOSTIC)
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		result.raw_error = imu_raw_batch_init(&raw_batch, measurement_session, 0,
						      imu_raw_mqtt_submit, NULL);
		raw_batch.clock = raw_tick_to_unix;
		if (!raw_tick_clock_ready) {
			imu_tick_clock_init(&raw_tick_clock);
			raw_tick_clock_ready = true;
		}
		imu_tick_clock_restart(&raw_tick_clock);
		raw_unix_unavailable = 0;
#else
		const uint8_t diagnostic_session[16] = {0};
		result.raw_error = imu_raw_batch_init(&raw_batch, diagnostic_session, 0,
						      raw_diagnostic_sink, NULL);
#endif
#endif
		lsm6dso_fifo_get_stats(dev, &initial_fifo);
		expected_generation = initial_fifo.generation + 1U;
		dropped_before = (uint32_t)atomic_get(&imu_measure_rtio.xcqcnt);
		imu_cpu_profile_start();
#ifdef CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK
		/* The idle-hook backend owns one global measurement window. */
		result.cpu_active_valid = cpu_load_get(true) >= 0;
#endif
		result.cpu_valid = k_thread_runtime_stats_cpu_get(0, &before) == 0;
		start = k_uptime_get();
		deadline = start + (int64_t)requested_seconds * 1000;
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		next_raw_flush = start + IMU_RAW_FLUSH_INTERVAL_MS;
		observed_raw_batch = 0;
		raw_pending_timed = false;
		next_raw_report = start;
		next_time_point = start;
		raw_time_point.valid = false;
		raw_time_point.failures = 0;
		raw_info_sent = false;
#endif
		rc = sensor_stream(&imu_measure_iodev, &imu_measure_rtio, NULL, &active_handle);
		if (rc != 0) {
			result.error = rc;
		}
		while (!result.error && !result.terminal_seen && !atomic_get(&measure_stop) &&
		       k_uptime_get() < deadline) {
			struct k_poll_event events[2];
			int64_t remaining, poll_remaining;

#if defined(CONFIG_IMU_FAULT_INJECTION)
			uint32_t stall_ms = (uint32_t)atomic_set(&inject_stall_ms, 0);

			if (stall_ms) {
				k_busy_wait(stall_ms * 1000U);
			}
#endif
			consume_cqes(&result, true);
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
			if (result.terminal_seen && !result.error) {
				/* The driver ended the stream and stopped the FIFO, losing
				 * an unknown number of samples. Continue in a new epoch
				 * unless the sensor cannot be restarted. */
				struct lsm6dso_fifo_stats ended;
				int64_t release_deadline = k_uptime_get() + IMU_RECOVERY_RELEASE_MS;
				bool released;

				for (;;) {
					lsm6dso_fifo_get_stats(dev, &ended);
					released = !ended.active &&
						!rtio_iodev_sqe_from_handle(&imu_measure_rtio, active_handle);
					if (released || k_uptime_get() >= release_deadline) {
						break;
					}
					k_sleep(K_MSEC(1));
				}
				restarts_without_data = result.counts.batches == batches_at_restart ?
					restarts_without_data + 1U : 0U;
				if (!released || ended.fault ||
				    restarts_without_data >= IMU_RECOVERY_MAX_CONSECUTIVE) {
					result.error = result.stream_rc;
				} else if ((rc = imu_raw_batch_new_epoch(&raw_batch)) != 0) {
					result.raw_error = rc;
					result.error = rc;
				} else {
					if (ended.generation == expected_generation) {
						fifo_stats_add(&earlier_fifo, &ended);
					}
					result.recoveries++;
					result.last_recovery_error = result.stream_rc;
					batches_at_restart = result.counts.batches;
					expected_generation = ended.generation + 1U;
					result.terminal_seen = false;
					rc = sensor_stream(&imu_measure_iodev, &imu_measure_rtio, NULL,
							   &active_handle);
					if (rc != 0) {
						result.error = rc;
					}
					raw_info_sent = false;
					next_raw_report = 0;
				}
			}
#endif
			remaining = deadline - k_uptime_get();
			poll_remaining = remaining;
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
			if (!raw_batch.count) {
				raw_pending_timed = false;
			} else if (!raw_pending_timed || observed_raw_batch != raw_batch.next_batch) {
				observed_raw_batch = raw_batch.next_batch;
				next_raw_flush = k_uptime_get() + IMU_RAW_FLUSH_INTERVAL_MS;
				raw_pending_timed = true;
			}
			if (raw_pending_timed && !result.raw_error &&
			    k_uptime_get() >= next_raw_flush) {
				result.raw_error = imu_raw_batch_flush(&raw_batch);
				if (result.raw_error && !result.error) {
					result.error = result.raw_error;
				}
				raw_pending_timed = false;
			}
			if (raw_pending_timed && poll_remaining > next_raw_flush - k_uptime_get()) {
				poll_remaining = MAX((int64_t)0, next_raw_flush - k_uptime_get());
			}
			if (k_uptime_get() >= next_time_point) {
				struct lsm6dso_fifo_stats running;

				/* The sensor's counter is only meaningful once this
				 * run's FIFO has been configured. Until the first
				 * reading, look again soon: batches wait for it. */
				lsm6dso_fifo_get_stats(dev, &running);
				if (running.active && running.generation == expected_generation) {
					imu_tick_clock_set_trim(&raw_tick_clock, running.freq_fine);
					raw_time_point_take(dev);
				}
				next_time_point = k_uptime_get() +
					(raw_time_point.valid ? IMU_RAW_TIME_POINT_INTERVAL_MS : 5);
				if (raw_time_point.valid) {
					/* Report while the point is fresh. */
					next_raw_report = 0;
				}
			}
			if (k_uptime_get() >= next_raw_report) {
				raw_info_sent = raw_report(dev, result.raw_error, result.error,
							   expected_generation, &earlier_fifo,
							   IMU_RAW_STATE_MEASURING, !raw_info_sent) ||
						raw_info_sent;
				next_raw_report = k_uptime_get() + IMU_RAW_REPORT_INTERVAL_MS;
			}
			if (poll_remaining > next_raw_report - k_uptime_get()) {
				poll_remaining = MAX((int64_t)0, next_raw_report - k_uptime_get());
			}
			if (poll_remaining > next_time_point - k_uptime_get()) {
				poll_remaining = MAX((int64_t)0, next_time_point - k_uptime_get());
			}
#endif
			if (result.error || result.terminal_seen || atomic_get(&measure_stop) ||
			    remaining <= 0) {
				break;
			}
			k_poll_event_init(&events[0], K_POLL_TYPE_SEM_AVAILABLE,
					  K_POLL_MODE_NOTIFY_ONLY, imu_measure_rtio.consume_sem);
			k_poll_event_init(&events[1], K_POLL_TYPE_SEM_AVAILABLE,
					  K_POLL_MODE_NOTIFY_ONLY, &imu_measure_stop_signal);
			rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(poll_remaining));
			if (rc != 0 && rc != -EAGAIN) {
				result.error = rc;
			}
		}
		consume_cqes(&result, true);
		if (result.terminal_seen && !result.error && result.stream_rc != -ECANCELED) {
			/* Ended by the driver too late to restart. */
			result.error = result.stream_rc;
		}
		requested_at = k_uptime_get();
		result.stop_requested_ms = requested_at - start;
		result.stopped_early = atomic_get(&measure_stop) != 0;
		if (active_handle != RTIO_SQE_HANDLE_INVALID) {
			/* Stop the producer without canceling the in-flight RTIO SQE.
			 * Cancellation could free a completed FIFO buffer before the
			 * consumer receives it. The driver instead pauses batching,
			 * delivers the remaining words, then posts a terminal CQE.
			 */
			result.graceful_requested = true;
			rc = lsm6dso_fifo_request_stop(dev, &imu_measure_rtio, active_handle);
			if (rc != 0 && rc != -ENOENT) {
				result.stop_error = rc;
			}
			cleanup_deadline = k_uptime_get() + IMU_MEASURE_CLEANUP_MS;
			while (!result.terminal_seen && k_uptime_get() < cleanup_deadline) {
				struct k_poll_event event;
				int64_t remaining;

				consume_cqes(&result, true);
				remaining = cleanup_deadline - k_uptime_get();
				if (result.terminal_seen || remaining <= 0) {
					break;
				}
				k_poll_event_init(&event, K_POLL_TYPE_SEM_AVAILABLE,
						  K_POLL_MODE_NOTIFY_ONLY, imu_measure_rtio.consume_sem);
				rc = k_poll(&event, 1, K_MSEC(remaining));
				if (rc != 0 && rc != -EAGAIN && result.stop_error == 0) {
					result.stop_error = rc;
					break;
				}
			}
			consume_cqes(&result, true);
			lsm6dso_fifo_get_stats(dev, &cleanup_fifo);
			result.cleanup_timeout =
				!result.terminal_seen ||
				rtio_iodev_sqe_from_handle(&imu_measure_rtio, active_handle) != NULL ||
				(result.error != -EBUSY && cleanup_fifo.active &&
				 cleanup_fifo.generation == expected_generation);
			if (result.cleanup_timeout) {
				/* Forced cancellation can lose in-flight data. Refuse another
				 * run until reboot so a late CQE cannot contaminate it.
				 */
				(void)rtio_sqe_cancel(&imu_measure_rtio, active_handle);
				atomic_set(&recovery_required, 1);
			} else {
				active_handle = RTIO_SQE_HANDLE_INVALID;
			}
		}
#if defined(CONFIG_IMU_RAW_BATCH_DIAGNOSTIC)
		if (!result.raw_error) {
			result.raw_error = imu_raw_batch_flush(&raw_batch);
		}
		result.raw_batches = raw_batch.delivered_batches;
		result.raw_accel = raw_batch.next_accel;
		result.raw_gyro = raw_batch.next_gyro;
		result.raw_timestamps = raw_batch.timestamp_words;
		result.raw_dropped_batches = raw_batch.dropped_batches;
		result.raw_dropped_accel = raw_batch.dropped_accel;
		result.raw_dropped_gyro = raw_batch.dropped_gyro;
		result.raw_epoch = raw_batch.meta.epoch_id;
		result.raw_time = raw_batch.time;
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		result.raw_time_point_failures = raw_time_point.failures;
		result.raw_time_point_window_us = raw_time_point.window_us;
		result.raw_unix_unavailable = raw_unix_unavailable;
		result.raw_tick_holds = raw_tick_clock.held;
		result.raw_us_per_million_ticks =
			raw_tick_clock.rate_us * 1000000LL / (int64_t)raw_tick_clock.rate_ticks;
#endif
#endif
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		(void)raw_report(dev, result.raw_error, result.error, expected_generation,
				 &earlier_fifo, IMU_RAW_STATE_STOPPED, !raw_info_sent);
#endif
		end = k_uptime_get();
		result.elapsed_ms = end - start;
#ifdef CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK
		result.cpu_active_permille = cpu_load_get(true);
		result.cpu_active_valid = result.cpu_active_valid && result.cpu_active_permille >= 0;
#endif
		result.cpu_valid = result.cpu_valid && k_thread_runtime_stats_cpu_get(0, &after) == 0 &&
				   after.execution_cycles >= before.execution_cycles &&
				   after.total_cycles >= before.total_cycles;
		if (result.cpu_valid) {
			result.cpu_total_cycles = after.execution_cycles - before.execution_cycles;
			result.cpu_busy_cycles = after.total_cycles - before.total_cycles;
		}
		imu_cpu_profile_stop();
		lsm6dso_fifo_get_stats(dev, &result.fifo);
		/* A different stream could start before the stats snapshot. Never
		 * compare that run's counters with this run's completions.
		 */
		result.fifo_valid = result.error != -EBUSY &&
			result.fifo.generation == expected_generation;
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		/* Driver statistics restart with each FIFO start; report the run. */
		fifo_stats_add(&result.fifo, &earlier_fifo);
#endif
		result.dropped_cqes = (uint32_t)atomic_get(&imu_measure_rtio.xcqcnt) - dropped_before;
		k_mutex_lock(&imu_measure_result_lock, K_FOREVER);
		last_result = result;
		k_mutex_unlock(&imu_measure_result_lock);
		atomic_clear(&measure_running);
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
		imu_raw_mqtt_end();
#endif
		shell_print(measure_shell, "IMU measurement finished:");
		print_result(measure_shell, &result);
		imu_cpu_profile_print(measure_shell, result.cpu_total_cycles, result.cpu_busy_cycles);
	}
}
K_THREAD_DEFINE(imu_measure_tid, IMU_MEASURE_STACK_SIZE, measure_thread, NULL, NULL, NULL, 9, 0, 0);

static int cmd_measure_start(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
	struct lsm6dso_fifo_stats fifo;
	char *end;
	unsigned long seconds;

	ARG_UNUSED(argc);
	seconds = strtoul(argv[1], &end, 10);
	if (argv[1][0] < '0' || argv[1][0] > '9' || *end || seconds == 0 ||
	    seconds > IMU_MEASURE_MAX_SECONDS) {
		shell_error(shell, "Duration must be 1..%u seconds", IMU_MEASURE_MAX_SECONDS);
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	if (atomic_get(&recovery_required)) {
		shell_error(shell, "Previous cleanup timed out; reboot before another measurement");
		return -EBUSY;
	}
	if (!atomic_cas(&measure_running, 0, 1)) {
		shell_error(shell, "Measurement already running");
		return -EBUSY;
	}
	lsm6dso_fifo_get_stats(dev, &fifo);
	if (fifo.active || (active_handle != RTIO_SQE_HANDLE_INVALID &&
	    rtio_iodev_sqe_from_handle(&imu_measure_rtio, active_handle))) {
		atomic_clear(&measure_running);
		shell_error(shell, "IMU is busy or recovery is incomplete");
		return -EBUSY;
	}
#if defined(CONFIG_IMU_RAW_MQTT_EXPERIMENTAL)
	if (imu_raw_mqtt_begin(measurement_session)) {
		atomic_clear(&measure_running);
		shell_error(shell, "Configure a new RAW MQTT session before measuring");
		return -EINVAL;
	}
#endif
	active_handle = RTIO_SQE_HANDLE_INVALID;
	requested_seconds = seconds;
	measure_shell = shell;
	atomic_clear(&measure_stop);
	k_sem_reset(&imu_measure_stop_signal);
	shell_print(shell, "Starting measurement for %lu seconds", seconds);
	k_sem_give(&imu_measure_request);
	return 0;
}

static int cmd_measure_stop(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!atomic_get(&measure_running)) {
		shell_print(shell, "No measurement running");
		return 0;
	}
	atomic_set(&measure_stop, 1);
	k_sem_give(&imu_measure_stop_signal);
	shell_print(shell, "Stopping measurement");
	return 0;
}

static int cmd_measure_result(const struct shell *shell, size_t argc, char **argv)
{
	struct imu_measure_result result;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_mutex_lock(&imu_measure_result_lock, K_FOREVER);
	result = last_result;
	k_mutex_unlock(&imu_measure_result_lock);
	if (atomic_get(&measure_running)) {
		shell_print(shell, "Measurement running; showing previous result");
	}
	print_result(shell, &result);
	return 0;
}

#if defined(CONFIG_IMU_FAULT_INJECTION)
static int cmd_inject_stall(const struct shell *shell, size_t argc, char **argv)
{
	char *end;
	unsigned long ms = strtoul(argv[1], &end, 10);

	ARG_UNUSED(argc);
	if (argv[1][0] < '0' || argv[1][0] > '9' || *end || ms == 0 || ms > 2000) {
		shell_error(shell, "Stall must be 1..2000 ms");
		return -EINVAL;
	}
	if (!atomic_get(&measure_running)) {
		shell_error(shell, "No measurement running");
		return -EINVAL;
	}
	atomic_set(&inject_stall_ms, (atomic_val_t)ms);
	shell_print(shell, "Stalling FIFO handling for %lu ms", ms);
	return 0;
}
#endif

static int cmd_measure_stats(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
	struct lsm6dso_fifo_stats stats;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	lsm6dso_fifo_get_stats(dev, &stats);
	shell_print(shell, "words=%" PRIu64 " transfers=%" PRIu32,
		    stats.words_read, stats.transfers);
	shell_print(shell, "transfer_ns=%" PRIu64 " max_transfer_ns=%" PRIu64,
		    stats.transfer_time_ns, stats.max_transfer_time_ns);
	shell_print(shell, "i2c_errors=%" PRIu32 " buffer_too_small=%" PRIu32
		    " overrun_observations=%" PRIu32,
		    stats.i2c_errors, stats.buffer_too_small, stats.overrun_observations);
	return 0;
}

#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
static int cmd_timestamp_decimation(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
	char *end;
	unsigned long value;
	int rc;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	if (argc == 1) {
		shell_print(shell, "timestamp_decimation=%u",
			    lsm6dso_fifo_get_timestamp_decimation(dev));
		return 0;
	}
	value = strtoul(argv[1], &end, 10);
	if (argv[1][0] < '0' || argv[1][0] > '9' || *end ||
	    (value != 0 && value != 1 && value != 8 && value != 32)) {
		shell_error(shell, "Use 0 (off), 1, 8, or 32");
		return -EINVAL;
	}
	rc = lsm6dso_fifo_set_timestamp_decimation(dev, (unsigned int)value);
	if (rc) {
		shell_error(shell, "Cannot change decimation while FIFO is active or recovering (%d)", rc);
		return rc;
	}
	shell_print(shell, "timestamp_decimation=%lu (applied at next stream start)", value);
	return 0;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(imu_measure_commands,
	SHELL_CMD_ARG(start, NULL, "Start: imu_measure start <seconds>", cmd_measure_start, 2, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop the current measurement", cmd_measure_stop, 1, 0),
	SHELL_CMD_ARG(result, NULL, "Show the last measurement", cmd_measure_result, 1, 0),
	SHELL_CMD_ARG(stats, NULL, "Show current IMU FIFO counters", cmd_measure_stats, 1, 0),
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
	SHELL_CMD_ARG(timestamp_decimation, NULL, "Get or set FIFO timestamp decimation (0, 1, 8, 32)",
		      cmd_timestamp_decimation, 1, 1),
#endif
#if defined(CONFIG_IMU_FAULT_INJECTION)
	SHELL_CMD_ARG(inject_stall, NULL, "Test: stall FIFO handling for <ms> to force an overrun",
		      cmd_inject_stall, 2, 0),
#endif
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(imu_measure, &imu_measure_commands, "Measure IMU FIFO streaming", NULL);
