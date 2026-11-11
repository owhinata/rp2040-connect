/* SPDX-License-Identifier: Apache-2.0 */
/* IMU acquisition service: reads the LSM6DSO FIFO stream, numbers and times
 * the samples, and hands RAW batches to the MQTT transport. Started and
 * stopped from the shell ("imu start", "imu stop"); acquisition does not
 * depend on the network once it runs. See design/imu-raw-mqtt-design-ja.md.
 */
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/cpu_load.h>

#include "imu_raw_batch.h"
#include "imu_raw_mqtt.h"
#include "imu_tick_clock.h"
#include "kv_store.h"
#include "time_sync.h"
#include "../drivers/lsm6dso/lsm6dso_fifo.h"
#include "../drivers/lsm6dso/lsm6dso_fifo_decoder.h"
#include "../drivers/lsm6dso/lsm6dso_fifo_stats.h"

/* Settings read from the key-value store by "imu start"; all are optional. */
#define KEY_HOST      "imu.host"      /* IPv4 text; default: the default gateway,
				       * looked up at every connection attempt */
#define KEY_PORT      "imu.port"      /* decimal text; default DEFAULT_PORT */
#define KEY_DEVICE    "imu.device"    /* topic name part; default DEFAULT_DEVICE */
#define KEY_ODR       "imu.odr"       /* Hz for both sensors; default DEFAULT_ODR_HZ */
#define KEY_WATERMARK "imu.watermark" /* FIFO words; default DEFAULT_WATERMARK */
#define KEY_SESSION   "imu.session"   /* 4-byte counter kept by this module */
#define DEFAULT_PORT      1883
#define DEFAULT_DEVICE    "rp2040"
#define DEFAULT_ODR_HZ    6667
#define DEFAULT_WATERMARK 128

#define CLEANUP_MS              2000
#define CONNECT_WAIT_MS         3000
#define CQE_BATCH               8
/* A partly filled batch is sent this long after its last sample. */
#define FLUSH_INTERVAL_MS       100
#define REPORT_INTERVAL_MS      500
#define TIME_POINT_INTERVAL_MS  1000
/* After the driver ends a stream with an error it has already stopped the
 * FIFO. Give it this long to release the stream, and give up when this many
 * restarts in a row deliver no data. */
#define RECOVERY_RELEASE_MS      200
#define RECOVERY_MAX_CONSECUTIVE 3

SENSOR_DT_STREAM_IODEV(imu_iodev, DT_NODELABEL(lsm6dsox),
	{ SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE });
RTIO_DEFINE_WITH_MEMPOOL(imu_rtio, 4, 16, 128, 64, 4);
K_SEM_DEFINE(start_request, 0, 1);
K_SEM_DEFINE(stop_signal, 0, 1);
K_MUTEX_DEFINE(snapshot_lock);

/* What "imu status" shows. The stream thread works on its own copy and
 * publishes it under snapshot_lock. */
struct imu_stream_status {
	bool started, running;
	uint8_t session_id[16];
	int64_t start_us, end_us; /* board clock (time_sync_mono_us) */
	uint64_t packets, accel, gyro, batches, timestamp_words;
	uint64_t unsent_batches; /* refused by the transport: disconnected or queue full */
	uint32_t epoch, recoveries, discontinuities;
	int last_recovery_error;
	/* error: why the stream ended, 0 for a requested stop. */
	int error, batch_error, stop_error;
	struct lsm6dso_fifo_stats fifo; /* summed over FIFO restarts */
	uint32_t dropped_completions, bufferless_completions, invalid_packets;
	bool cleanup_timeout;
	struct imu_raw_time time;
	uint32_t time_point_failures, time_point_window_us, unix_unavailable, monotonic_holds;
	int64_t us_per_million_ticks;
	int cpu_permille; /* at the end of the run; negative if unavailable */
};

static struct imu_stream_status snapshot;
static atomic_t stream_running, stop_requested, restart_blocked;
static struct imu_raw_mqtt_config transport_config;
static rtio_sqe_handle_t active_handle = RTIO_SQE_HANDLE_INVALID;

/* Owned by the stream thread. */
static struct imu_stream_status run;
static struct imu_raw_batch batch;
static struct imu_tick_clock tick_clock;
static bool tick_clock_ready, terminal_seen, stopping;
static int stream_rc;
static struct lsm6dso_fifo_stats earlier_fifo;
static uint32_t expected_generation;
static struct {
	bool valid, unix_valid;
	uint64_t tick;
	int64_t mono_us, unix_us, unix_estimate_us;
	uint32_t sync_delay_us;
} time_point;

static void fifo_stats_add(struct lsm6dso_fifo_stats *total, const struct lsm6dso_fifo_stats *part)
{
	total->words_read += part->words_read;
	total->transfer_time_ns += part->transfer_time_ns;
	total->max_transfer_time_ns = MAX(total->max_transfer_time_ns, part->max_transfer_time_ns);
	total->transfers += part->transfers;
	total->i2c_errors += part->i2c_errors;
	total->buffer_too_small += part->buffer_too_small;
	total->overrun_observations += part->overrun_observations;
}

/* Errors after which the driver has stopped the FIFO and a restart can
 * continue the session in a new epoch. */
static bool recoverable(int rc)
{
	return rc == -EOVERFLOW || rc == -ENOMEM || rc == -ENOBUFS || rc == -EIO;
}

/* Server time of a sample for the batch header. */
static bool tick_to_unix(void *context, uint64_t tick, uint64_t *unix_us)
{
	int64_t mono_us, time_us;

	ARG_UNUSED(context);
	if (!imu_tick_clock_convert(&tick_clock, tick, &mono_us) ||
	    !time_sync_unix_us(mono_us, &time_us, NULL) || time_us <= 0) {
		run.unix_unavailable++;
		return false;
	}
	*unix_us = (uint64_t)time_us;
	return true;
}

/* Reads the sensor's tick counter together with the board clock. */
static void take_time_point(const struct device *dev)
{
	const struct imu_raw_time *fifo_time = &batch.time;
	int64_t before, after;
	uint64_t wraps;
	uint32_t tick;

	if (lsm6dso_timestamp_read(dev, time_sync_mono_us, &tick, &before, &after)) {
		run.time_point_failures++;
		return;
	}
	/* Extend like the FIFO timestamps. The register is read later than
	 * the newest FIFO word, so it may already be past a wrap. */
	wraps = fifo_time->wraps;
	if (fifo_time->raw_seen && tick < fifo_time->last_raw &&
	    fifo_time->last_raw - tick > UINT32_MAX / 2U) {
		wraps++;
	}
	time_point.tick = (wraps << 32) | tick;
	time_point.mono_us = before + (after - before) / 2;
	run.time_point_window_us = (uint32_t)MIN(after - before, (int64_t)UINT32_MAX);
	imu_tick_clock_add(&tick_clock, time_point.tick, time_point.mono_us);
	/* Converted now: a point is reported more than once, and the smoothed
	 * clock only answers for recent times. */
	time_point.unix_valid =
		time_sync_unix_us(time_point.mono_us, &time_point.unix_us,
				  &time_point.sync_delay_us) &&
		time_sync_estimate_us(time_point.mono_us, &time_point.unix_estimate_us, NULL);
	time_point.valid = true;
}

/* Refreshes the counters, publishes them for "imu status" and hands them to
 * the transport. Returns true once the driver statistics belong to this run
 * (the FIFO starts asynchronously), which is when info can be sent. */
static bool report(const struct device *dev, enum imu_raw_state state, bool send_info)
{
	struct imu_raw_status status = {.state = state};
	struct lsm6dso_fifo_stats fifo;
	bool current;

	lsm6dso_fifo_get_stats(dev, &fifo);
	current = fifo.generation == expected_generation;
	run.fifo = earlier_fifo;
	if (current) {
		fifo_stats_add(&run.fifo, &fifo);
		run.fifo.accel_hz = fifo.accel_hz;
		run.fifo.gyro_hz = fifo.gyro_hz;
		run.fifo.fault = fifo.fault;
	}
	run.accel = batch.next_accel;
	run.gyro = batch.next_gyro;
	run.batches = batch.next_batch;
	run.unsent_batches = batch.dropped_batches;
	run.timestamp_words = batch.timestamp_words;
	run.epoch = batch.meta.epoch_id;
	run.time = batch.time;
	run.monotonic_holds = tick_clock.held;
	run.us_per_million_ticks = tick_clock.rate_us * 1000000LL / (int64_t)tick_clock.rate_ticks;
	run.end_us = time_sync_mono_us();
	k_mutex_lock(&snapshot_lock, K_FOREVER);
	snapshot = run;
	k_mutex_unlock(&snapshot_lock);

	memcpy(status.session_id, run.session_id, sizeof(status.session_id));
	status.epoch_id = run.epoch;
	status.uptime_ms = (uint64_t)k_uptime_get();
	status.accel_acquired = run.accel;
	status.gyro_acquired = run.gyro;
	status.next_batch = run.batches;
	status.timestamp_words = run.timestamp_words;
	status.fifo_overrun_observations = run.fifo.overrun_observations;
	status.i2c_errors = run.fifo.i2c_errors;
	status.raw_error = run.batch_error;
	status.measure_error = run.error;
	if (time_point.valid) {
		status.tick_valid = true;
		status.tick = time_point.tick;
		status.tick_mono_us = time_point.mono_us;
		status.tick_window_us = run.time_point_window_us;
		status.unix_valid = time_point.unix_valid;
		status.unix_us = time_point.unix_us;
		status.unix_estimate_us = time_point.unix_estimate_us;
		status.sync_delay_us = time_point.sync_delay_us;
	}
	if (current && send_info) {
		struct imu_raw_info info = {
			.epoch_id = run.epoch,
			.accel_odr_hz = fifo.accel_hz, .gyro_odr_hz = fifo.gyro_hz,
			.accel_ug_per_lsb = fifo.accel_ug_per_lsb,
			.gyro_udps_per_lsb = fifo.gyro_udps_per_lsb,
			.fifo_watermark = lsm6dso_fifo_get_watermark(dev),
			.timestamp_decimation = (uint8_t)lsm6dso_fifo_get_timestamp_decimation(dev),
			.freq_fine = fifo.freq_fine,
		};

		memcpy(info.session_id, run.session_id, sizeof(info.session_id));
		imu_raw_mqtt_set_info(&info);
	}
	imu_raw_mqtt_report(&status);
	return current;
}

static void consume_packet(const uint8_t *buffer, uint32_t length)
{
	const struct lsm6dso_fifo_packet *packet = (const void *)buffer;
	size_t cursor = 0;
	int rc;

	if (length < offsetof(struct lsm6dso_fifo_packet, data) ||
	    packet->marker != LSM6DSO_FIFO_PACKET_MARKER ||
	    packet->words > LSM6DSO_FIFO_MAX_WORDS ||
	    length - offsetof(struct lsm6dso_fifo_packet, data) <
		    (size_t)packet->words * LSM6DSO_FIFO_WORD_SIZE) {
		run.invalid_packets++;
		if (!run.error) {
			run.error = -EBADMSG;
		}
		return;
	}
	run.packets++;
	rc = imu_raw_batch_consume(&batch, packet->data,
				   packet->words * LSM6DSO_FIFO_WORD_SIZE, &cursor);
	if (rc == -EPROTO) {
		/* An unknown word: the rest of this packet cannot be trusted.
		 * Mark the gap instead of ending the stream. */
		run.discontinuities++;
		rc = imu_raw_batch_new_epoch(&batch);
	}
	if (rc) {
		run.batch_error = rc;
		if (!run.error) {
			run.error = rc;
		}
	}
}

static void consume_completions(void)
{
	struct rtio_cqe *cqe;

	for (unsigned int i = 0; i < CQE_BATCH && (cqe = rtio_cqe_consume(&imu_rtio)) != NULL; i++) {
		uint8_t *buffer = NULL;
		uint32_t length = 0;
		int rc = cqe->result;
		int buffer_rc = rtio_cqe_get_mempool_buffer(&imu_rtio, cqe, &buffer, &length);

		rtio_cqe_release(&imu_rtio, cqe);
		if (rc < 0) {
			terminal_seen = true;
			stream_rc = rc;
			/* A recoverable error while running is left to the stream
			 * loop, which restarts the stream or records the error. */
			if ((rc != -ECANCELED || !stopping) && !run.error &&
			    (stopping || !recoverable(rc))) {
				run.error = rc;
			}
		} else if (buffer_rc != 0 || !buffer || !length) {
			run.bufferless_completions++;
			if (!run.error) {
				run.error = -ENODATA;
			}
		} else {
			consume_packet(buffer, length);
		}
		if (buffer) {
			rtio_release_buffer(&imu_rtio, buffer, length);
		}
	}
}

/* The driver ended the stream and stopped the FIFO, losing an unknown number
 * of samples. Continue in a new epoch unless the sensor cannot be restarted.
 * Returns false with run.error set when giving up. */
static bool restart_stream(const struct device *dev, uint64_t *packets_at_restart,
			   unsigned int *restarts_without_data)
{
	struct lsm6dso_fifo_stats ended;
	int64_t release_deadline = k_uptime_get() + RECOVERY_RELEASE_MS;
	bool released;
	int rc;

	for (;;) {
		lsm6dso_fifo_get_stats(dev, &ended);
		released = !ended.active && !rtio_iodev_sqe_from_handle(&imu_rtio, active_handle);
		if (released || k_uptime_get() >= release_deadline) {
			break;
		}
		k_sleep(K_MSEC(1));
	}
	*restarts_without_data = run.packets == *packets_at_restart ? *restarts_without_data + 1U : 0U;
	if (!released || ended.fault || *restarts_without_data >= RECOVERY_MAX_CONSECUTIVE) {
		run.error = stream_rc;
		return false;
	}
	rc = imu_raw_batch_new_epoch(&batch);
	if (rc) {
		run.batch_error = rc;
		run.error = rc;
		return false;
	}
	if (ended.generation == expected_generation) {
		fifo_stats_add(&earlier_fifo, &ended);
	}
	run.recoveries++;
	run.last_recovery_error = stream_rc;
	*packets_at_restart = run.packets;
	expected_generation = ended.generation + 1U;
	terminal_seen = false;
	rc = sensor_stream(&imu_iodev, &imu_rtio, NULL, &active_handle);
	if (rc) {
		run.error = rc;
		return false;
	}
	return true;
}

/* Stops the producer without canceling the in-flight RTIO request, which
 * could free a completed FIFO buffer before it is consumed. The driver pauses
 * batching, delivers the remaining words, then posts a terminal completion. */
static void stop_stream(const struct device *dev)
{
	struct lsm6dso_fifo_stats after;
	int64_t deadline;
	int rc;

	if (active_handle == RTIO_SQE_HANDLE_INVALID) {
		return;
	}
	stopping = true;
	rc = lsm6dso_fifo_request_stop(dev, &imu_rtio, active_handle);
	if (rc != 0 && rc != -ENOENT) {
		run.stop_error = rc;
	}
	deadline = k_uptime_get() + CLEANUP_MS;
	while (!terminal_seen && k_uptime_get() < deadline) {
		struct k_poll_event event;
		int64_t remaining;

		consume_completions();
		remaining = deadline - k_uptime_get();
		if (terminal_seen || remaining <= 0) {
			break;
		}
		k_poll_event_init(&event, K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
				  imu_rtio.consume_sem);
		rc = k_poll(&event, 1, K_MSEC(remaining));
		if (rc != 0 && rc != -EAGAIN && !run.stop_error) {
			run.stop_error = rc;
			break;
		}
	}
	consume_completions();
	lsm6dso_fifo_get_stats(dev, &after);
	run.cleanup_timeout = !terminal_seen ||
		rtio_iodev_sqe_from_handle(&imu_rtio, active_handle) != NULL ||
		(after.active && after.generation == expected_generation);
	if (run.cleanup_timeout) {
		/* Forced cancellation can lose in-flight data. Refuse another
		 * start until reboot so a late completion cannot contaminate it. */
		(void)rtio_sqe_cancel(&imu_rtio, active_handle);
		atomic_set(&restart_blocked, 1);
	} else {
		active_handle = RTIO_SQE_HANDLE_INVALID;
	}
}

static void stream_once(const struct device *dev)
{
	struct lsm6dso_fifo_stats initial;
	int64_t next_report = 0, next_time_point = 0, next_flush = 0;
	uint64_t observed_batch = 0, packets_at_restart = 0;
	unsigned int restarts_without_data = 0;
	uint32_t dropped_before;
	bool flush_timed = false, info_sent = false;
	int rc;

	/* Completions left over from an earlier run are not ours. */
	for (struct rtio_cqe *cqe; (cqe = rtio_cqe_consume(&imu_rtio)) != NULL;) {
		uint8_t *buffer = NULL;
		uint32_t length = 0;

		(void)rtio_cqe_get_mempool_buffer(&imu_rtio, cqe, &buffer, &length);
		rtio_cqe_release(&imu_rtio, cqe);
		if (buffer) {
			rtio_release_buffer(&imu_rtio, buffer, length);
		}
	}
	memset(&run, 0, sizeof(run));
	memset(&earlier_fifo, 0, sizeof(earlier_fifo));
	memset(&time_point, 0, sizeof(time_point));
	terminal_seen = false;
	stopping = false;
	stream_rc = 0;
	run.started = true;
	run.running = true;
	run.cpu_permille = -1;
	memcpy(run.session_id, transport_config.session_id, sizeof(run.session_id));
	run.batch_error = imu_raw_batch_init(&batch, run.session_id, 0, imu_raw_mqtt_submit, NULL);
	batch.clock = tick_to_unix;
	if (!tick_clock_ready) {
		imu_tick_clock_init(&tick_clock);
		tick_clock_ready = true;
	}
	imu_tick_clock_restart(&tick_clock);
	lsm6dso_fifo_get_stats(dev, &initial);
	expected_generation = initial.generation + 1U;
	dropped_before = (uint32_t)atomic_get(&imu_rtio.xcqcnt);
	/* Give the transport a moment to connect so that the first batches are
	 * not dropped. Acquisition starts regardless when the time is up. */
	for (int64_t deadline = k_uptime_get() + CONNECT_WAIT_MS;
	     k_uptime_get() < deadline && !atomic_get(&stop_requested);) {
		struct imu_raw_mqtt_stats transport;

		imu_raw_mqtt_get_stats(&transport);
		if (transport.connected) {
			break;
		}
		k_sleep(K_MSEC(20));
	}
	(void)cpu_load_get(true);
	run.start_us = time_sync_mono_us();
	run.error = run.batch_error;
	if (!run.error) {
		run.error = sensor_stream(&imu_iodev, &imu_rtio, NULL, &active_handle);
	}
	while (!run.error && !atomic_get(&stop_requested)) {
		struct k_poll_event events[2];
		int64_t wait = REPORT_INTERVAL_MS;

		consume_completions();
		if (terminal_seen && !run.error) {
			if (!restart_stream(dev, &packets_at_restart, &restarts_without_data)) {
				break;
			}
			info_sent = false;
			next_report = 0;
		}
		if (run.error) {
			break;
		}
		/* Send a partly filled batch once it has been idle long enough. */
		if (!batch.count) {
			flush_timed = false;
		} else if (!flush_timed || observed_batch != batch.next_batch) {
			observed_batch = batch.next_batch;
			next_flush = k_uptime_get() + FLUSH_INTERVAL_MS;
			flush_timed = true;
		}
		if (flush_timed && k_uptime_get() >= next_flush) {
			rc = imu_raw_batch_flush(&batch);
			if (rc) {
				run.batch_error = rc;
				run.error = rc;
				break;
			}
			flush_timed = false;
		}
		if (k_uptime_get() >= next_time_point) {
			struct lsm6dso_fifo_stats running;

			/* The sensor's counter is only meaningful once this run's
			 * FIFO has been configured. Until the first reading, look
			 * again soon: batches wait for it. */
			lsm6dso_fifo_get_stats(dev, &running);
			if (running.active && running.generation == expected_generation) {
				imu_tick_clock_set_trim(&tick_clock, running.freq_fine);
				take_time_point(dev);
			}
			next_time_point = k_uptime_get() +
				(time_point.valid ? TIME_POINT_INTERVAL_MS : 5);
			if (time_point.valid) {
				next_report = 0; /* report while the point is fresh */
			}
		}
		if (k_uptime_get() >= next_report) {
			info_sent = report(dev, IMU_RAW_STATE_MEASURING, !info_sent) || info_sent;
			next_report = k_uptime_get() + REPORT_INTERVAL_MS;
		}
		wait = MIN(wait, next_report - k_uptime_get());
		wait = MIN(wait, next_time_point - k_uptime_get());
		if (flush_timed) {
			wait = MIN(wait, next_flush - k_uptime_get());
		}
		k_poll_event_init(&events[0], K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
				  imu_rtio.consume_sem);
		k_poll_event_init(&events[1], K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
				  &stop_signal);
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(MAX(wait, (int64_t)0)));
		if (rc != 0 && rc != -EAGAIN) {
			run.error = rc;
		}
	}
	consume_completions();
	if (terminal_seen && !run.error && stream_rc != -ECANCELED) {
		run.error = stream_rc; /* ended by the driver too late to restart */
	}
	stop_stream(dev);
	if (!run.batch_error) {
		run.batch_error = imu_raw_batch_flush(&batch);
	}
	run.dropped_completions = (uint32_t)atomic_get(&imu_rtio.xcqcnt) - dropped_before;
	run.cpu_permille = cpu_load_get(false);
	run.running = false;
	(void)report(dev, IMU_RAW_STATE_STOPPED, !info_sent);
	imu_raw_mqtt_stop();
}

static void stream_thread(void *a, void *b, void *c)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&start_request, K_FOREVER);
		stream_once(dev);
		atomic_clear(&stream_running);
	}
}
/* Above the FIFO worker (10) so that batches are built as they arrive. */
K_THREAD_DEFINE(imu_stream_tid, 3072, stream_thread, NULL, NULL, NULL, 9, 0, 0);

static bool kv_text(const char *key, char *out, size_t capacity)
{
	ssize_t n = kv_store_get(key, out, capacity - 1);

	if (n <= 0) {
		return false;
	}
	out[n] = '\0';
	return true;
}

static bool kv_number(const char *key, unsigned long *value, unsigned long min, unsigned long max)
{
	char text[12], *end;

	if (!kv_text(key, text, sizeof(text))) {
		return false;
	}
	*value = strtoul(text, &end, 10);
	return text[0] >= '0' && text[0] <= '9' && !*end && *value >= min && *value <= max;
}

/* Unique per start across reboots: the chip's ID and a stored counter. */
static int next_session(uint8_t session[16])
{
	uint8_t stored[4];
	uint32_t counter = 0;
	ssize_t n;
	int rc;

	memset(session, 0, 16);
	n = hwinfo_get_device_id(session, 8);
	if (n <= 0) {
		return n < 0 ? (int)n : -ENODEV;
	}
	if (kv_store_get(KEY_SESSION, stored, sizeof(stored)) == sizeof(stored)) {
		counter = sys_get_be32(stored);
	}
	counter++;
	sys_put_be32(counter, stored);
	rc = kv_store_set(KEY_SESSION, stored, sizeof(stored));
	if (rc) {
		return rc;
	}
	sys_put_be32(counter, session + 8);
	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
	const struct device *bus = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6dsox)));
	struct imu_raw_mqtt_config *cfg = &transport_config;
	struct sensor_value odr = {.val1 = DEFAULT_ODR_HZ};
	unsigned long number, watermark = DEFAULT_WATERMARK;
	char text[IMU_RAW_MQTT_DEVICE_ID_MAX + 1], address[32];
	struct lsm6dso_fifo_stats fifo;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!device_is_ready(dev) || !device_is_ready(bus)) {
		return -ENODEV;
	}
	if (atomic_get(&restart_blocked)) {
		shell_error(sh, "The previous stop timed out; reboot before starting again");
		return -EBUSY;
	}
	if (!atomic_cas(&stream_running, 0, 1)) {
		shell_error(sh, "Already running");
		return -EBUSY;
	}
	lsm6dso_fifo_get_stats(dev, &fifo);
	if (fifo.active) {
		shell_error(sh, "The IMU FIFO is in use (sensor stream?)");
		rc = -EBUSY;
		goto failed;
	}
	memset(cfg, 0, sizeof(*cfg));
	if (kv_text(KEY_HOST, text, sizeof(text))) {
		if (zsock_inet_pton(AF_INET, text, &cfg->address) != 1) {
			shell_error(sh, KEY_HOST " is not an IPv4 address");
			rc = -EINVAL;
			goto failed;
		}
	}
	cfg->port = kv_number(KEY_PORT, &number, 1, 65535) ? (uint16_t)number : DEFAULT_PORT;
	strcpy(cfg->device_id, kv_text(KEY_DEVICE, text, sizeof(text)) ? text : DEFAULT_DEVICE);
	if (kv_number(KEY_ODR, &number, 1, 6667)) {
		odr.val1 = (int32_t)number;
	}
	(void)kv_number(KEY_WATERMARK, &watermark, 1, LSM6DSO_FIFO_MAX_WATERMARK);
	/* 1 MHz: at 6667 Hz the FIFO fills faster than 400 kHz can empty it. */
	rc = i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS));
	if (!rc) {
		rc = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	}
	if (!rc) {
		rc = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	}
	if (!rc) {
		rc = lsm6dso_fifo_set_watermark(dev, (uint16_t)watermark);
	}
	if (rc) {
		shell_error(sh, "Sensor setup failed (%d)", rc);
		goto failed;
	}
	rc = next_session(cfg->session_id);
	if (rc) {
		shell_error(sh, "Cannot assign a session ID (%d)", rc);
		goto failed;
	}
	rc = imu_raw_mqtt_start(cfg);
	if (rc) {
		shell_error(sh, "Transport not ready (%d)", rc);
		goto failed;
	}
	atomic_clear(&stop_requested);
	k_sem_reset(&stop_signal);
	if (cfg->address.s_addr) {
		(void)zsock_inet_ntop(AF_INET, &cfg->address, address, sizeof(address));
	} else {
		strcpy(address, "the default gateway");
	}
	shell_print(sh, "Streaming %d Hz to %s, port %u, as imu/%s/raw", odr.val1, address,
		    cfg->port, cfg->device_id);
	k_sem_give(&start_request);
	return 0;
failed:
	atomic_clear(&stream_running);
	return rc;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!atomic_get(&stream_running)) {
		shell_print(sh, "Not running");
		return 0;
	}
	atomic_set(&stop_requested, 1);
	k_sem_give(&stop_signal);
	return 0;
}

/* One item per line, grouped, so that a value that is not zero stands out. */
#define ITEM(sh, name, format, ...) shell_print(sh, "  %-23s " format, name ":", __VA_ARGS__)

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	static struct imu_stream_status s;
	struct imu_raw_mqtt_stats m;
	char text[33];
	int cpu;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_mutex_lock(&snapshot_lock, K_FOREVER);
	s = snapshot;
	k_mutex_unlock(&snapshot_lock);
	imu_raw_mqtt_get_stats(&m);
	if (!s.started) {
		shell_print(sh, "state: %s", atomic_get(&stream_running) ? "starting" : "idle");
		return 0;
	}
	for (size_t i = 0; i < 16; i++) {
		snprintk(text + i * 2, 3, "%02x", s.session_id[i]);
	}
	shell_print(sh, "state: %s", s.running ? "running" : "stopped");
	ITEM(sh, "session", "%s", text);
	ITEM(sh, "epoch", "%" PRIu32, s.epoch);
	ITEM(sh, "elapsed_ms", "%" PRId64, (s.end_us - s.start_us) / 1000);
	/* Share of time not idle, interrupts included, since the start. */
	cpu = s.running ? cpu_load_get(false) : s.cpu_permille;
	if (cpu >= 0) {
		ITEM(sh, "cpu_active", "%d.%d%%", cpu / 10, cpu % 10);
	} else {
		ITEM(sh, "cpu_active", "%s", "unavailable");
	}
	shell_print(sh, "acquired:");
	ITEM(sh, "accel", "%" PRIu64, s.accel);
	ITEM(sh, "gyro", "%" PRIu64, s.gyro);
	ITEM(sh, "batches", "%" PRIu64, s.batches);
	ITEM(sh, "fifo_packets", "%" PRIu64, s.packets);
	ITEM(sh, "timestamp_words", "%" PRIu64, s.timestamp_words);
	shell_print(sh, "errors:");
	ITEM(sh, "stream", "%d", s.error);
	ITEM(sh, "batch", "%d", s.batch_error);
	ITEM(sh, "stop", "%d", s.stop_error);
	ITEM(sh, "stop_timeout", "%d", s.cleanup_timeout);
	shell_print(sh, "fifo:");
	ITEM(sh, "overruns", "%" PRIu32, s.fifo.overrun_observations);
	ITEM(sh, "i2c_errors", "%" PRIu32, s.fifo.i2c_errors);
	ITEM(sh, "buffer_too_small", "%" PRIu32, s.fifo.buffer_too_small);
	ITEM(sh, "fault", "%d", s.fifo.fault);
	ITEM(sh, "recoveries", "%" PRIu32, s.recoveries);
	ITEM(sh, "last_recovery_error", "%d", s.last_recovery_error);
	ITEM(sh, "discontinuities", "%" PRIu32, s.discontinuities);
	ITEM(sh, "dropped_completions", "%" PRIu32, s.dropped_completions);
	ITEM(sh, "bufferless_completions", "%" PRIu32, s.bufferless_completions);
	ITEM(sh, "invalid_packets", "%" PRIu32, s.invalid_packets);
	ITEM(sh, "max_transfer_us", "%" PRIu64, s.fifo.max_transfer_time_ns / 1000);
	shell_print(sh, "time:");
	ITEM(sh, "synchronized", "%d", time_sync_is_ready());
	ITEM(sh, "unix_unavailable", "%" PRIu32, s.unix_unavailable);
	ITEM(sh, "timestamp_anchors", "%" PRIu64, s.time.anchors);
	ITEM(sh, "irregular_periods", "%" PRIu64, s.time.irregular_periods);
	ITEM(sh, "counter_mismatches", "%" PRIu64, s.time.counter_mismatches);
	ITEM(sh, "counter_skips", "%" PRIu64, s.time.counter_skips);
	ITEM(sh, "tick_read_failures", "%" PRIu32, s.time_point_failures);
	ITEM(sh, "tick_read_us", "%" PRIu32, s.time_point_window_us);
	ITEM(sh, "monotonic_holds", "%" PRIu32, s.monotonic_holds);
	ITEM(sh, "us_per_1e6_ticks", "%" PRId64, s.us_per_million_ticks);
	shell_print(sh, "mqtt:");
	if (m.address.s_addr) {
		(void)zsock_inet_ntop(AF_INET, &m.address, text, sizeof(text));
	} else {
		strcpy(text, "none");
	}
	ITEM(sh, "server", "%s:%u%s", text, m.port, m.via_gateway ? " (default gateway)" : "");
	ITEM(sh, "connected", "%d", m.connected);
	ITEM(sh, "connects", "%" PRIu32, m.connects);
	ITEM(sh, "failures", "%" PRIu32, m.failures);
	ITEM(sh, "published", "%" PRIu32, m.published);
	ITEM(sh, "dropped_batches", "%" PRIu64, m.dropped_batches);
	ITEM(sh, "dropped_accel", "%" PRIu64, m.dropped_accel);
	ITEM(sh, "dropped_gyro", "%" PRIu64, m.dropped_gyro);
	ITEM(sh, "queued", "%zu", m.queued);
	ITEM(sh, "queue_high", "%zu", m.queue_high);
	ITEM(sh, "lock_wait_max_us", "%" PRIu64, m.lock_wait_max_us);
	ITEM(sh, "publish_max_us", "%" PRIu64, m.publish_max_us);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(imu_commands,
	SHELL_CMD_ARG(start, NULL, "Start streaming to the MQTT server (settings: kv imu.*)",
		      cmd_start, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop streaming", cmd_stop, 1, 0),
	SHELL_CMD_ARG(status, NULL, "Show counters and errors of the current or last run",
		      cmd_status, 1, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(imu, &imu_commands, "IMU RAW streaming", NULL);
