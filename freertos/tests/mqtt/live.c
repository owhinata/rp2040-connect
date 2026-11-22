/* SPDX-License-Identifier: MIT */
/* The FreeRTOS build's MQTT transport against the host's receiver
 * (host/imu_host, "imu-host check"), on the FreeRTOS POSIX port: the same
 * sources as freertos/tests/mqtt/main.c (common/imu/imu_raw_mqtt.c,
 * imu_raw_status.c, mqtt_min.c, mqtt_min_sock.c, imu_raw_queue.c,
 * imu_raw_packet.c with the prelude; the build's interface,
 * common/compat/net_offload/zephyr_net_if.c) on the zsock_*() calls of
 * freertos/tests/telnet/host_sock.c (the host's sockets with the NINA's
 * limits and waits; a small send buffer, so that a receiver that stops
 * reading blocks the sends soon).  freertos/script/host_tests.sh starts the
 * receiver on 127.0.0.1 and then this program:
 *
 *   live <port> <seconds>
 *
 * The default gateway is 127.0.0.1 (the build's default, imu.host unset),
 * the device "rp2040".  A task at the acquisition's priority submits a
 * batch every 28 ms (380 words, half accel, half gyro, with the sensor tick
 * and the Unix time valid, as at 6667 Hz) and reports the status (tick and
 * time points) every 100 ms, for <seconds>; then the transport is stopped
 * ("imu stop") and the transport's counters are printed as
 *
 *   BOARD {"published": N, "dropped_batches": N, ...}
 *
 * for the script to compare with the receiver's result.  The session ID is
 * the board's: the chip ID 503533748a8c9218 and the counter 1.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>

#include "host_sock.h"
#include "imu_raw_mqtt.h"
#include "imu_raw_packet.h"
#include "imu_raw_status.h"
#include "priorities.h"
#include "zephyr_compat.h"
#include "zephyr_os_port.h"

#define TASK_STACK 16384

#define BATCH_WORDS      380
#define BATCH_ACCEL      (BATCH_WORDS / 2)
#define BATCH_GYRO       (BATCH_WORDS - BATCH_ACCEL)
#define BATCH_PERIOD_MS  28
#define STATUS_PERIOD_MS 100
/* The sensor's tick: 40 kHz, 6 ticks a sample at 6667 Hz; 150 us a sample. */
#define TICKS_PER_SAMPLE 6
#define US_PER_SAMPLE    150
#define SENDBUF          4096

_Thread_local int z_host_isr_depth;

void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

extern k_tid_t imu_raw_mqtt_tid;

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	(void)level;
	(void)module;
	(void)func;
	(void)fmt;
	(void)ap;
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	(void)level;
	(void)module;
	(void)func;
	(void)fmt;
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	(void)last_ms;
	(void)skipped;
	(void)rate_ms;
	*skipped_out = 0;
	return true;
}

static uint16_t port;
static int seconds;
static uint8_t session[16] = {0x50, 0x35, 0x33, 0x74, 0x8a, 0x8c, 0x92, 0x18, 0, 0, 0, 1};
static struct net_if iface;

static void die(const char *what)
{
	printf("live: %s\n", what);
	fflush(stdout);
	_exit(1);
}

static int64_t unix_base_us;

static int submit(uint64_t seq)
{
	static struct lsm6dso_fifo_sample samples[BATCH_WORDS];
	static uint8_t payload[IMU_RAW_MAX_SIZE];
	struct imu_raw_metadata m = {
		.batch_seq = seq,
		.accel_first_index = seq * BATCH_ACCEL,
		.gyro_first_index = seq * BATCH_GYRO,
		.epoch_id = 1,
		.flags = IMU_RAW_ACCEL_TIME_VALID | IMU_RAW_GYRO_TIME_VALID |
			 IMU_RAW_ACCEL_UNIX_VALID | IMU_RAW_GYRO_UNIX_VALID,
	};
	size_t length;

	memcpy(m.session_id, session, sizeof(session));
	m.accel_first_tick = m.accel_first_index * TICKS_PER_SAMPLE;
	m.gyro_first_tick = m.gyro_first_index * TICKS_PER_SAMPLE;
	m.accel_first_unix_us = (uint64_t)unix_base_us + m.accel_first_index * US_PER_SAMPLE;
	m.gyro_first_unix_us = (uint64_t)unix_base_us + m.gyro_first_index * US_PER_SAMPLE;
	for (int i = 0; i < BATCH_WORDS; i++) {
		samples[i].kind = (i & 1) ? LSM6DSO_FIFO_GYRO : LSM6DSO_FIFO_ACCEL;
		samples[i].xyz[0] = (int16_t)i;
		samples[i].xyz[1] = (int16_t)-i;
		samples[i].xyz[2] = (int16_t)(seq & 0x7fff);
	}
	if (imu_raw_encode(payload, sizeof(payload), &length, &m, samples, BATCH_WORDS)) {
		die("imu_raw_encode");
	}
	return imu_raw_mqtt_submit(NULL, payload, length);
}

static void report(enum imu_raw_state state, uint64_t samples_done, uint64_t next_batch)
{
	struct imu_raw_status s = {
		.state = state,
		.epoch_id = 1,
		.accel_acquired = samples_done,
		.gyro_acquired = samples_done,
		.next_batch = next_batch,
		.tick_valid = true,
		.unix_valid = true,
		.tick_window_us = 120,
		.sync_delay_us = 600,
	};

	memcpy(s.session_id, session, sizeof(session));
	s.tick = samples_done * TICKS_PER_SAMPLE;
	s.tick_mono_us = (int64_t)nano_mono_us();
	s.unix_us = unix_base_us + (int64_t)(samples_done * US_PER_SAMPLE);
	s.unix_estimate_us = s.unix_us;
	imu_raw_mqtt_report(&s);
}

static void acquisition(void *arg)
{
	static const struct imu_raw_info base_info = {
		.epoch_id = 1,
		.accel_odr_hz = 6667,
		.gyro_odr_hz = 6667,
		.accel_ug_per_lsb = 244,
		.gyro_udps_per_lsb = 70000,
		.fifo_watermark = 128,
		.timestamp_decimation = 1,
	};
	struct imu_raw_info info = base_info;
	struct imu_raw_mqtt_config cfg = {.port = port};
	struct imu_raw_mqtt_stats st;
	int64_t start_ms, end_ms, next_batch_ms, next_status_ms;
	uint64_t seq = 0;
	uint32_t refused = 0;

	(void)arg;
	strcpy(cfg.device_id, "rp2040");
	memcpy(cfg.session_id, session, sizeof(session));
	memcpy(info.session_id, session, sizeof(session));
	unix_base_us = (int64_t)1794900000000000LL;
	if (imu_raw_mqtt_start(&cfg)) {
		die("imu_raw_mqtt_start");
	}
	/* The acquisition's wait for the connection (CONNECT_WAIT_MS). */
	for (int i = 0; i < 150; i++) {
		imu_raw_mqtt_get_stats(&st);
		if (st.connected) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(20) + 1);
	}
	imu_raw_mqtt_set_info(&info);
	report(IMU_RAW_STATE_MEASURING, 0, 0);
	start_ms = (int64_t)(nano_mono_us() / 1000u);
	end_ms = start_ms + seconds * 1000;
	next_batch_ms = start_ms + BATCH_PERIOD_MS;
	next_status_ms = start_ms + STATUS_PERIOD_MS;
	for (;;) {
		int64_t now = (int64_t)(nano_mono_us() / 1000u);

		if (now >= end_ms) {
			break;
		}
		if (now >= next_batch_ms) {
			int rc = submit(seq);

			if (rc && rc != -EAGAIN) {
				die("submit");
			}
			refused += rc == -EAGAIN;
			seq++;
			next_batch_ms += BATCH_PERIOD_MS;
		}
		if (now >= next_status_ms) {
			report(IMU_RAW_STATE_MEASURING, seq * BATCH_ACCEL, seq);
			next_status_ms += STATUS_PERIOD_MS;
		}
		vTaskDelay(pdMS_TO_TICKS(1) + 1);
	}
	report(IMU_RAW_STATE_STOPPED, seq * BATCH_ACCEL, seq);
	imu_raw_mqtt_stop();
	imu_raw_mqtt_get_stats(&st);
	printf("BOARD {\"batches\": %llu, \"refused\": %u, \"published\": %u, "
	       "\"dropped_batches\": %llu, \"dropped_accel\": %llu, \"dropped_gyro\": %llu, "
	       "\"connects\": %u, \"failures\": %u, \"status_published\": %u, "
	       "\"info_published\": %u, \"queue_high\": %zu, \"publish_max_us\": %llu}\n",
	       (unsigned long long)seq, refused, st.published,
	       (unsigned long long)st.dropped_batches, (unsigned long long)st.dropped_accel,
	       (unsigned long long)st.dropped_gyro, st.connects, st.failures, st.status_published,
	       st.info_published, st.queue_high, (unsigned long long)st.publish_max_us);
	fflush(stdout);
	_exit(0);
}

int main(int argc, char **argv)
{
	static char out[1 << 12];
	static const struct net_in_addr loopback = {.s_addr = 0x0100007f}; /* 127.0.0.1 */

	if (argc != 3) {
		fprintf(stderr, "usage: live <port> <seconds>\n");
		return 2;
	}
	port = (uint16_t)atoi(argv[1]);
	seconds = atoi(argv[2]);
	if (!port || seconds <= 0) {
		return 2;
	}
	setvbuf(stdout, out, _IOLBF, sizeof(out));
	host_sock_connect_sndbuf = SENDBUF;
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	z_net_if_register(&iface);
	net_if_ipv4_set_gw(&iface, &loopback);
	net_if_dormant_off(&iface);
	if (K_THREAD_DEFINE_START(imu_raw_mqtt_tid) != 0 ||
	    xTaskCreate(acquisition, "acquisition", TASK_STACK / sizeof(StackType_t), NULL,
			NANO_PRIO_IMU_STREAM, NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
