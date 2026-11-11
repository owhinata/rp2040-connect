/* SPDX-License-Identifier: Apache-2.0 */
/* Experimental RAW-only transport. Session and device IDs are supplied by the
 * operator; info, status and time synchronization are not implemented yet. */
#include "imu_raw_mqtt.h"
#include "imu_raw_queue.h"

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

K_MUTEX_DEFINE(raw_queue_lock);
K_SEM_DEFINE(raw_start, 0, 1);
static struct imu_raw_queue queue;
static struct imu_raw_queue_item outgoing;
static struct mqtt_client client;
static struct sockaddr_in broker;
static uint8_t rx_buf[512], tx_buf[512], session[16];
static char topic[80], status_topic[80], info_topic[80], client_id[64], device_id[41];
static char json[512];
#define STATUS_INTERVAL_MS 1000
/* Protected by report_lock. */
static struct k_spinlock report_lock;
static struct imu_raw_status reported_status;
static struct imu_raw_info reported_info;
static bool info_valid, info_pending, status_forced;
static atomic_t connects, status_published, info_published;
static uint16_t next_message_id;
static atomic_t running, worker_active, acquisition_active, session_claimed, connected;
static atomic_t published, failures, lock_drops;
static struct k_spinlock publish_stats_lock;
static uint64_t publish_total_us, publish_max_us, publish_total_bytes;
/* Protected by raw_queue_lock. Includes uncontended acquisition overhead. */
static uint64_t queue_lock_wait_total_us, queue_lock_wait_max_us, queue_lock_wait_count;

static void mqtt_event(struct mqtt_client *unused, const struct mqtt_evt *evt)
{
	ARG_UNUSED(unused);
	if (evt->type == MQTT_EVT_CONNACK) {
		atomic_set(&connected, evt->result == 0);
	} else if (evt->type == MQTT_EVT_DISCONNECT) {
		atomic_clear(&connected);
	}
}

static void set_connected(bool value)
{
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
	imu_raw_queue_set_connected(&queue, value);
	k_mutex_unlock(&raw_queue_lock);
	atomic_set(&connected, value);
}

int imu_raw_mqtt_begin(uint8_t session_id[16])
{
	if (!session_id || !atomic_get(&running) ||
	    !atomic_cas(&session_claimed, 0, 1)) {
		return -EINVAL;
	}
	atomic_set(&acquisition_active, 1);
	memcpy(session_id, session, 16);
	return 0;
}

void imu_raw_mqtt_end(void)
{
	atomic_clear(&acquisition_active);
}

int imu_raw_mqtt_submit(void *context, const uint8_t *payload, size_t length)
{
	int rc;
	uint64_t started;
	uint64_t wait_us;

	ARG_UNUSED(context);
	started = k_cycle_get_64();
	rc = k_mutex_lock(&raw_queue_lock, K_FOREVER);
	if (rc) {
		atomic_inc(&lock_drops);
		return rc;
	}
	wait_us = k_cyc_to_us_ceil64(k_cycle_get_64() - started);
	queue_lock_wait_count++;
	queue_lock_wait_total_us += wait_us;
	queue_lock_wait_max_us = MAX(queue_lock_wait_max_us, wait_us);
	rc = imu_raw_queue_put(&queue, payload, length);
	k_mutex_unlock(&raw_queue_lock);
	return rc;
}

void imu_raw_mqtt_set_info(const struct imu_raw_info *info)
{
	k_spinlock_key_t key = k_spin_lock(&report_lock);

	reported_info = *info;
	info_valid = true;
	info_pending = true;
	k_spin_unlock(&report_lock, key);
}

void imu_raw_mqtt_report(const struct imu_raw_status *status)
{
	k_spinlock_key_t key = k_spin_lock(&report_lock);

	status_forced = status_forced || status->state != reported_status.state;
	reported_status = *status;
	k_spin_unlock(&report_lock, key);
}

static int publish_json(const char *json_topic, int length, enum mqtt_qos qos, bool retain)
{
	struct mqtt_publish_param p = {0};

	if (length < 0) {
		return length;
	}
	p.message.topic.topic.utf8 = (const uint8_t *)json_topic;
	p.message.topic.topic.size = strlen(json_topic);
	p.message.topic.qos = qos;
	p.message.payload.data = (uint8_t *)json;
	p.message.payload.len = length;
	p.retain_flag = retain;
	if (qos != MQTT_QOS_0_AT_MOST_ONCE) {
		if (++next_message_id == 0) {
			next_message_id = 1;
		}
		p.message_id = next_message_id;
	}
	return mqtt_publish(&client, &p);
}

/* QoS 1 only asks the broker to acknowledge; a lost info is not resent
 * until the next (re)connection or change. */
static int publish_info_if_pending(void)
{
	struct imu_raw_info info;
	k_spinlock_key_t key = k_spin_lock(&report_lock);
	bool pending = info_pending && info_valid;
	int rc;

	info = reported_info;
	info_pending = false;
	k_spin_unlock(&report_lock, key);
	if (!pending) {
		return 0;
	}
	rc = publish_json(info_topic, imu_raw_info_encode(json, sizeof(json), &info, device_id),
			  MQTT_QOS_1_AT_LEAST_ONCE, true);
	if (!rc) {
		atomic_inc(&info_published);
	}
	return rc;
}

static int publish_status(void)
{
	struct imu_raw_status status;
	k_spinlock_key_t key = k_spin_lock(&report_lock);
	int rc;

	status = reported_status;
	status_forced = false;
	k_spin_unlock(&report_lock, key);
	if (status.state == IMU_RAW_STATE_IDLE) {
		memcpy(status.session_id, session, sizeof(status.session_id));
		status.uptime_ms = k_uptime_get();
	}
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
	status.dropped_batches = queue.dropped_batches;
	status.dropped_accel = queue.dropped_accel;
	status.dropped_gyro = queue.dropped_gyro;
	status.queue_high_watermark = queue.high_watermark;
	k_mutex_unlock(&raw_queue_lock);
	status.published = (uint32_t)atomic_get(&published);
	status.connects = (uint32_t)atomic_get(&connects);
	status.failures = (uint32_t)atomic_get(&failures);
	rc = publish_json(status_topic, imu_raw_status_encode(json, sizeof(json), &status),
			  MQTT_QOS_0_AT_MOST_ONCE, false);
	if (!rc) {
		atomic_inc(&status_published);
	}
	return rc;
}

static bool status_is_forced(void)
{
	k_spinlock_key_t key = k_spin_lock(&report_lock);
	bool forced = status_forced;

	k_spin_unlock(&report_lock, key);
	return forced;
}

static int open_connection(void)
{
	struct zsock_pollfd fd;
	int rc;

	mqtt_client_init(&client);
	client.broker = (struct sockaddr *)&broker;
	client.evt_cb = mqtt_event;
	client.client_id.utf8 = (uint8_t *)client_id;
	client.client_id.size = strlen(client_id);
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.transport.type = MQTT_TRANSPORT_NON_SECURE;
	client.rx_buf = rx_buf;
	client.rx_buf_size = sizeof(rx_buf);
	client.tx_buf = tx_buf;
	client.tx_buf_size = sizeof(tx_buf);
	rc = mqtt_connect(&client);
	if (rc) return rc;
	fd.fd = client.transport.tcp.sock;
	fd.events = ZSOCK_POLLIN;
	rc = zsock_poll(&fd, 1, 5000);
	if (rc <= 0) return rc ? -errno : -ETIMEDOUT;
	rc = mqtt_input(&client);
	if (rc) return rc;
	return atomic_get(&connected) ? 0 : -ECONNREFUSED;
}

static int publish_item(void)
{
	struct mqtt_publish_param p = {0};

	p.message.topic.topic.utf8 = (uint8_t *)topic;
	p.message.topic.topic.size = strlen(topic);
	p.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
	p.message.payload.data = outgoing.payload;
	p.message.payload.len = outgoing.length;
	return mqtt_publish(&client, &p);
}

static void raw_mqtt_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&raw_start, K_FOREVER);
		while (atomic_get(&running)) {
			int rc = open_connection();
			if (rc || !atomic_get(&running)) {
				(void)mqtt_abort(&client);
				set_connected(false);
				if (rc) atomic_inc(&failures);
				k_sleep(K_SECONDS(1));
				continue;
			}
			set_connected(true);
			atomic_inc(&connects);
			int64_t next_input_poll = 0;
			int64_t next_status = 0;
			k_spinlock_key_t report_key = k_spin_lock(&report_lock);

			info_pending = info_valid;
			k_spin_unlock(&report_lock, report_key);
			while (atomic_get(&running) && atomic_get(&connected)) {
				struct zsock_pollfd fd = {.fd = client.transport.tcp.sock,
							 .events = ZSOCK_POLLIN};
				int taken;

				rc = publish_info_if_pending();
				if (rc) break;
				if (k_uptime_get() >= next_status || status_is_forced()) {
					rc = publish_status();
					if (rc) break;
					next_status = k_uptime_get() + STATUS_INTERVAL_MS;
				}
				k_mutex_lock(&raw_queue_lock, K_FOREVER);
				taken = imu_raw_queue_take(&queue, &outgoing);
				k_mutex_unlock(&raw_queue_lock);
				if (!taken) {
					uint32_t started = k_cycle_get_32();
					rc = publish_item();
					uint64_t elapsed = k_cyc_to_us_floor64(
						(uint32_t)(k_cycle_get_32() - started));
					k_spinlock_key_t key = k_spin_lock(&publish_stats_lock);
					publish_total_us += elapsed;
					publish_max_us = MAX(publish_max_us, elapsed);
					publish_total_bytes += outgoing.length;
					k_spin_unlock(&publish_stats_lock, key);
					if (rc) {
						k_mutex_lock(&raw_queue_lock, K_FOREVER);
						imu_raw_queue_discard(&queue, &outgoing);
						k_mutex_unlock(&raw_queue_lock);
						break;
					}
					atomic_inc(&published);
				}
				if (k_uptime_get() >= next_input_poll) {
					rc = zsock_poll(&fd, 1, 0);
					if (rc < 0 || (rc > 0 && (fd.revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP | ZSOCK_POLLNVAL)))) {
						rc = -EIO;
						break;
					}
					if (rc > 0 && (fd.revents & ZSOCK_POLLIN) &&
					    (rc = mqtt_input(&client))) break;
					rc = mqtt_live(&client);
					if (rc && rc != -EAGAIN) break;
					next_input_poll = k_uptime_get() + 100;
				}
				if (taken) {
					k_sleep(K_MSEC(5));
				}
			}
			set_connected(false);
			(void)mqtt_abort(&client);
			if (rc && rc != -EAGAIN) atomic_inc(&failures);
		}
		atomic_clear(&worker_active);
	}
}
/* FIFO worker priority is 10; mutex priority inheritance may temporarily boost
 * this transport thread while it releases the queue lock. */
K_THREAD_DEFINE(imu_raw_mqtt_tid, 3072, raw_mqtt_thread, NULL, NULL, NULL, 11, 0, 0);

static int hex_digit(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static int raw_cmd(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 2 && !strcmp(argv[1], "stop")) {
		atomic_clear(&running);
		set_connected(false);
	} else if (argc == 6 && !strcmp(argv[1], "start")) {
		struct in_addr address;
		uint8_t new_session[16];
		char *end;
		unsigned long port = strtoul(argv[3], &end, 10);
		size_t device_len = strlen(argv[4]);

		if (atomic_get(&worker_active) || atomic_get(&acquisition_active) ||
		    argv[3][0] < '0' || argv[3][0] > '9' || *end || !port || port > 65535 ||
		    !device_len || device_len > 40 || strlen(argv[5]) != 32 ||
		    zsock_inet_pton(AF_INET, argv[2], &address) != 1) return -EINVAL;
		for (size_t i = 0; i < device_len; i++) {
			char c = argv[4][i];
			if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
			      (c >= '0' && c <= '9') || c == '-' || c == '_')) return -EINVAL;
		}
		for (size_t i = 0; i < 16; i++) {
			int hi = hex_digit(argv[5][i * 2]), lo = hex_digit(argv[5][i * 2 + 1]);
			if (hi < 0 || lo < 0) return -EINVAL;
			new_session[i] = (hi << 4) | lo;
		}
		broker.sin_family = AF_INET; broker.sin_port = htons(port); broker.sin_addr = address;
		memcpy(session, new_session, sizeof(session));
		snprintk(topic, sizeof(topic), "imu/%s/raw", argv[4]);
		snprintk(status_topic, sizeof(status_topic), "imu/%s/status", argv[4]);
		snprintk(info_topic, sizeof(info_topic), "imu/%s/info", argv[4]);
		snprintk(device_id, sizeof(device_id), "%s", argv[4]);
		k_spinlock_key_t report_key = k_spin_lock(&report_lock);
		memset(&reported_status, 0, sizeof(reported_status));
		info_valid = false;
		info_pending = false;
		status_forced = false;
		k_spin_unlock(&report_lock, report_key);
		atomic_clear(&connects); atomic_clear(&status_published); atomic_clear(&info_published);
		snprintk(client_id, sizeof(client_id), "imu-%s", argv[4]);
		k_mutex_lock(&raw_queue_lock, K_FOREVER);
		imu_raw_queue_init(&queue);
		queue_lock_wait_count = 0;
		queue_lock_wait_total_us = 0;
		queue_lock_wait_max_us = 0;
		k_mutex_unlock(&raw_queue_lock);
		atomic_clear(&session_claimed); atomic_clear(&published); atomic_clear(&failures);
		atomic_clear(&lock_drops); atomic_set(&worker_active, 1); atomic_set(&running, 1);
		k_spinlock_key_t key = k_spin_lock(&publish_stats_lock);
		publish_total_us = 0;
		publish_max_us = 0;
		publish_total_bytes = 0;
		k_spin_unlock(&publish_stats_lock, key);
		k_sem_give(&raw_start);
	} else if (argc != 2 || strcmp(argv[1], "status")) {
		shell_error(sh, "Use start <IPv4> <port> <device_id> <32-hex-session> | stop | status");
		return -EINVAL;
	}
	size_t queued, high_watermark;
	uint64_t dropped_batches, dropped_accel, dropped_gyro;
	uint64_t total_us, max_us, total_bytes;
	uint64_t wait_count, wait_total_us, wait_max_us;
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
	queued = queue.count;
	high_watermark = queue.high_watermark;
	dropped_batches = queue.dropped_batches;
	dropped_accel = queue.dropped_accel;
	dropped_gyro = queue.dropped_gyro;
	wait_count = queue_lock_wait_count;
	wait_total_us = queue_lock_wait_total_us;
	wait_max_us = queue_lock_wait_max_us;
	k_mutex_unlock(&raw_queue_lock);
	k_spinlock_key_t key = k_spin_lock(&publish_stats_lock);
	total_us = publish_total_us;
	max_us = publish_max_us;
	total_bytes = publish_total_bytes;
	k_spin_unlock(&publish_stats_lock, key);
	shell_print(sh, "running=%d connected=%d claimed=%d queued=%zu high_watermark=%zu published=%d failures=%d lock_drops=%d",
		    (int)atomic_get(&running), (int)atomic_get(&connected), (int)atomic_get(&session_claimed),
		    queued, high_watermark, (int)atomic_get(&published), (int)atomic_get(&failures),
		    (int)atomic_get(&lock_drops));
	shell_print(sh, "queue_dropped_batches=%" PRIu64 " accel=%" PRIu64 " gyro=%" PRIu64,
		    dropped_batches, dropped_accel, dropped_gyro);
	shell_print(sh, "queue_lock_wait_count=%" PRIu64 " total_us=%" PRIu64 " max_us=%" PRIu64,
		    wait_count, wait_total_us, wait_max_us);
	shell_print(sh, "publish_us=%" PRIu64 " publish_max_us=%" PRIu64
		    " publish_bytes=%" PRIu64, total_us, max_us, total_bytes);
	shell_print(sh, "connects=%d status_published=%d info_published=%d",
		    (int)atomic_get(&connects), (int)atomic_get(&status_published),
		    (int)atomic_get(&info_published));
	return 0;
}
SHELL_CMD_REGISTER(imu_raw_mqtt, NULL, "Experimental RAW MQTT transport", raw_cmd);
