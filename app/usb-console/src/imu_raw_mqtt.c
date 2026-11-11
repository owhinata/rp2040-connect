/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_raw_mqtt.h"
#include "imu_raw_queue.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/atomic.h>

#define STATUS_INTERVAL_MS  1000
#define INPUT_POLL_MS       100
#define RECONNECT_DELAY_MS  1000
#define CONNACK_WAIT_MS     5000
#define STOP_FLUSH_MS       500

K_MUTEX_DEFINE(raw_queue_lock);
K_SEM_DEFINE(raw_start, 0, 1);
static struct imu_raw_queue queue;
static struct imu_raw_queue_item outgoing;
static struct mqtt_client client;
static struct sockaddr_in broker;
static struct in_addr configured_address; /* 0.0.0.0: use the default gateway */
static uint8_t rx_buf[512], tx_buf[512], session[16];
static char topic[80], status_topic[80], info_topic[80], client_id[64];
static char device_id[IMU_RAW_MQTT_DEVICE_ID_MAX + 1];
static char json[768];
static atomic_t running, worker_active, connected;
static atomic_t published, failures, connects, status_published, info_published;
static struct k_spinlock publish_stats_lock;
static uint64_t publish_total_us, publish_max_us, publish_total_bytes;
/* Protected by raw_queue_lock. Includes uncontended acquisition overhead. */
static uint64_t queue_lock_wait_total_us, queue_lock_wait_max_us, queue_lock_wait_count;
/* Protected by report_lock. */
static struct k_spinlock report_lock;
static struct imu_raw_status reported_status;
static struct imu_raw_info reported_info;
static bool info_valid, info_pending, status_forced;
static uint16_t next_message_id;

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

int imu_raw_mqtt_submit(void *context, const uint8_t *payload, size_t length)
{
	uint64_t started, wait_us;
	int rc;

	ARG_UNUSED(context);
	started = k_cycle_get_64();
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
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

	if (configured_address.s_addr) {
		broker.sin_addr = configured_address;
	} else {
		struct net_if *iface = net_if_get_default();

		broker.sin_addr.s_addr = 0;
		if (iface) {
			broker.sin_addr = net_if_ipv4_get_gw(iface);
		}
		if (!broker.sin_addr.s_addr) {
			return -ENETUNREACH; /* not on a network yet */
		}
	}
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
	if (rc) {
		return rc;
	}
	fd.fd = client.transport.tcp.sock;
	fd.events = ZSOCK_POLLIN;
	rc = zsock_poll(&fd, 1, CONNACK_WAIT_MS);
	if (rc <= 0) {
		return rc ? -errno : -ETIMEDOUT;
	}
	rc = mqtt_input(&client);
	if (rc) {
		return rc;
	}
	return atomic_get(&connected) ? 0 : -ECONNREFUSED;
}

static int publish_batch(void)
{
	struct mqtt_publish_param p = {0};
	uint32_t started = k_cycle_get_32();
	uint64_t elapsed;
	k_spinlock_key_t key;
	int rc;

	p.message.topic.topic.utf8 = (uint8_t *)topic;
	p.message.topic.topic.size = strlen(topic);
	p.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
	p.message.payload.data = outgoing.payload;
	p.message.payload.len = outgoing.length;
	rc = mqtt_publish(&client, &p);
	elapsed = k_cyc_to_us_floor64((uint32_t)(k_cycle_get_32() - started));
	key = k_spin_lock(&publish_stats_lock);
	publish_total_us += elapsed;
	publish_max_us = MAX(publish_max_us, elapsed);
	publish_total_bytes += outgoing.length;
	k_spin_unlock(&publish_stats_lock, key);
	if (rc) {
		k_mutex_lock(&raw_queue_lock, K_FOREVER);
		imu_raw_queue_discard(&queue, &outgoing);
		k_mutex_unlock(&raw_queue_lock);
	} else {
		atomic_inc(&published);
	}
	return rc;
}

/* Keep-alive and anything the broker sends (CONNACK aside, only PUBACK and
 * PINGRESP are expected). Each check is a command to the Wi-Fi module. */
static int service_input(void)
{
	struct zsock_pollfd fd = {.fd = client.transport.tcp.sock, .events = ZSOCK_POLLIN};
	int rc = zsock_poll(&fd, 1, 0);

	if (rc < 0 || (rc > 0 && (fd.revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP | ZSOCK_POLLNVAL)))) {
		return -EIO;
	}
	if (rc > 0 && (fd.revents & ZSOCK_POLLIN)) {
		rc = mqtt_input(&client);
		if (rc) {
			return rc;
		}
	}
	rc = mqtt_live(&client);
	return rc == -EAGAIN ? 0 : rc;
}

static int serve_connection(void)
{
	int64_t next_input_poll = 0, next_status = 0;
	k_spinlock_key_t key = k_spin_lock(&report_lock);
	int rc = 0;

	info_pending = info_valid;
	k_spin_unlock(&report_lock, key);
	while (!rc && atomic_get(&running) && atomic_get(&connected)) {
		int taken;

		rc = publish_info_if_pending();
		if (rc) {
			break;
		}
		if (k_uptime_get() >= next_status || status_is_forced()) {
			rc = publish_status();
			if (rc) {
				break;
			}
			next_status = k_uptime_get() + STATUS_INTERVAL_MS;
		}
		k_mutex_lock(&raw_queue_lock, K_FOREVER);
		taken = imu_raw_queue_take(&queue, &outgoing);
		k_mutex_unlock(&raw_queue_lock);
		if (!taken) {
			rc = publish_batch();
			if (rc) {
				break;
			}
		}
		if (k_uptime_get() >= next_input_poll) {
			rc = service_input();
			next_input_poll = k_uptime_get() + INPUT_POLL_MS;
		}
		if (taken) {
			k_sleep(K_MSEC(5));
		}
	}
	return rc;
}

static void raw_mqtt_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&raw_start, K_FOREVER);
		while (atomic_get(&running)) {
			int rc = open_connection();

			if (!rc && atomic_get(&running)) {
				set_connected(true);
				atomic_inc(&connects);
				rc = serve_connection();
			}
			set_connected(false);
			(void)mqtt_abort(&client);
			if (rc) {
				atomic_inc(&failures);
			}
			if (atomic_get(&running)) {
				k_sleep(K_MSEC(RECONNECT_DELAY_MS));
			}
		}
		atomic_clear(&worker_active);
	}
}
/* The FIFO worker has priority 10 and the batch builder 9; mutex priority
 * inheritance may briefly boost this thread while it holds the queue lock. */
/* Measured peak stack use: 1256 bytes, including a reconnection. */
K_THREAD_DEFINE(imu_raw_mqtt_tid, 2048, raw_mqtt_thread, NULL, NULL, NULL, 11, 0, 0);

int imu_raw_mqtt_start(const struct imu_raw_mqtt_config *config)
{
	size_t length = strlen(config->device_id);
	k_spinlock_key_t key;

	if (atomic_get(&worker_active)) {
		return -EBUSY;
	}
	if (!config->port || !length || length > IMU_RAW_MQTT_DEVICE_ID_MAX) {
		return -EINVAL;
	}
	for (size_t i = 0; i < length; i++) {
		char c = config->device_id[i];

		if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
		      c == '-' || c == '_')) {
			return -EINVAL;
		}
	}
	broker.sin_family = AF_INET;
	broker.sin_port = htons(config->port);
	broker.sin_addr = config->address;
	configured_address = config->address;
	memcpy(session, config->session_id, sizeof(session));
	strcpy(device_id, config->device_id);
	snprintk(topic, sizeof(topic), "imu/%s/raw", device_id);
	snprintk(status_topic, sizeof(status_topic), "imu/%s/status", device_id);
	snprintk(info_topic, sizeof(info_topic), "imu/%s/info", device_id);
	snprintk(client_id, sizeof(client_id), "imu-%s", device_id);
	key = k_spin_lock(&report_lock);
	memset(&reported_status, 0, sizeof(reported_status));
	info_valid = false;
	info_pending = false;
	status_forced = false;
	k_spin_unlock(&report_lock, key);
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
	imu_raw_queue_init(&queue);
	queue_lock_wait_count = 0;
	queue_lock_wait_total_us = 0;
	queue_lock_wait_max_us = 0;
	k_mutex_unlock(&raw_queue_lock);
	key = k_spin_lock(&publish_stats_lock);
	publish_total_us = 0;
	publish_max_us = 0;
	publish_total_bytes = 0;
	k_spin_unlock(&publish_stats_lock, key);
	atomic_clear(&published);
	atomic_clear(&failures);
	atomic_clear(&connects);
	atomic_clear(&status_published);
	atomic_clear(&info_published);
	atomic_set(&worker_active, 1);
	atomic_set(&running, 1);
	k_sem_give(&raw_start);
	return 0;
}

void imu_raw_mqtt_stop(void)
{
	int64_t deadline = k_uptime_get() + STOP_FLUSH_MS;

	while (atomic_get(&connected) && k_uptime_get() < deadline) {
		size_t queued;

		k_mutex_lock(&raw_queue_lock, K_FOREVER);
		queued = queue.count;
		k_mutex_unlock(&raw_queue_lock);
		if (!queued && !status_is_forced()) {
			break;
		}
		k_sleep(K_MSEC(10));
	}
	atomic_clear(&running);
	set_connected(false);
}

void imu_raw_mqtt_get_stats(struct imu_raw_mqtt_stats *stats)
{
	k_spinlock_key_t key;

	memset(stats, 0, sizeof(*stats));
	stats->running = atomic_get(&running);
	stats->connected = atomic_get(&connected);
	stats->via_gateway = !configured_address.s_addr;
	stats->address = broker.sin_addr;
	stats->port = ntohs(broker.sin_port);
	stats->published = (uint32_t)atomic_get(&published);
	stats->failures = (uint32_t)atomic_get(&failures);
	stats->connects = (uint32_t)atomic_get(&connects);
	stats->status_published = (uint32_t)atomic_get(&status_published);
	stats->info_published = (uint32_t)atomic_get(&info_published);
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
	stats->queued = queue.count;
	stats->queue_high = queue.high_watermark;
	stats->dropped_batches = queue.dropped_batches;
	stats->dropped_accel = queue.dropped_accel;
	stats->dropped_gyro = queue.dropped_gyro;
	stats->lock_wait_count = queue_lock_wait_count;
	stats->lock_wait_total_us = queue_lock_wait_total_us;
	stats->lock_wait_max_us = queue_lock_wait_max_us;
	k_mutex_unlock(&raw_queue_lock);
	key = k_spin_lock(&publish_stats_lock);
	stats->publish_total_us = publish_total_us;
	stats->publish_max_us = publish_max_us;
	stats->publish_bytes = publish_total_bytes;
	k_spin_unlock(&publish_stats_lock, key);
}
