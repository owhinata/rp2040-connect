/* SPDX-License-Identifier: Apache-2.0 */
/* STUB, replaced by the port of src/imu_raw_mqtt.c (T21 of
 * nuttx/PORTING_PLAN.md) once the port has sockets (T14-T16) and an MQTT
 * client (T20).
 *
 * The API of imu_raw_mqtt.h behaving as the Zephyr build's transport does
 * while the board has no network: the connection never comes up, so every
 * batch is refused and counted as dropped, and nothing is published. It is
 * the Zephyr code (zephyr/app/usb-console/src/imu_raw_mqtt.c) without the
 * MQTT client and its thread:
 *
 * - imu_raw_mqtt_start(): the same checks (-EBUSY, -EINVAL) and resets.
 *   The transport thread is not there; in the Zephyr build it would try to
 *   connect every RECONNECT_DELAY_MS and fail with -ENETUNREACH (no default
 *   gateway) or a connection error, counting each try in "failures". Here
 *   "failures" and "connects" stay 0.
 * - imu_raw_mqtt_submit(): the queue (imu_raw_queue.c) stays disconnected,
 *   so imu_raw_queue_put() checks the batch, counts it in dropped_batches,
 *   dropped_accel and dropped_gyro, and returns -EAGAIN, which
 *   imu_raw_batch counts as an unsent batch. The lock wait is measured on
 *   the board clock (time_sync_mono_us()) instead of the cycle counter.
 * - imu_raw_mqtt_set_info() and imu_raw_mqtt_report(): kept, as the Zephyr
 *   code does for the next connection; nothing reads them.
 * - imu_raw_mqtt_stop(): never connected, so it does not wait; the Zephyr
 *   transport thread clears worker_active when it notices, up to about a
 *   second later, here it is cleared at once.
 * - imu_raw_mqtt_get_stats(): the same fields. The address is 0.0.0.0 as
 *   after a failed lookup of the default gateway ("server: none:<port>").
 */
#include <nuttx/config.h> /* NuttX's headers before the compatibility layer's */
#include <arpa/inet.h>
#include "imu_raw_mqtt.h"
#include "imu_raw_queue.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "time_sync.h"

K_MUTEX_DEFINE(raw_queue_lock);
static struct imu_raw_queue queue;
static struct sockaddr_in broker;
static struct in_addr configured_address; /* 0.0.0.0: use the default gateway */
static uint8_t session[16];
static char device_id[IMU_RAW_MQTT_DEVICE_ID_MAX + 1];
static atomic_t running, worker_active, connected;
static atomic_t published, failures, connects, status_published, info_published;
/* Protected by raw_queue_lock. Includes uncontended acquisition overhead. */
static uint64_t queue_lock_wait_total_us, queue_lock_wait_max_us, queue_lock_wait_count;
/* Protected by report_lock. */
static struct k_spinlock report_lock;
static struct imu_raw_status reported_status;
static struct imu_raw_info reported_info;
static bool info_valid, info_pending, status_forced;

static void set_connected(bool value)
{
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
	imu_raw_queue_set_connected(&queue, value);
	k_mutex_unlock(&raw_queue_lock);
	atomic_set(&connected, value);
}

int imu_raw_mqtt_submit(void *context, const uint8_t *payload, size_t length)
{
	int64_t started;
	uint64_t wait_us;
	int rc;

	ARG_UNUSED(context);
	started = time_sync_mono_us();
	k_mutex_lock(&raw_queue_lock, K_FOREVER);
	wait_us = (uint64_t)(time_sync_mono_us() - started);
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
	atomic_clear(&published);
	atomic_clear(&failures);
	atomic_clear(&connects);
	atomic_clear(&status_published);
	atomic_clear(&info_published);
	atomic_set(&worker_active, 1);
	atomic_set(&running, 1);
	/* Zephyr: the transport thread starts here and looks up the default
	 * gateway for its first connection attempt; with no network the
	 * address it records is 0.0.0.0. */
	if (!configured_address.s_addr) {
		broker.sin_addr.s_addr = 0;
	}
	return 0;
}

void imu_raw_mqtt_stop(void)
{
	atomic_clear(&running);
	set_connected(false);
	atomic_clear(&worker_active);
}

void imu_raw_mqtt_get_stats(struct imu_raw_mqtt_stats *stats)
{
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
	/* No publishes: publish_total_us, publish_max_us, publish_bytes are 0. */
}
