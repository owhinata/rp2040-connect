/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_MQTT_H_
#define IMU_RAW_MQTT_H_
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <netinet/in.h> /* NuttX: struct in_addr (Zephyr: <zephyr/net/net_ip.h>) */
#include "imu_raw_status.h"

/* MQTT transport for RAW batches (QoS 0), info and status. It owns a bounded
 * queue: batches are dropped and counted while disconnected or when the
 * queue is full, never retried. Thread context only. */
#define IMU_RAW_MQTT_DEVICE_ID_MAX 40

struct imu_raw_mqtt_config {
	/* 0.0.0.0 means the default gateway, looked up again for every
	 * connection attempt: the session then follows the network the board
	 * is on, and can be started before Wi-Fi is connected. */
	struct in_addr address;
	uint16_t port;
	/* Letters, digits, '-' and '_'; used in topics and the client ID. */
	char device_id[IMU_RAW_MQTT_DEVICE_ID_MAX + 1];
	uint8_t session_id[16];
};

struct imu_raw_mqtt_stats {
	bool running, connected;
	bool via_gateway;       /* no fixed address configured */
	struct in_addr address; /* of the latest connection attempt; 0 if none */
	uint16_t port;
	size_t queued, queue_high;
	uint32_t published; /* accepted by the socket, not confirmed delivered */
	uint32_t failures, connects, status_published, info_published;
	uint64_t dropped_batches, dropped_accel, dropped_gyro;
	uint64_t lock_wait_count, lock_wait_total_us, lock_wait_max_us;
	uint64_t publish_total_us, publish_max_us, publish_bytes;
};

/* -EBUSY while a previous session is still shutting down, -EINVAL for a bad
 * device ID or port. Counters restart. */
int imu_raw_mqtt_start(const struct imu_raw_mqtt_config *config);
/* Gives a final status up to half a second to leave, then disconnects. */
void imu_raw_mqtt_stop(void);
/* Sink for imu_raw_batch. May wait for the queue mutex; never waits on the
 * network. Returns -EAGAIN for a batch that was dropped. */
int imu_raw_mqtt_submit(void *context, const uint8_t *payload, size_t length);
/* Neither waits on the network. Info is published on every (re)connection and
 * when it changes; status periodically, on (re)connection and when the state
 * changes. Only the acquisition fields of *status are used. Nothing is kept
 * for a disconnected period. */
void imu_raw_mqtt_set_info(const struct imu_raw_info *info);
void imu_raw_mqtt_report(const struct imu_raw_status *status);
void imu_raw_mqtt_get_stats(struct imu_raw_mqtt_stats *stats);
#endif
