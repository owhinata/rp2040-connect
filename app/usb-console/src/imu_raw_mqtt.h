/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_MQTT_H_
#define IMU_RAW_MQTT_H_
#include <stddef.h>
#include <stdint.h>
#include "imu_raw_status.h"

/* Claim one externally supplied session for the next measurement. */
int imu_raw_mqtt_begin(uint8_t session_id[16]);
void imu_raw_mqtt_end(void);
/* Thread context only. May wait for the queue mutex; never waits on the network. */
int imu_raw_mqtt_submit(void *context, const uint8_t *payload, size_t length);
/* Thread context only; neither waits on the network. The transport publishes
 * info on every (re)connection and when it changes, and status periodically,
 * on (re)connection and when the state changes. Only the acquisition fields of
 * *status are used. Nothing is retained for a disconnected period. */
void imu_raw_mqtt_set_info(const struct imu_raw_info *info);
void imu_raw_mqtt_report(const struct imu_raw_status *status);
#endif
