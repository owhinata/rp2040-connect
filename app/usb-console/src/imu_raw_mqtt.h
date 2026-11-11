/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_MQTT_H_
#define IMU_RAW_MQTT_H_
#include <stddef.h>
#include <stdint.h>

/* Claim one externally supplied session for the next measurement. */
int imu_raw_mqtt_begin(uint8_t session_id[16]);
void imu_raw_mqtt_end(void);
/* Thread context only. May wait for the queue mutex; never waits on the network. */
int imu_raw_mqtt_submit(void *context, const uint8_t *payload, size_t length);
#endif
