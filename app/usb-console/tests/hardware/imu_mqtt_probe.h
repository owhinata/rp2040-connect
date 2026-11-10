/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_MQTT_PROBE_H_
#define IMU_MQTT_PROBE_H_
#include <stddef.h>
#include <stdint.h>
int imu_mqtt_probe_submit(const uint8_t *buffer, size_t length);
#endif
