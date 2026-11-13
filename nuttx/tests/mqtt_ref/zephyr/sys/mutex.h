/* SPDX-License-Identifier: Apache-2.0 */
/* Host build of Zephyr's own MQTT library: sys_mutex is a k_mutex without
 * user mode (Zephyr's <zephyr/sys/mutex.h>); the client's field is the
 * compatibility layer's struct k_mutex.
 */
#ifndef TEST_MQTT_REF_SYS_MUTEX_H
#define TEST_MQTT_REF_SYS_MUTEX_H
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

#define sys_mutex_init(m) k_mutex_init(m)
#define sys_mutex_lock(m, t) k_mutex_lock(m, t)
#define sys_mutex_unlock(m) k_mutex_unlock(m)
#endif
