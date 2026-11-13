/* SPDX-License-Identifier: Apache-2.0 */
/* Host build of Zephyr's own MQTT library: what mqtt_os.h takes from the
 * kernel beyond the compatibility layer's <zephyr/kernel.h>
 * (k_uptime_get_32(), the byte order helpers).
 */
#ifndef TEST_MQTT_REF_NET_CORE_H
#define TEST_MQTT_REF_NET_CORE_H
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

static inline uint32_t k_uptime_get_32(void)
{
	return (uint32_t)k_uptime_get();
}
#endif
