/* SPDX-License-Identifier: Apache-2.0 */
/* Host build of Zephyr's own MQTT library: the network log macros, compiled
 * out as at CONFIG_MQTT_LOG_LEVEL=0.
 */
#ifndef TEST_MQTT_REF_NET_LOG_H
#define TEST_MQTT_REF_NET_LOG_H
#define NET_DBG(...) do { } while (0)
#define NET_INFO(...) do { } while (0)
#define NET_WARN(...) do { } while (0)
#define NET_ERR(...) do { } while (0)
#define NET_HEXDUMP_DBG(...) do { } while (0)
#endif
