/* SPDX-License-Identifier: Apache-2.0 */
/* Host build of Zephyr's own MQTT library (the reference for
 * tests/mqtt_min.c): CONFIG_MQTT_LOG_LEVEL=0, so no logging.
 */
#ifndef TEST_MQTT_REF_LOG_H
#define TEST_MQTT_REF_LOG_H
#define LOG_MODULE_REGISTER(...) extern int mqtt_ref_log_unused
#endif
