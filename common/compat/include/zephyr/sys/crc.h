/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/sys/crc.h> (Zephyr v4.5.0) that the copies
 * of Zephyr's NVS and settings (common/compat/subsys) refer to, with
 * Zephyr's prototypes:
 *
 *   crc8_ccitt()  the CRC of NVS's allocation table entries; the copy of
 *                 Zephyr's subsys/crc/crc8_sw.c (common/compat/subsys/crc)
 *                 implements it with crc8_rohc() and crc8(),
 *   crc16_ccitt() only with CONFIG_SETTINGS_NVS_NAME_CACHE (off),
 *   crc32_ieee()  only with CONFIG_NVS_DATA_CRC (off).
 *
 * The last two are declared, not implemented: the code that calls them is
 * compiled out with the Zephyr build's configuration
 * (zephyr_kv_config.h).  Part of the small Zephyr compatibility layer of
 * the port (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_CRC_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_CRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/toolchain.h>

uint8_t crc8_ccitt(uint8_t initial_value, const void *buf, size_t len);
uint8_t crc8_rohc(uint8_t initial_value, const void *buf, size_t len);
uint8_t crc8(const uint8_t *src, size_t len, uint8_t polynomial,
             uint8_t initial_value, bool reversed);
uint16_t crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len);
uint32_t crc32_ieee(const uint8_t *data, size_t len);

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_CRC_H */
