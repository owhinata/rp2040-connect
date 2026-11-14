/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* NuttX port: Zephyr's subsys/net/lib/wifi_credentials/
 * wifi_credentials_backend_settings.c (Zephyr v4.5.0) on the settings of
 * the Zephyr compatibility layer (<zephyr/settings/settings.h>: NuttX's
 * NVS, /dev/config), with the same names ("wifi_cred/<index>"), checks,
 * log messages and errors.  Zephyr reads the subtree "wifi_cred" with
 * settings_load_subtree_direct() and calls a callback for each stored
 * name; the layer reads one name at a time, and the library writes only
 * "wifi_cred/0" to "wifi_cred/<CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES - 1>",
 * so the two callbacks become reads of those names:
 * - zephyr_settings_backend_load_key_cb() (the SSID cache at start) runs for
 *   each index that is stored; its "index too large" and "no key" cases
 *   cannot occur (no other name is read),
 * - zephyr_settings_backend_load_val_cb() (one entry) reads the entry's
 *   name: -ENOENT when it is not stored, as when Zephyr finds no match.
 * The entry buffer is aligned for the struct cast on it (the Cortex-M0+
 * faults on an unaligned word access), and the index formats are %zu
 * (-Wformat of NuttX's build).
 */

#include <nuttx/config.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "app_log.h"
#include <stdlib.h>
#include <zephyr/settings/settings.h>

#include "wifi_credentials_internal.h"

LOG_MODULE_REGISTER(wifi_credentials_backend, CONFIG_WIFI_CREDENTIALS_LOG_LEVEL);

#define WIFI_CREDENTIALS_SBE_BASE_KEY "wifi_cred"
#define WIFI_CREDENTIALS_SBE_KEY_SIZE                                                              \
	sizeof(WIFI_CREDENTIALS_SBE_BASE_KEY "/" STRINGIFY(CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES))
#define WIFI_CREDENTIALS_SBE_KEY_FMT WIFI_CREDENTIALS_SBE_BASE_KEY "/%zu"

/* NuttX: the name of index idx (the formatting of store_entry()). */
static int settings_name(char *buf, size_t size, size_t idx)
{
	int ret = snprintk(buf, size, WIFI_CREDENTIALS_SBE_KEY_FMT, idx);

	if (ret < 0 || ret == size) {
		LOG_ERR("WiFi credentials settings key could not be generated, idx: %zu", idx);
		return -EFAULT;
	}
	return 0;
}

/* zephyr_settings_backend_load_key_cb() for the entry of index idx, of len
 * stored bytes, read into buf.
 */
static int zephyr_settings_backend_load_key_cb(size_t idx, size_t len, uint8_t *buf,
					       size_t length_read)
{
	if (len < sizeof(struct wifi_credentials_header)) {
		LOG_ERR("Settings error: invalid settings length");
		return -EINVAL;
	}

	/* value validation */
	if (length_read < len) {
		LOG_ERR("Settings error: entry incomplete");
		return -ENODATA;
	}

	wifi_credentials_cache_ssid(idx, (struct wifi_credentials_header *)buf);
	return 0;
}

int wifi_credentials_backend_init(void)
{
	int ret = settings_subsys_init();

	if (ret) {
		LOG_ERR("Initializing settings subsystem failed: %d", ret);
		return ret;
	}

	for (size_t idx = 0; idx < CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES; idx++) {
		char settings_name_buf[WIFI_CREDENTIALS_SBE_KEY_SIZE] = {0};
		uint8_t buf[ENTRY_MAX_LEN] __aligned(8);
		ssize_t len;

		ret = settings_name(settings_name_buf, ARRAY_SIZE(settings_name_buf), idx);
		if (ret) {
			return ret;
		}
		len = settings_load_one(settings_name_buf, buf, ARRAY_SIZE(buf));
		if (len < 0) {
			return len;
		}
		if (len == 0) {
			continue;
		}
		ret = zephyr_settings_backend_load_key_cb(idx, len, buf,
							  MIN((size_t)len, ARRAY_SIZE(buf)));
		if (ret) {
			return ret;
		}
	}

	return 0;
}

int wifi_credentials_store_entry(size_t idx, const void *buf, size_t buf_len)
{
	char settings_name_buf[WIFI_CREDENTIALS_SBE_KEY_SIZE] = {0};

	int ret = snprintk(settings_name_buf, ARRAY_SIZE(settings_name_buf),
			   WIFI_CREDENTIALS_SBE_KEY_FMT, idx);

	if (ret < 0 || ret == ARRAY_SIZE(settings_name_buf)) {
		LOG_ERR("WiFi credentials settings key could not be generated, idx: %zu", idx);
		return -EFAULT;
	}

	return settings_save_one(settings_name_buf, buf, buf_len);
}

int wifi_credentials_delete_entry(size_t idx)
{
	char settings_name_buf[WIFI_CREDENTIALS_SBE_KEY_SIZE] = {0};

	int ret = snprintk(settings_name_buf, ARRAY_SIZE(settings_name_buf),
			   WIFI_CREDENTIALS_SBE_KEY_FMT, idx);

	if (ret < 0 || ret == ARRAY_SIZE(settings_name_buf)) {
		LOG_ERR("WiFi credentials settings key could not be generated, idx: %zu", idx);
		return -EFAULT;
	}

	return settings_delete(settings_name_buf);
}

int wifi_credentials_load_entry(size_t idx, void *buf, size_t buf_len)
{
	char settings_name_buf[WIFI_CREDENTIALS_SBE_KEY_SIZE] = {0};
	ssize_t len;
	int ret;

	ret = settings_name(settings_name_buf, ARRAY_SIZE(settings_name_buf), idx);
	if (ret) {
		return ret;
	}

	/* zephyr_settings_backend_load_val_cb() on the entry's name. */
	len = settings_load_one(settings_name_buf, buf, buf_len);
	if (len < 0) {
		return len;
	}

	if (len == 0) {
		return -ENOENT;
	}

	if (len != buf_len) {
		LOG_ERR("Settings error: invalid settings length");
		return -EINVAL;
	}

	return 0;
}
