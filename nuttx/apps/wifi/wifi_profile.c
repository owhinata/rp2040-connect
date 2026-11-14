/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX port: a copy of zephyr/app/usb-console/src/wifi_profile.c with only
 * the system calls replaced (nuttx/drivers/compat, the Zephyr compatibility
 * layer, keeps the Zephyr names; the log is app_log.h, the RAM log of
 * "dmesg"), and wifi_reconnect_tid declared without const (the layer's
 * K_THREAD_DEFINE() sets it when the init task starts the thread).  The old
 * profile formats that migrate_profile() converts (the KV key and the
 * settings item of earlier Zephyr builds) never exist in the port's store;
 * the function looks for them and finds nothing. */
#include <nuttx/config.h>
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "app_log.h"
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_credentials.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/settings/settings.h>
#include "kv_store.h"
#include "wifi_profile.h"

LOG_MODULE_REGISTER(wifi_profile, LOG_LEVEL_INF);

#define PROFILE_VERSION 1
#define OLD_KV_KEY      "sys.wifi_profile"
#define LEGACY_KEY      "wifi_last/last"

struct old_wifi_profile {
	uint8_t version, ssid_len, password_len, security;
	uint8_t ssid[32];
	uint8_t password[63];
};

static bool auto_enabled;
static struct net_mgmt_event_callback wifi_events;
K_SEM_DEFINE(reconnect_request, 0, 1);
K_MUTEX_DEFINE(profile_lock);
extern k_tid_t wifi_reconnect_tid;

static bool old_profile_valid(const struct old_wifi_profile *p)
{
	return p->version == PROFILE_VERSION && p->ssid_len > 0 &&
	       p->ssid_len <= sizeof(p->ssid) &&
	       ((p->security == WIFI_SECURITY_TYPE_NONE &&
		 p->password_len == 0) ||
		(p->security == WIFI_SECURITY_TYPE_PSK &&
		 p->password_len >= 8 &&
		 p->password_len <= sizeof(p->password)));
}

static int migrate_profile(void)
{
	struct old_wifi_profile old = {0};
	struct wifi_credentials_personal existing = {0};
	ssize_t len;
	int ret = 0;

	len = kv_store_get(OLD_KV_KEY, &old, sizeof(old));
	if (len == -ENOENT) {
		len = settings_load_one(LEGACY_KEY, &old, sizeof(old));
		if (len == 0) {
			goto out;
		}
	}
	if (len != sizeof(old) || !old_profile_valid(&old)) {
		ret = len < 0 ? (int)len : -EINVAL;
		goto out;
	}

	ret = wifi_credentials_get_by_ssid_personal_struct(
		(const char *)old.ssid, old.ssid_len, &existing);
	if (ret == -ENOENT) {
		ret = wifi_credentials_set_personal(
			(const char *)old.ssid, old.ssid_len, old.security,
			NULL, 0, (const char *)old.password, old.password_len,
			0, 0, 0);
	}
	if (ret) {
		goto out;
	}
	/* Never erase an old copy until standard credentials exist. */
	ret = settings_delete(LEGACY_KEY);
	if (!ret) {
		ret = kv_store_delete(OLD_KV_KEY);
	}
	if (!ret) {
		LOG_INF("Wi-Fi profile migrated to Zephyr credentials");
	}
out:
	memset(&old, 0, sizeof(old));
	memset(&existing, 0, sizeof(existing));
	return ret;
}

static void wifi_profile_connected(void)
{
	bool has_credentials = !wifi_credentials_is_empty();

	k_mutex_lock(&profile_lock, K_FOREVER);
	auto_enabled = has_credentials;
	k_mutex_unlock(&profile_lock);
}

static void wifi_event(struct net_mgmt_event_callback *cb, uint64_t event,
		       struct net_if *iface)
{
	const struct wifi_status *status = cb->info;

	ARG_UNUSED(event);
	ARG_UNUSED(iface);
	if (status && !status->status) {
		wifi_profile_connected();
	}
}

void wifi_profile_pause_reconnect(void)
{
	k_mutex_lock(&profile_lock, K_FOREVER);
	auto_enabled = false;
	k_mutex_unlock(&profile_lock);
}

bool wifi_profile_connection_allowed(void)
{
	bool allowed;

	if (k_current_get() != wifi_reconnect_tid) {
		return true;
	}
	k_mutex_lock(&profile_lock, K_FOREVER);
	allowed = auto_enabled;
	k_mutex_unlock(&profile_lock);
	return allowed;
}

void wifi_profile_request_reconnect(void)
{
	k_mutex_lock(&profile_lock, K_FOREVER);
	if (auto_enabled) {
		k_sem_give(&reconnect_request);
	}
	k_mutex_unlock(&profile_lock);
}

static void reconnect_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		struct net_if *iface;

		k_sem_take(&reconnect_request, K_FOREVER);
		k_sleep(K_SECONDS(4));
		iface = net_if_get_wifi_sta();
		if (!iface) {
			continue;
		}

		for (;;) {
			bool enabled;
			int ret;

			k_mutex_lock(&profile_lock, K_FOREVER);
			enabled = auto_enabled;
			k_mutex_unlock(&profile_lock);
			if (!enabled || wifi_credentials_is_empty() ||
			    net_if_ipv4_get_global_addr(iface,
							NET_ADDR_PREFERRED)) {
				break;
			}

			ret = net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface,
				       NULL, 0);
			if (ret && ret != -EBUSY && ret != -EALREADY) {
				LOG_WRN("Stored Wi-Fi request failed: %d", ret);
			}

			for (int i = 0; i < 32; i++) {
				k_sleep(K_SECONDS(1));
				k_mutex_lock(&profile_lock, K_FOREVER);
				enabled = auto_enabled;
				k_mutex_unlock(&profile_lock);
				if (!enabled ||
				    net_if_ipv4_get_global_addr(
					    iface, NET_ADDR_PREFERRED)) {
					break;
				}
			}
		}
	}
}
/* Measured peak stack use: 1120 bytes. */
K_THREAD_DEFINE(wifi_reconnect_tid, 2048, reconnect_thread, NULL, NULL, NULL, 8,
		0, 0);

int wifi_profile_init(void)
{
	int ret = migrate_profile();

	if (ret) {
		LOG_WRN("Wi-Fi profile migration deferred: %d", ret);
	}
	net_mgmt_init_event_callback(&wifi_events, wifi_event,
				     NET_EVENT_WIFI_CONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_events);
	wifi_profile_connected();
	wifi_profile_request_reconnect();
	return ret;
}
