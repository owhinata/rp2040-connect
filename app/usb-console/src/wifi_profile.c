/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include "wifi_profile.h"

LOG_MODULE_REGISTER(wifi_profile, LOG_LEVEL_INF);
#define PROFILE_VERSION 1
struct wifi_profile {
	uint8_t version, ssid_len, password_len, security;
	uint8_t ssid[32];
	uint8_t password[63];
};
static struct wifi_profile saved, pending;
static bool saved_valid, pending_valid, settings_ready;
K_MUTEX_DEFINE(profile_lock);

static bool profile_valid(const struct wifi_profile *p)
{
	return p->version == PROFILE_VERSION && p->ssid_len > 0 &&
	       p->ssid_len <= sizeof(p->ssid) &&
	       ((p->security == WIFI_SECURITY_TYPE_NONE &&
		 p->password_len == 0) ||
		(p->security == WIFI_SECURITY_TYPE_PSK &&
		 p->password_len >= 8 &&
		 p->password_len <= sizeof(p->password)));
}

static int load_profile(const char *name, size_t len, settings_read_cb read_cb,
			void *cb_arg)
{
	struct wifi_profile value;
	ssize_t n;

	if (strcmp(name, "last")) {
		return -ENOENT;
	}
	if (len != sizeof(value)) {
		return -EINVAL;
	}
	n = read_cb(cb_arg, &value, sizeof(value));
	if (n < 0) {
		return n;
	}
	if (n != sizeof(value) || !profile_valid(&value)) {
		return -EINVAL;
	}
	k_mutex_lock(&profile_lock, K_FOREVER);
	saved = value;
	saved_valid = true;
	k_mutex_unlock(&profile_lock);
	memset(&value, 0, sizeof(value));
	return 0;
}
SETTINGS_STATIC_HANDLER_DEFINE(wifi_last, "wifi_last", NULL, load_profile, NULL,
			       NULL);

void wifi_profile_prepare(const struct wifi_connect_req_params *params)
{
	k_mutex_lock(&profile_lock, K_FOREVER);
	memset(&pending, 0, sizeof(pending));
	pending.version = PROFILE_VERSION;
	pending.ssid_len = params->ssid_length;
	pending.security = params->security;
	memcpy(pending.ssid, params->ssid, pending.ssid_len);
	if (params->security == WIFI_SECURITY_TYPE_PSK) {
		pending.password_len = params->psk_length;
		memcpy(pending.password, params->psk, pending.password_len);
	}
	pending_valid = true;
	k_mutex_unlock(&profile_lock);
}

void wifi_profile_discard(void)
{
	k_mutex_lock(&profile_lock, K_FOREVER);
	memset(&pending, 0, sizeof(pending));
	pending_valid = false;
	k_mutex_unlock(&profile_lock);
}

void wifi_profile_commit(void)
{
	int ret = 0;

	k_mutex_lock(&profile_lock, K_FOREVER);
	if (settings_ready && pending_valid &&
	    (!saved_valid || memcmp(&saved, &pending, sizeof(saved)))) {
		ret = settings_save_one("wifi_last/last", &pending,
					sizeof(pending));
		if (!ret) {
			saved = pending;
			saved_valid = true;
		}
	}
	memset(&pending, 0, sizeof(pending));
	pending_valid = false;
	k_mutex_unlock(&profile_lock);
	if (ret) {
		LOG_ERR("Could not save Wi-Fi profile: %d", ret);
	}
}

int wifi_profile_init(void)
{
	struct wifi_profile copy;
	struct wifi_connect_req_params params = {0};
	struct net_if *iface;
	int ret = settings_subsys_init();

	if (!ret) {
		ret = settings_load_subtree("wifi_last");
	}
	if (ret) {
		LOG_ERR("Could not load Wi-Fi profile: %d", ret);
		return ret;
	}
	settings_ready = true;

	k_mutex_lock(&profile_lock, K_FOREVER);
	if (!saved_valid) {
		k_mutex_unlock(&profile_lock);
		return 0;
	}
	copy = saved;
	k_mutex_unlock(&profile_lock);

	/* Allow NINA's Wi-Fi service to finish starting after its SPI reset. */
	k_sleep(K_SECONDS(4));
	iface = net_if_get_wifi_sta();
	if (!iface) {
		memset(&copy, 0, sizeof(copy));
		return -ENODEV;
	}
	params.ssid = copy.ssid;
	params.ssid_length = copy.ssid_len;
	params.security = copy.security;
	params.channel = WIFI_CHANNEL_ANY;
	params.psk = copy.password;
	params.psk_length = copy.password_len;

	/* NINA may initially report disconnected after reset. Retry a bounded
	 * number of times, while allowing the user to cancel with wifi_forget.
	 */
	for (int attempt = 0; attempt < 3; attempt++) {
		bool unchanged;

		k_mutex_lock(&profile_lock, K_FOREVER);
		unchanged = saved_valid && !memcmp(&saved, &copy, sizeof(copy));
		k_mutex_unlock(&profile_lock);
		if (!unchanged ||
		    net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)) {
			break;
		}
		ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params,
			       sizeof(params));
		if (ret && ret != -EBUSY && ret != -EALREADY) {
			LOG_WRN("Stored Wi-Fi connection request failed: %d",
				ret);
		} else if (!ret) {
			LOG_INF("Stored Wi-Fi connection requested");
		}
		for (int i = 0; i < 32; i++) {
			k_sleep(K_SECONDS(1));
			if (net_if_ipv4_get_global_addr(iface,
							NET_ADDR_PREFERRED)) {
				break;
			}
			k_mutex_lock(&profile_lock, K_FOREVER);
			unchanged = saved_valid &&
				    !memcmp(&saved, &copy, sizeof(copy));
			k_mutex_unlock(&profile_lock);
			if (!unchanged) {
				break;
			}
		}
	}
	memset(&copy, 0, sizeof(copy));
	return 0;
}

static int forget_profile(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_mutex_lock(&profile_lock, K_FOREVER);
	ret = settings_ready ? settings_delete("wifi_last/last") : -EIO;
	if (!ret) {
		memset(&saved, 0, sizeof(saved));
		saved_valid = false;
		memset(&pending, 0, sizeof(pending));
		pending_valid = false;
	}
	k_mutex_unlock(&profile_lock);
	if (ret) {
		shell_error(sh, "Could not delete Wi-Fi profile: %d", ret);
	} else {
		shell_print(sh, "Saved Wi-Fi profile deleted");
	}
	return ret;
}
SHELL_CMD_REGISTER(wifi_forget, NULL, "Delete saved Wi-Fi profile",
		   forget_profile);
