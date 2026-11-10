/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/sys/byteorder.h>
#include "nina_proto.h"
#include "wifi_profile.h"

static struct net_if *nina_iface;
static uint8_t mac[6];
static struct net_in_addr assigned_ip;
static bool connected;
static atomic_t generation;
static atomic_t connecting;
static atomic_t request_generation;
static atomic_t scanning;
static struct net_if *scan_iface;
static scan_result_cb_t scan_cb;
static bool scan_reconnect;
K_MUTEX_DEFINE(connect_state_lock);
K_SEM_DEFINE(connect_request, 0, 1);
K_SEM_DEFINE(scan_request, 0, 1);
int nina_socket_create(int family, int type, int proto);

static int request_one(uint8_t cmd, const uint8_t *data, size_t length,
		       uint8_t *result, size_t cap, size_t *n)
{
	const struct nina_param p = {data, length};

	return nina_request(cmd, data ? &p : NULL, data ? 1 : 0, false, false,
			    result, cap, n);
}

static void clear_ip_config(struct net_if *iface)
{
	const struct net_in_addr no_gateway = {0};

	net_if_ipv4_addr_rm(iface, &assigned_ip);
	net_if_ipv4_set_gw(iface, &no_gateway);
	net_if_dormant_on(iface);
}

static enum wifi_security_type scan_security(uint8_t type)
{
	switch (type) {
	case 7:
		return WIFI_SECURITY_TYPE_NONE;
	case 5:
		return WIFI_SECURITY_TYPE_WEP;
	case 2:
		return WIFI_SECURITY_TYPE_WPA_PSK;
	case 4:
		return WIFI_SECURITY_TYPE_PSK;
	default:
		return WIFI_SECURITY_TYPE_UNKNOWN;
	}
}

static void scan_thread(void *a, void *b, void *c)
{
	uint8_t records[10 * (WIFI_SSID_MAX_LEN + 1)];

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		size_t len, offset = 0, n;
		uint8_t index = 0;
		struct net_if *iface;
		scan_result_cb_t cb;
		bool reconnect;
		int ret;

		k_sem_take(&scan_request, K_FOREVER);
		iface = scan_iface;
		cb = scan_cb;
		reconnect = scan_reconnect;
		ret = nina_request(0x27, NULL, 0, false, false, records,
				   sizeof(records), &len);
		while (!ret && offset < len) {
			struct wifi_scan_result result = {0};
			uint8_t value[6];
			size_t ssid_len = records[offset++];

			if (ssid_len > WIFI_SSID_MAX_LEN ||
			    ssid_len > len - offset || index >= 10) {
				ret = -EPROTO;
				break;
			}
			memcpy(result.ssid, records + offset, ssid_len);
			result.ssid_length = ssid_len;
			offset += ssid_len;
			result.band = WIFI_FREQ_BAND_2_4_GHZ;
			result.mfp = WIFI_MFP_UNKNOWN;
			ret = request_one(0x32, &index, 1, value, sizeof(value),
					  &n);
			if (ret || n != 4) {
				ret = ret ? ret : -EPROTO;
				break;
			}
			result.rssi = (int32_t)sys_get_le32(value);
			ret = request_one(0x33, &index, 1, value, sizeof(value),
					  &n);
			if (ret || n != 1) {
				ret = ret ? ret : -EPROTO;
				break;
			}
			result.security = scan_security(value[0]);
			ret = request_one(0x3c, &index, 1, value, sizeof(value),
					  &n);
			if (ret || n != WIFI_MAC_ADDR_LEN) {
				ret = ret ? ret : -EPROTO;
				break;
			}
			for (size_t i = 0; i < WIFI_MAC_ADDR_LEN; i++) {
				result.mac[i] =
					value[WIFI_MAC_ADDR_LEN - 1 - i];
			}
			result.mac_length = WIFI_MAC_ADDR_LEN;
			ret = request_one(0x3d, &index, 1, value, sizeof(value),
					  &n);
			if (ret || n != 1) {
				ret = ret ? ret : -EPROTO;
				break;
			}
			result.channel = value[0];
			cb(iface, 0, &result);
			index++;
		}
		atomic_clear(&scanning);
		cb(iface, ret, NULL);
		if (reconnect) {
			wifi_profile_request_reconnect();
		}
	}
}
K_THREAD_DEFINE(nina_scan_tid, 2048, scan_thread, NULL, NULL, NULL, 8, 0, 0);

static int wifi_scan(const struct device *dev, struct net_if *iface,
		     struct wifi_scan_params *params, scan_result_cb_t cb)
{
	bool was_connected;

	ARG_UNUSED(dev);
	if (!iface || !cb) {
		return -EINVAL;
	}
	if (params) {
		if (params->scan_type != WIFI_SCAN_TYPE_ACTIVE ||
		    params->bands || params->dwell_time_active ||
		    params->dwell_time_passive || params->max_bss_cnt) {
			return -ENOTSUP;
		}
		for (size_t i = 0; i < ARRAY_SIZE(params->ssids); i++) {
			if (params->ssids[i]) {
				return -ENOTSUP;
			}
		}
		for (size_t i = 0; i < ARRAY_SIZE(params->band_chan); i++) {
			if (params->band_chan[i].channel) {
				return -ENOTSUP;
			}
		}
	}
	k_mutex_lock(&connect_state_lock, K_FOREVER);
	if (atomic_get(&connecting) || !atomic_cas(&scanning, 0, 1)) {
		k_mutex_unlock(&connect_state_lock);
		return -EBUSY;
	}
	was_connected = connected;
	if (was_connected) {
		atomic_inc(&generation);
		connected = false;
		clear_ip_config(iface);
	}
	scan_iface = iface;
	scan_cb = cb;
	scan_reconnect = was_connected;
	k_mutex_unlock(&connect_state_lock);
	if (was_connected) {
		wifi_mgmt_raise_disconnect_result_event(iface, 0);
	}
	k_sem_give(&scan_request);
	return 0;
}

static void connect_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		uint8_t value, ip[12];
		size_t n;
		int ret = -ETIMEDOUT;
		atomic_val_t id;

		k_sem_take(&connect_request, K_FOREVER);
		id = atomic_get(&request_generation);

		for (int i = 0; i < 60 && id == atomic_get(&generation); i++) {
			ret = request_one(0x20, NULL, 0, &value, 1, &n);
			if (ret || n != 1) {
				if (!ret) {
					ret = -EPROTO;
				}
				break;
			}
			if (value == 3) {
				uint8_t no_power_save = 0, power_reply;

				/* Reapply after every association, including reconnects. */
				ret = request_one(0x17, &no_power_save, 1, &power_reply, 1, &n);
				if (ret || n != 1 || power_reply != 1) {
					ret = ret ? ret : -EIO;
					break;
				}
				ret = nina_ip_config(ip);
				if (ret) {
					break;
				}
				if (ip[0] || ip[1] || ip[2] || ip[3]) {
					k_mutex_lock(&connect_state_lock,
						     K_FOREVER);
					if (id != atomic_get(&generation)) {
						k_mutex_unlock(
							&connect_state_lock);
						ret = -ECANCELED;
						break;
					}
					memcpy(&assigned_ip, ip, 4);
					if (!net_if_ipv4_addr_add(
						    nina_iface, &assigned_ip,
						    NET_ADDR_DHCP, 0)) {
						k_mutex_unlock(
							&connect_state_lock);
						ret = -ENOMEM;
						break;
					}
					struct net_in_addr mask, gateway;
					memcpy(&mask, ip + 4, 4);
					memcpy(&gateway, ip + 8, 4);
					net_if_ipv4_set_netmask_by_addr(
						nina_iface, &assigned_ip,
						&mask);
					net_if_ipv4_set_gw(nina_iface,
							   &gateway);
					connected = true;
					net_if_dormant_off(nina_iface);
					k_mutex_unlock(&connect_state_lock);
					ret = 0;
					break;
				}
			}
			/* NINA can report the previous failure or NO_SSID while
			 * the new asynchronous connection attempt is still
			 * starting. */
			ret = -ETIMEDOUT;
			k_sleep(K_MSEC(500));
		}

		k_mutex_lock(&connect_state_lock, K_FOREVER);
		if (id == atomic_get(&generation)) {
			wifi_mgmt_raise_connect_result_event(nina_iface, ret);
		}
		atomic_clear(&connecting);
		k_mutex_unlock(&connect_state_lock);
	}
}
K_THREAD_DEFINE(nina_connect_tid, 2048, connect_thread, NULL, NULL, NULL, 8, 0,
		0);

static int wifi_connect(const struct device *dev, struct net_if *iface,
			struct wifi_connect_req_params *params)
{
	uint8_t answer;
	size_t n;
	struct nina_param args[2];
	int ret;

	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	if (!params || !params->ssid || !params->ssid_length ||
	    params->ssid_length > 32) {
		return -EINVAL;
	}
	if (params->security != WIFI_SECURITY_TYPE_NONE &&
	    params->security != WIFI_SECURITY_TYPE_PSK) {
		return -ENOTSUP;
	}
	if (params->security == WIFI_SECURITY_TYPE_PSK &&
	    (!params->psk || params->psk_length < 8 ||
	     params->psk_length > 63 || params->psk_is_pbkdf2)) {
		return -EINVAL;
	}
	k_mutex_lock(&connect_state_lock, K_FOREVER);
	if (!wifi_profile_connection_allowed()) {
		k_mutex_unlock(&connect_state_lock);
		return -ECANCELED;
	}
	if (connected) {
		k_mutex_unlock(&connect_state_lock);
		return -EALREADY;
	}
	if (atomic_get(&scanning) || !atomic_cas(&connecting, 0, 1)) {
		k_mutex_unlock(&connect_state_lock);
		return -EBUSY;
	}

	args[0] = (struct nina_param){params->ssid, params->ssid_length};
	args[1] = (struct nina_param){params->psk, params->psk_length};
	atomic_inc(&generation);
	atomic_set(&request_generation, atomic_get(&generation));
	ret = nina_request(
		params->security == WIFI_SECURITY_TYPE_NONE ? 0x10 : 0x11, args,
		params->security == WIFI_SECURITY_TYPE_NONE ? 1 : 2, false,
		false, &answer, 1, &n);
	if (ret || n != 1 || answer != 1) {
		atomic_clear(&connecting);
		k_mutex_unlock(&connect_state_lock);
		return ret ? ret : -EIO;
	}

	k_sem_give(&connect_request);
	k_mutex_unlock(&connect_state_lock);
	return 0;
}

static int wifi_disconnect(const struct device *dev, struct net_if *iface)
{
	uint8_t dummy = 0xff, answer;
	size_t n;
	int ret;

	ARG_UNUSED(dev);

	k_mutex_lock(&connect_state_lock, K_FOREVER);
	wifi_profile_pause_reconnect();
	atomic_inc(&generation);
	ret = request_one(0x30, &dummy, 1, &answer, 1, &n);
	if (ret || n != 1 || answer != 1) {
		k_mutex_unlock(&connect_state_lock);
		return ret ? ret : -EIO;
	}
	if (connected) {
		clear_ip_config(iface);
	}

	connected = false;
	k_mutex_unlock(&connect_state_lock);
	wifi_mgmt_raise_disconnect_result_event(iface, 0);
	return 0;
}

static void monitor_thread(void *a, void *b, void *c)
{
	unsigned int disconnected_checks = 0;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		uint8_t state;
		size_t n;
		int ret;
		bool lost = false;

		k_sleep(K_SECONDS(2));
		k_mutex_lock(&connect_state_lock, K_FOREVER);
		if (!connected) {
			disconnected_checks = 0;
			k_mutex_unlock(&connect_state_lock);
			continue;
		}

		ret = request_one(0x20, NULL, 0, &state, 1, &n);
		if (!ret && n == 1) {
			if (state == 3) {
				disconnected_checks = 0;
			} else if (++disconnected_checks >= 2) {
				atomic_inc(&generation);
				connected = false;
				clear_ip_config(nina_iface);
				lost = true;
				disconnected_checks = 0;
			}
		}
		k_mutex_unlock(&connect_state_lock);

		if (lost) {
			wifi_mgmt_raise_disconnect_result_event(nina_iface, 0);
			wifi_profile_request_reconnect();
		}
	}
}
K_THREAD_DEFINE(nina_monitor_tid, 1536, monitor_thread, NULL, NULL, NULL, 9, 0,
		0);

static int wifi_status(const struct device *dev, struct net_if *iface,
		       struct wifi_iface_status *status)
{
	uint8_t state, dummy = 0xff;
	uint8_t rssi[4], bssid[WIFI_MAC_ADDR_LEN];
	size_t n;
	int ret;

	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	ret = request_one(0x20, NULL, 0, &state, 1, &n);
	if (ret || n != 1) {
		return ret ? ret : -EPROTO;
	}
	status->state =
		state == 3 ? WIFI_STATE_COMPLETED : WIFI_STATE_DISCONNECTED;
	status->security = WIFI_SECURITY_TYPE_UNKNOWN;
	status->mfp = WIFI_MFP_UNKNOWN;
	status->link_mode = WIFI_LINK_MODE_UNKNOWN;
	status->band = WIFI_FREQ_BAND_UNKNOWN;
	status->iface_mode = WIFI_MODE_INFRA;
	if (state != 3) {
		return 0;
	}

	ret = request_one(0x23, &dummy, 1, (uint8_t *)status->ssid,
			  WIFI_SSID_MAX_LEN, &n);
	if (ret) {
		return ret;
	}
	status->ssid_len = n;
	status->ssid[n] = '\0';

	ret = request_one(0x24, &dummy, 1, bssid, sizeof(bssid), &n);
	if (ret || n != sizeof(bssid)) {
		return ret ? ret : -EPROTO;
	}
	/* NINA's Arduino firmware reports this address in reverse byte order.
	 */
	for (size_t i = 0; i < sizeof(bssid); i++) {
		status->bssid[i] = bssid[sizeof(bssid) - 1 - i];
	}

	ret = request_one(0x25, &dummy, 1, rssi, sizeof(rssi), &n);
	if (ret || n != sizeof(rssi)) {
		return ret ? ret : -EPROTO;
	}
	status->rssi = (int32_t)sys_get_le32(rssi);
	status->band = WIFI_FREQ_BAND_2_4_GHZ;
	return 0;
}

static void iface_init(struct net_if *iface)
{
	uint8_t dummy = 0xff;
	size_t n = 0;

	nina_iface = iface;
	net_if_dormant_on(iface);
	if (!request_one(0x22, &dummy, 1, mac, sizeof(mac), &n) &&
	    n == sizeof(mac)) {
		net_if_set_link_addr(iface, mac, sizeof(mac),
				     NET_LINK_ETHERNET);
	}
	net_if_socket_offload_set(iface, nina_socket_create);
	nina_dns_init();
}

static int driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static enum offloaded_net_if_types get_type(void)
{
	return L2_OFFLOADED_NET_IF_TYPE_WIFI;
}
static const struct wifi_mgmt_ops mgmt = {.scan = wifi_scan,
					  .connect = wifi_connect,
					  .disconnect = wifi_disconnect,
					  .iface_status = wifi_status};
static const struct net_wifi_mgmt_offload api = {
	.wifi_iface.iface_api.init = iface_init,
	.wifi_iface.get_type = get_type,
	.wifi_mgmt_api = &mgmt};
NET_DEVICE_DT_OFFLOAD_DEFINE(DT_NODELABEL(nina_w102), driver_init, NULL, NULL,
			     NULL, CONFIG_WIFI_INIT_PRIORITY, &api, 1500);
