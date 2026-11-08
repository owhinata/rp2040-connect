/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include "nina_proto.h"
#include "wifi_profile.h"

static struct net_if *nina_iface;
static uint8_t mac[6];
static struct net_in_addr assigned_ip;
static bool connected;
static atomic_t generation;
static atomic_t connecting;
K_MUTEX_DEFINE(connect_state_lock);
K_SEM_DEFINE(connect_request, 0, 1);
int nina_socket_create(int family, int type, int proto);

static int request_one(uint8_t cmd, const uint8_t *data, size_t length,
		       uint8_t *result, size_t cap, size_t *n)
{
	const struct nina_param p = {data, length};

	return nina_request(cmd, data ? &p : NULL, data ? 1 : 0, false, false,
			    result, cap, n);
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
		id = atomic_get(&generation);

		for (int i = 0; i < 60 && id == atomic_get(&generation); i++) {
			ret = request_one(0x20, NULL, 0, &value, 1, &n);
			if (ret || n != 1) {
				if (!ret) {
					ret = -EPROTO;
				}
				break;
			}
			if (value == 3) {
				ret = nina_ip_config(ip);
				if (ret) {
					break;
				}
				if (ip[0] || ip[1] || ip[2] || ip[3]) {
					memcpy(&assigned_ip, ip, 4);
					if (!net_if_ipv4_addr_add(
						    nina_iface, &assigned_ip,
						    NET_ADDR_DHCP, 0)) {
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
			if (!ret) {
				wifi_profile_commit();
			} else {
				wifi_profile_discard();
			}
			wifi_mgmt_raise_connect_result_event(nina_iface, ret);
		} else {
			wifi_profile_discard();
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
	if (connected) {
		return -EALREADY;
	}
	if (!atomic_cas(&connecting, 0, 1)) {
		return -EBUSY;
	}

	wifi_profile_prepare(params);
	args[0] = (struct nina_param){params->ssid, params->ssid_length};
	args[1] = (struct nina_param){params->psk, params->psk_length};
	atomic_inc(&generation);
	ret = nina_request(
		params->security == WIFI_SECURITY_TYPE_NONE ? 0x10 : 0x11, args,
		params->security == WIFI_SECURITY_TYPE_NONE ? 1 : 2, false,
		false, &answer, 1, &n);
	if (ret || n != 1 || answer != 1) {
		wifi_profile_discard();
		atomic_clear(&connecting);
		return ret ? ret : -EIO;
	}

	k_sem_give(&connect_request);
	return 0;
}

static int wifi_disconnect(const struct device *dev, struct net_if *iface)
{
	uint8_t dummy = 0xff, answer;
	size_t n;
	int ret;

	ARG_UNUSED(dev);

	k_mutex_lock(&connect_state_lock, K_FOREVER);
	atomic_inc(&generation);
	k_mutex_unlock(&connect_state_lock);
	ret = request_one(0x30, &dummy, 1, &answer, 1, &n);
	if (ret || n != 1 || answer != 1) {
		return ret ? ret : -EIO;
	}
	if (connected) {
		net_if_ipv4_addr_rm(iface, &assigned_ip);
	}

	connected = false;
	wifi_mgmt_raise_disconnect_result_event(iface, 0);
	return 0;
}

static int wifi_status(const struct device *dev, struct net_if *iface,
		       struct wifi_iface_status *status)
{
	uint8_t state;
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
	return 0;
}

static void iface_init(struct net_if *iface)
{
	uint8_t dummy = 0xff;
	size_t n = 0;

	nina_iface = iface;
	if (!request_one(0x22, &dummy, 1, mac, sizeof(mac), &n) &&
	    n == sizeof(mac)) {
		net_if_set_link_addr(iface, mac, sizeof(mac),
				     NET_LINK_ETHERNET);
	}
	net_if_socket_offload_set(iface, nina_socket_create);
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
static const struct wifi_mgmt_ops mgmt = {.connect = wifi_connect,
					  .disconnect = wifi_disconnect,
					  .iface_status = wifi_status};
static const struct net_wifi_mgmt_offload api = {
	.wifi_iface.iface_api.init = iface_init,
	.wifi_iface.get_type = get_type,
	.wifi_mgmt_api = &mgmt};
NET_DEVICE_DT_OFFLOAD_DEFINE(DT_NODELABEL(nina_w102), driver_init, NULL, NULL,
			     NULL, CONFIG_WIFI_INIT_PRIORITY, &api, 1500);
