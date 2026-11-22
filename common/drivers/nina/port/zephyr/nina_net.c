/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA-W102 as Zephyr's offloaded Wi-Fi interface: the Zephyr binding
 * of the core's Wi-Fi operations (nina_wifi.h).  NET_DEVICE_DT_OFFLOAD_DEFINE()
 * registers the device of the module's node (compatible "arduino,nina-spi"); its operations
 * (struct wifi_mgmt_ops: scan, connect, disconnect, iface_status) check
 * Zephyr's parameters and call the core, and the core's struct
 * nina_wifi_ops puts what happens on the interface (the IPv4 address,
 * netmask and gateway, dormant on and off) and raises Zephyr's events
 * (NET_EVENT_WIFI_CONNECT_RESULT, _DISCONNECT_RESULT; the scan's results
 * go to the callback of Zephyr's Wi-Fi management, which raises
 * NET_EVENT_WIFI_SCAN_RESULT and _SCAN_DONE).  The core's threads are K_THREAD_DEFINE() threads here.
 *
 * The FreeRTOS build compiles this file on its Zephyr compatibility layer
 * (freertos/drivers/wifi/wifi.cmake, common/compat/net_offload/zephyr_net_init.c).
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include "nina_dns.h"
#include "nina_proto.h"
#include "nina_wifi.h"

static struct net_if *nina_iface;
/* The interface's link address points here (net_if_set_link_addr()). */
static uint8_t mac[NINA_WIFI_MAC_LEN];
static scan_result_cb_t scan_cb;
int nina_socket_create(int family, int type, int proto);

/* The core's operations ----------------------------------------------- */

static int ip_set(const uint8_t *addr, const uint8_t *netmask, const uint8_t *gateway)
{
	struct net_in_addr address, mask, gw;

	memcpy(&address, addr, 4);
	memcpy(&mask, netmask, 4);
	memcpy(&gw, gateway, 4);
	if (!net_if_ipv4_addr_add(nina_iface, &address, NET_ADDR_DHCP, 0)) {
		return -ENOMEM;
	}
	net_if_ipv4_set_netmask_by_addr(nina_iface, &address, &mask);
	net_if_ipv4_set_gw(nina_iface, &gw);
	return 0;
}

static void link_up(void)
{
	net_if_dormant_off(nina_iface);
}

static void link_down(const uint8_t *addr)
{
	const struct net_in_addr no_gateway = {0};
	struct net_in_addr address;

	memcpy(&address, addr, 4);
	net_if_ipv4_addr_rm(nina_iface, &address);
	net_if_ipv4_set_gw(nina_iface, &no_gateway);
	net_if_dormant_on(nina_iface);
}

static void connect_result(int status)
{
	wifi_mgmt_raise_connect_result_event(nina_iface, status);
}

static void disconnected(enum nina_wifi_disconnect_reason reason)
{
	ARG_UNUSED(reason);
	wifi_mgmt_raise_disconnect_result_event(nina_iface, 0);
}

static const struct nina_wifi_ops ops = {
	.ip_set = ip_set,
	.link_up = link_up,
	.link_down = link_down,
	.connect_result = connect_result,
	.disconnected = disconnected,
};

/* Zephyr's Wi-Fi management ------------------------------------------- */

static enum wifi_security_type scan_security(uint8_t type)
{
	switch (type) {
	case NINA_ENC_TYPE_NONE:
		return WIFI_SECURITY_TYPE_NONE;
	case NINA_ENC_TYPE_WEP:
		return WIFI_SECURITY_TYPE_WEP;
	case NINA_ENC_TYPE_TKIP:
		return WIFI_SECURITY_TYPE_WPA_PSK;
	case NINA_ENC_TYPE_CCMP:
		return WIFI_SECURITY_TYPE_PSK;
	default:
		return WIFI_SECURITY_TYPE_UNKNOWN;
	}
}

/* In the core's scan thread; ARG is the interface of the request. */
static void scan_result(void *arg, int status, const struct nina_wifi_scan_result *found)
{
	struct net_if *iface = arg;
	struct wifi_scan_result result = {0};

	if (!found) {
		scan_cb(iface, status, NULL);
		return;
	}
	memcpy(result.ssid, found->ssid, found->ssid_len);
	result.ssid_length = found->ssid_len;
	result.band = WIFI_FREQ_BAND_2_4_GHZ;
	result.mfp = WIFI_MFP_UNKNOWN;
	result.rssi = found->rssi;
	result.security = scan_security(found->enc_type);
	memcpy(result.mac, found->bssid, WIFI_MAC_ADDR_LEN);
	result.mac_length = WIFI_MAC_ADDR_LEN;
	result.channel = found->channel;
	scan_cb(iface, 0, &result);
}

static int wifi_scan(const struct device *dev, struct net_if *iface,
		     struct wifi_scan_params *params, scan_result_cb_t cb)
{
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
	/* Zephyr's Wi-Fi management passes its one callback with every
	 * request (scan_result_cb() of subsys/net/l2/wifi/wifi_mgmt.c), so a
	 * request that the core refuses leaves a running scan's as it was.
	 */
	scan_cb = cb;
	return nina_wifi_scan(scan_result, iface);
}

static int wifi_connect(const struct device *dev, struct net_if *iface,
			struct wifi_connect_req_params *params)
{
	struct nina_wifi_connect_params p;

	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	if (!params) {
		return -EINVAL;
	}
	p = (struct nina_wifi_connect_params){
		.ssid = params->ssid,
		.ssid_len = params->ssid_length,
		.security = params->security == WIFI_SECURITY_TYPE_NONE ? NINA_WIFI_SECURITY_NONE
			    : params->security == WIFI_SECURITY_TYPE_PSK ? NINA_WIFI_SECURITY_PSK
									  : NINA_WIFI_SECURITY_OTHER,
		.psk = params->psk,
		.psk_len = params->psk_length,
		.psk_is_pbkdf2 = params->psk_is_pbkdf2,
	};
	return nina_wifi_connect(&p);
}

static int wifi_disconnect(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);
	return nina_wifi_disconnect();
}

static int wifi_status(const struct device *dev, struct net_if *iface,
		       struct wifi_iface_status *status)
{
	struct nina_wifi_status s;
	int ret;

	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	ret = nina_wifi_status(&s);
	if (ret) {
		return ret;
	}
	status->state = s.connected ? WIFI_STATE_COMPLETED : WIFI_STATE_DISCONNECTED;
	status->security = WIFI_SECURITY_TYPE_UNKNOWN;
	status->mfp = WIFI_MFP_UNKNOWN;
	status->link_mode = WIFI_LINK_MODE_UNKNOWN;
	status->band = WIFI_FREQ_BAND_UNKNOWN;
	status->iface_mode = WIFI_MODE_INFRA;
	if (!s.connected) {
		return 0;
	}
	memcpy(status->ssid, s.ssid, s.ssid_len + 1);
	status->ssid_len = s.ssid_len;
	memcpy(status->bssid, s.bssid, WIFI_MAC_ADDR_LEN);
	status->rssi = s.rssi;
	status->band = WIFI_FREQ_BAND_2_4_GHZ;
	return 0;
}

static void iface_init(struct net_if *iface)
{
	nina_iface = iface;
	net_if_dormant_on(iface);
	if (!nina_wifi_init(&ops, mac)) {
		net_if_set_link_addr(iface, mac, sizeof(mac), NET_LINK_ETHERNET);
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
NET_DEVICE_DT_OFFLOAD_DEFINE(DT_COMPAT_GET_ANY_STATUS_OKAY(arduino_nina_spi), driver_init, NULL, NULL,
			     NULL, CONFIG_WIFI_INIT_PRIORITY, &api, 1500);

/* The core's threads -------------------------------------------------- */

static void scan_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	nina_wifi_scan_thread();
}

static void connect_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	nina_wifi_connect_thread();
}

static void monitor_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	nina_wifi_monitor_thread();
}

K_THREAD_DEFINE(nina_scan_tid, 1536, scan_thread, NULL, NULL, NULL, 8, 0, 0);
K_THREAD_DEFINE(nina_connect_tid, 1024, connect_thread, NULL, NULL, NULL, 8, 0, 0);
K_THREAD_DEFINE(nina_monitor_tid, 1536, monitor_thread, NULL, NULL, NULL, 9, 0, 0);
