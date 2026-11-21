/* SPDX-License-Identifier: Apache-2.0 */
/* Fakes of the Zephyr declarations that common/drivers/nina/port/zephyr/nina_net.c
 * uses (<zephyr/kernel.h>, <zephyr/net/net_if.h>, <zephyr/net/wifi_mgmt.h>),
 * for the host test tests/nina_net.c, which defines the functions.  The
 * structures have Zephyr's member names (Zephyr v4.5.0); only the members
 * the binding uses are here.
 */
#ifndef NET_TEST_H_
#define NET_TEST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ARG_UNUSED(x)             (void)(x)
#define ARRAY_SIZE(a)             (sizeof(a) / sizeof((a)[0]))
#define DT_NODELABEL(x)           0
#define CONFIG_WIFI_INIT_PRIORITY 80

/* K_THREAD_DEFINE(): the thread's description, which the test reads. */
struct fake_thread {
	const char *name;
	size_t stack_size;
	void (*entry)(void *p1, void *p2, void *p3);
	int prio;
};
#define K_THREAD_DEFINE(name, size, entry, p1, p2, p3, prio, options, delay)                       \
	const struct fake_thread name = {#name, (size), (entry), (prio)}

struct device {
	int unused;
};

/* <zephyr/net/net_if.h> */
struct net_in_addr {
	uint32_t s_addr;
};
struct net_if {
	int unused;
};
struct net_if_addr {
	struct net_in_addr address;
};
enum net_addr_type {
	NET_ADDR_ANY,
	NET_ADDR_AUTOCONF,
	NET_ADDR_DHCP,
};
enum net_link_type {
	NET_LINK_UNKNOWN,
	NET_LINK_ETHERNET,
};
struct net_if_addr *net_if_ipv4_addr_add(struct net_if *iface, struct net_in_addr *addr,
					 enum net_addr_type addr_type, uint32_t vlifetime);
bool net_if_ipv4_addr_rm(struct net_if *iface, const struct net_in_addr *addr);
bool net_if_ipv4_set_netmask_by_addr(struct net_if *iface, const struct net_in_addr *addr,
				     const struct net_in_addr *netmask);
void net_if_ipv4_set_gw(struct net_if *iface, const struct net_in_addr *gw);
void net_if_dormant_on(struct net_if *iface);
void net_if_dormant_off(struct net_if *iface);
int net_if_set_link_addr(struct net_if *iface, uint8_t *addr, uint8_t len,
			 enum net_link_type type);
void net_if_socket_offload_set(struct net_if *iface, int (*socket_create)(int, int, int));

/* <zephyr/net/wifi_mgmt.h> */
#define WIFI_SSID_MAX_LEN              32
#define WIFI_MAC_ADDR_LEN              6
#define WIFI_MGMT_SCAN_SSID_FILT_MAX   1
#define WIFI_MGMT_SCAN_CHAN_MAX_MANUAL 3

enum wifi_security_type {
	WIFI_SECURITY_TYPE_NONE,
	WIFI_SECURITY_TYPE_PSK,
	WIFI_SECURITY_TYPE_PSK_SHA256,
	WIFI_SECURITY_TYPE_SAE,
	WIFI_SECURITY_TYPE_WAPI,
	WIFI_SECURITY_TYPE_EAP,
	WIFI_SECURITY_TYPE_WEP,
	WIFI_SECURITY_TYPE_WPA_PSK,
	WIFI_SECURITY_TYPE_UNKNOWN = 100,
};
enum wifi_frequency_bands {
	WIFI_FREQ_BAND_2_4_GHZ,
	WIFI_FREQ_BAND_5_GHZ,
	WIFI_FREQ_BAND_UNKNOWN = 50,
};
enum wifi_mfp_options {
	WIFI_MFP_DISABLE,
	WIFI_MFP_OPTIONAL,
	WIFI_MFP_REQUIRED,
	WIFI_MFP_UNKNOWN,
};
enum wifi_scan_type {
	WIFI_SCAN_TYPE_ACTIVE,
	WIFI_SCAN_TYPE_PASSIVE,
};
enum wifi_iface_state {
	WIFI_STATE_DISCONNECTED,
	WIFI_STATE_COMPLETED = 9,
};
enum wifi_link_mode {
	WIFI_LINK_MODE_UNKNOWN = 20,
};
enum wifi_iface_mode {
	WIFI_MODE_INFRA,
};

struct wifi_band_channel {
	uint8_t band;
	uint8_t channel;
};
struct wifi_scan_params {
	enum wifi_scan_type scan_type;
	uint8_t bands;
	uint16_t dwell_time_active;
	uint16_t dwell_time_passive;
	const char *ssids[WIFI_MGMT_SCAN_SSID_FILT_MAX];
	uint16_t max_bss_cnt;
	struct wifi_band_channel band_chan[WIFI_MGMT_SCAN_CHAN_MAX_MANUAL];
};
struct wifi_scan_result {
	uint8_t ssid[WIFI_SSID_MAX_LEN + 1];
	uint8_t ssid_length;
	uint8_t band;
	uint8_t channel;
	enum wifi_security_type security;
	enum wifi_mfp_options mfp;
	int8_t rssi;
	uint8_t mac[WIFI_MAC_ADDR_LEN];
	uint8_t mac_length;
};
struct wifi_connect_req_params {
	const uint8_t *ssid;
	uint8_t ssid_length;
	const uint8_t *psk;
	uint8_t psk_length;
	enum wifi_security_type security;
	bool psk_is_pbkdf2;
};
struct wifi_iface_status {
	int state;
	unsigned int ssid_len;
	char ssid[WIFI_SSID_MAX_LEN + 1];
	char bssid[WIFI_MAC_ADDR_LEN];
	enum wifi_frequency_bands band;
	unsigned int channel;
	enum wifi_iface_mode iface_mode;
	enum wifi_link_mode link_mode;
	enum wifi_security_type security;
	enum wifi_mfp_options mfp;
	int rssi;
};

typedef void (*scan_result_cb_t)(struct net_if *iface, int status,
				 struct wifi_scan_result *entry);

struct wifi_mgmt_ops {
	int (*scan)(const struct device *dev, struct net_if *iface,
		    struct wifi_scan_params *params, scan_result_cb_t cb);
	int (*connect)(const struct device *dev, struct net_if *iface,
		       struct wifi_connect_req_params *params);
	int (*disconnect)(const struct device *dev, struct net_if *iface);
	int (*iface_status)(const struct device *dev, struct net_if *iface,
			    struct wifi_iface_status *status);
};
enum offloaded_net_if_types {
	L2_OFFLOADED_NET_IF_TYPE_UNKNOWN,
	L2_OFFLOADED_NET_IF_TYPE_WIFI = 3,
};
struct net_if_api {
	void (*init)(struct net_if *iface);
};
struct offloaded_if_api {
	struct net_if_api iface_api;
	enum offloaded_net_if_types (*get_type)(void);
};
struct net_wifi_mgmt_offload {
	struct offloaded_if_api wifi_iface;
	const struct wifi_mgmt_ops *const wifi_mgmt_api;
};

/* The device: its init and the interface's API, which the test reads. */
#define NET_DEVICE_DT_OFFLOAD_DEFINE(node_id, init_fn, pm, data, config, prio, api, mtu)          \
	int (*const fake_net_init)(const struct device *dev) = (init_fn);                          \
	const struct net_wifi_mgmt_offload *const fake_net_api = (api);                            \
	const int fake_net_mtu = (mtu)

void wifi_mgmt_raise_connect_result_event(struct net_if *iface, int status);
void wifi_mgmt_raise_disconnect_result_event(struct net_if *iface, int status);

#endif
