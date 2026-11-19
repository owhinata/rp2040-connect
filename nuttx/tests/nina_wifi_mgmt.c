/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the applications' Zephyr Wi-Fi management on NuttX:
 * common/drivers/nina/nuttx/nina_wifi_mgmt.c under the compatibility
 * layer's Wi-Fi requests (common/compat/zephyr_wifi_mgmt.c) and interface
 * (nuttx/drivers/compat/zephyr_net_if.c), as nina/Make.defs and
 * compat/Make.defs build them, on fakes of the network device's lock
 * (tests/nina_netdev_fakes), of net_mgmt's event thread and events, of the
 * core's nina_wifi_connect() and of the Wi-Fi profile.  Built twice: with
 * CONFIG_NANO_WIFI_PROFILE (the profile's three functions as the core's
 * hooks) and without (every connection allowed, nothing else).
 *
 * - The start: net_mgmt's event thread first (its failure is returned),
 *   then the interface of net_if_get_default() and net_if_get_wifi_sta()
 *   on the device.
 * - NET_REQUEST_WIFI_CONNECT (what NET_REQUEST_WIFI_CONNECT_STORED makes of
 *   each stored network): the layer's checks (-ENETDOWN while the device is
 *   down), then the core with the SSID, the security (none, PSK, the
 *   others as "other"), the passphrase and pbkdf2; the core's result.  The
 *   other requests of the layer are not served (-ENOTSUP): wapi calls the
 *   core itself (nina_netdev.c).
 * - The results as the layer's events: NET_EVENT_WIFI_CONNECT_RESULT with
 *   the status, NET_EVENT_WIFI_DISCONNECT_RESULT with 0, on the interface.
 * - The interface's addresses for the applications: the address only when
 *   the device has one (NET_ADDR_PREFERRED or any state), the gateway.
 */

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <net/if.h>
#include <nuttx/net/netdev.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "nina_wifi.h"
#include "nina_wifi_mgmt.h"

static struct net_driver_s dev;
static int locked;

void netdev_lock(struct net_driver_s *d)
{
	assert(d == &dev && !locked);
	locked = 1;
}

void netdev_unlock(struct net_driver_s *d)
{
	assert(d == &dev && locked);
	locked = 0;
}

static int event_init_ret;
static unsigned int event_inits;

int net_mgmt_event_init(void)
{
	event_inits++;
	return event_init_ret;
}

static uint64_t last_event;
static struct net_if *last_iface;
static struct wifi_status last_status;
static unsigned int events;

void net_mgmt_event_notify_with_info(uint64_t mgmt_event, struct net_if *iface, const void *info,
				     size_t length)
{
	assert(length == sizeof(last_status));
	memcpy(&last_status, info, sizeof(last_status));
	last_event = mgmt_event;
	last_iface = iface;
	events++;
}

static struct nina_wifi_connect_params seen;
static unsigned int connects;
static int connect_ret;

int nina_wifi_connect(const struct nina_wifi_connect_params *params)
{
	seen = *params;
	connects++;
	return connect_ret;
}

#ifdef CONFIG_NANO_WIFI_PROFILE
static unsigned int allowed_calls, pauses, requests;

bool wifi_profile_connection_allowed(void)
{
	allowed_calls++;
	return false;
}

void wifi_profile_pause_reconnect(void)
{
	pauses++;
}

void wifi_profile_request_reconnect(void)
{
	requests++;
}
#endif

static void test_start(void)
{
	event_init_ret = -ENOMEM;
	assert(nina_wifi_mgmt_initialize(&dev) == -ENOMEM);
	assert(net_if_get_default() == NULL && net_if_get_wifi_sta() == NULL);
	event_init_ret = 0;
	assert(nina_wifi_mgmt_initialize(&dev) == 0);
	assert(event_inits == 2);
	assert(net_if_get_default() && net_if_get_default() == net_if_get_wifi_sta());
	assert(net_if_get_default()->dev == &dev);
}

static void test_connect(void)
{
	static const uint8_t ssid[] = "Net", psk[] = "password";
	struct net_if *iface = net_if_get_wifi_sta();
	struct wifi_connect_req_params p = {
		.ssid = ssid,
		.ssid_length = 3,
		.psk = psk,
		.psk_length = 8,
		.security = WIFI_SECURITY_TYPE_PSK,
		.channel = WIFI_CHANNEL_ANY,
	};
	struct wifi_iface_status status;

	dev.d_flags = 0;
	assert(net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &p, sizeof(p)) == -ENETDOWN);
	assert(connects == 0);
	dev.d_flags = IFF_UP;
	assert(net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &p, sizeof(p)) == 0);
	assert(connects == 1 && seen.ssid == ssid && seen.ssid_len == 3);
	assert(seen.security == NINA_WIFI_SECURITY_PSK && seen.psk == psk && seen.psk_len == 8);
	assert(!seen.psk_is_pbkdf2);
	p.security = WIFI_SECURITY_TYPE_NONE;
	p.psk_length = 0;
	p.psk_is_pbkdf2 = true;
	connect_ret = -EALREADY;
	assert(net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &p, sizeof(p)) == -EALREADY);
	connect_ret = 0;
	assert(seen.security == NINA_WIFI_SECURITY_NONE && seen.psk_len == 0 && seen.psk_is_pbkdf2);
	p.security = WIFI_SECURITY_TYPE_OWE;
	assert(net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &p, sizeof(p)) == 0);
	assert(seen.security == NINA_WIFI_SECURITY_OTHER);
	p.security = WIFI_SECURITY_TYPE_WPA_PSK;
	p.psk_length = 8;
	assert(net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &p, sizeof(p)) == 0);
	assert(seen.security == NINA_WIFI_SECURITY_OTHER && connects == 4);

	/* The rest is wapi's, through the core (nina_netdev.c). */
	assert(net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0) == -ENOTSUP);
	assert(net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0) == -ENOTSUP);
	assert(net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status)) ==
	       -ENOTSUP);
}

static void test_events(void)
{
	nina_wifi_mgmt_connect_result(-ETIMEDOUT);
	assert(events == 1 && last_event == NET_EVENT_WIFI_CONNECT_RESULT);
	assert(last_iface == net_if_get_default() && last_status.status == -ETIMEDOUT);
	nina_wifi_mgmt_connect_result(0);
	assert(events == 2 && last_status.status == 0);
	last_status.status = 9;
	nina_wifi_mgmt_disconnect_result();
	assert(events == 3 && last_event == NET_EVENT_WIFI_DISCONNECT_RESULT);
	assert(last_iface == net_if_get_default() && last_status.status == 0);
}

static void test_addresses(void)
{
	struct net_if *iface = net_if_get_default();
	struct net_in_addr *a;

	dev.d_ipaddr = 0;
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == NULL);
	assert(net_if_ipv4_get_global_addr(NULL, NET_ADDR_PREFERRED) == NULL);
	dev.d_ipaddr = 0x6100a80a;
	dev.d_draddr = 0x0100a80a;
	a = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
	assert(a && a->s_addr == 0x6100a80a);
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_ANY_STATE) == a);
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_TENTATIVE) == NULL);
	assert(net_if_ipv4_get_gw(iface).s_addr == 0x0100a80a);
	assert(!locked);
}

static void test_hooks(void)
{
#ifdef CONFIG_NANO_WIFI_PROFILE
	assert(!nina_wifi_mgmt_connection_allowed() && allowed_calls == 1);
	nina_wifi_mgmt_pause_reconnect();
	nina_wifi_mgmt_request_reconnect();
	assert(pauses == 1 && requests == 1);
#else
	assert(nina_wifi_mgmt_connection_allowed());
	nina_wifi_mgmt_pause_reconnect();
	nina_wifi_mgmt_request_reconnect();
#endif
}

int main(void)
{
	test_start();
	test_connect();
	test_events();
	test_addresses();
	test_hooks();
	printf("nina_wifi_mgmt: ok\n");
	return 0;
}
