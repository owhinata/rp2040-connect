/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the NINA core's Zephyr binding,
 * common/drivers/nina/port/zephyr/nina_net.c (included), on fakes of the Zephyr
 * declarations it uses (tests/net_fakes) and of the core's Wi-Fi functions
 * (nina_wifi.h), which record what they are asked.  The mapping:
 *
 * - The device: its init returns 0, the interface is Zephyr's offloaded
 *   Wi-Fi type with an MTU of 1500 and the four operations.
 * - The interface's init: dormant, the core's init (the MAC address as the
 *   link address, kept in a buffer of the binding; none when the core
 *   fails), the socket offload, the resolver.
 * - Scan: -EINVAL without an interface or a callback, -ENOTSUP for each
 *   parameter the module cannot follow, the core's error; each result as
 *   a struct wifi_scan_result (the SSID, 2.4 GHz, MFP unknown, the RSSI,
 *   the security of each encryption type, the BSSID, the channel) and the
 *   end with its status, to Zephyr's callback with the request's
 *   interface.
 * - Connect: the security (none, PSK, and the others as the core's
 *   "other"), the SSID, the passphrase and pbkdf2 passed on; the core's
 *   result returned.  Disconnect: the core's.
 * - Status: the core's error; not connected (the fields Zephyr's shell
 *   reads, band unknown); connected (SSID, BSSID, RSSI, 2.4 GHz).
 * - The core's operations: the address with its netmask and gateway on
 *   the interface (-ENOMEM when Zephyr has no room), dormant off; the
 *   address removed, the gateway 0.0.0.0, dormant on; the connect result
 *   and the disconnection (status 0, for every reason) as Zephyr's events;
 *   the Wi-Fi profile's three functions as the hooks.
 * - The threads: nina_scan_tid (1536 bytes, priority 8), nina_connect_tid
 *   (1024, 8), nina_monitor_tid (1536, 9), each running its core loop.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../../../../common/drivers/nina/port/zephyr/nina_net.c"

static char log_buf[4096];

static void note(const char *fmt, ...)
{
	char line[256];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	assert(strlen(log_buf) + strlen(line) + 2 < sizeof(log_buf));
	strcat(log_buf, line);
	strcat(log_buf, " ");
}

static void expect_log(const char *expected)
{
	size_t n = strlen(log_buf);

	if (n && log_buf[n - 1] == ' ') {
		log_buf[n - 1] = '\0';
	}
	if (strcmp(log_buf, expected) != 0) {
		fprintf(stderr, "expected: %s\n     got: %s\n", expected, log_buf);
		assert(0);
	}
	log_buf[0] = '\0';
}

/* Zephyr ------------------------------------------------------------------ */

static struct net_if iface0;
static bool addr_room = true;
static struct net_if_addr if_addr;

static const char *ip(const struct net_in_addr *a)
{
	static char text[4][16];
	static unsigned int at;
	const uint8_t *b = (const uint8_t *)&a->s_addr;
	char *t = text[at++ % 4];

	snprintf(t, 16, "%u.%u.%u.%u", b[0], b[1], b[2], b[3]);
	return t;
}

struct net_if_addr *net_if_ipv4_addr_add(struct net_if *iface, struct net_in_addr *addr,
					 enum net_addr_type addr_type, uint32_t vlifetime)
{
	assert(iface == &iface0 && addr_type == NET_ADDR_DHCP && vlifetime == 0);
	note("add(%s)", ip(addr));
	return addr_room ? &if_addr : NULL;
}

bool net_if_ipv4_addr_rm(struct net_if *iface, const struct net_in_addr *addr)
{
	assert(iface == &iface0);
	note("rm(%s)", ip(addr));
	return true;
}

bool net_if_ipv4_set_netmask_by_addr(struct net_if *iface, const struct net_in_addr *addr,
				     const struct net_in_addr *netmask)
{
	assert(iface == &iface0);
	note("netmask(%s,%s)", ip(addr), ip(netmask));
	return true;
}

void net_if_ipv4_set_gw(struct net_if *iface, const struct net_in_addr *gw)
{
	assert(iface == &iface0);
	note("gw(%s)", ip(gw));
}

void net_if_dormant_on(struct net_if *iface)
{
	assert(iface == &iface0);
	note("dormant_on");
}

void net_if_dormant_off(struct net_if *iface)
{
	assert(iface == &iface0);
	note("dormant_off");
}

static uint8_t *link_addr;

int net_if_set_link_addr(struct net_if *iface, uint8_t *addr, uint8_t len,
			 enum net_link_type type)
{
	assert(iface == &iface0 && len == 6 && type == NET_LINK_ETHERNET);
	link_addr = addr;
	note("link(%02x..%02x)", addr[0], addr[5]);
	return 0;
}

int nina_socket_create(int family, int type, int proto)
{
	(void)family;
	(void)type;
	(void)proto;
	return -1;
}

void net_if_socket_offload_set(struct net_if *iface, int (*socket_create)(int, int, int))
{
	assert(iface == &iface0 && socket_create == nina_socket_create);
	note("offload");
}

void wifi_mgmt_raise_connect_result_event(struct net_if *iface, int status)
{
	assert(iface == &iface0);
	note("connect_event(%d)", status);
}

void wifi_mgmt_raise_disconnect_result_event(struct net_if *iface, int status)
{
	assert(iface == &iface0);
	note("disconnect_event(%d)", status);
}


void nina_dns_init(void)
{
	note("dns");
}

/* The core ---------------------------------------------------------------- */

static const struct nina_wifi_ops *core_ops;
static int core_ret;
static nina_wifi_scan_cb_t core_scan_cb;
static void *core_scan_arg;
static struct nina_wifi_connect_params core_params;
static struct nina_wifi_status core_status;

int nina_wifi_init(const struct nina_wifi_ops *with, uint8_t out[NINA_WIFI_MAC_LEN])
{
	static const uint8_t m[6] = {0xa1, 2, 3, 4, 5, 0xa6};

	core_ops = with;
	memcpy(out, m, 6);
	note("init");
	return core_ret;
}

int nina_wifi_scan(nina_wifi_scan_cb_t cb, void *arg)
{
	core_scan_cb = cb;
	core_scan_arg = arg;
	note("scan");
	return core_ret;
}

int nina_wifi_connect(const struct nina_wifi_connect_params *params)
{
	core_params = *params;
	note("connect");
	return core_ret;
}

int nina_wifi_disconnect(void)
{
	note("disconnect");
	return core_ret;
}

int nina_wifi_status(struct nina_wifi_status *status)
{
	note("status");
	*status = core_status;
	return core_ret;
}

void nina_wifi_scan_thread(void)
{
	note("scan_thread");
}

void nina_wifi_connect_thread(void)
{
	note("connect_thread");
}

void nina_wifi_monitor_thread(void)
{
	note("monitor_thread");
}

/* Zephyr's scan callback ---------------------------------------------------- */

static struct wifi_scan_result seen[4];
static unsigned int nseen;

static void zephyr_scan_cb(struct net_if *iface, int status, struct wifi_scan_result *entry)
{
	assert(iface == &iface0);
	if (entry) {
		assert(status == 0 && nseen < 4);
		seen[nseen++] = *entry;
		note("result");
	} else {
		note("done(%d)", status);
	}
}

/* The tests --------------------------------------------------------------- */

static void test_device(void)
{
	assert(fake_net_init(NULL) == 0);
	assert(fake_net_api->wifi_iface.get_type() == L2_OFFLOADED_NET_IF_TYPE_WIFI);
	assert(fake_net_mtu == 1500);
	assert(fake_net_api->wifi_mgmt_api->scan == wifi_scan);
	assert(fake_net_api->wifi_mgmt_api->connect == wifi_connect);
	assert(fake_net_api->wifi_mgmt_api->disconnect == wifi_disconnect);
	assert(fake_net_api->wifi_mgmt_api->iface_status == wifi_status);

	/* A module that does not answer: no link address. */
	core_ret = -ETIMEDOUT;
	fake_net_api->wifi_iface.iface_api.init(&iface0);
	expect_log("dormant_on init offload dns");
	core_ret = 0;
	fake_net_api->wifi_iface.iface_api.init(&iface0);
	expect_log("dormant_on init link(a1..a6) offload dns");
	assert(link_addr == mac && core_ops == &ops);
}

static void test_scan(void)
{
	struct wifi_scan_params params;
	struct nina_wifi_scan_result r = {
		.ssid = "AP",
		.ssid_len = 2,
		.rssi = -48,
		.bssid = {1, 2, 3, 4, 5, 6},
		.channel = 11,
	};
	static const struct {
		uint8_t enc;
		enum wifi_security_type security;
	} encs[] = {
		{NINA_ENC_TYPE_NONE, WIFI_SECURITY_TYPE_NONE},
		{NINA_ENC_TYPE_WEP, WIFI_SECURITY_TYPE_WEP},
		{NINA_ENC_TYPE_TKIP, WIFI_SECURITY_TYPE_WPA_PSK},
		{NINA_ENC_TYPE_CCMP, WIFI_SECURITY_TYPE_PSK},
		{8, WIFI_SECURITY_TYPE_UNKNOWN},
		{0, WIFI_SECURITY_TYPE_UNKNOWN},
	};

	assert(wifi_scan(NULL, NULL, NULL, zephyr_scan_cb) == -EINVAL);
	assert(wifi_scan(NULL, &iface0, NULL, NULL) == -EINVAL);
	for (int i = 0; i < 7; i++) {
		memset(&params, 0, sizeof(params));
		switch (i) {
		case 0:
			params.scan_type = WIFI_SCAN_TYPE_PASSIVE;
			break;
		case 1:
			params.bands = 1;
			break;
		case 2:
			params.dwell_time_active = 10;
			break;
		case 3:
			params.dwell_time_passive = 10;
			break;
		case 4:
			params.max_bss_cnt = 3;
			break;
		case 5:
			params.ssids[0] = "x";
			break;
		default:
			params.band_chan[2].channel = 6;
			break;
		}
		assert(wifi_scan(NULL, &iface0, &params, zephyr_scan_cb) == -ENOTSUP);
	}
	expect_log("");

	core_ret = -EBUSY;
	assert(wifi_scan(NULL, &iface0, NULL, zephyr_scan_cb) == -EBUSY);
	core_ret = 0;
	memset(&params, 0, sizeof(params));
	assert(wifi_scan(NULL, &iface0, &params, zephyr_scan_cb) == 0);
	expect_log("scan scan");
	assert(core_scan_arg == &iface0);

	nseen = 0;
	core_scan_cb(core_scan_arg, 0, &r);
	assert(nseen == 1);
	assert(seen[0].ssid_length == 2 && memcmp(seen[0].ssid, "AP", 3) == 0);
	assert(seen[0].band == WIFI_FREQ_BAND_2_4_GHZ && seen[0].mfp == WIFI_MFP_UNKNOWN);
	assert(seen[0].rssi == -48 && seen[0].channel == 11 && seen[0].mac_length == 6);
	assert(seen[0].mac[0] == 1 && seen[0].mac[5] == 6);
	for (size_t i = 0; i < ARRAY_SIZE(encs); i++) {
		nseen = 0;
		r.enc_type = encs[i].enc;
		core_scan_cb(core_scan_arg, 0, &r);
		assert(nseen == 1 && seen[0].security == encs[i].security);
	}
	core_scan_cb(core_scan_arg, -EIO, NULL);
	expect_log("result result result result result result result done(-5)");
}

static void test_connect(void)
{
	static const uint8_t ssid[] = "Net", psk[] = "password";
	struct wifi_connect_req_params p = {
		.ssid = ssid,
		.ssid_length = 3,
		.psk = psk,
		.psk_length = 8,
		.security = WIFI_SECURITY_TYPE_PSK,
	};

	assert(wifi_connect(NULL, &iface0, NULL) == -EINVAL);
	core_ret = -EALREADY;
	assert(wifi_connect(NULL, &iface0, &p) == -EALREADY);
	core_ret = 0;
	assert(core_params.ssid == ssid && core_params.ssid_len == 3);
	assert(core_params.psk == psk && core_params.psk_len == 8 && !core_params.psk_is_pbkdf2);
	assert(core_params.security == NINA_WIFI_SECURITY_PSK);
	p.security = WIFI_SECURITY_TYPE_NONE;
	p.psk_is_pbkdf2 = true;
	assert(wifi_connect(NULL, &iface0, &p) == 0);
	assert(core_params.security == NINA_WIFI_SECURITY_NONE && core_params.psk_is_pbkdf2);
	for (int s = WIFI_SECURITY_TYPE_PSK_SHA256; s <= WIFI_SECURITY_TYPE_WPA_PSK; s++) {
		p.security = s;
		assert(wifi_connect(NULL, &iface0, &p) == 0);
		assert(core_params.security == NINA_WIFI_SECURITY_OTHER);
	}
	expect_log("connect connect connect connect connect connect connect connect");

	core_ret = -EIO;
	assert(wifi_disconnect(NULL, &iface0) == -EIO);
	core_ret = 0;
	assert(wifi_disconnect(NULL, &iface0) == 0);
	expect_log("disconnect disconnect");
}

static void test_status(void)
{
	struct wifi_iface_status st;

	memset(&st, 0x5a, sizeof(st));
	core_ret = -EPROTO;
	assert(wifi_status(NULL, &iface0, &st) == -EPROTO);
	core_ret = 0;
	core_status = (struct nina_wifi_status){.connected = false};
	assert(wifi_status(NULL, &iface0, &st) == 0);
	assert(st.state == WIFI_STATE_DISCONNECTED && st.security == WIFI_SECURITY_TYPE_UNKNOWN);
	assert(st.mfp == WIFI_MFP_UNKNOWN && st.link_mode == WIFI_LINK_MODE_UNKNOWN);
	assert(st.band == WIFI_FREQ_BAND_UNKNOWN && st.iface_mode == WIFI_MODE_INFRA);

	core_status = (struct nina_wifi_status){
		.connected = true,
		.ssid = "Net",
		.ssid_len = 3,
		.bssid = {9, 8, 7, 6, 5, 4},
		.rssi = -61,
	};
	assert(wifi_status(NULL, &iface0, &st) == 0);
	assert(st.state == WIFI_STATE_COMPLETED && st.ssid_len == 3 && strcmp(st.ssid, "Net") == 0);
	assert(st.bssid[0] == 9 && st.bssid[5] == 4 && st.rssi == -61);
	assert(st.band == WIFI_FREQ_BAND_2_4_GHZ && st.iface_mode == WIFI_MODE_INFRA);
	expect_log("status status status");
}

static void test_ops(void)
{
	static const uint8_t addr[4] = {10, 42, 0, 97}, mask[4] = {255, 255, 255, 0},
			     gw[4] = {10, 42, 0, 1};

	assert(core_ops->ip_set(addr, mask, gw) == 0);
	core_ops->link_up();
	expect_log("add(10.42.0.97) netmask(10.42.0.97,255.255.255.0) gw(10.42.0.1) dormant_off");
	addr_room = false;
	assert(core_ops->ip_set(addr, mask, gw) == -ENOMEM);
	addr_room = true;
	expect_log("add(10.42.0.97)");
	core_ops->link_down(addr);
	expect_log("rm(10.42.0.97) gw(0.0.0.0) dormant_on");
	core_ops->connect_result(0);
	core_ops->connect_result(-ETIMEDOUT);
	core_ops->disconnected(NINA_WIFI_DISCONNECT_SCAN);
	core_ops->disconnected(NINA_WIFI_DISCONNECT_REQUEST);
	core_ops->disconnected(NINA_WIFI_DISCONNECT_LINK_LOST);
	expect_log("connect_event(0) connect_event(-110) disconnect_event(0) disconnect_event(0) "
		   "disconnect_event(0)");
}

static void test_threads(void)
{
	assert(strcmp(nina_scan_tid.name, "nina_scan_tid") == 0);
	assert(nina_scan_tid.stack_size == 1536 && nina_scan_tid.prio == 8);
	assert(nina_connect_tid.stack_size == 1024 && nina_connect_tid.prio == 8);
	assert(nina_monitor_tid.stack_size == 1536 && nina_monitor_tid.prio == 9);
	nina_scan_tid.entry(NULL, NULL, NULL);
	nina_connect_tid.entry(NULL, NULL, NULL);
	nina_monitor_tid.entry(NULL, NULL, NULL);
	expect_log("scan_thread connect_thread monitor_thread");
}

int main(void)
{
	test_device();
	test_scan();
	test_connect();
	test_status();
	test_ops();
	test_threads();
	puts("NINA Zephyr binding: tests passed");
	return 0;
}
