/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the NINA core's NuttX binding, wlan0:
 * common/drivers/nina/nuttx/nina_netdev.c (included), on NuttX's own
 * <nuttx/wireless/wireless.h> (the fetched tree) and fakes of the other
 * NuttX declarations it uses (tests/nina_netdev_fakes: the network device,
 * mutexes, semaphores, kernel threads), of the core's Wi-Fi functions
 * (nina_wifi.h) and of the applications' Zephyr Wi-Fi management
 * (nina_wifi_mgmt.h), which record what they are asked.  The mapping:
 *
 * - nina_net_initialize(): the management first (its failure ends it),
 *   wlan0 registered as IEEE 802.11 with the ioctl handler, no link-layer
 *   header, an MTU of 1500, administratively up, no carrier, then the
 *   core's init and its MAC address on wlan0 (none when it fails).
 * - The core's operations: the address, netmask and gateway in d_ipaddr,
 *   d_netmask and d_draddr; the carrier on; the address (only if it is
 *   wlan0's) and the gateway cleared and the carrier off; the connect
 *   result and the disconnection (every reason) to the management; its
 *   hooks.
 * - SIOCSIWSCAN: -ENOTSUP for a passive scan, an ESSID, a channel (only
 *   the first three looked at), -ENETDOWN while wlan0 is down, -EBUSY
 *   while a scan runs, the core's error, which SIOCGIWSCAN then reports.
 *   SIOCGIWSCAN: -EAGAIN while scanning, the results as wireless events
 *   (BSSID, channel, RSSI in dBm, ESSID, open or encrypted, no encoding
 *   for an unknown type) last first, -E2BIG for a small buffer, nothing
 *   for no network, the scan's error.
 * - SIOCSIWAUTH, SIOCSIWENCODEEXT, SIOCSIWESSID: the securities (open,
 *   WPA2, a key alone; WPA, WPA3, WEP as "other"), the checks of Zephyr's
 *   Wi-Fi management on the key (-EINVAL), -ENETDOWN, the SSID's length,
 *   the settings forgotten after each attempt, wapi's "disconnect" (flags
 *   0) doing nothing.
 * - SIOCSIWAP: the two addresses that disconnect, -ENOTSUP for others,
 *   -ENETDOWN.  SIOCGIWESSID, SIOCGIWAP, SIOCGIWSENS from the core's
 *   status; the modes; -ENOTTY, -EINVAL.
 * - nina_net_start(): nina_scan_tid (priority 110, 1536 bytes),
 *   nina_connect_tid (110, 1024), nina_monitor_tid (109, 1536), each
 *   running its core loop before the next starts; a failure stops it.
 */

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

#include "../../common/drivers/nina/nuttx/nina_netdev.c"

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

/* NuttX ------------------------------------------------------------------- */

static int register_ret;
static int netdev_locked;

int netdev_register(struct net_driver_s *dev, enum net_lltype_e lltype)
{
	assert(dev == &g_dev && lltype == NET_LL_IEEE80211);
	note("register");
	return register_ret;
}

void netdev_lock(struct net_driver_s *dev)
{
	assert(dev == &g_dev && !netdev_locked);
	netdev_locked = 1;
}

void netdev_unlock(struct net_driver_s *dev)
{
	assert(dev == &g_dev && netdev_locked);
	netdev_locked = 0;
}

void netdev_carrier_on(struct net_driver_s *dev)
{
	assert(netdev_locked);
	dev->d_flags |= IFF_RUNNING;
	note("carrier_on");
}

void netdev_carrier_off(struct net_driver_s *dev)
{
	assert(netdev_locked);
	dev->d_flags &= ~IFF_RUNNING;
	note("carrier_off");
}

int nxmutex_lock(mutex_t *m)
{
	assert(!m->locked);
	m->locked = 1;
	return 0;
}

int nxmutex_unlock(mutex_t *m)
{
	assert(m->locked);
	m->locked = 0;
	return 0;
}

int nxsem_post(sem_t *sem)
{
	sem->count++;
	return 0;
}

int nxsem_wait_uninterruptible(sem_t *sem)
{
	assert(sem->count > 0);
	sem->count--;
	return 0;
}

static int kthread_ret;

/* The thread runs at once (it returns: the core's loops are fakes). */
int kthread_create_with_stack(const char *name, int priority, void *stack_addr, int stack_size,
			      main_t entry, char *const argv[])
{
	assert(argv == NULL && stack_addr != NULL && ((uintptr_t)stack_addr & 7) == 0);
	note("thread(%s,%d,%d)", name, priority, stack_size);
	if (kthread_ret) {
		return kthread_ret;
	}
	assert(entry(0, NULL) == 0);
	return 3;
}

/* The management (nina_wifi_mgmt.c) ------------------------------------------ */

static int mgmt_ret;

int nina_wifi_mgmt_initialize(struct net_driver_s *dev)
{
	assert(dev == &g_dev);
	note("mgmt");
	return mgmt_ret;
}

void nina_wifi_mgmt_connect_result(int status)
{
	note("connect_event(%d)", status);
}

void nina_wifi_mgmt_disconnect_result(void)
{
	note("disconnect_event");
}

bool nina_wifi_mgmt_connection_allowed(void)
{
	return true;
}

void nina_wifi_mgmt_pause_reconnect(void)
{
}

void nina_wifi_mgmt_request_reconnect(void)
{
}

/* The core ---------------------------------------------------------------- */

static const struct nina_wifi_ops *core_ops;
static int core_ret;
static nina_wifi_scan_cb_t core_scan_cb;
static void *core_scan_arg;
static struct nina_wifi_connect_params core_params;
static uint8_t core_psk[80];
static struct nina_wifi_status core_status;

int nina_wifi_init(const struct nina_wifi_ops *with, uint8_t out[NINA_WIFI_MAC_LEN])
{
	static const uint8_t m[6] = {0xb1, 2, 3, 4, 5, 0xb6};

	assert(IFF_IS_UP(g_dev.d_flags) && !IFF_IS_RUNNING(g_dev.d_flags));
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
	if (params->psk) {
		memcpy(core_psk, params->psk, params->psk_len);
		core_params.psk = core_psk;
	}
	note("connect(%.*s,%s,%zu)", (int)params->ssid_len, (const char *)params->ssid,
	     params->security == NINA_WIFI_SECURITY_NONE  ? "none"
	     : params->security == NINA_WIFI_SECURITY_PSK ? "psk"
							  : "other",
	     params->psk ? params->psk_len : 0);
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

/* Helpers ----------------------------------------------------------------- */

static int ioctl(int cmd, struct iwreq *iwr)
{
	return g_dev.d_ioctl(&g_dev, cmd, (unsigned long)(uintptr_t)iwr);
}

static in_addr_t addr(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	uint8_t bytes[4] = {a, b, c, d};
	in_addr_t v;

	memcpy(&v, bytes, 4);
	return v;
}

/* The tests --------------------------------------------------------------- */

static void test_initialize(void)
{
	mgmt_ret = -ENOMEM;
	assert(nina_net_initialize() == -ENOMEM);
	expect_log("mgmt");
	mgmt_ret = 0;
	register_ret = -EEXIST;
	assert(nina_net_initialize() == -EEXIST);
	expect_log("mgmt register");
	register_ret = 0;
	core_ret = -ETIMEDOUT;
	assert(nina_net_initialize() == 0);
	expect_log("mgmt register carrier_off init");
	assert(g_dev.d_mac.ether.ether_addr_octet[0] == 0);
	core_ret = 0;
	assert(nina_net_initialize() == 0);
	expect_log("mgmt register carrier_off init");
	assert(g_dev.d_ioctl == nina_ioctl && g_dev.d_llhdrlen == 0 && g_dev.d_pktsize == 1500);
	assert(IFF_IS_UP(g_dev.d_flags) && !IFF_IS_RUNNING(g_dev.d_flags));
	assert(g_dev.d_mac.ether.ether_addr_octet[0] == 0xb1);
	assert(g_dev.d_mac.ether.ether_addr_octet[5] == 0xb6);
	assert(core_ops == &g_ops && !netdev_locked);
}

static void test_ops(void)
{
	static const uint8_t a[4] = {10, 42, 0, 97}, m[4] = {255, 255, 255, 0},
			     gw[4] = {10, 42, 0, 1}, other[4] = {10, 42, 0, 98};

	assert(core_ops->ip_set(a, m, gw) == 0);
	core_ops->link_up();
	expect_log("carrier_on");
	assert(g_dev.d_ipaddr == addr(10, 42, 0, 97) && g_dev.d_netmask == addr(255, 255, 255, 0));
	assert(g_dev.d_draddr == addr(10, 42, 0, 1) && IFF_IS_RUNNING(g_dev.d_flags));
	/* Another address: wlan0's stays; the gateway and the carrier go. */
	core_ops->link_down(other);
	assert(g_dev.d_ipaddr == addr(10, 42, 0, 97) && g_dev.d_draddr == 0);
	core_ops->link_down(a);
	expect_log("carrier_off carrier_off");
	assert(g_dev.d_ipaddr == 0 && g_dev.d_draddr == 0 && !IFF_IS_RUNNING(g_dev.d_flags));
	assert(g_dev.d_netmask == addr(255, 255, 255, 0));
	core_ops->connect_result(0);
	core_ops->connect_result(-EIO);
	core_ops->disconnected(NINA_WIFI_DISCONNECT_SCAN);
	core_ops->disconnected(NINA_WIFI_DISCONNECT_REQUEST);
	core_ops->disconnected(NINA_WIFI_DISCONNECT_LINK_LOST);
	expect_log("connect_event(0) connect_event(-5) disconnect_event disconnect_event "
		   "disconnect_event");
	assert(core_ops->connection_allowed == nina_wifi_mgmt_connection_allowed);
	assert(core_ops->reconnect_pause == nina_wifi_mgmt_pause_reconnect);
	assert(core_ops->reconnect_request == nina_wifi_mgmt_request_reconnect);
	assert(!netdev_locked);
}

static void put_req(struct iwreq *iwr, struct iw_scan_req *req)
{
	memset(iwr, 0, sizeof(*iwr));
	iwr->u.data.pointer = req;
	iwr->u.data.length = sizeof(*req);
}

static void test_scan(void)
{
	struct iwreq iwr;
	struct iw_scan_req req;
	uint8_t buf[1024];
	struct nina_wifi_scan_result r = {
		.ssid = "AP-one",
		.ssid_len = 6,
		.rssi = -52,
		.enc_type = NINA_ENC_TYPE_CCMP,
		.bssid = {1, 2, 3, 4, 5, 6},
		.channel = 6,
	};
	struct iw_event ev;
	size_t at;

	/* What the module cannot follow; the status stays for SIOCGIWSCAN. */
	memset(&req, 0, sizeof(req));
	req.scan_type = IW_SCAN_TYPE_PASSIVE;
	put_req(&iwr, &req);
	assert(ioctl(SIOCSIWSCAN, &iwr) == -ENOTSUP);
	memset(&req, 0, sizeof(req));
	req.essid_len = 2;
	put_req(&iwr, &req);
	iwr.u.data.flags = IW_SCAN_THIS_ESSID;
	assert(ioctl(SIOCSIWSCAN, &iwr) == -ENOTSUP);
	memset(&req, 0, sizeof(req));
	req.num_channels = 3;
	req.channel_list[2].m = 11;
	put_req(&iwr, &req);
	assert(ioctl(SIOCSIWSCAN, &iwr) == -ENOTSUP);
	assert(ioctl(SIOCGIWSCAN, &iwr) == -ENOTSUP);
	/* A fourth channel is not looked at; an ESSID without the flag
	 * either. */
	memset(&req, 0, sizeof(req));
	req.num_channels = 4;
	req.channel_list[3].m = 11;
	req.essid_len = 2;
	put_req(&iwr, &req);
	assert(ioctl(SIOCSIWSCAN, &iwr) == 0);
	expect_log("scan");
	assert(core_scan_arg == NULL);
	assert(ioctl(SIOCSIWSCAN, &iwr) == -EBUSY);
	memset(&iwr, 0, sizeof(iwr));
	assert(ioctl(SIOCGIWSCAN, &iwr) == -EAGAIN);

	/* Three networks: WPA2, open, an unknown type. */
	core_scan_cb(core_scan_arg, 0, &r);
	r.enc_type = NINA_ENC_TYPE_NONE;
	r.ssid_len = 2;
	r.channel = 11;
	core_scan_cb(core_scan_arg, 0, &r);
	r.enc_type = 9;
	core_scan_cb(core_scan_arg, 0, &r);
	core_scan_cb(core_scan_arg, 0, NULL);
	memset(&iwr, 0, sizeof(iwr));
	iwr.u.data.pointer = buf;
	iwr.u.data.length = 2 * IWE_RESULT_LEN + IWE_RESULT_LEN - IWE_ENCODE_LEN - 1;
	assert(ioctl(SIOCGIWSCAN, &iwr) == -E2BIG);
	iwr.u.data.length = sizeof(buf);
	assert(ioctl(SIOCGIWSCAN, &iwr) == 0);
	assert(iwr.u.data.length == 3 * IWE_RESULT_LEN - IWE_ENCODE_LEN);

	/* Last first: the unknown type (no SIOCGIWENCODE), the open one, the
	 * WPA2 one. */
	at = 0;
	for (int n = 0; n < 3; n++) {
		const int enc = n == 0 ? -1 : n == 1 ? 0 : 1;

		memcpy(&ev, buf + at, IWE_AP_LEN);
		assert(ev.cmd == SIOCGIWAP && ev.len == IWE_AP_LEN);
		assert(ev.u.ap_addr.sa_family == ARPHRD_ETHER && ev.u.ap_addr.sa_data[5] == 6);
		at += IWE_AP_LEN;
		memcpy(&ev, buf + at, IWE_FREQ_LEN);
		assert(ev.cmd == SIOCGIWFREQ && ev.u.freq.m == (n == 2 ? 6 : 11) && ev.u.freq.e == 0);
		at += IWE_FREQ_LEN;
		memcpy(&ev, buf + at, IWE_QUAL_LEN);
		assert(ev.cmd == IWEVQUAL && (int8_t)ev.u.qual.level == -52);
		assert(ev.u.qual.updated == IW_QUAL_DBM);
		at += IWE_QUAL_LEN;
		memcpy(&ev, buf + at, IWE_HDR + sizeof(struct iw_point));
		assert(ev.cmd == SIOCGIWESSID && ev.len == IWE_ESSID_LEN);
		assert(ev.u.essid.length == (n == 2 ? 6 : 2) && ev.u.essid.flags == 1);
		assert((uintptr_t)ev.u.essid.pointer == sizeof(struct iw_point));
		assert(memcmp(buf + at + IWE_HDR + sizeof(struct iw_point), "AP-one",
			      n == 2 ? 6 : 2) == 0);
		at += IWE_ESSID_LEN;
		if (enc >= 0) {
			memcpy(&ev, buf + at, IWE_ENCODE_LEN);
			assert(ev.cmd == SIOCGIWENCODE);
			assert(ev.u.data.flags ==
			       (enc ? IW_ENCODE_ENABLED | IW_ENCODE_NOKEY : IW_ENCODE_DISABLED));
			at += IWE_ENCODE_LEN;
		}
	}
	assert(at == iwr.u.data.length);

	/* A scan without networks; a scan that fails; the core refuses. */
	memset(&iwr, 0, sizeof(iwr));
	assert(ioctl(SIOCSIWSCAN, &iwr) == 0);
	core_scan_cb(core_scan_arg, 0, NULL);
	iwr.u.data.length = 100;
	assert(ioctl(SIOCGIWSCAN, &iwr) == 0 && iwr.u.data.length == 0);
	assert(ioctl(SIOCSIWSCAN, &iwr) == 0);
	core_scan_cb(core_scan_arg, -ETIMEDOUT, NULL);
	assert(ioctl(SIOCGIWSCAN, &iwr) == -ETIMEDOUT);
	core_ret = -EBUSY;
	assert(ioctl(SIOCSIWSCAN, &iwr) == -EBUSY);
	assert(ioctl(SIOCGIWSCAN, &iwr) == -EBUSY);
	core_ret = 0;
	expect_log("scan scan scan");

	/* wlan0 down (ifconfig wlan0 down). */
	g_dev.d_flags &= ~IFF_UP;
	assert(ioctl(SIOCSIWSCAN, &iwr) == -ENETDOWN);
	assert(ioctl(SIOCGIWSCAN, &iwr) == -ENETDOWN);
	g_dev.d_flags |= IFF_UP;
	expect_log("");
	assert(!g_scan_lock.locked);
}

static void set_auth(int index, uint32_t value)
{
	struct iwreq iwr;

	memset(&iwr, 0, sizeof(iwr));
	iwr.u.param.flags = index;
	iwr.u.param.value = (int32_t)value;
	assert(ioctl(SIOCSIWAUTH, &iwr) == 0);
}

static void set_key(const char *key, uint16_t alg)
{
	struct iwreq iwr;
	uint8_t buf[sizeof(struct iw_encode_ext) + 80];
	struct iw_encode_ext ext;

	memset(&ext, 0, sizeof(ext));
	ext.alg = alg;
	ext.key_len = strlen(key);
	memcpy(buf, &ext, sizeof(ext));
	memcpy(buf + sizeof(ext), key, ext.key_len);
	memset(&iwr, 0, sizeof(iwr));
	iwr.u.encoding.pointer = buf;
	iwr.u.encoding.length = sizeof(ext) + ext.key_len;
	assert(ioctl(SIOCSIWENCODEEXT, &iwr) == 0);
}

static int essid(const char *ssid, uint16_t flags)
{
	struct iwreq iwr;

	memset(&iwr, 0, sizeof(iwr));
	iwr.u.essid.pointer = (void *)ssid;
	iwr.u.essid.length = ssid ? strlen(ssid) : 0;
	iwr.u.essid.flags = flags;
	return ioctl(SIOCSIWESSID, &iwr);
}

static void test_connect(void)
{
	struct iwreq iwr;
	static const char long_ssid[] = "123456789012345678901234567890123";
	uint8_t buf[sizeof(struct iw_encode_ext) + 70];
	struct iw_encode_ext ext;

	/* Open; WPA2; a key alone. */
	assert(essid("Net", 1) == 0);
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_WPA2);
	set_auth(IW_AUTH_CIPHER_PAIRWISE, IW_AUTH_CIPHER_CCMP);
	set_key("password", IW_ENCODE_ALG_CCMP);
	assert(essid("Net", 1) == 0);
	assert(memcmp(core_psk, "password", 8) == 0);
	set_key("password", IW_ENCODE_ALG_CCMP);
	assert(essid("Net", 1) == 0);
	/* Forgotten: open again. */
	assert(essid("Net", 1) == 0);
	expect_log("connect(Net,none,0) connect(Net,psk,8) connect(Net,psk,8) connect(Net,none,0)");

	/* The other securities, after Zephyr's checks of the key. */
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_WPA);
	set_key("password", IW_ENCODE_ALG_TKIP);
	assert(essid("Net", 1) == 0);
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_WPA3);
	set_key("password", IW_ENCODE_ALG_CCMP);
	assert(essid("Net", 1) == 0);
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_DISABLED);
	set_auth(IW_AUTH_CIPHER_PAIRWISE, IW_AUTH_CIPHER_WEP40);
	set_key("12345", IW_ENCODE_ALG_WEP);
	assert(essid("Net", 1) == 0);
	expect_log("connect(Net,other,8) connect(Net,other,8) connect(Net,other,5)");
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_DISABLED);
	set_auth(IW_AUTH_CIPHER_PAIRWISE, IW_AUTH_CIPHER_WEP104);
	set_key("123456", IW_ENCODE_ALG_WEP);
	assert(essid("Net", 1) == -EINVAL); /* a WEP key of 6 bytes */
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_DISABLED);
	set_auth(IW_AUTH_CIPHER_PAIRWISE, IW_AUTH_CIPHER_WEP40);
	assert(essid("Net", 1) == -EINVAL); /* WEP without a key */
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_WPA3);
	assert(essid("Net", 1) == -EINVAL); /* WPA3 without a key */
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_WPA2);
	assert(essid("Net", 1) == -EINVAL); /* WPA2 without a key */
	set_key("1234567", IW_ENCODE_ALG_CCMP);
	assert(essid("Net", 1) == -EINVAL); /* a key of 7 bytes */
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_DISABLED);
	set_key("1234567", IW_ENCODE_ALG_CCMP);
	assert(essid("Net", 1) == -EINVAL); /* open, with a key of 7 */
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_DISABLED);
	set_key("12345678", IW_ENCODE_ALG_CCMP);
	assert(essid("Net", 1) == 0); /* open, with a key the core ignores */
	expect_log("connect(Net,none,8)");

	/* A key of 64 bytes passes Zephyr's checks (the core refuses it). */
	set_key("1234567890123456789012345678901234567890123456789012345678901234",
		IW_ENCODE_ALG_CCMP);
	core_ret = -EINVAL;
	assert(essid("Net", 1) == -EINVAL);
	core_ret = 0;
	expect_log("connect(Net,psk,64)");
	/* 65 bytes are refused by SIOCSIWENCODEEXT. */
	memset(&ext, 0, sizeof(ext));
	ext.alg = IW_ENCODE_ALG_CCMP;
	ext.key_len = 65;
	memcpy(buf, &ext, sizeof(ext));
	memset(&iwr, 0, sizeof(iwr));
	iwr.u.encoding.pointer = buf;
	iwr.u.encoding.length = sizeof(ext) + 65;
	assert(ioctl(SIOCSIWENCODEEXT, &iwr) == -EINVAL);
	iwr.u.encoding.length = sizeof(ext) - 1;
	assert(ioctl(SIOCSIWENCODEEXT, &iwr) == -EINVAL);
	/* An unknown SIOCSIWAUTH index. */
	memset(&iwr, 0, sizeof(iwr));
	iwr.u.param.flags = IW_AUTH_KEY_MGMT;
	assert(ioctl(SIOCSIWAUTH, &iwr) == -ENOTSUP);

	/* The SSID; wapi's "disconnect"; wlan0 down (the settings are
	 * forgotten all the same). */
	assert(essid("", 1) == -EINVAL);
	assert(essid(NULL, 1) == -EINVAL);
	assert(essid(long_ssid, 1) == -EINVAL);
	assert(essid("12345678901234567890123456789012", 1) == 0);
	assert(essid("random", 0) == 0);
	expect_log("connect(12345678901234567890123456789012,none,0)");
	g_dev.d_flags &= ~IFF_UP;
	set_auth(IW_AUTH_WPA_VERSION, IW_AUTH_WPA_VERSION_WPA2);
	set_key("password", IW_ENCODE_ALG_CCMP);
	assert(essid("Net", 1) == -ENETDOWN);
	g_dev.d_flags |= IFF_UP;
	assert(essid("Net", 1) == 0);
	expect_log("connect(Net,none,0)");
	core_ret = -EALREADY;
	assert(essid("Net", 1) == -EALREADY);
	core_ret = 0;
	expect_log("connect(Net,none,0)");
	assert(!g_wext_lock.locked && g_key_len == 0);
}

static void test_disconnect_status(void)
{
	struct iwreq iwr;
	char ssid[40];

	memset(&iwr, 0, sizeof(iwr));
	assert(ioctl(SIOCSIWAP, &iwr) == 0);
	memset(iwr.u.ap_addr.sa_data, 0xff, 6);
	assert(ioctl(SIOCSIWAP, &iwr) == 0);
	iwr.u.ap_addr.sa_data[2] = 0;
	assert(ioctl(SIOCSIWAP, &iwr) == -ENOTSUP);
	memset(&iwr, 0, sizeof(iwr));
	g_dev.d_flags &= ~IFF_UP;
	assert(ioctl(SIOCSIWAP, &iwr) == -ENETDOWN);
	g_dev.d_flags |= IFF_UP;
	core_ret = -EIO;
	assert(ioctl(SIOCSIWAP, &iwr) == -EIO);
	core_ret = 0;
	expect_log("disconnect disconnect disconnect");

	/* Not connected. */
	core_status = (struct nina_wifi_status){.connected = false};
	memset(&iwr, 0, sizeof(iwr));
	assert(ioctl(SIOCGIWESSID, &iwr) == -EINVAL);
	iwr.u.essid.pointer = ssid;
	iwr.u.essid.length = sizeof(ssid);
	assert(ioctl(SIOCGIWESSID, &iwr) == 0);
	assert(iwr.u.essid.length == 0 && iwr.u.essid.flags == 0);
	memset(iwr.u.ap_addr.sa_data, 0x77, 6);
	assert(ioctl(SIOCGIWAP, &iwr) == 0);
	assert(iwr.u.ap_addr.sa_family == ARPHRD_ETHER && iwr.u.ap_addr.sa_data[0] == 0);
	assert(ioctl(SIOCGIWSENS, &iwr) == -ENOTCONN);

	/* Connected. */
	core_status = (struct nina_wifi_status){
		.connected = true,
		.ssid = "Net",
		.ssid_len = 3,
		.bssid = {9, 8, 7, 6, 5, 4},
		.rssi = -63,
	};
	memset(&iwr, 0, sizeof(iwr));
	memset(ssid, 0, sizeof(ssid));
	iwr.u.essid.pointer = ssid;
	iwr.u.essid.length = 2; /* too short: cut, the whole length reported */
	assert(ioctl(SIOCGIWESSID, &iwr) == 0);
	assert(iwr.u.essid.length == 3 && iwr.u.essid.flags == 1 && strcmp(ssid, "Ne") == 0);
	assert(ioctl(SIOCGIWAP, &iwr) == 0);
	assert(iwr.u.ap_addr.sa_data[0] == 9 && iwr.u.ap_addr.sa_data[5] == 4);
	assert(ioctl(SIOCGIWSENS, &iwr) == 0 && iwr.u.sens.value == 63);
	core_ret = -EPROTO;
	assert(ioctl(SIOCGIWAP, &iwr) == -EPROTO);
	assert(ioctl(SIOCGIWSENS, &iwr) == -EPROTO);
	iwr.u.essid.pointer = ssid;
	assert(ioctl(SIOCGIWESSID, &iwr) == -EPROTO);
	core_ret = 0;
	expect_log("status status status status status status status status status");

	/* The modes, the others, no argument. */
	assert(ioctl(SIOCGIWMODE, &iwr) == 0 && iwr.u.mode == IW_MODE_INFRA);
	iwr.u.mode = IW_MODE_INFRA;
	assert(ioctl(SIOCSIWMODE, &iwr) == 0);
	iwr.u.mode = IW_MODE_ADHOC;
	assert(ioctl(SIOCSIWMODE, &iwr) == -ENOTSUP);
	assert(ioctl(SIOCGIWNAME, &iwr) == -ENOTTY);
	assert(g_dev.d_ioctl(&g_dev, SIOCGIWMODE, 0) == -EINVAL);
}

static void test_start(void)
{
	assert(nina_net_start() == 0);
	expect_log("thread(nina_scan_tid,110,1536) scan_thread "
		   "thread(nina_connect_tid,110,1024) connect_thread "
		   "thread(nina_monitor_tid,109,1536) monitor_thread");
	assert(g_thread_started.count == 0);
	kthread_ret = -ENOMEM;
	assert(nina_net_start() == -ENOMEM);
	expect_log("thread(nina_scan_tid,110,1536)");
	kthread_ret = 0;
}

int main(void)
{
	test_initialize();
	test_ops();
	test_scan();
	test_connect();
	test_disconnect_status();
	test_start();
	printf("nina_netdev: ok\n");
	return 0;
}
