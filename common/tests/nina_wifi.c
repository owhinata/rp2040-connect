/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of common/drivers/nina/nina_wifi.c (included) on fakes of its
 * boundary: the module's commands (nina_proto_request(), nina_proto_ip_config(), the
 * capabilities), the kernel services of nina_os.h (the Wi-Fi lock and
 * wake-up, the sleeps and the clock) and a binding's struct
 * nina_wifi_ops, which record what they are asked.  The Wi-Fi thread's
 * passes (wifi_once()) run in the test's thread, after the request that
 * posts them; the fake wait returns at once when a post is pending (logged
 * "wake"), else it moves the clock by its time (logged "wait<ms>").
 *
 * - nina_wifi_init(): the MAC address (0x22 with the dummy), a short
 *   answer or an error, the capability probe after it either way.
 * - nina_wifi_connect(): the checks of the parameters in their order, the
 *   application's refusal, -EALREADY, -EBUSY during an attempt or a scan;
 *   0x11 with the SSID and the passphrase, 0x10 with the SSID alone; the
 *   module's refusal; then the Wi-Fi thread: 0x20 every 500 ms until
 *   connected, 0x17 (no power save), 0x21 until it has an address, the
 *   address, netmask and gateway, link up and the result 0 under the lock;
 *   60 checks then -ETIMEDOUT; a power mode refused (-EIO); a bad answer
 *   (-EPROTO); a binding's ip_set() error; a disconnection during the
 *   attempt (no result, and -ECANCELED at the address); the next attempt
 *   after each.
 * - nina_wifi_disconnect(): the pause, 0x30 with the dummy, link down
 *   (only when connected), the event; the module's refusal.
 * - The monitor: IDLE waits until its deadline, then the check, every 2 s
 *   after the end of the last; no command while not connected; a link seen
 *   down twice in a row is lost (link down, the event, the reconnection);
 *   once is not.
 * - nina_wifi_scan(): 0x27 on 3.0.2 with the link given up first and the
 *   reconnection asked after; the link kept with NINA_CAP_SCAN_KEEPS_LINK;
 *   the asynchronous scan (0x36, 0x6B every 100 ms) and its ends (not
 *   started, stopped, failed, past 10 s without 0x27); each network's
 *   0x32, 0x33, 0x3C (reversed) and 0x3D; a bad record, more than 10
 *   networks, a short answer; -EBUSY while scanning or connecting.
 * - nina_wifi_status(): not connected (one command), connected (0x23, the
 *   BSSID reversed, the RSSI), a short answer.
 * - Without the optional hooks: connections allowed, nothing paused or
 *   asked.
 * - The one thread's states together: the monitor's checks at their
 *   deadlines within an asynchronous scan that keeps the link (the sleep
 *   split there; a loss seen twice is reported once, during the scan,
 *   whose results still come), and IDLE's wait after it until the next
 *   deadline; a connection and a scan refused (-EBUSY) while the results
 *   are fetched; the checks within an attempt asking nothing (the count
 *   reset) and the period started again from the connection; a request
 *   posted at the deadline (the check first, then the request); the 10 s
 *   of a scan with a check every 2 s, and the 30 s of an attempt with the
 *   checks between its polls.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../drivers/nina/nina_wifi.c"

/* The module ------------------------------------------------------------ */

struct step {
	uint8_t cmd;
	int ret;
	uint8_t reply[340];
	size_t len;
};

static struct step steps[200];
static unsigned int nsteps, step_at;
static char log_buf[16384];
static uint32_t caps;
static unsigned int probes;
static uint8_t ip_replies[4][12];
static unsigned int nips, ip_at;
static int ip_ret;
/* Called once at the command HOOK_ON (0x21: nina_proto_ip_config()), before its
 * answer. */
static void (*hook)(void);
static uint8_t hook_on;

static void run_hook(uint8_t cmd)
{
	void (*h)(void) = hook;

	if (h && cmd == hook_on) {
		hook = NULL;
		h();
	}
}

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

/* Whether the log ends with EXPECTED (and clears it). */
static bool log_ends(const char *expected)
{
	size_t n = strlen(log_buf), m = strlen(expected);
	bool ends;

	if (n && log_buf[n - 1] == ' ') {
		log_buf[--n] = '\0';
	}
	ends = n >= m && strcmp(log_buf + n - m, expected) == 0;
	if (!ends) {
		fprintf(stderr, "expected the end: %s\n               got: %s\n", expected, log_buf);
	}
	log_buf[0] = '\0';
	return ends;
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

static void answer(uint8_t cmd, int ret, const void *reply, size_t len)
{
	assert(nsteps < sizeof(steps) / sizeof(steps[0]));
	assert(len <= sizeof(steps[0].reply));
	steps[nsteps].cmd = cmd;
	steps[nsteps].ret = ret;
	if (len) {
		memcpy(steps[nsteps].reply, reply, len);
	}
	steps[nsteps].len = len;
	nsteps++;
}

static void answer_byte(uint8_t cmd, uint8_t value)
{
	answer(cmd, 0, &value, 1);
}

static void steps_done(void)
{
	assert(step_at == nsteps);
	step_at = nsteps = 0;
}

int nina_proto_request(uint8_t cmd, const struct nina_proto_param *params, size_t nparams, bool wide_params,
		 bool wide_response, uint8_t *out, size_t cap, size_t *out_len)
{
	char line[200];
	size_t at;
	const struct step *s;

	assert(!wide_params && !wide_response);
	at = (size_t)snprintf(line, sizeof(line), "%02x", cmd);
	for (size_t i = 0; i < nparams; i++) {
		at += (size_t)snprintf(line + at, sizeof(line) - at, "(");
		for (size_t j = 0; j < params[i].len; j++) {
			at += (size_t)snprintf(line + at, sizeof(line) - at, "%02x",
					       params[i].data[j]);
		}
		at += (size_t)snprintf(line + at, sizeof(line) - at, ")");
	}
	note("%s", line);
	assert(step_at < nsteps);
	s = &steps[step_at++];
	assert(s->cmd == cmd);
	run_hook(cmd);
	if (s->ret) {
		return s->ret;
	}
	if (s->len > cap) {
		return -EMSGSIZE; /* as nina_proto.c */
	}
	memcpy(out, s->reply, s->len);
	*out_len = s->len;
	return 0;
}

int nina_proto_ip_config(uint8_t data[12])
{
	note("21");
	run_hook(NINA_CMD_GET_IPADDR);
	if (ip_ret) {
		return ip_ret;
	}
	assert(nips);
	memcpy(data, ip_replies[ip_at < nips ? ip_at : nips - 1], 12);
	ip_at++;
	return 0;
}

uint32_t nina_proto_probe_capabilities(void)
{
	probes++;
	return caps;
}

uint32_t nina_proto_capabilities(void)
{
	return caps;
}

/* The kernel ------------------------------------------------------------ */

static int lock_depth;
static bool woken;
static int64_t now_ms;

void nina_os_wifi_lock(void)
{
	assert(lock_depth == 0);
	lock_depth++;
}

void nina_os_wifi_unlock(void)
{
	assert(lock_depth == 1);
	lock_depth--;
}

void nina_os_wifi_post(void)
{
	note("post");
	woken = true;
}

void nina_os_wifi_wait(uint32_t ms)
{
	assert(lock_depth == 0);
	if (woken) {
		woken = false;
		note("wake");
		return;
	}
	assert(ms > 0 && ms <= NINA_MONITOR_MS);
	note("wait%u", (unsigned int)ms);
	now_ms += ms;
}

void nina_os_sleep_ms(uint32_t ms)
{
	assert(lock_depth == 0);
	note("sleep%u", (unsigned int)ms);
	now_ms += ms;
}

int64_t nina_os_uptime_ms(void)
{
	return now_ms;
}

/* The binding ----------------------------------------------------------- */

static int ip_set_ret;
static bool allowed = true;

static int fake_ip_set(const uint8_t *addr, const uint8_t *netmask, const uint8_t *gateway)
{
	assert(lock_depth == 1);
	note("ip_set(%u.%u.%u.%u/%u.%u.%u.%u,%u.%u.%u.%u)", addr[0], addr[1], addr[2], addr[3],
	     netmask[0], netmask[1], netmask[2], netmask[3], gateway[0], gateway[1], gateway[2],
	     gateway[3]);
	return ip_set_ret;
}

static void fake_link_up(void)
{
	assert(lock_depth == 1);
	note("up");
}

static void fake_link_down(const uint8_t *addr)
{
	assert(lock_depth == 1);
	note("down(%u.%u.%u.%u)", addr[0], addr[1], addr[2], addr[3]);
}

static void fake_connect_result(int status)
{
	assert(lock_depth == 1);
	note("result(%d)", status);
}

static void fake_disconnected(enum nina_wifi_disconnect_reason reason)
{
	assert(lock_depth == 0);
	note("disconnected(%s)", reason == NINA_WIFI_DISCONNECT_SCAN      ? "scan"
				 : reason == NINA_WIFI_DISCONNECT_REQUEST ? "request"
									  : "lost");
}

static bool fake_connection_allowed(void)
{
	assert(lock_depth == 1);
	note("allowed?");
	return allowed;
}

static void fake_reconnect_pause(void)
{
	assert(lock_depth == 1);
	note("pause");
}

static void fake_reconnect_request(void)
{
	assert(lock_depth == 0);
	note("reconnect");
}

static const struct nina_wifi_ops fake_ops = {
	.ip_set = fake_ip_set,
	.link_up = fake_link_up,
	.link_down = fake_link_down,
	.connect_result = fake_connect_result,
	.disconnected = fake_disconnected,
};

static const struct nina_wifi_reconnect_ops fake_reconnect_ops = {
	.connection_allowed = fake_connection_allowed,
	.pause = fake_reconnect_pause,
	.request = fake_reconnect_request,
};

/* The same, for the runs without the application's reconnection. */
static const struct nina_wifi_ops bare_ops = {
	.ip_set = fake_ip_set,
	.link_up = fake_link_up,
	.link_down = fake_link_down,
	.connect_result = fake_connect_result,
	.disconnected = fake_disconnected,
};

/* Scan results ------------------------------------------------------------ */

static struct nina_wifi_scan_result results[12];
static unsigned int nresults;
static int scan_arg_seen;

static void scan_cb_fake(void *arg, int status, const struct nina_wifi_scan_result *result)
{
	assert(lock_depth == 0);
	assert(arg == &scan_arg_seen);
	if (result) {
		assert(status == 0 && nresults < 12);
		results[nresults++] = *result;
		note("net");
	} else {
		note("done(%d)", status);
	}
}

/* Helpers ----------------------------------------------------------------- */

/* A monitor's deadline that the tests do not reach. */
#define NO_DEADLINE (INT64_MAX / 4)

static uint8_t thread_records[NINA_WIFI_SCAN_MAX * (NINA_WIFI_SSID_MAX_LEN + 1)];

/* One pass of the Wi-Fi thread. */
static void thread_pass(void)
{
	wifi_once(thread_records, sizeof(thread_records));
}

static const uint8_t mac_reply[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
static const uint8_t ssid[] = "Net";
static const uint8_t psk[] = "password";
static const uint8_t ip_ok[12] = {10, 42, 0, 97, 255, 255, 255, 0, 10, 42, 0, 1};

static struct nina_wifi_connect_params psk_params(void)
{
	return (struct nina_wifi_connect_params){
		.ssid = ssid,
		.ssid_len = 3,
		.security = NINA_WIFI_SECURITY_PSK,
		.psk = psk,
		.psk_len = 8,
	};
}

static void ip_answers(unsigned int n, ...)
{
	va_list ap;

	va_start(ap, n);
	for (unsigned int i = 0; i < n; i++) {
		memcpy(ip_replies[i], va_arg(ap, const uint8_t *), 12);
	}
	va_end(ap);
	nips = n;
	ip_at = 0;
	ip_ret = 0;
}

/* A fresh state, as after the start, with OPS. */
static void reset(const struct nina_wifi_ops *with)
{
	uint8_t mac[6];

	connected = false;
	connecting = false;
	atomic_store(&scanning, false);
	disconnected_checks = 0;
	requests = 0;
	woken = false;
	monitor_deadline = NO_DEADLINE;
	caps = 0;
	hook = NULL;
	ip_set_ret = 0;
	answer(NINA_CMD_GET_MACADDR, 0, mac_reply, 6);
	assert(nina_wifi_init(with, mac) == 0);
	nina_wifi_set_reconnect_ops(with == &bare_ops ? NULL : &fake_reconnect_ops);
	steps_done();
	log_buf[0] = '\0';
}

/* An attempt that connects at once (10.42.0.97/24, gateway 10.42.0.1). */
static void connect_now(void)
{
	struct nina_wifi_connect_params p = psk_params();

	answer_byte(NINA_CMD_SET_PASSPHRASE, 1);
	assert(nina_wifi_connect(&p) == 0);
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	ip_answers(1, ip_ok);
	thread_pass();
	steps_done();
	/* The wake-up of the post, left: the next wait returns at once. */
	assert(woken);
	thread_pass();
	log_buf[0] = '\0';
	assert(connected && !connecting);
	monitor_deadline = NO_DEADLINE;
}

/* The tests --------------------------------------------------------------- */

static void test_init(void)
{
	uint8_t mac[6] = {0};
	static const uint8_t short_mac[5] = {1, 2, 3, 4, 5};

	probes = 0;
	answer(NINA_CMD_GET_MACADDR, 0, mac_reply, 6);
	assert(nina_wifi_init(&fake_ops, mac) == 0);
	steps_done();
	expect_log("22(ff)");
	assert(memcmp(mac, mac_reply, 6) == 0 && probes == 1);

	answer(NINA_CMD_GET_MACADDR, 0, short_mac, 5);
	assert(nina_wifi_init(&fake_ops, mac) == -EPROTO);
	answer(NINA_CMD_GET_MACADDR, -ETIMEDOUT, NULL, 0);
	assert(nina_wifi_init(&fake_ops, mac) == -ETIMEDOUT);
	steps_done();
	expect_log("22(ff) 22(ff)");
	assert(probes == 3);
}

static void test_connect_checks(void)
{
	static const uint8_t long_ssid[33] = {0};
	static const uint8_t long_psk[64] = {0};
	struct nina_wifi_connect_params p;

	reset(&fake_ops);
	assert(nina_wifi_connect(NULL) == -EINVAL);
	p = psk_params();
	p.ssid = NULL;
	assert(nina_wifi_connect(&p) == -EINVAL);
	p = psk_params();
	p.ssid_len = 0;
	assert(nina_wifi_connect(&p) == -EINVAL);
	p = psk_params();
	p.ssid = long_ssid;
	p.ssid_len = 33;
	p.security = NINA_WIFI_SECURITY_OTHER; /* the SSID is checked first */
	assert(nina_wifi_connect(&p) == -EINVAL);
	p = psk_params();
	p.security = NINA_WIFI_SECURITY_OTHER;
	p.psk_len = 3; /* the security before the passphrase */
	assert(nina_wifi_connect(&p) == -ENOTSUP);
	p = psk_params();
	p.psk_len = 7;
	assert(nina_wifi_connect(&p) == -EINVAL);
	p.psk = long_psk;
	p.psk_len = 64;
	assert(nina_wifi_connect(&p) == -EINVAL);
	p.psk_len = 63; /* the longest one */
	answer_byte(NINA_CMD_SET_PASSPHRASE, 0);
	assert(nina_wifi_connect(&p) == -EIO);
	steps_done();
	p = psk_params();
	p.psk = NULL;
	assert(nina_wifi_connect(&p) == -EINVAL);
	p = psk_params();
	p.psk_is_pbkdf2 = true;
	assert(nina_wifi_connect(&p) == -EINVAL);
	log_buf[0] = '\0';

	/* The application refuses: nothing asked of the module. */
	allowed = false;
	p = psk_params();
	assert(nina_wifi_connect(&p) == -ECANCELED);
	allowed = true;
	expect_log("allowed?");
	assert(!connecting);

	/* The module refuses the network, or does not answer. */
	answer_byte(NINA_CMD_SET_PASSPHRASE, 0);
	assert(nina_wifi_connect(&p) == -EIO);
	answer(NINA_CMD_SET_PASSPHRASE, -ETIMEDOUT, NULL, 0);
	assert(nina_wifi_connect(&p) == -ETIMEDOUT);
	steps_done();
	expect_log("allowed? 11(4e6574)(70617373776f7264) allowed? 11(4e6574)(70617373776f7264)");
	assert(!connecting && !(requests & WIFI_REQUEST_CONNECT));
}

static void test_connect(void)
{
	struct nina_wifi_connect_params p = psk_params();
	static const uint8_t no_ip[12] = {0};
	long gen;

	reset(&fake_ops);
	gen = atomic_load(&generation);
	answer_byte(NINA_CMD_SET_PASSPHRASE, 1);
	assert(nina_wifi_connect(&p) == 0);
	steps_done();
	expect_log("allowed? 11(4e6574)(70617373776f7264) post");
	assert(atomic_load(&generation) == gen + 1 && atomic_load(&request_generation) == gen + 1);
	/* Busy during the attempt, for another attempt and for a scan. */
	assert(nina_wifi_connect(&p) == -EBUSY);
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == -EBUSY);
	expect_log("allowed?");

	/* Not yet (idle, then a failure the module still reports), connected
	 * without an address, then with it. */
	answer_byte(NINA_CMD_GET_CONN_STATUS, 0);
	answer_byte(NINA_CMD_GET_CONN_STATUS, 4);
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	ip_answers(2, no_ip, ip_ok);
	thread_pass();
	steps_done();
	expect_log("20 sleep500 20 sleep500 20 17(00) 21 sleep500 20 17(00) 21 "
		   "ip_set(10.42.0.97/255.255.255.0,10.42.0.1) up result(0)");
	assert(connected && !connecting);
	assert(nina_wifi_connect(&p) == -EALREADY);
	expect_log("allowed?");

	/* The open network: 0x10 with the SSID alone. */
	reset(&fake_ops);
	p.security = NINA_WIFI_SECURITY_NONE;
	p.psk = NULL;
	p.psk_len = 0;
	answer_byte(NINA_CMD_SET_NET, 1);
	assert(nina_wifi_connect(&p) == 0);
	steps_done();
	expect_log("allowed? 10(4e6574) post");
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	ip_answers(1, ip_ok);
	thread_pass();
	steps_done();
	expect_log("20 17(00) 21 ip_set(10.42.0.97/255.255.255.0,10.42.0.1) up result(0)");
}

static void start_attempt(void)
{
	struct nina_wifi_connect_params p = psk_params();

	answer_byte(NINA_CMD_SET_PASSPHRASE, 1);
	assert(nina_wifi_connect(&p) == 0);
	steps_done();
	log_buf[0] = '\0';
}

/* A disconnection from another thread, at a command of the Wi-Fi thread. */
static void disconnect_hook(void)
{
	assert(nina_wifi_disconnect() == 0);
}

static void test_connect_failures(void)
{
	static const uint8_t short_reply[1] = {3};

	/* 60 checks without a connection. */
	reset(&fake_ops);
	start_attempt();
	for (int i = 0; i < 60; i++) {
		answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	}
	now_ms = 0;
	thread_pass();
	steps_done();
	assert(now_ms == 60 * 500);
	{
		char expected[60 * 13 + 32] = "";

		for (int i = 0; i < 60; i++) {
			strcat(expected, "20 sleep500 ");
		}
		snprintf(expected + strlen(expected), 32, "result(%d)", -ETIMEDOUT);
		expect_log(expected);
	}
	assert(!connected && !connecting);

	/* The power mode refused. */
	start_attempt();
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 0);
	thread_pass();
	steps_done();
	{
		char expected[64];

		snprintf(expected, sizeof(expected), "20 17(00) result(%d)", -EIO);
		expect_log(expected);
	}

	/* A short answer of 0x20; then an error of 0x21. */
	start_attempt();
	answer(NINA_CMD_GET_CONN_STATUS, 0, short_reply, 0);
	thread_pass();
	steps_done();
	{
		char expected[64];

		snprintf(expected, sizeof(expected), "20 result(%d)", -EPROTO);
		expect_log(expected);
	}
	start_attempt();
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	ip_ret = -EIO;
	thread_pass();
	steps_done();
	{
		char expected[64];

		snprintf(expected, sizeof(expected), "20 17(00) 21 result(%d)", -EIO);
		expect_log(expected);
	}

	/* The binding cannot set the address: not connected. */
	start_attempt();
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	ip_answers(1, ip_ok);
	ip_set_ret = -ENOMEM;
	thread_pass();
	steps_done();
	ip_set_ret = 0;
	{
		char expected[128];

		snprintf(expected, sizeof(expected),
			 "20 17(00) 21 ip_set(10.42.0.97/255.255.255.0,10.42.0.1) result(%d)",
			 -ENOMEM);
		expect_log(expected);
	}
	assert(!connected && !connecting);

	/* A disconnection while the thread asks 0x20: the attempt ends without
	 * a result. */
	start_attempt();
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	answer_byte(NINA_CMD_DISCONNECT, 1);
	hook = disconnect_hook;
	hook_on = NINA_CMD_GET_CONN_STATUS;
	thread_pass();
	steps_done();
	expect_log("20 pause 30(ff) disconnected(request) sleep500");
	assert(!connecting && !connected);

	/* A disconnection between 0x21 and the address: -ECANCELED, no
	 * result, no address. */
	start_attempt();
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	answer_byte(NINA_CMD_DISCONNECT, 1);
	ip_answers(1, ip_ok);
	hook = disconnect_hook;
	hook_on = NINA_CMD_GET_IPADDR;
	thread_pass();
	steps_done();
	expect_log("20 17(00) 21 pause 30(ff) disconnected(request)");
	assert(!connecting && !connected);

	/* The next attempt works. */
	connect_now();
}

static void test_disconnect(void)
{
	reset(&fake_ops);
	connect_now();
	answer_byte(NINA_CMD_DISCONNECT, 1);
	assert(nina_wifi_disconnect() == 0);
	steps_done();
	expect_log("pause 30(ff) down(10.42.0.97) disconnected(request)");
	assert(!connected);

	/* Not connected: no link down, but the event. */
	answer_byte(NINA_CMD_DISCONNECT, 1);
	assert(nina_wifi_disconnect() == 0);
	steps_done();
	expect_log("pause 30(ff) disconnected(request)");

	/* The module refuses: nothing changes but the pause. */
	connect_now();
	answer_byte(NINA_CMD_DISCONNECT, 0);
	assert(nina_wifi_disconnect() == -EIO);
	answer(NINA_CMD_DISCONNECT, -ETIMEDOUT, NULL, 0);
	assert(nina_wifi_disconnect() == -ETIMEDOUT);
	steps_done();
	expect_log("pause 30(ff) pause 30(ff)");
	assert(connected);
}

/* IDLE until the monitor's deadline, then the check. */
static void monitor_pass(void)
{
	thread_pass();
	thread_pass();
}

static void test_monitor(void)
{
	reset(&fake_ops);
	now_ms = 0;
	monitor_deadline = NINA_MONITOR_MS;
	monitor_pass();
	expect_log("wait2000");
	assert(now_ms == 2000 && monitor_deadline == 4000);

	connect_now();
	monitor_deadline = now_ms + NINA_MONITOR_MS;
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6); /* once down */
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	answer(NINA_CMD_GET_CONN_STATUS, -ETIMEDOUT, NULL, 0); /* not counted */
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6); /* twice: lost */
	for (int i = 0; i < 6; i++) {
		monitor_pass();
	}
	steps_done();
	expect_log("wait2000 20 wait2000 20 wait2000 20 wait2000 20 wait2000 20 wait2000 20 "
		   "down(10.42.0.97) disconnected(lost) reconnect");
	assert(!connected);
	monitor_pass();
	expect_log("wait2000");

	/* A connection after the loss starts the count again. */
	connect_now();
	monitor_deadline = now_ms;
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	thread_pass();
	steps_done();
	assert(connected && disconnected_checks == 1);
	log_buf[0] = '\0';
}

/* The answer of 0x27: N networks named "AP<i>". */
static size_t scan_records(uint8_t *out, unsigned int n)
{
	size_t len = 0;

	for (unsigned int i = 0; i < n; i++) {
		out[len++] = 3;
		out[len++] = 'A';
		out[len++] = 'P';
		out[len++] = (uint8_t)('0' + i);
	}
	return len;
}

static void network_answers(unsigned int i)
{
	uint8_t rssi[4] = {(uint8_t)(-40 - (int)i), 0xff, 0xff, 0xff};
	uint8_t bssid[6] = {6, 5, 4, 3, 2, (uint8_t)i};

	answer(NINA_CMD_GET_IDX_RSSI, 0, rssi, 4);
	answer_byte(NINA_CMD_GET_IDX_ENCT, i ? NINA_ENC_TYPE_NONE : NINA_ENC_TYPE_CCMP);
	answer(NINA_CMD_GET_IDX_BSSID, 0, bssid, 6);
	answer_byte(NINA_CMD_GET_IDX_CHANNEL, (uint8_t)(1 + i));
}

static void test_scan(void)
{
	uint8_t records[340];
	size_t len;

	assert(nina_wifi_scan(NULL, NULL) == -EINVAL);

	/* 3.0.2, connected: the link is given up and asked for again. */
	reset(&fake_ops);
	connect_now();
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	expect_log("down(10.42.0.97) disconnected(scan) post");
	assert(!connected);
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == -EBUSY);
	{
		struct nina_wifi_connect_params p = psk_params();

		assert(nina_wifi_connect(&p) == -EBUSY);
	}
	expect_log("allowed?");
	len = scan_records(records, 2);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, len);
	network_answers(0);
	network_answers(1);
	nresults = 0;
	thread_pass();
	steps_done();
	expect_log("27 32(00) 33(00) 3c(00) 3d(00) net 32(01) 33(01) 3c(01) 3d(01) net done(0) "
		   "reconnect");
	assert(nresults == 2);
	assert(results[0].ssid_len == 3 && memcmp(results[0].ssid, "AP0", 3) == 0);
	assert(results[0].rssi == -40 && results[0].enc_type == NINA_ENC_TYPE_CCMP);
	assert(results[0].bssid[0] == 0 && results[0].bssid[5] == 6 && results[0].channel == 1);
	assert(results[1].rssi == -41 && results[1].enc_type == NINA_ENC_TYPE_NONE);
	assert(results[1].bssid[0] == 1 && results[1].channel == 2);
	assert(!atomic_load(&scanning));

	/* The link kept: nothing given up, no reconnection. */
	connect_now();
	caps = NINA_CAP_SCAN_KEEPS_LINK;
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	expect_log("post");
	assert(connected);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, 0);
	thread_pass();
	steps_done();
	expect_log("27 done(0)");

	/* The asynchronous scan: 0x36, 0x6B every 100 ms, then 0x27. */
	caps = NINA_CAP_SCAN_KEEPS_LINK | NINA_CAP_ASYNC_SCAN;
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer_byte(NINA_CMD_START_SCAN_NETWORKS, 1);
	answer_byte(NINA_CMD_GET_SCAN_STATE, NINA_SCAN_RUNNING);
	answer_byte(NINA_CMD_GET_SCAN_STATE, NINA_SCAN_DONE);
	len = scan_records(records, 1);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, len);
	network_answers(0);
	thread_pass();
	steps_done();
	expect_log("post 36 sleep100 6b sleep100 6b 27 32(00) 33(00) 3c(00) 3d(00) net done(0)");

	/* Its ends: not started, stopped, failed, a short answer, too long. */
	static const struct {
		uint8_t start, state;
		int status;
	} ends[] = {
		{0, 0, -EIO},
		{1, NINA_SCAN_IDLE, -ECANCELED},
		{1, NINA_SCAN_FAILED, -EIO},
	};
	for (size_t i = 0; i < sizeof(ends) / sizeof(ends[0]); i++) {
		char expected[64];

		assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
		answer_byte(NINA_CMD_START_SCAN_NETWORKS, ends[i].start);
		if (ends[i].start) {
			answer_byte(NINA_CMD_GET_SCAN_STATE, ends[i].state);
		}
		thread_pass();
		steps_done();
		snprintf(expected, sizeof(expected),
			 ends[i].start ? "post 36 sleep100 6b done(%d)" : "post 36 done(%d)",
			 ends[i].status);
		expect_log(expected);
	}
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer(NINA_CMD_START_SCAN_NETWORKS, 0, records, 0);
	thread_pass();
	steps_done();
	{
		char expected[64];

		snprintf(expected, sizeof(expected), "post 36 done(%d)", -EPROTO);
		expect_log(expected);
	}
	/* Still running after 10 s: -ETIMEDOUT, without 0x27. */
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer_byte(NINA_CMD_START_SCAN_NETWORKS, 1);
	for (int i = 0; i < 100; i++) {
		answer_byte(NINA_CMD_GET_SCAN_STATE, NINA_SCAN_RUNNING);
	}
	now_ms = 0;
	thread_pass();
	steps_done();
	assert(now_ms == 10000);
	{
		char expected[64];

		snprintf(expected, sizeof(expected), "sleep100 6b done(%d)", -ETIMEDOUT);
		assert(log_ends(expected));
	}

	/* Bad answers: an SSID too long, past the end, more than 10 networks,
	 * a short RSSI. */
	caps = NINA_CAP_SCAN_KEEPS_LINK;
	records[0] = 33;
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, 34);
	thread_pass();
	records[0] = 5;
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, 4);
	thread_pass();
	len = scan_records(records, 11);
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, len);
	for (unsigned int i = 0; i < 10; i++) {
		network_answers(i);
	}
	nresults = 0;
	thread_pass();
	assert(nresults == 10);
	len = scan_records(records, 1);
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, len);
	answer(NINA_CMD_GET_IDX_RSSI, 0, records, 3);
	thread_pass();
	steps_done();
	{
		char done[32];

		snprintf(done, sizeof(done), "done(%d)", -EPROTO);
		/* Each of the four ends with -EPROTO. */
		unsigned int count = 0;

		for (const char *at = log_buf; (at = strstr(at, done)) != NULL; at++) {
			count++;
		}
		assert(count == 4);
	}
	log_buf[0] = '\0';
	assert(connected);
}

static void test_status(void)
{
	struct nina_wifi_status st;
	static const uint8_t bssid[6] = {6, 5, 4, 3, 2, 1};
	static const uint8_t rssi[4] = {0xc4, 0xff, 0xff, 0xff};

	reset(&fake_ops);
	memset(&st, 0xaa, sizeof(st));
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	assert(nina_wifi_status(&st) == 0);
	steps_done();
	expect_log("20");
	assert(!st.connected);

	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer(NINA_CMD_GET_CURR_SSID, 0, "Net", 3);
	answer(NINA_CMD_GET_CURR_BSSID, 0, bssid, 6);
	answer(NINA_CMD_GET_CURR_RSSI, 0, rssi, 4);
	assert(nina_wifi_status(&st) == 0);
	steps_done();
	expect_log("20 23(ff) 24(ff) 25(ff)");
	assert(st.connected && st.ssid_len == 3 && strcmp((char *)st.ssid, "Net") == 0);
	assert(st.bssid[0] == 1 && st.bssid[5] == 6 && st.rssi == -60);

	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer(NINA_CMD_GET_CURR_SSID, 0, "Net", 3);
	answer(NINA_CMD_GET_CURR_BSSID, 0, bssid, 5);
	assert(nina_wifi_status(&st) == -EPROTO);
	answer(NINA_CMD_GET_CONN_STATUS, -ETIMEDOUT, NULL, 0);
	assert(nina_wifi_status(&st) == -ETIMEDOUT);
	steps_done();
	log_buf[0] = '\0';
}

static void test_bare_ops(void)
{
	struct nina_wifi_connect_params p = psk_params();

	reset(&bare_ops);
	answer_byte(NINA_CMD_SET_PASSPHRASE, 1);
	assert(nina_wifi_connect(&p) == 0);
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	ip_answers(1, ip_ok);
	thread_pass();
	answer_byte(NINA_CMD_DISCONNECT, 1);
	assert(nina_wifi_disconnect() == 0);
	steps_done();
	expect_log("11(4e6574)(70617373776f7264) post 20 17(00) 21 "
		   "ip_set(10.42.0.97/255.255.255.0,10.42.0.1) up result(0) 30(ff) "
		   "down(10.42.0.97) disconnected(request)");
}

/* The thread's states together ------------------------------------------- */

static void test_monitor_in_scan(void)
{
	uint8_t records[340];
	char expected[1024];
	size_t len;

	/* Connected, the link kept through an asynchronous scan, the monitor's
	 * deadline 150 ms after the start: checks at 150 and at 2150 ms, each
	 * within a poll's sleep; down twice: lost during the scan, once.  The
	 * scan goes on and its result comes. */
	reset(&fake_ops);
	connect_now();
	caps = NINA_CAP_SCAN_KEEPS_LINK | NINA_CAP_ASYNC_SCAN;
	now_ms = 0;
	monitor_deadline = 150;
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer_byte(NINA_CMD_START_SCAN_NETWORKS, 1);
	answer_byte(NINA_CMD_GET_SCAN_STATE, NINA_SCAN_RUNNING); /* 100 ms */
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);               /* 150: once down */
	for (int i = 0; i < 20; i++) {                         /* 200 .. 2100 */
		answer_byte(NINA_CMD_GET_SCAN_STATE, NINA_SCAN_RUNNING);
	}
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);            /* 2150: twice */
	answer_byte(NINA_CMD_GET_SCAN_STATE, NINA_SCAN_DONE); /* 2200 */
	len = scan_records(records, 1);
	answer(NINA_CMD_SCAN_NETWORKS, 0, records, len);
	network_answers(0);
	nresults = 0;
	thread_pass();
	steps_done();
	strcpy(expected, "post 36 sleep100 6b sleep50 20 sleep50 6b");
	for (int i = 0; i < 19; i++) {
		strcat(expected, " sleep100 6b");
	}
	strcat(expected, " sleep50 20 down(10.42.0.97) disconnected(lost) reconnect sleep50 6b 27 "
			 "32(00) 33(00) 3c(00) 3d(00) net done(0)");
	expect_log(expected);
	assert(now_ms == 2200 && nresults == 1 && !connected && !atomic_load(&scanning));
	assert(monitor_deadline == 4150);

	/* IDLE: the post's wake-up left, then the wait until the deadline, then
	 * the check, which asks nothing now. */
	thread_pass();
	thread_pass();
	expect_log("wake wait1950");
	thread_pass();
	expect_log("");
	assert(now_ms == 4150 && monitor_deadline == 6150);
}

/* Another thread's requests while the results are fetched. */
static void requests_during_fetch(void)
{
	struct nina_wifi_connect_params p = psk_params();

	assert(nina_wifi_connect(&p) == -EBUSY);
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == -EBUSY);
}

static void test_busy_in_fetch(void)
{
	reset(&fake_ops);
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer(NINA_CMD_SCAN_NETWORKS, 0, NULL, 0);
	hook = requests_during_fetch;
	hook_on = NINA_CMD_SCAN_NETWORKS;
	thread_pass();
	steps_done();
	expect_log("post 27 allowed? done(0)");
	assert(!connecting && !atomic_load(&scanning) && requests == 0);
}

static void test_monitor_in_connect(void)
{
	/* Not connected during the attempt: the check at 700 ms (between two
	 * polls) asks nothing and resets the count left from an earlier link;
	 * the connection starts the period again. */
	reset(&fake_ops);
	start_attempt();
	now_ms = 0;
	monitor_deadline = 700;
	disconnected_checks = 1;
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	answer_byte(NINA_CMD_SET_POWER_MODE, 1);
	ip_answers(1, ip_ok);
	thread_pass();
	steps_done();
	expect_log("20 sleep500 20 sleep200 sleep300 20 17(00) 21 "
		   "ip_set(10.42.0.97/255.255.255.0,10.42.0.1) up result(0)");
	assert(connected && disconnected_checks == 0);
	assert(now_ms == 1000 && monitor_deadline == 3000);
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	thread_pass();
	monitor_pass();
	steps_done();
	expect_log("wake wait2000 20");
	assert(now_ms == 3000 && monitor_deadline == 5000);
}

static void test_request_at_deadline(void)
{
	/* The check first, then the scan. */
	reset(&fake_ops);
	connect_now();
	caps = NINA_CAP_SCAN_KEEPS_LINK;
	now_ms = 5000;
	monitor_deadline = 5000;
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
	thread_pass();
	steps_done();
	expect_log("post 20");
	assert(monitor_deadline == 7000 && atomic_load(&scanning));
	answer(NINA_CMD_SCAN_NETWORKS, 0, NULL, 0);
	thread_pass();
	steps_done();
	expect_log("27 done(0)");
	assert(connected && now_ms == 5000);
}

static void test_timeouts(void)
{
	char expected[64];

	/* A scan that never ends, connected with the link kept: given up at
	 * 10 s as before; a check every 2 s meanwhile (at 2, 4, 6, 8 and 10 s,
	 * before the poll of that time). */
	reset(&fake_ops);
	connect_now();
	caps = NINA_CAP_SCAN_KEEPS_LINK | NINA_CAP_ASYNC_SCAN;
	now_ms = 0;
	monitor_deadline = 2000;
	assert(nina_wifi_scan(scan_cb_fake, &scan_arg_seen) == 0);
	answer_byte(NINA_CMD_START_SCAN_NETWORKS, 1);
	for (int k = 1; k <= 100; k++) {
		if (k % 20 == 0) {
			answer_byte(NINA_CMD_GET_CONN_STATUS, NINA_WL_CONNECTED);
		}
		answer_byte(NINA_CMD_GET_SCAN_STATE, NINA_SCAN_RUNNING);
	}
	thread_pass();
	steps_done();
	assert(now_ms == 10000 && monitor_deadline == 12000 && connected);
	snprintf(expected, sizeof(expected), "sleep100 20 6b done(%d)", -ETIMEDOUT);
	assert(log_ends(expected));

	/* An attempt that never connects: 60 polls in 30 s as before, the
	 * checks (at 1.7 s, then every 2 s) between them asking nothing. */
	reset(&fake_ops);
	start_attempt();
	now_ms = 0;
	monitor_deadline = 1700;
	for (int i = 0; i < 60; i++) {
		answer_byte(NINA_CMD_GET_CONN_STATUS, 6);
	}
	thread_pass();
	steps_done();
	assert(now_ms == 30000 && monitor_deadline == 31700 && !connected && !connecting);
	snprintf(expected, sizeof(expected), "20 sleep200 sleep300 result(%d)", -ETIMEDOUT);
	assert(log_ends(expected));
}

int main(void)
{
	test_init();
	test_connect_checks();
	test_connect();
	test_connect_failures();
	test_disconnect();
	test_monitor();
	test_scan();
	test_status();
	test_bare_ops();
	test_monitor_in_scan();
	test_busy_in_fetch();
	test_monitor_in_connect();
	test_request_at_deadline();
	test_timeouts();
	puts("nina_wifi: tests passed");
	return 0;
}
