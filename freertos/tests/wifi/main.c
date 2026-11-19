/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's Wi-Fi on the FreeRTOS POSIX port:
 * the build's network parts (freertos/drivers/compat/zephyr_net_if.c, the
 * thread "net_mgmt" of zephyr_net_mgmt_os.h, freertos/drivers/nina/
 * nina_netif.c) under the shared ones as freertos/drivers/wifi/wifi.cmake
 * builds them (common/drivers/nina/nina_wifi.c and its Zephyr binding
 * common/drivers/nina/zephyr/nina_net.c, common/compat/
 * zephyr_net_mgmt.c, zephyr_wifi_mgmt.c, zephyr_wifi_utils.c,
 * wifi_credentials.c and its settings backend, common/imu/wifi_profile.c,
 * the "wifi" command of common/compat/zephyr_wifi_shell.c and the copy of
 * Zephyr's wifi_credentials_shell.c), on the store as the board builds it
 * (common/imu/kv_store.c, the copies of Zephyr's settings and NVS) over the
 * shared fake of the key-value area's flash (common/tests/kv_flash_fake.c),
 * the build's shell (freertos/drivers/shell) on a console that captures,
 * and a fake NINA-W102 (nina_request() and nina_ip_config() here).  The
 * credentials are made up for the test.
 *
 * - The start with a module that does not answer: the interface comes up
 *   without a MAC address, "wifi status" fails, nothing waits.
 * - The start in the build's order with no stored network: no connection
 *   is attempted.  The help has only the options the driver serves; the
 *   others (channel, -i, another -k, any of "wifi scan") are errors that
 *   ask nothing of the module.
 * - "wifi cred add": the entry is the setting "wifi_cred/0" in the
 *   Zephyr build's layout, "wifi cred list" shows it, "wifi cred delete" of
 *   another SSID fails.
 * - "wifi connect": the events (Zephyr's messages on the session from the
 *   "net_mgmt" thread), the address, netmask and gateway on the interface,
 *   dormant off; "wifi status"; "wifi disconnect" clears them and pauses
 *   the automatic reconnection; "wifi scan" prints the module's networks;
 *   "wifi connect -k 0" joins an open network with the SSID alone.
 * - "wifi cred auto_connect" connects with the stored network; a lost link
 *   (the module's status, seen twice by nina_monitor_tid) raises the
 *   disconnection, and wifi_reconnect_tid connects again 4 s later.
 * - Garbage collection of the store (many writes): the entry is still read.
 * - "ifconfig" (freertos/drivers/wifi/nano_ifconfig.c), with and without
 *   "wlan0": nothing before the interface exists, then the NuttX build's
 *   lines (the state UP while dormant, RUNNING while connected, the
 *   addresses of the connection, 0.0.0.0 without them, an empty line at
 *   the end); another name and more arguments are errors.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_credentials.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

#include "kv_flash_fake.h"
#include "kv_store.h"
#include "nano_console.h"
#include "nano_shell.h"
#include "nina_net.h"
#include "nina_os.h"
#include "nina_proto.h"
#include "pico/stdio.h"
#include "priorities.h"
#include "wifi_profile.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 16384
#define CAPTURE_SIZE 16384
#define LOG_SIZE     16384

/* Made-up network of the test. */
#define TEST_SSID "TestNet"
#define TEST_PSK  "test-pass-123"

_Thread_local int z_host_isr_depth;

void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

int wifi_shell_init(void);

/* The allocations of zshell.c and nano_console.c, with the scheduler
 * suspended (as in freertos/tests/shell).
 */
void *test_calloc(size_t n, size_t size);
void *test_malloc(size_t size);
void test_free(void *p);

void *test_calloc(size_t n, size_t size)
{
	void *p;

	vTaskSuspendAll();
	p = calloc(n, size);
	(void)xTaskResumeAll();
	return p;
}

void *test_malloc(size_t size)
{
	void *p;

	vTaskSuspendAll();
	p = malloc(size);
	(void)xTaskResumeAll();
	return p;
}

void test_free(void *p)
{
	vTaskSuspendAll();
	free(p);
	(void)xTaskResumeAll();
}

/* The Pico SDK's stdio, as pico_stdio (as in freertos/tests/shell) -------*/

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation)
{
	struct nano_console *con = nano_console_current();

	(void)cr_translation;
	if (len < 0) {
		len = (int)strlen(s);
	}
	nano_console_write(con, s, (size_t)len);
	if (newline) {
		nano_console_write(con, "\n", 1);
	}
	return len;
}

int __wrap_vprintf(const char *fmt, va_list args);
int __wrap_printf(const char *fmt, ...);
int __wrap_puts(const char *s);
int __wrap_putchar(int c);

int __wrap_vprintf(const char *fmt, va_list args)
{
	char buf[512];
	int n = vsnprintf(buf, sizeof(buf), fmt, args);

	if (n > 0) {
		stdio_put_string(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1, false, true);
	}
	return n;
}

int __wrap_printf(const char *fmt, ...)
{
	va_list args;
	int n;

	va_start(args, fmt);
	n = __wrap_vprintf(fmt, args);
	va_end(args);
	return n;
}

int __wrap_puts(const char *s)
{
	return stdio_put_string(s, -1, true, true);
}

int __wrap_putchar(int c)
{
	char ch = (char)c;

	stdio_put_string(&ch, 1, false, true);
	return (unsigned char)ch;
}

/* The log ------------------------------------------------------------------*/

static char log_buf[LOG_SIZE];

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	static const char *const names[] = {"", "err", "wrn", "inf", "dbg"};
	size_t len;

	(void)func;
	vTaskSuspendAll();
	len = strlen(log_buf);
	snprintf(log_buf + len, sizeof(log_buf) - len, "<%s> %s: ", names[level], module);
	len = strlen(log_buf);
	vsnprintf(log_buf + len, sizeof(log_buf) - len, fmt, ap);
	len = strlen(log_buf);
	snprintf(log_buf + len, sizeof(log_buf) - len, "\n");
	(void)xTaskResumeAll();
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	app_log_vprintf(level, module, func, fmt, ap);
	va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	(void)last_ms;
	(void)skipped;
	(void)rate_ms;
	*skipped_out = 0;
	return true;
}

/* The test's own lines on fd 1 and 2, past the wrapped stdio (which is the
 * session's: the runner is bound to the console).
 */
static void say(int fd, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void say(int fd, const char *fmt, ...)
{
	char line[256];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n > 0) {
		(void)!write(fd, line, (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1);
	}
}

/* The fake NINA-W102 ---------------------------------------------------------*/

#define WL_NO_SSID_AVAIL 1
#define WL_CONNECTED     3
#define WL_DISCONNECTED  6

static struct {
	bool dead;    /* every request fails as a silent module's */
	uint8_t link; /* the status the module reports (0x20) */
	int up_after; /* polls of 0x20 after a connection until WL_CONNECTED */
	unsigned int connects, disconnects, scans, mac_reads;
	/* The asynchronous scan (0x36, 0x6B): 0x36's answer, the states 0x6B
	 * gives in turn (the last one again after them), the commands seen. */
	uint8_t scan_start_reply;
	uint8_t scan_states[64];
	unsigned int nscan_states, scan_state_polls, scan_starts;
	uint8_t scan_state;      /* the last state 0x6B gave */
	bool lose_link_at_start; /* 0x36 also drops the link (WL_DISCONNECTED) */
	char ssid[33];           /* of the last connection */
	char psk[65];
	bool with_psk;
} fw = {.link = WL_DISCONNECTED};

static void reply(uint8_t *out, size_t cap, size_t *out_len, const void *data, size_t len)
{
	assert(len <= cap);
	memcpy(out, data, len);
	*out_len = len;
}

static void put_le32(uint8_t *b, int32_t v)
{
	uint32_t u = (uint32_t)v;

	for (int i = 0; i < 4; i++) {
		b[i] = (uint8_t)(u >> (8 * i));
	}
}

/* The kernel services of nina_os.h that the Wi-Fi operations use
 * (common/drivers/nina/nina_wifi.c), as fakes on the layer with the same
 * behaviour as the FreeRTOS binding's (common/drivers/nina/freertos/nina_os.c,
 * tested in tests/nina); the rest of the binding, the waits on the module's
 * lines, is not needed here. */
static K_MUTEX_DEFINE(connect_state_lock);
static K_SEM_DEFINE(scan_request, 0, 1);
static K_SEM_DEFINE(connect_request, 0, 1);

void nina_os_wifi_lock(void)
{
	(void)k_mutex_lock(&connect_state_lock, K_FOREVER);
}

void nina_os_wifi_unlock(void)
{
	(void)k_mutex_unlock(&connect_state_lock);
}

void nina_os_wifi_post(unsigned int which)
{
	k_sem_give(which == NINA_OS_WIFI_SCAN ? &scan_request : &connect_request);
}

void nina_os_wifi_wait(unsigned int which)
{
	(void)k_sem_take(which == NINA_OS_WIFI_SCAN ? &scan_request : &connect_request, K_FOREVER);
}

void nina_os_sleep_ms(uint32_t ms)
{
	(void)k_msleep((int32_t)ms);
}

int64_t nina_os_uptime_ms(void)
{
	return k_uptime_get();
}

/* The firmware's capability bits (common/drivers/nina/nina_spi.c, here a
 * fake): what the test sets; the probe counted. */
static uint32_t fake_caps;
static unsigned int probes;

uint32_t nina_probe_capabilities(void)
{
	probes++;
	return fake_caps;
}

uint32_t nina_capabilities(void)
{
	return fake_caps;
}

uint32_t nina_module_capabilities(void)
{
	return fake_caps;
}

/* Two networks: index 0 WPA2 on channel 6, 1 open on channel 11. */
static const char *const scan_ssids[] = {"TestAP-1", "TestAP-2"};

int nina_request(uint8_t cmd, const struct nina_param *p, size_t nparams, bool wide_params,
		 bool wide_response, uint8_t *out, size_t cap, size_t *out_len)
{
	uint8_t b[64];
	int ret = 0;

	(void)wide_params;
	(void)wide_response;
	vTaskSuspendAll();
	if (fw.dead) {
		ret = -ETIMEDOUT;
		goto out;
	}
	switch (cmd) {
	case 0x22: /* MAC address, reversed */
		fw.mac_reads++;
		for (int i = 0; i < 6; i++) {
			b[i] = (uint8_t)(0xa6 - i);
		}
		reply(out, cap, out_len, b, 6);
		break;
	case 0x10: /* open network */
	case 0x11: /* WPA2 passphrase */
		assert(nparams == (cmd == 0x10 ? 1u : 2u) && p[0].len <= 32);
		fw.connects++;
		memcpy(fw.ssid, p[0].data, p[0].len);
		fw.ssid[p[0].len] = '\0';
		fw.with_psk = cmd == 0x11;
		fw.psk[0] = '\0';
		if (fw.with_psk) {
			assert(p[1].len <= 64);
			memcpy(fw.psk, p[1].data, p[1].len);
			fw.psk[p[1].len] = '\0';
		}
		fw.link = WL_DISCONNECTED;
		fw.up_after = 1;
		reply(out, cap, out_len, "\x01", 1);
		break;
	case 0x20: /* status */
		if (fw.up_after > 0 && --fw.up_after == 0) {
			fw.link = WL_CONNECTED;
		}
		reply(out, cap, out_len, &fw.link, 1);
		break;
	case 0x17: /* power save off */
		reply(out, cap, out_len, "\x01", 1);
		break;
	case 0x30: /* disconnect */
		fw.disconnects++;
		fw.link = WL_DISCONNECTED;
		fw.up_after = 0;
		reply(out, cap, out_len, "\x01", 1);
		break;
	case 0x23: /* current SSID */
		reply(out, cap, out_len, fw.ssid, strlen(fw.ssid));
		break;
	case 0x24: /* current BSSID, reversed */
		reply(out, cap, out_len, "\x66\x55\x44\x33\x22\x11", 6);
		break;
	case 0x25: /* current RSSI */
		put_le32(b, -45);
		reply(out, cap, out_len, b, 4);
		break;
	case 0x36: /* start a scan (0005) */
		assert(fake_caps & NINA_CAP_ASYNC_SCAN);
		fw.scan_starts++;
		fw.scan_state_polls = 0;
		fw.scan_state = 1;
		if (fw.lose_link_at_start) {
			fw.link = WL_DISCONNECTED;
		}
		reply(out, cap, out_len, &fw.scan_start_reply, 1);
		break;
	case 0x6b: { /* the scan's state (0005) */
		unsigned int i = fw.scan_state_polls++;

		assert(fake_caps & NINA_CAP_ASYNC_SCAN);
		fw.scan_state = fw.scan_states[i < fw.nscan_states ? i : fw.nscan_states - 1];
		reply(out, cap, out_len, &fw.scan_state, 1);
		break;
	}
	case 0x27: { /* scan */
		size_t n = 0;

		/* After the asynchronous scan only when it is done. */
		assert(!(fake_caps & NINA_CAP_ASYNC_SCAN) || fw.scan_state == 2);
		fw.scans++;
		for (size_t i = 0; i < ARRAY_SIZE(scan_ssids); i++) {
			size_t len = strlen(scan_ssids[i]);

			b[n++] = (uint8_t)len;
			memcpy(b + n, scan_ssids[i], len);
			n += len;
		}
		reply(out, cap, out_len, b, n);
		break;
	}
	case 0x32: /* RSSI of a result */
		put_le32(b, p[0].data[0] == 0 ? -50 : -71);
		reply(out, cap, out_len, b, 4);
		break;
	case 0x33: /* encryption of a result: 4 WPA2, 7 none */
		reply(out, cap, out_len, p[0].data[0] == 0 ? "\x04" : "\x07", 1);
		break;
	case 0x3c: /* BSSID of a result, reversed */
		reply(out, cap, out_len,
		      p[0].data[0] == 0 ? "\x06\x05\x04\x03\x02\x01" : "\x16\x15\x14\x13\x12\x11",
		      6);
		break;
	case 0x3d: /* channel of a result */
		reply(out, cap, out_len, p[0].data[0] == 0 ? "\x06" : "\x0b", 1);
		break;
	default:
		say(2, "unexpected NINA command 0x%02x\n", cmd);
		_exit(1);
	}
out:
	(void)xTaskResumeAll();
	return ret;
}

/* 10.42.0.97/24 via 10.42.0.1 (common/drivers/nina/nina_spi.c's layout). */
int nina_ip_config(uint8_t data[12])
{
	static const uint8_t config[12] = {10, 42, 0, 97, 255, 255, 255, 0, 10, 42, 0, 1};

	if (fw.dead) {
		return -ETIMEDOUT;
	}
	memcpy(data, config, sizeof(config));
	return 0;
}

void nina_dns_init(void)
{
}

/* The session: a console that captures, bound to the runner ----------------*/

static struct {
	char text[CAPTURE_SIZE];
	size_t len;
} cap;

static struct nano_console con;

static void capture_out(struct nano_console *c, const char *buf, size_t len)
{
	(void)c;
	vTaskSuspendAll();
	assert(cap.len + len < sizeof(cap.text));
	memcpy(cap.text + cap.len, buf, len);
	cap.len += len;
	cap.text[cap.len] = '\0';
	(void)xTaskResumeAll();
}

static void cap_clear(void)
{
	vTaskSuspendAll();
	cap.len = 0;
	cap.text[0] = '\0';
	(void)xTaskResumeAll();
}

static int run(const char *line)
{
	char buf[256];

	strcpy(buf, line);
	return nano_shell_execute(buf);
}

static void fail(const char *what)
{
	say(2, "%s\n--- session ---\n", what);
	(void)!write(2, cap.text, cap.len);
	say(2, "\n--- log ---\n");
	(void)!write(2, log_buf, strlen(log_buf));
	_exit(1);
}

static void expect_text(const char *want)
{
	if (strstr(cap.text, want) == NULL) {
		say(2, "missing \"%s\"\n", want);
		fail("session");
	}
}

/* The session's whole output is WANT (after cap_clear()). */
static void expect_exact(const char *want)
{
	if (strcmp(cap.text, want) != 0) {
		say(2, "want exactly \"%s\"\n", want);
		fail("session");
	}
}

/* "ifconfig" and "ifconfig wlan0": the NuttX build's two lines and the empty
 * one, with STATE and the addresses (the MAC address stays 0: the module did
 * not answer when the interface read it).  The console ends lines with CR LF.
 */
static void expect_ifconfig(const char *state, const char *inet)
{
	char want[256];

	snprintf(want, sizeof(want),
		 "wlan0\tLink encap:Ethernet HWaddr 00:00:00:00:00:00 at %s mtu 1500\r\n"
		 "\tinet addr:%s\r\n\r\n",
		 state, inet);
	cap_clear();
	assert(run("ifconfig") == 0);
	expect_exact(want);
	cap_clear();
	assert(run("ifconfig wlan0") == 0);
	expect_exact(want);
}

/* Waits up to MS for WANT in the session's output. */
static void wait_text(const char *want, int ms)
{
	for (int t = 0; t < ms; t += 10) {
		if (strstr(cap.text, want) != NULL) {
			return;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	say(2, "timed out waiting for \"%s\"\n", want);
	fail("session");
}

NANO_SHELL_CMD_DECLARE(ifconfig);
NANO_SHELL_CMD_DECLARE(wifi);

const struct nano_shell_cmd nano_shell_cmds[] = {
	NANO_SHELL_CMD(ifconfig),
	NANO_SHELL_CMD(wifi),
};

const size_t nano_shell_cmd_count = sizeof(nano_shell_cmds) / sizeof(nano_shell_cmds[0]);

/* The events, in order -------------------------------------------------------*/

static struct net_mgmt_event_callback events_cb;
static char events[256];

static void event_handler(struct net_mgmt_event_callback *cb, uint64_t event, struct net_if *iface)
{
	const struct wifi_status *status = cb->info;
	size_t len = strlen(events);

	(void)iface;
	assert(status != NULL && cb->info_length == sizeof(*status));
	snprintf(events + len, sizeof(events) - len, "%s%d ",
		 event == NET_EVENT_WIFI_CONNECT_RESULT ? "C" : "D", status->status);
}

static void wait_events(const char *want, int ms)
{
	for (int t = 0; t < ms; t += 10) {
		if (strcmp(events, want) == 0) {
			return;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	say(2, "events \"%s\", want \"%s\"\n", events, want);
	fail("events");
}

static uint32_t addr(const struct net_in_addr *a)
{
	return a == NULL ? 0 : ntohl(a->s_addr);
}

#define report(...) say(1, __VA_ARGS__)

/* The tests ------------------------------------------------------------------*/

static void test_start(void)
{
	struct net_if *iface;
	int ret;

	/* No interface yet: "ifconfig" shows nothing, and there is no wlan0. */
	cap_clear();
	assert(run("ifconfig") == 0);
	expect_exact("");
	assert(run("ifconfig wlan0") == 1);
	expect_text("wlan0: no such interface");

	/* A module that does not answer: nothing waits for it. */
	fw.dead = true;
	assert(nina_net_initialize() == 0);
	iface = net_if_get_default();
	assert(iface != NULL && net_if_get_wifi_sta() == iface);
	assert(net_if_is_admin_up(iface) && net_if_is_dormant(iface));
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == NULL);

	/* The rest of the build's start (main.c), the store being empty. */
	assert(wifi_credentials_init() == 0);
	assert(wifi_shell_init() == 0);
	assert(K_THREAD_DEFINE_START(nina_scan_tid) == 0);
	assert(K_THREAD_DEFINE_START(nina_connect_tid) == 0);
	assert(K_THREAD_DEFINE_START(nina_monitor_tid) == 0);
	assert(K_THREAD_DEFINE_START(wifi_reconnect_tid) == 0);
	assert(kv_store_init() == 0);
	ret = wifi_profile_init();
	assert(ret == 0);
	assert(wifi_credentials_is_empty());

	net_mgmt_init_event_callback(&events_cb, event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT |
					     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&events_cb);

	/* The commands run at the shell's priority. */
	vTaskPrioritySet(NULL, NANO_PRIO_SHELL);

	assert(run("wifi status") == 1);
	expect_text("Status request failed");
	fw.dead = false;

	/* Up, not on a network, no address; other names and arguments. */
	expect_ifconfig("UP", "0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0");
	cap_clear();
	assert(run("ifconfig eth0") == 1);
	expect_text("eth0: no such interface");
	cap_clear();
	assert(run("ifconfig wlan0 10.0.0.2") == 1);
	expect_text("ifconfig: wrong parameter count");

	/* The subcommands, in Zephyr's order, and the help of one. */
	cap_clear();
	(void)run("wifi");
	expect_text("wifi - Wi-Fi commands");
	{
		const char *c = strstr(cap.text, "  connect");
		const char *r = strstr(cap.text, "  cred");
		const char *d = strstr(cap.text, "  disconnect");
		const char *sc = strstr(cap.text, "  scan");
		const char *st = strstr(cap.text, "  status");

		if (!(c && r && d && sc && st && c < r && r < d && d < sc && sc < st)) {
			fail("wifi help");
		}
	}
	expect_text("connect     : Connect to a Wi-Fi AP");
	if (strstr(cap.text, "--channel") != NULL || strstr(cap.text, "--iface") != NULL ||
	    strstr(cap.text, "--type") != NULL || strstr(cap.text, "eap") != NULL) {
		fail("wifi help: options the driver cannot serve");
	}
	cap_clear();
	assert(run("wifi connect --help") == 1);
	expect_text("Usage: connect <-s --ssid \"<SSID>\">: SSID to connect");
	expect_text("0:None, 1:WPA2-PSK");
	if (strstr(cap.text, "2:WPA2-PSK-256") != NULL) {
		fail("wifi connect --help: key management types");
	}
	cap_clear();
	assert(run("wifi scan -h") == 1);
	expect_text("scan: wrong parameter count");
	expect_text("Scan for Wi-Fi APs");

	/* The options the driver cannot serve are errors: nothing is asked of
	 * the module.
	 */
	cap_clear();
	assert(run("wifi connect -s " TEST_SSID " -c 6") == 1);
	expect_text("Invalid option c");
	expect_text("Usage: connect");
	cap_clear();
	assert(run("wifi connect -s " TEST_SSID " --channel=6") == 1);
	expect_text("Invalid option");
	cap_clear();
	assert(run("wifi connect -s " TEST_SSID " -i 1") == 1);
	expect_text("Invalid option i");
	cap_clear();
	assert(run("wifi connect -s " TEST_SSID " -k 3 -p " TEST_PSK) == 1);
	expect_text("Invalid key management type: 3");
	cap_clear();
	assert(run("wifi connect -s " TEST_SSID " -k psk -p " TEST_PSK) == 1);
	expect_text("Invalid key management type: psk");
	cap_clear();
	assert(run("wifi connect -k 1 -p " TEST_PSK) == 1);
	expect_text("SSID not provided");
	cap_clear();
	assert(run("wifi scan -t active") == 1);
	expect_text("scan: wrong parameter count");
	cap_clear();
	assert(run("wifi status -i 1") == 1);
	expect_text("status: wrong parameter count");
	cap_clear();
	assert(run("wifi disconnect -i 1") == 1);
	expect_text("disconnect: wrong parameter count");
	assert(fw.connects == 0 && fw.scans == 0 && fw.disconnects == 0);

	/* No stored network: no connection is tried. */
	vTaskDelay(pdMS_TO_TICKS(5000));
	assert(fw.connects == 0 && events[0] == '\0');
	assert(strstr(log_buf, "<err>") == NULL);
	report("ok  start\n");
}

static void test_cred(void)
{
	struct wifi_credentials_personal entry;
	ssize_t len;

	cap_clear();
	assert(run("wifi cred add -s " TEST_SSID " -k 1 -p " TEST_PSK) == 0);
	assert(!wifi_credentials_is_empty());

	/* The Zephyr build's name and layout in the settings. */
	memset(&entry, 0, sizeof(entry));
	len = settings_load_one("wifi_cred/0", &entry, sizeof(entry));
	assert(len == (ssize_t)sizeof(entry));
	assert(entry.header.type == WIFI_SECURITY_TYPE_PSK && entry.header.ssid_len == 7 &&
	       memcmp(entry.header.ssid, TEST_SSID, 7) == 0);
	assert(entry.password_len == strlen(TEST_PSK) &&
	       memcmp(entry.password, TEST_PSK, entry.password_len) == 0);
	assert(settings_load_one("wifi_cred/1", &entry, sizeof(entry)) <= 0);

	assert(run("wifi cred list") == 0);
	expect_text("network ssid: \"" TEST_SSID "\", ssid_len: 7, type: WPA2-PSK");

	cap_clear();
	assert(run("wifi cred delete Other") == 1);
	expect_text("No matching network with SSID: \"Other\" found");
	assert(!wifi_credentials_is_empty());
	report("ok  cred\n");
}

static void test_connect(void)
{
	struct net_if *iface = net_if_get_default();
	struct net_in_addr gw;
	char text[128];

	cap_clear();
	assert(run("wifi connect -s " TEST_SSID " -k 1 -p " TEST_PSK) == 0);
	expect_text("Connection requested");
	wait_text("Connected", 5000);
	wait_events("C0 ", 1000);
	assert(strcmp(fw.ssid, TEST_SSID) == 0 && fw.with_psk && strcmp(fw.psk, TEST_PSK) == 0);
	assert(addr(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)) == 0x0a2a0061);
	gw = net_if_ipv4_get_gw(iface);
	assert(addr(&gw) == 0x0a2a0001);
	assert(!net_if_is_dormant(iface));
	expect_ifconfig("RUNNING", "10.42.0.97 DRaddr:10.42.0.1 Mask:255.255.255.0");

	/* Connected already: -EALREADY, in the log (Zephyr: printk()). */
	cap_clear();
	assert(run("wifi connect -s " TEST_SSID " -k 1 -p " TEST_PSK) == 1);
	snprintf(text, sizeof(text),
		 "<err> net_wifi_shell: Connection request failed with error: %d", -EALREADY);
	assert(strstr(log_buf, text) != NULL);
	log_buf[0] = '\0';

	cap_clear();
	assert(run("wifi status") == 0);
	expect_text("Status: successful");
	expect_text("State: COMPLETED");
	expect_text("Interface Mode: STATION");
	expect_text("SSID: " TEST_SSID);
	expect_text("BSSID: 11:22:33:44:55:66");
	expect_text("Band: 2.4GHz");
	expect_text("Security:  UNKNOWN");
	expect_text("RSSI: -45");
	report("ok  connect, status\n");
}

static void test_disconnect_scan(void)
{
	struct net_if *iface = net_if_get_default();
	struct net_in_addr gw;

	cap_clear();
	events[0] = '\0';
	assert(run("wifi disconnect") == 0);
	expect_text("Disconnect requested");
	wait_text("Disconnection request done", 1000);
	wait_events("D0 ", 1000);
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == NULL);
	gw = net_if_ipv4_get_gw(iface);
	assert(addr(&gw) == 0 && net_if_is_dormant(iface));
	/* The netmask goes with the address (the NuttX build keeps its own). */
	expect_ifconfig("UP", "0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0");

	cap_clear();
	assert(run("wifi scan") == 0);
	expect_text("Scan requested");
	wait_text("Scan request done", 2000);
	expect_text("Num  | SSID                             (len) | Chan (Band)   | RSSI | "
		    "Security             | BSSID             | MFP     ");
	expect_text("1    | TestAP-1                         8     | 6    (2.4GHz) | -50  | "
		    "WPA2-PSK             | 01:02:03:04:05:06 | UNKNOWN ");
	expect_text("2    | TestAP-2                         8     | 11   (2.4GHz) | -71  | "
		    "OPEN                 | 11:12:13:14:15:16 | UNKNOWN ");

	/* A second scan: the callback is not added twice. */
	cap_clear();
	assert(run("wifi scan") == 0);
	wait_text("Scan request done", 2000);
	assert(fw.scans == 2);

	/* The disconnection asked for pauses the reconnection. */
	vTaskDelay(pdMS_TO_TICKS(5000));
	assert(strcmp(events, "D0 ") == 0 && fw.link == WL_DISCONNECTED);
	report("ok  disconnect, scan\n");
}

static void test_open(void)
{
	struct net_if *iface = net_if_get_default();

	/* -k 0: an open network, the SSID alone (0x10). */
	cap_clear();
	events[0] = '\0';
	assert(run("wifi connect -s TestAP-2 -k 0") == 0);
	expect_text("Connection requested");
	if (strstr(cap.text, "Passphrase provided") != NULL) {
		fail("open network: warning");
	}
	wait_text("Connected", 5000);
	wait_events("C0 ", 1000);
	assert(strcmp(fw.ssid, "TestAP-2") == 0 && !fw.with_psk);
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL);

	cap_clear();
	assert(run("wifi disconnect") == 0);
	wait_text("Disconnection request done", 1000);
	wait_events("C0 D0 ", 1000);
	report("ok  open network\n");
}

static void test_auto_connect(void)
{
	struct net_if *iface = net_if_get_default();
	unsigned int connects;

	cap_clear();
	events[0] = '\0';
	fw.ssid[0] = '\0';
	assert(run("wifi cred auto_connect") == 0);
	wait_text("Connected", 5000);
	wait_events("C0 ", 1000);
	assert(strcmp(fw.ssid, TEST_SSID) == 0 && strcmp(fw.psk, TEST_PSK) == 0);
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL);

	/* The link is lost: the monitor sees it twice (2 s apart), the
	 * reconnection thread connects 4 s later.
	 */
	connects = fw.connects;
	cap_clear();
	events[0] = '\0';
	vTaskSuspendAll();
	fw.link = WL_NO_SSID_AVAIL;
	(void)xTaskResumeAll();
	wait_events("D0 ", 7000);
	expect_text("Disconnected");
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == NULL);
	wait_events("D0 C0 ", 10000);
	assert(fw.connects == connects + 1);
	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL);
	report("ok  auto_connect, reconnect\n");
}

/* The scan's states that 0x6B gives: RUNNING times 1, then LAST. */
static void scan_script(uint8_t start, unsigned int running, uint8_t last)
{
	vTaskSuspendAll();
	fw.scan_start_reply = start;
	assert(running + 1 <= sizeof(fw.scan_states));
	for (unsigned int i = 0; i < running; i++) {
		fw.scan_states[i] = 1;
	}
	fw.scan_states[running] = last;
	fw.nscan_states = running + 1;
	fw.scan_starts = fw.scan_state_polls = fw.scans = 0;
	fw.scan_state = 0;
	(void)xTaskResumeAll();
}

/* "Scan request failed (ERR)" of the shell. */
static void wait_scan_failed(int err, int ms)
{
	char text[64];

	snprintf(text, sizeof(text), "Scan request failed (%d)", err);
	wait_text(text, ms);
}

static void expect_connected(void)
{
	struct net_if *iface = net_if_get_default();

	assert(net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL);
	assert(!net_if_is_dormant(iface));
}

static void test_scan_capabilities(void)
{
	unsigned int connects;

	/* Connected, with a firmware that keeps the link through a scan and
	 * scans asynchronously: 0x36, 0x6B every 100 ms until done, then 0x27;
	 * the link and the address stay, no disconnection, no reconnection. */
	assert(probes == 1);
	expect_connected();
	fake_caps = NINA_CAP_SCAN_KEEPS_LINK | NINA_CAP_ASYNC_SCAN;
	scan_script(1, 3, 2);
	connects = fw.connects;
	cap_clear();
	events[0] = '\0';
	assert(run("wifi scan") == 0);
	expect_text("Scan requested");
	wait_text("Scan request done", 3000);
	expect_text("1    | TestAP-1 ");
	assert(fw.scan_starts == 1 && fw.scan_state_polls == 4 && fw.scans == 1);
	vTaskDelay(pdMS_TO_TICKS(500));
	assert(events[0] == '\0' && fw.connects == connects);
	expect_connected();
	expect_ifconfig("RUNNING", "10.42.0.97 DRaddr:10.42.0.1 Mask:255.255.255.0");

	/* It could not start (0x36 answers 0): failed, no 0x6B, no 0x27. */
	scan_script(0, 0, 2);
	cap_clear();
	assert(run("wifi scan") == 0);
	wait_scan_failed(-EIO, 1000);
	assert(fw.scan_starts == 1 && fw.scan_state_polls == 0 && fw.scans == 0);
	/* It failed while running (3), or was stopped (0): no 0x27. */
	scan_script(1, 2, 3);
	cap_clear();
	assert(run("wifi scan") == 0);
	wait_scan_failed(-EIO, 1000);
	assert(fw.scan_state_polls == 3 && fw.scans == 0);
	scan_script(1, 1, 0);
	cap_clear();
	assert(run("wifi scan") == 0);
	wait_scan_failed(-ECANCELED, 1000);
	assert(fw.scan_state_polls == 2 && fw.scans == 0);
	/* It never ends: given up after NINA_SCAN_WAIT_MS (1 s in this test),
	 * without 0x27, which would wait for it. */
	scan_script(1, 63, 1);
	cap_clear();
	assert(run("wifi scan") == 0);
	vTaskDelay(pdMS_TO_TICKS(500));
	assert(strstr(cap.text, "Scan request failed") == NULL);
	wait_scan_failed(-ETIMEDOUT, 1500);
	assert(fw.scan_state_polls >= 9 && fw.scan_state_polls <= 11 && fw.scans == 0);
	assert(events[0] == '\0');
	expect_connected();

	/* The link is lost when a scan starts: the scan ends as usual, the
	 * monitor sees the loss as any other (twice, 2 s apart), and the
	 * reconnection follows. */
	scan_script(1, 8, 2);
	vTaskSuspendAll();
	fw.lose_link_at_start = true;
	(void)xTaskResumeAll();
	connects = fw.connects;
	cap_clear();
	events[0] = '\0';
	assert(run("wifi scan") == 0);
	wait_text("Scan request done", 2000);
	wait_events("D0 ", 7000);
	vTaskSuspendAll();
	fw.lose_link_at_start = false;
	(void)xTaskResumeAll();
	wait_events("D0 C0 ", 45000);
	assert(fw.connects > connects);
	expect_connected();

	/* The 3.0.2 firmware (no bits): a scan while connected gives the link
	 * up (a disconnection, the address gone) and the reconnection takes it
	 * up again; 0x27 alone. */
	fake_caps = 0;
	scan_script(1, 0, 2);
	connects = fw.connects;
	cap_clear();
	events[0] = '\0';
	assert(run("wifi scan") == 0);
	wait_events("D0 ", 1000);
	wait_text("Scan request done", 2000);
	assert(fw.scans == 1 && fw.scan_starts == 0);
	wait_events("D0 C0 ", 45000);
	assert(fw.connects == connects + 1);
	expect_connected();
	report("ok  scans: link kept, asynchronous, failures, link lost, 3.0.2\n");
}

static void test_gc(void)
{
	struct wifi_credentials_personal entry;
	char value[100];

	/* 90 writes of 100 bytes go through the 8 KiB area several times. */
	memset(value, 'x', sizeof(value));
	for (int i = 0; i < 90; i++) {
		value[0] = (char)('a' + i % 26);
		assert(kv_store_set("test.fill", value, sizeof(value)) == 0);
	}
	assert(kv_fake->erases >= 1);
	memset(&entry, 0, sizeof(entry));
	assert(wifi_credentials_get_by_ssid_personal_struct(TEST_SSID, 7, &entry) == 0);
	assert(entry.password_len == strlen(TEST_PSK) &&
	       memcmp(entry.password, TEST_PSK, entry.password_len) == 0);
	assert(settings_load_one("wifi_cred/0", &entry, sizeof(entry)) == (ssize_t)sizeof(entry));
	report("ok  credentials after garbage collection (%lu erases)\n", kv_fake->erases);
}

static void runner(void *arg)
{
	(void)arg;
	nano_console_bind(NULL, &con);
	test_start();
	test_cred();
	test_connect();
	test_disconnect_scan();
	test_open();
	test_auto_connect();
	test_scan_capabilities();
	test_gc();
	if (strstr(log_buf, "<err>") != NULL) {
		fail("errors in the log");
	}
	_exit(0);
}

int main(void)
{
	kv_fake_init();
	nano_console_init(&con, "usb", capture_out, NULL);
	nano_console_set_default(&con);
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_MAIN,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
