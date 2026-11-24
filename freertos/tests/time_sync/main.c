/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's time synchronization on the FreeRTOS
 * POSIX port: the shared common/imu/time_sync.c and time_offset.c with the
 * prelude, as common/pico_sdk/drivers/imu/imu.cmake builds them, on the build's OS
 * parts (the realtime clock of the layer's <zephyr/sys/clock.h>,
 * common/pico_sdk/drivers/compat/zephyr_sys_clock.c; the interface,
 * zephyr_net_if.c; the net_mgmt events with the thread "net_mgmt",
 * common/compat/zephyr_net_mgmt.c; the thread time_sync_tid of
 * K_THREAD_DEFINE(); the layer's semaphore, mutex, sleeps and clock), the
 * build's shell on a console that captures, fake sockets (the zsock_*()
 * calls, here) with a fake NTP server, and the TIMER's registers read from
 * the test port's clock (tests/compat/rp2040_timer_us.h).  The
 * checks of the NTP replies and the estimate are the shared code's, tested
 * by nuttx/tests/time_sync.c and common/tests/time_offset.c; this test is
 * about the FreeRTOS parts:
 *
 * - The realtime clock: SYS_CLOCK_REALTIME is the TIMER plus a base (0 at
 *   the start), SYS_CLOCK_MONOTONIC the TIMER, other clocks -EINVAL; setting
 *   it moves the base only, SYS_CLOCK_MONOTONIC and a bad tv_nsec are
 *   refused, a time before 1970 reads back normalized.
 * - time_sync_mono_us(): TIMERAWH, TIMERAWL, TIMERAWH (offsets 0x24 and 0x28
 *   of the shared rp2040_timer_us()), the same clock as the layer's.
 * - The thread: "time_sync_tid" at Zephyr's 9 (APP_PRIO_TIME_SYNC) on its
 *   2048-byte stack, started by K_THREAD_DEFINE_START(); nothing is sent
 *   before Wi-Fi connects.
 * - A connection event (NET_EVENT_WIFI_CONNECT_RESULT through "net_mgmt"):
 *   one UDP socket, a request to the default gateway's port 123 at once,
 *   the reply polled for with MSG_DONTWAIT and k_sleep(K_MSEC(1)) between
 *   the polls; ready, the server's time within a few ms, the realtime clock
 *   stepped to it (realtime_clock_steps 1); the next request 2 s later
 *   (CONFIG_NINA_NTP_INTERVAL_MS of the prelude).
 * - A reply that does not come: -ETIMEDOUT after 250 ms (the layer's
 *   k_uptime_get()), counted in "timeouts", the estimate kept.
 * - A disconnection event: no more requests, the socket closed; a new
 *   connection starts a new estimate (replies from 1 again) at once.
 * - The realtime clock is only pushed forward: behind by 1 s, it is
 *   stepped; ahead by 1 s, it is left alone.
 * - "ntp status" and "ntp" through the build's shell: the Zephyr build's
 *   lines and values (server 10.42.0.1:123 (default gateway), interval_ms
 *   2000, the counters).
 *
 * The test port's tick is 1 ms (tests/compat/FreeRTOSConfig.h), as the
 * board's, so the polls are 2 ticks apart.  It waits for the real intervals: about
 * 12 s.  Built and run by freertos/script/host_tests.sh.
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
#include <time.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/clock.h>

#include "shell_console.h"
#include "shell_engine.h"
#include "priorities.h"
#include "time_sync.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 16384
#define CAPTURE_SIZE 8192
#define LOG_SIZE     4096

/* The server's time is the board's plus this (2026-11-17, in us). */
#define SERVER_OFFSET_US 1794900000000000LL
#define NTP_UNIX_SECONDS 2208988800LL
/* The fake network's round trip and the server's hold time. */
#define ROUND_TRIP_US    5000
#define SERVER_HOLD_US   50

_Thread_local int z_host_isr_depth;

void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

extern k_tid_t time_sync_tid;

/* The allocations of zshell.c and shell_console.c, with the scheduler
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

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation);

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation)
{
	struct shell_console *con = shell_console_current();

	(void)cr_translation;
	if (len < 0) {
		len = (int)strlen(s);
	}
	shell_console_write(con, s, (size_t)len);
	if (newline) {
		shell_console_write(con, "\n", 1);
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

/* The test's own lines on fd 1 and 2, past the wrapped stdio. */
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

/* The TIMER's reads (tests/compat/rp2040_timer_us.h) -------------------------*/

/* Only the reads of the recording thread: the tick hook reads the clock too. */
static char timer_reads[16];
static unsigned int timer_read_count;
static _Thread_local bool timer_recording;

void test_timer_read(char reg)
{
	if (timer_recording && timer_read_count < sizeof(timer_reads) - 1) {
		timer_reads[timer_read_count++] = reg;
		timer_reads[timer_read_count] = '\0';
	}
}

/* The fake sockets and NTP server --------------------------------------------*/

#define FAKE_FD 3

enum server_mode {
	SERVER_REPLIES,
	SERVER_SILENT,
};

static struct {
	enum server_mode mode;
	int sockets_open, sockets_made, closes;
	int sends, polls;
	struct sockaddr_in to;
	uint8_t request[48];
	int64_t sent_us;      /* rp2040_timer_us() at the last sendto() */
	bool pending;         /* a reply to the last request is on its way */
	int64_t last_poll_us; /* for the intervals between polls */
	int64_t min_poll_gap_us;
	int64_t first_poll_us; /* of the last request's wait */
	int64_t last_poll_of_wait_us;
} net = {.min_poll_gap_us = INT64_MAX};

static void lock(void)
{
	vTaskSuspendAll();
}

static void unlock(void)
{
	(void)xTaskResumeAll();
}

static void put_be32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24);
	p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);
	p[3] = (uint8_t)v;
}

static void put_ntp(uint8_t *p, int64_t unix_us)
{
	int64_t s = unix_us / 1000000, us = unix_us % 1000000;

	put_be32(p, (uint32_t)(s + NTP_UNIX_SECONDS));
	put_be32(p + 4, (uint32_t)((((uint64_t)us << 32) + 999999) / 1000000));
}

int zsock_socket(int family, int type, int proto)
{
	assert(family == AF_INET && type == SOCK_DGRAM && proto == IPPROTO_UDP);
	lock();
	assert(net.sockets_open == 0);
	net.sockets_open++;
	net.sockets_made++;
	unlock();
	return FAKE_FD;
}

int zsock_close(int sock)
{
	assert(sock == FAKE_FD);
	lock();
	assert(net.sockets_open == 1);
	net.sockets_open--;
	net.closes++;
	unlock();
	return 0;
}

ssize_t zsock_sendto(int sock, const void *buf, size_t len, int flags, const struct sockaddr *to,
		     socklen_t tolen)
{
	assert(sock == FAKE_FD && len == 48 && flags == 0 && to != NULL &&
	       tolen == sizeof(struct sockaddr_in));
	lock();
	memcpy(net.request, buf, len);
	memcpy(&net.to, to, sizeof(net.to));
	net.sent_us = (int64_t)rp2040_timer_us();
	net.pending = net.mode == SERVER_REPLIES;
	net.sends++;
	net.first_poll_us = 0;
	unlock();
	return (ssize_t)len;
}

ssize_t zsock_recvfrom(int sock, void *buf, size_t max_len, int flags, struct sockaddr *src_addr,
		       socklen_t *addrlen)
{
	int64_t now = (int64_t)rp2040_timer_us();
	uint8_t *reply = buf;
	int64_t t2;

	assert(sock == FAKE_FD && max_len >= 48 && flags == ZSOCK_MSG_DONTWAIT &&
	       src_addr == NULL && addrlen == NULL);
	lock();
	net.polls++;
	if (net.first_poll_us == 0) {
		net.first_poll_us = now;
	} else if (now - net.last_poll_us < net.min_poll_gap_us) {
		net.min_poll_gap_us = now - net.last_poll_us;
	}
	net.last_poll_us = now;
	net.last_poll_of_wait_us = now;
	if (!net.pending || now < net.sent_us + ROUND_TRIP_US) {
		unlock();
		errno = EAGAIN;
		return -1;
	}
	net.pending = false;
	/* Server mode 4, version 4, stratum 2, the request's transmit time as
	 * the originate time, receive and transmit times on the server's clock
	 * (the board's plus SERVER_OFFSET_US) halfway along the round trip.
	 */
	memset(reply, 0, 48);
	reply[0] = 0x24;
	reply[1] = 2;
	memcpy(reply + 24, net.request + 40, 8);
	t2 = net.sent_us + ROUND_TRIP_US / 2 + SERVER_OFFSET_US;
	put_ntp(reply + 32, t2);
	put_ntp(reply + 40, t2 + SERVER_HOLD_US);
	unlock();
	return 48;
}

/* Only with a configured server name (CONFIG_NINA_NTP_SERVER is empty). */
int zsock_getaddrinfo(const char *host, const char *service, const struct zsock_addrinfo *hints,
		      struct zsock_addrinfo **res)
{
	(void)host;
	(void)service;
	(void)hints;
	(void)res;
	abort();
}

void zsock_freeaddrinfo(struct zsock_addrinfo *ai)
{
	(void)ai;
	abort();
}

/* The session: a console that captures, bound to the runner ----------------*/

static struct {
	char text[CAPTURE_SIZE];
	size_t len;
} cap;

static struct shell_console con;

static void capture_out(struct shell_console *c, const char *buf, size_t len)
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
	char buf[128];

	strcpy(buf, line);
	return shell_engine_execute(buf);
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

/* One line of "ntp status": "  <name>:" padded to 27, a space, the value
 * (the console ends lines with CR LF).
 */
static void expect_item(const char *name, const char *value)
{
	char item[32], line[128];

	snprintf(item, sizeof(item), "%s:", name);
	snprintf(line, sizeof(line), "\r\n  %-27s %s\r\n", item, value);
	expect_text(line);
}

SHELL_ENGINE_CMD_DECLARE(ntp);

const struct shell_engine_cmd shell_engine_cmds[] = {
	SHELL_ENGINE_CMD(ntp),
};

const size_t shell_engine_cmd_count = sizeof(shell_engine_cmds) / sizeof(shell_engine_cmds[0]);

/* Helpers ----------------------------------------------------------------------*/

static struct net_if iface;

static void notify(uint64_t event)
{
	struct wifi_status status = {.status = 0};

	net_mgmt_event_notify_with_info(event, &iface, &status, sizeof(status));
}

static int get(int (*value)(void))
{
	int v;

	lock();
	v = value();
	unlock();
	return v;
}

static int sends(void)
{
	return net.sends;
}

static int sockets_open(void)
{
	return net.sockets_open;
}

/* Waits up to MS for COND. */
#define wait_for(cond, ms)                                                                         \
	do {                                                                                       \
		int waited_ = 0;                                                                   \
		while (!(cond)) {                                                                  \
			if (waited_ >= (ms)) {                                                     \
				say(2, "%s:%d: timed out: %s\n", __FILE__, __LINE__, #cond);       \
				fail("wait");                                                      \
			}                                                                          \
			vTaskDelay(pdMS_TO_TICKS(5));                                              \
			waited_ += 5;                                                              \
		}                                                                                  \
	} while (0)

static int64_t realtime_us(void)
{
	struct timespec ts;

	assert(sys_clock_gettime(SYS_CLOCK_REALTIME, &ts) == 0);
	assert(ts.tv_nsec >= 0 && ts.tv_nsec < 1000000000 && ts.tv_nsec % 1000 == 0);
	return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static void set_realtime_us(int64_t us)
{
	struct timespec ts = {.tv_sec = (time_t)(us / 1000000),
			      .tv_nsec = (long)(us % 1000000) * 1000};

	assert(sys_clock_settime(SYS_CLOCK_REALTIME, &ts) == 0);
}

static int64_t abs64(int64_t v)
{
	return v < 0 ? -v : v;
}

#define report(...) say(1, __VA_ARGS__)

/* The tests ------------------------------------------------------------------*/

static void test_realtime_clock(void)
{
	struct timespec ts, mono;
	int64_t before, after, rt;

	/* At the start: the realtime clock is the TIMER (base 0). */
	before = (int64_t)rp2040_timer_us();
	rt = realtime_us();
	assert(sys_clock_gettime(SYS_CLOCK_MONOTONIC, &mono) == 0);
	after = (int64_t)rp2040_timer_us();
	assert(rt >= before && rt <= after);
	assert((int64_t)mono.tv_sec * 1000000 + mono.tv_nsec / 1000 >= rt &&
	       (int64_t)mono.tv_sec * 1000000 + mono.tv_nsec / 1000 <= after);
	assert(sys_clock_gettime(0, &ts) == -EINVAL && sys_clock_gettime(2, &ts) == -EINVAL);

	/* Setting it moves the base: it goes on from the new time. */
	ts.tv_sec = 1794900000;
	ts.tv_nsec = 123456789;
	assert(sys_clock_settime(SYS_CLOCK_REALTIME, &ts) == 0);
	rt = realtime_us();
	assert(rt >= 1794900000123456LL && rt < 1794900000123456LL + 100000);
	vTaskDelay(pdMS_TO_TICKS(20));
	assert(realtime_us() >= rt + 19000);
	assert(sys_clock_gettime(SYS_CLOCK_MONOTONIC, &mono) == 0 && mono.tv_sec < 1000000000);

	/* Refused: the monotonic clock, another clock, a bad tv_nsec. */
	assert(sys_clock_settime(SYS_CLOCK_MONOTONIC, &ts) == -EINVAL);
	assert(sys_clock_settime(0, &ts) == -EINVAL);
	ts.tv_nsec = 1000000000;
	assert(sys_clock_settime(SYS_CLOCK_REALTIME, &ts) == -EINVAL);
	ts.tv_nsec = -1;
	assert(sys_clock_settime(SYS_CLOCK_REALTIME, &ts) == -EINVAL);
	assert(realtime_us() >= rt + 19000 && realtime_us() < rt + 1000000);

	/* Before 1970: tv_nsec stays in 0..999999999. */
	ts.tv_sec = -2;
	ts.tv_nsec = 500000000;
	assert(sys_clock_settime(SYS_CLOCK_REALTIME, &ts) == 0);
	assert(sys_clock_gettime(SYS_CLOCK_REALTIME, &ts) == 0);
	assert(ts.tv_sec == -2 && ts.tv_nsec >= 500000000 && ts.tv_nsec < 1000000000);

	/* Back to the TIMER for the rest. */
	set_realtime_us((int64_t)rp2040_timer_us());
	report("ok  realtime clock\n");
}

static void test_mono(void)
{
	int64_t before, m, after;

	timer_read_count = 0;
	before = (int64_t)rp2040_timer_us();
	timer_recording = true;
	m = time_sync_mono_us();
	timer_recording = false;
	after = (int64_t)rp2040_timer_us();
	assert(m >= before && m <= after);
	if (strcmp(timer_reads, "HLH") != 0) {
		say(2, "TIMER reads \"%s\"\n", timer_reads);
		fail("time_sync_mono_us");
	}
	report("ok  time_sync_mono_us (TIMERAWH, TIMERAWL, TIMERAWH)\n");
}

static void test_start(void)
{
	struct net_in_addr address = {.s_addr = htonl(0x0a2a0061)};
	struct net_in_addr netmask = {.s_addr = htonl(0xffffff00)};
	struct net_in_addr gateway = {.s_addr = htonl(0x0a2a0001)};

	/* The interface as the NINA driver leaves it on a connection
	 * (10.42.0.97/24 via 10.42.0.1), and the build's start (main.c).
	 */
	assert(net_mgmt_event_init() == 0);
	z_net_if_register(&iface);
	assert(net_if_ipv4_addr_add(&iface, &address, NET_ADDR_DHCP, 0) != NULL);
	assert(net_if_ipv4_set_netmask_by_addr(&iface, &address, &netmask));
	net_if_ipv4_set_gw(&iface, &gateway);
	net_if_dormant_off(&iface);

	assert(K_THREAD_DEFINE_START(time_sync_tid) == 0);
	assert(time_sync_tid != NULL);
	assert(strcmp(pcTaskGetName(time_sync_tid), "time_sync_tid") == 0);
	assert(uxTaskPriorityGet(time_sync_tid) == APP_PRIO_TIME_SYNC &&
	       APP_PRIO_TIME_SYNC == APP_PRIO_FROM_ZEPHYR(9));
	time_sync_init();

	/* The commands run at the shell's priority. */
	vTaskPrioritySet(NULL, APP_PRIO_SHELL);

	/* Not connected: nothing is sent. */
	vTaskDelay(pdMS_TO_TICKS(300));
	assert(get(sends) == 0 && net.sockets_made == 0);
	assert(!time_sync_is_ready());

	cap_clear();
	assert(run("ntp status") == 0);
	expect_text("state: not synchronized\r\n");
	expect_item("server", "none:123 (default gateway)");
	expect_item("interval_ms", "2000");
	expect_item("queries", "0");
	expect_item("replies", "0");
	expect_item("last_error", "0");
	if (strstr(cap.text, "round_trip:") != NULL) {
		fail("ntp status: round_trip before a reply");
	}

	/* "ntp" alone: the help of the set (Zephyr's shell). */
	cap_clear();
	(void)run("ntp");
	expect_text("ntp - Network time synchronization");
	expect_text("status  : Show time synchronization status");
	report("ok  start (thread, nothing before Wi-Fi)\n");
}

static void test_sync(void)
{
	int64_t now, unix_us, rt;
	uint32_t delay;
	char text[64];

	notify(NET_EVENT_WIFI_CONNECT_RESULT);
	wait_for(time_sync_is_ready(), 1000);
	assert(get(sockets_open) == 1 && get(sends) == 1);
	assert(net.to.sin_family == AF_INET && net.to.sin_port == htons(123) &&
	       net.to.sin_addr.s_addr == htonl(0x0a2a0001));
	assert(net.request[0] == 0x23);
	for (int i = 1; i < 40; i++) {
		assert(net.request[i] == 0);
	}

	/* The server's time, from the TIMER: within the polls' granularity
	 * (half the time between the reply's arrival and the poll that finds
	 * it) and the host's scheduling.
	 */
	now = time_sync_mono_us();
	assert(time_sync_unix_us(now, &unix_us, &delay));
	if (abs64(unix_us - (now + SERVER_OFFSET_US)) > 5000 || delay < ROUND_TRIP_US - 100 ||
	    delay > ROUND_TRIP_US + 20000) {
		say(2, "error %lld us, round trip %u us\n",
		    (long long)(unix_us - (now + SERVER_OFFSET_US)), (unsigned int)delay);
		fail("estimate");
	}
	assert(time_sync_estimate_us(now, &unix_us, NULL));
	assert(abs64(unix_us - (now + SERVER_OFFSET_US)) <= 5000);

	/* The realtime clock was 1970 + uptime: stepped once to the server's. */
	rt = realtime_us();
	assert(abs64(rt - ((int64_t)rp2040_timer_us() + SERVER_OFFSET_US)) <= 25000);

	/* The polls for the reply: MSG_DONTWAIT, k_sleep(K_MSEC(1)) between. */
	assert(net.polls >= 2 && net.min_poll_gap_us >= 1000);

	cap_clear();
	assert(run("ntp status") == 0);
	expect_text("state: synchronized\r\n");
	expect_item("server", "10.42.0.1:123 (default gateway)");
	expect_item("interval_ms", "2000");
	/* The server's seconds (the offset plus the host's uptime), to the
	 * thousands.
	 */
	snprintf(text, sizeof(text), "  unix_time:                  %lld",
		 (long long)((now + SERVER_OFFSET_US) / 1000000 / 1000));
	expect_text(text);
	expect_text("  last_reply_ms_ago:          ");
	expect_text("\r\nexchanges:\r\n");
	expect_item("queries", "1");
	expect_item("replies", "1");
	expect_item("timeouts", "0");
	expect_item("rejected", "0");
	expect_item("last_error", "0");
	expect_text("\r\nround_trip:\r\n");
	snprintf(text, sizeof(text), "%u", (unsigned int)delay);
	expect_item("last_us", text);
	expect_item("min_us", text);
	expect_item("in_use_us", text);
	expect_text("\r\nclock:\r\n");
	expect_item("rate_valid", "0");
	expect_item("rate_ppb", "0");
	expect_item("last_residual_us", "0");
	/* The smoothed clock's first value is not a step; the realtime
	 * clock's is.
	 */
	expect_item("steps", "0");
	expect_item("realtime_clock_steps", "1");

	/* The next request after CONFIG_NINA_NTP_INTERVAL_MS. */
	now = (int64_t)rp2040_timer_us();
	wait_for(get(sends) == 2, 2500);
	assert((int64_t)rp2040_timer_us() - now >= 1500);
	assert(get(sockets_open) == 1 && net.sockets_made == 1);
	report("ok  sync (request, reply polls, estimate, realtime clock step, ntp status)\n");
}

static void test_timeout(void)
{
	int64_t sent;

	/* The next request finds no reply: 250 ms of polls, then a timeout. */
	wait_for(!net.pending, 1000);
	lock();
	net.mode = SERVER_SILENT;
	unlock();
	wait_for(get(sends) == 3, 2500);
	sent = net.sent_us;
	vTaskDelay(pdMS_TO_TICKS(400));
	/* k_uptime_get() is in whole ms: the last poll is 249..250 ms on. */
	assert(net.last_poll_of_wait_us - sent >= 249000 &&
	       net.last_poll_of_wait_us - sent < 300000);
	assert(time_sync_is_ready());

	cap_clear();
	assert(run("ntp status") == 0);
	expect_text("state: synchronized\r\n");
	expect_item("queries", "3");
	expect_item("replies", "2");
	expect_item("timeouts", "1");
	expect_item("last_error", "0");

	lock();
	net.mode = SERVER_REPLIES;
	unlock();
	report("ok  timeout (250 ms, counted, estimate kept)\n");
}

static void test_disconnect(void)
{
	int sent, steps_before;
	int64_t rt;

	notify(NET_EVENT_WIFI_DISCONNECT_RESULT);
	wait_for(get(sockets_open) == 0, 2500);
	sent = get(sends);
	vTaskDelay(pdMS_TO_TICKS(2500));
	assert(get(sends) == sent && net.closes == 1);

	/* Behind by 1 s: the next connection steps it forward. */
	cap_clear();
	assert(run("ntp status") == 0);
	steps_before = strstr(cap.text, "realtime_clock_steps:       1\r\n") != NULL ? 1 : -1;
	assert(steps_before == 1);
	set_realtime_us((int64_t)rp2040_timer_us() + SERVER_OFFSET_US - 1000000);

	notify(NET_EVENT_WIFI_CONNECT_RESULT);
	wait_for(get(sends) == sent + 1 && !net.pending, 1000);
	wait_for(time_sync_is_ready(), 1000);
	vTaskDelay(pdMS_TO_TICKS(20));
	rt = realtime_us();
	assert(abs64(rt - ((int64_t)rp2040_timer_us() + SERVER_OFFSET_US)) <= 25000);
	cap_clear();
	assert(run("ntp status") == 0);
	expect_text("state: synchronized\r\n");
	/* A new estimate: one reply on this connection. */
	expect_item("replies", "1");
	expect_item("realtime_clock_steps", "2");

	/* Ahead by 1 s: left alone. */
	set_realtime_us((int64_t)rp2040_timer_us() + SERVER_OFFSET_US + 1000000);
	wait_for(get(sends) == sent + 2 && !net.pending, 2500);
	vTaskDelay(pdMS_TO_TICKS(50));
	rt = realtime_us();
	assert(rt - ((int64_t)rp2040_timer_us() + SERVER_OFFSET_US) > 900000);
	cap_clear();
	assert(run("ntp status") == 0);
	expect_item("replies", "2");
	expect_item("realtime_clock_steps", "2");
	report("ok  disconnect, reconnect, realtime clock forward only\n");
}

static void runner(void *arg)
{
	(void)arg;
	shell_console_bind(&con);
	test_realtime_clock();
	test_mono();
	test_start();
	test_sync();
	test_timeout();
	test_disconnect();
	if (strstr(log_buf, "<err>") != NULL) {
		fail("errors in the log");
	}
	_exit(0);
}

int main(void)
{
	shell_console_init(&con, "usb", capture_out, NULL);
	shell_console_set_default(&con);
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, APP_PRIO_MAIN,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
