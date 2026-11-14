/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the time synchronization (nuttx/apps/imu/time_sync.c, the
 * copy of the Zephyr build's src/time_sync.c) on the Zephyr compatibility
 * layer, with a fake NINA socket, clock, network interface and realtime
 * clock.  The source is included, so its static functions are tested
 * directly:
 *
 * - The request: 48 bytes, 0x23 (version 4, client), the board time of
 *   the first read in bytes 40..47 (big endian), the rest zero, to the
 *   server's address.  t1 is the clock read right before sendto(), t4 the
 *   read right after the receive that returned the reply.
 * - The wait (the layer's zsock_recv() with ZSOCK_MSG_DONTWAIT): one
 *   FIONREAD (one poll of the module) per pass, recv() only when it says
 *   something is there, k_sleep(K_MSEC(1)) between passes that found
 *   nothing, the next pass at once after a datagram that is not the reply
 *   (a late reply to an earlier request), -ETIMEDOUT once 250 ms have gone
 *   by, other errors passed on.
 * - The socket (the layer's zsock_socket()): socket() marked
 *   NINA_SO_CALLER_POLLS, so that only this thread asks the module about
 *   it, as in the Zephyr build; a refused mark closes it (-1, the errno).
 *   A receive that waits (the layer's zsock_recv() without
 *   ZSOCK_MSG_DONTWAIT, not used by the file): one FIONREAD now and after
 *   each k_sleep(K_MSEC(20)), as the Zephyr build's udp_recv(), then recv().
 * - The checks of the reply (-EPROTO): mode, leap indicator 3, stratum
 *   0 and 16, no transmit time, transmit before receive, receive time
 *   before 2020, a round trip shorter than the server's time.  The NTP era
 *   that starts in 2036.  The sample: midpoint, offset and round trip.
 * - The estimate: ready after a sample, still at 60 s, not after.
 * - The realtime clock: stepped forward when 20 ms or more behind, never
 *   back.
 * - The Wi-Fi events: connection generation, state and wake-up.
 * - The server address: the default gateway, -ENETDOWN, -EHOSTUNREACH.
 * - "ntp status" (through nuttx/apps/subcmd): the Zephyr build's lines.
 */
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

/* The test's clock: nano_mono_us() (tests/time_sync_fakes), k_uptime_get()
 * and k_sleep() of the file under test.
 */
int64_t test_now_us = 1000000;

#define k_uptime_get test_uptime_get
#define k_sleep      test_sleep

/* The fake socket calls and realtime clock that the file under test (and
 * the layer's inline zsock_recv() and sys_clock_*()) call.
 */
#define socket(f, t, p) fake_socket(f, t, p)
#define setsockopt(fd, l, o, v, n) fake_setsockopt(fd, l, o, v, n)
#define close(fd) fake_close(fd)
#define ioctl(fd, req, ...) fake_ioctl(fd, req, __VA_ARGS__)
#define recv(fd, buf, len, flags) fake_recv(fd, buf, len, flags)
#define sendto(fd, buf, len, flags, to, tolen) fake_sendto(fd, buf, len, flags, to, tolen)
#define clock_gettime(id, ts) fake_clock_gettime(id, ts)
#define clock_settime(id, ts) fake_clock_settime(id, ts)

struct timespec;
int fake_socket(int family, int type, int protocol);
int fake_setsockopt(int fd, int level, int option, const void *value, socklen_t len);
int fake_close(int fd);
int fake_ioctl(int fd, int req, int *arg);
ssize_t fake_recv(int fd, void *buf, size_t len, int flags);
ssize_t fake_sendto(int fd, const void *buf, size_t len, int flags, const struct sockaddr *to,
		    socklen_t tolen);
int fake_clock_gettime(clockid_t id, struct timespec *ts);
int fake_clock_settime(clockid_t id, const struct timespec *ts);

#include "../apps/imu/time_sync.c"

int64_t test_uptime_get(void)
{
	return test_now_us / 1000;
}

static int sleeps;
static int64_t slept_ticks;

int32_t test_sleep(k_timeout_t timeout)
{
	sleeps++;
	slept_ticks = timeout.ticks;
	/* Zephyr and NuttX wait timeout + 1 ticks of 100 us. */
	test_now_us += (timeout.ticks + 1) * 100;
	return 0;
}

/* Fake socket ------------------------------------------------------------*/

#define FAKE_FD 7

static struct {
	uint8_t data[64];
	size_t len;
	int64_t arrival_us;
} queue[4];
static int queued;
static int polls, recvs, sends;
static int ioctl_error, recv_error, send_error;
static int64_t send_entry_us, recv_exit_us;
static uint8_t sent[64];
static size_t sent_len;
static struct sockaddr_in sent_to;
/* What one call costs on the module (SPI and its wait). */
static int64_t poll_cost_us = 0, recv_cost_us = 400, send_cost_us = 300;
/* The server's answer to the next request: arrives this long after the
 * request has been sent, takes server_hold_us, its clock is the board's
 * plus server_offset_us.
 */
static int64_t reply_after_us = 5000, server_hold_us = 50, server_offset_us;
static uint8_t reply_mode = 0x24, reply_stratum = 2;
static bool reply_wanted = true, reply_no_transmit, reply_swap_times;
static int64_t reply_t2_override, reply_t3_extra_us;

static void put_be32(uint8_t *p, uint32_t v)
{
	p[0] = v >> 24;
	p[1] = v >> 16;
	p[2] = v >> 8;
	p[3] = v;
}

static void put_ntp(uint8_t *p, int64_t unix_us)
{
	int64_t s = unix_us / 1000000, us = unix_us % 1000000;
	uint64_t frac = (((uint64_t)us << 32) + 999999) / 1000000;

	put_be32(p, (uint32_t)(s + NTP_UNIX_OFFSET));
	put_be32(p + 4, (uint32_t)frac);
}

static void push(const uint8_t *data, size_t len, int64_t arrival_us)
{
	assert(queued < 4);
	memcpy(queue[queued].data, data, len);
	queue[queued].len = len;
	queue[queued].arrival_us = arrival_us;
	queued++;
}

static bool available(void)
{
	return queued > 0 && queue[0].arrival_us <= test_now_us;
}

static int sockets_open, marks, mark_error;

int fake_socket(int family, int type, int protocol)
{
	assert(family == AF_INET && type == SOCK_DGRAM && protocol == IPPROTO_UDP);
	assert(sockets_open == 0);
	sockets_open++;
	return FAKE_FD;
}

int fake_setsockopt(int fd, int level, int option, const void *value, socklen_t len)
{
	assert(fd == FAKE_FD && sockets_open == 1 && level == NINA_SOL &&
	       option == NINA_SO_CALLER_POLLS && len == sizeof(int) && *(const int *)value == 1);
	if (mark_error) {
		errno = mark_error;
		return -1;
	}
	marks++;
	return 0;
}

/* The test's own descriptors (the pipes of run_ntp()) are the host's. */
int fake_close(int fd)
{
	if (fd != FAKE_FD) {
		return (close)(fd);
	}
	assert(sockets_open == 1);
	sockets_open--;
	return 0;
}

int fake_ioctl(int fd, int req, int *arg)
{
	assert(fd == FAKE_FD && req == FIONREAD);
	polls++;
	test_now_us += poll_cost_us;
	if (ioctl_error) {
		errno = ioctl_error;
		return -1;
	}
	*arg = available() ? 1 : 0;
	return 0;
}

ssize_t fake_recv(int fd, void *buf, size_t len, int flags)
{
	size_t n;

	assert(fd == FAKE_FD && flags == MSG_DONTWAIT);
	recvs++;
	test_now_us += recv_cost_us;
	recv_exit_us = test_now_us;
	if (recv_error) {
		errno = recv_error;
		return -1;
	}
	/* FIONREAD said there is something. */
	assert(available());
	n = queue[0].len < len ? queue[0].len : len;
	memcpy(buf, queue[0].data, n);
	queued--;
	memmove(&queue[0], &queue[1], sizeof(queue[0]) * queued);
	return (ssize_t)n;
}

ssize_t fake_sendto(int fd, const void *buf, size_t len, int flags, const struct sockaddr *to,
		    socklen_t tolen)
{
	uint8_t reply[NTP_PACKET_SIZE] = {0};
	int64_t t2, t3;

	assert(fd == FAKE_FD && flags == 0 && tolen == sizeof(sent_to));
	sends++;
	send_entry_us = test_now_us;
	memcpy(sent, buf, len);
	sent_len = len;
	memcpy(&sent_to, to, sizeof(sent_to));
	test_now_us += send_cost_us;
	if (send_error) {
		errno = send_error;
		return -1;
	}
	if (!reply_wanted) {
		return (ssize_t)len;
	}
	/* The server receives half way and answers after server_hold_us. */
	t2 = test_now_us + reply_after_us / 2 + server_offset_us;
	t3 = t2 + server_hold_us + reply_t3_extra_us;
	if (reply_t2_override) {
		t2 = reply_t2_override;
	}
	if (reply_swap_times) {
		int64_t t = t2;

		t2 = t3;
		t3 = t;
	}
	reply[0] = reply_mode;
	reply[1] = reply_stratum;
	memcpy(reply + 24, sent + 40, 8);
	put_ntp(reply + 32, t2);
	if (!reply_no_transmit) {
		put_ntp(reply + 40, t3);
	}
	push(reply, sizeof(reply), test_now_us + reply_after_us + server_hold_us);
	return (ssize_t)len;
}

/* Fake realtime clock. */
static int64_t realtime_us;
static int settimes;

int fake_clock_gettime(clockid_t id, struct timespec *ts)
{
	assert(id == CLOCK_REALTIME);
	ts->tv_sec = realtime_us / 1000000;
	ts->tv_nsec = (realtime_us % 1000000) * 1000;
	return 0;
}

int fake_clock_settime(clockid_t id, const struct timespec *ts)
{
	assert(id == CLOCK_REALTIME);
	settimes++;
	realtime_us = (int64_t)ts->tv_sec * 1000000 + ts->tv_nsec / 1000;
	return 0;
}

/* Fake network interface, events and resolver ----------------------------*/

static struct net_if fake_iface;
static bool have_iface = true;
static struct in_addr gateway;
static struct net_mgmt_event_callback *added;

struct net_if *net_if_get_default(void)
{
	return have_iface ? &fake_iface : NULL;
}

struct net_in_addr net_if_ipv4_get_gw(struct net_if *iface)
{
	assert(iface == &fake_iface);
	return gateway;
}

void net_mgmt_add_event_callback(struct net_mgmt_event_callback *cb)
{
	added = cb;
}

int nina_getaddrinfo(const char *host, const char *service, const struct addrinfo *hints,
		     struct addrinfo **res)
{
	(void)host;
	(void)service;
	(void)hints;
	(void)res;
	assert(!"no resolver with the default gateway");
	return EAI_FAIL;
}

void nina_freeaddrinfo(struct addrinfo *res)
{
	(void)res;
}

/* Helpers ----------------------------------------------------------------*/

static void reset_net(void)
{
	queued = 0;
	polls = recvs = sends = 0;
	sleeps = 0;
	ioctl_error = recv_error = send_error = 0;
	poll_cost_us = 0;
	recv_cost_us = 400;
	send_cost_us = 300;
	reply_after_us = 5000;
	server_hold_us = 50;
	server_offset_us = 0;
	reply_mode = 0x24;
	reply_stratum = 2;
	reply_wanted = true;
	reply_no_transmit = reply_swap_times = false;
	reply_t2_override = reply_t3_extra_us = 0;
}

static int64_t be64(const uint8_t *p)
{
	return (int64_t)(((uint64_t)be32(p) << 32) | be32(p + 4));
}

static char out[4096];

static void slurp(FILE *f, char *buf, size_t size)
{
	size_t n;

	rewind(f);
	n = fread(buf, 1, size - 1, f);
	buf[n] = '\0';
}

static int run_ntp(const char *word)
{
	char *argv[] = {"ntp", (char *)word, NULL};
	int ret, saved;
	FILE *fo = tmpfile();

	assert(fo);
	fflush(stdout);
	saved = dup(1);
	dup2(fileno(fo), 1);
	ret = subcmd_main(&ntp_root, word ? 2 : 1, argv);
	fflush(stdout);
	dup2(saved, 1);
	close(saved);
	slurp(fo, out, sizeof(out));
	fclose(fo);
	return ret;
}

/* Tests ------------------------------------------------------------------*/

static void test_ntp_time(void)
{
	uint8_t p[8];

	/* 2026-11-14 00:00:00.5 UTC. */
	put_be32(p, (uint32_t)(1794614400LL + NTP_UNIX_OFFSET));
	put_be32(p + 4, 0x80000000u);
	assert(ntp_to_unix_us(p) == 1794614400LL * 1000000 + 500000);
	/* The era from 2036: seconds below the 1970 offset. */
	put_be32(p, 10);
	put_be32(p + 4, 0);
	assert(ntp_to_unix_us(p) == ((1LL << 32) + 10 - NTP_UNIX_OFFSET) * 1000000);
	/* Fractions round down to the microsecond. */
	put_be32(p, (uint32_t)NTP_UNIX_OFFSET);
	put_be32(p + 4, 0xffffffffu);
	assert(ntp_to_unix_us(p) == 999999);
}

static void test_exchange_ok(void)
{
	struct time_offset_sample sample;
	int64_t t1, t4, t2, t3;
	int ret;

	reset_net();
	server.sin_family = AF_INET;
	server.sin_port = htons(123);
	server.sin_addr.s_addr = htonl(0x0a2a0001);
	server_offset_us = 1794614400LL * 1000000 - test_now_us;
	ret = exchange(FAKE_FD, &sample);
	assert(ret == 0);
	/* The request. */
	assert(sends == 1 && sent_len == NTP_PACKET_SIZE && sent[0] == 0x23);
	for (int i = 1; i < 40; i++) {
		assert(sent[i] == 0);
	}
	/* The first read (bytes 40..47), then t1 right before sendto(). */
	t1 = send_entry_us - 1;
	assert(be64(sent + 40) == t1 - 1);
	assert(sent_to.sin_family == AF_INET && sent_to.sin_port == htons(123) &&
	       sent_to.sin_addr.s_addr == htonl(0x0a2a0001));
	/* Polled every K_MSEC(1) (10 + 1 ticks) until the reply was there
	 * (5.05 ms after the send ended): passes at 0, 1.1, ..., 5.5 ms, the
	 * last one found it.  One FIONREAD per pass, recv() only then.
	 */
	assert(polls == 6 && recvs == 1 && sleeps == 5 && slept_ticks == 10);
	/* t4 right after the receive. */
	t4 = recv_exit_us;
	t2 = send_entry_us + send_cost_us + reply_after_us / 2 + server_offset_us;
	t3 = t2 + server_hold_us;
	assert(sample.mono_us == (t1 + t4) / 2);
	assert(sample.offset_us == ((t2 - t1) + (t3 - t4)) / 2);
	assert(sample.delay_us == (uint32_t)((t4 - t1) - (t3 - t2)));
}

static void test_exchange_late_reply(void)
{
	struct time_offset_sample sample;
	uint8_t late[NTP_PACKET_SIZE] = {0x24, 2};
	int64_t before;

	reset_net();
	server_offset_us = 1794614400LL * 1000000 - test_now_us;
	reply_after_us = 0;
	server_hold_us = 0;
	/* A late reply to an earlier request is already there. */
	memset(late + 24, 0x55, 8);
	push(late, sizeof(late), test_now_us);
	before = test_now_us;
	assert(exchange(FAKE_FD, &sample) == 0);
	/* Read at once, then the reply, without a sleep between. */
	assert(recvs == 2 && polls == 2 && sleeps == 0);
	assert(test_now_us - before < 2000);
	/* A datagram of another size is skipped the same way. */
	reset_net();
	server_offset_us = 1794614400LL * 1000000 - test_now_us;
	reply_after_us = 0;
	server_hold_us = 0;
	push(late, 20, test_now_us);
	assert(exchange(FAKE_FD, &sample) == 0 && recvs == 2 && sleeps == 0);
}

static void test_exchange_timeout(void)
{
	struct time_offset_sample sample;
	int64_t start;

	reset_net();
	reply_wanted = false;
	start = test_now_us;
	assert(exchange(FAKE_FD, &sample) == -ETIMEDOUT);
	/* 250 ms after the send, on k_uptime_get(). */
	assert(test_now_us - start >= 250000 && test_now_us - start < 252000);
	assert(recvs == 0 && polls > 200 && polls == sleeps + 1);
}

static void test_socket(void)
{
	int fd;

	marks = 0;
	fd = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	assert(fd == FAKE_FD && sockets_open == 1 && marks == 1);
	assert(zsock_close(fd) == 0 && sockets_open == 0);
	mark_error = ENOPROTOOPT;
	errno = 0;
	assert(zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) == -1 && errno == ENOPROTOOPT);
	assert(sockets_open == 0 && marks == 1);
	mark_error = 0;
}

static void test_recv_waits(void)
{
	uint8_t data[4] = {1, 2, 3, 4}, buf[8];
	int64_t start;

	reset_net();
	start = test_now_us;
	push(data, sizeof(data), start + 50000);
	/* FIONREAD at 0, 20.1, 40.2 ms: nothing; 60.3 ms: the datagram. */
	assert(zsock_recv(FAKE_FD, buf, sizeof(buf), 0) == 4 && !memcmp(buf, data, 4));
	assert(polls == 4 && sleeps == 3 && slept_ticks == 200 && recvs == 1);
	assert(recv_exit_us - start == 3 * 20100 + recv_cost_us);
	/* A failed poll ends it. */
	reset_net();
	ioctl_error = EIO;
	assert(zsock_recv(FAKE_FD, buf, sizeof(buf), 0) == -1 && errno == EIO);
	assert(polls == 1 && sleeps == 0);
	ioctl_error = 0;
}

static void test_exchange_errors(void)
{
	struct time_offset_sample sample;

	reset_net();
	send_error = ENETUNREACH;
	assert(exchange(FAKE_FD, &sample) == -ENETUNREACH);
	reset_net();
	ioctl_error = ENOTCONN;
	assert(exchange(FAKE_FD, &sample) == -ENOTCONN);
	reset_net();
	recv_error = EIO;
	assert(exchange(FAKE_FD, &sample) == -EIO);
	/* The layer's EAGAIN when FIONREAD says nothing is there. */
	reset_net();
	reply_wanted = false;
	errno = 0;
	assert(zsock_recv(FAKE_FD, out, 48, ZSOCK_MSG_DONTWAIT) == -1 && errno == EAGAIN);
	assert(polls == 1 && recvs == 0);
}

static void expect_rejected(void)
{
	struct time_offset_sample sample;

	server_offset_us = 1794614400LL * 1000000 - test_now_us;
	assert(exchange(FAKE_FD, &sample) == -EPROTO);
}

static void test_exchange_rejected(void)
{
	reset_net();
	reply_mode = 0x23; /* client */
	expect_rejected();
	reset_net();
	reply_mode = 0xe4; /* leap indicator 3: not synchronized */
	expect_rejected();
	reset_net();
	reply_stratum = 0;
	expect_rejected();
	reset_net();
	reply_stratum = 16;
	expect_rejected();
	reset_net();
	reply_no_transmit = true;
	expect_rejected();
	reset_net();
	reply_swap_times = true; /* transmit before receive */
	expect_rejected();
	reset_net();
	reply_t2_override = (NTP_MIN_UNIX_TIME - 1) * 1000000; /* before 2020 */
	expect_rejected();
	reset_net();
	reply_t3_extra_us = 20000; /* the server's time longer than the round trip */
	expect_rejected();
	/* Accepted: leap indicator 1, stratum 15. */
	reset_net();
	reply_mode = 0x64;
	reply_stratum = 15;
	{
		struct time_offset_sample sample;

		server_offset_us = 1794614400LL * 1000000 - test_now_us;
		assert(exchange(FAKE_FD, &sample) == 0);
	}
}

static void test_estimate_age(void)
{
	struct time_offset_sample sample;
	int64_t unix_us;
	uint32_t delay;

	time_offset_init(&estimate);
	time_slew_init(&slewed);
	assert(!time_sync_is_ready());
	assert(!time_sync_unix_us(time_sync_mono_us(), &unix_us, &delay));
	reset_net();
	realtime_us = 0;
	settimes = 0;
	server_offset_us = 1794614400LL * 1000000 - test_now_us;
	assert(exchange(FAKE_FD, &sample) == 0);
	record(&sample);
	assert(time_sync_is_ready());
	assert(time_sync_unix_us(time_sync_mono_us(), &unix_us, &delay));
	assert(delay == sample.delay_us);
	assert(llabs(unix_us - (time_sync_mono_us() + server_offset_us)) < 1000);
	assert(time_sync_estimate_us(time_sync_mono_us(), &unix_us, &delay));
	/* The realtime clock was 0: stepped forward to the estimate. */
	assert(settimes == 1 && atomic_get(&clock_steps) == 1);
	assert(llabs(realtime_us - (test_now_us + server_offset_us)) < 1000);
	/* 60 s after the reply: still recent; a millisecond later: not. */
	test_now_us = last_sample_uptime * 1000 + NTP_MAX_AGE_MS * 1000;
	assert(time_sync_is_ready());
	test_now_us += 1000;
	assert(!time_sync_is_ready());
	assert(!time_sync_unix_us(time_sync_mono_us(), &unix_us, &delay));
	assert(!time_sync_estimate_us(time_sync_mono_us(), &unix_us, &delay));
}

static void test_realtime_forward_only(void)
{
	int64_t unix_us;

	/* A fresh estimate with no rate. */
	time_offset_init(&estimate);
	time_slew_init(&slewed);
	{
		struct time_offset_sample sample;

		reset_net();
		server_offset_us = 1794614400LL * 1000000 - test_now_us;
		assert(exchange(FAKE_FD, &sample) == 0);
		realtime_us = 0;
		record(&sample);
	}
	settimes = 0;
	assert(time_slew_get(&slewed, test_now_us, &unix_us));
	/* Ahead by 10 s: left alone. */
	realtime_us = unix_us + 10000000;
	follow_realtime_clock();
	assert(settimes == 0);
	/* Behind by less than 20 ms: left alone. */
	assert(time_slew_get(&slewed, test_now_us, &unix_us));
	realtime_us = unix_us - 19000;
	follow_realtime_clock();
	assert(settimes == 0);
	/* Behind by 20 ms or more: moved forward to the estimate. */
	assert(time_slew_get(&slewed, test_now_us, &unix_us));
	realtime_us = unix_us - 21000;
	follow_realtime_clock();
	assert(settimes == 1 && llabs(realtime_us - unix_us) < 100);
}

static void test_wifi_events(void)
{
	struct wifi_status ok = {.status = 0}, failed = {.status = -110};
	atomic_val_t generation;

	time_sync_init();
	assert(added == &wifi_events);
	assert(wifi_events.event_mask ==
	       (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT));
	k_sem_reset(&sync_request);
	generation = atomic_get(&wifi_generation);
	wifi_events.info = &failed;
	wifi_event(&wifi_events, NET_EVENT_WIFI_CONNECT_RESULT, &fake_iface);
	assert(atomic_get(&wifi_generation) == generation && !atomic_get(&wifi_connected));
	assert(k_sem_count_get(&sync_request) == 0);
	wifi_events.info = NULL;
	wifi_event(&wifi_events, NET_EVENT_WIFI_CONNECT_RESULT, &fake_iface);
	assert(atomic_get(&wifi_generation) == generation);
	wifi_events.info = &ok;
	wifi_event(&wifi_events, NET_EVENT_WIFI_CONNECT_RESULT, &fake_iface);
	assert(atomic_get(&wifi_generation) == generation + 1 && atomic_get(&wifi_connected));
	assert(k_sem_count_get(&sync_request) == 1);
	wifi_events.info = NULL;
	wifi_event(&wifi_events, NET_EVENT_WIFI_DISCONNECT_RESULT, &fake_iface);
	assert(atomic_get(&wifi_generation) == generation + 2 && !atomic_get(&wifi_connected));
	assert(k_sem_count_get(&sync_request) == 1); /* limit 1 */
}

static void test_resolve_server(void)
{
	have_iface = false;
	assert(resolve_server() == -ENETDOWN);
	have_iface = true;
	gateway.s_addr = 0;
	assert(resolve_server() == -EHOSTUNREACH);
	gateway.s_addr = htonl(0x0a2a0001);
	assert(resolve_server() == 0);
	assert(server.sin_family == AF_INET && server.sin_port == htons(123) &&
	       server.sin_addr.s_addr == htonl(0x0a2a0001));
}

static void test_status_output(void)
{
	static const char not_synced[] =
		"state: not synchronized\n"
		"  server:                     none:123 (default gateway)\n"
		"  interval_ms:                2000\n"
		"exchanges:\n"
		"  queries:                    0\n"
		"  replies:                    0\n"
		"  timeouts:                   0\n"
		"  rejected:                   0\n"
		"  last_error:                 0\n";
	char expected[1024];

	time_offset_init(&estimate);
	time_slew_init(&slewed);
	memset(&server, 0, sizeof(server));
	atomic_set(&clock_steps, 0);
	assert(run_ntp("status") == 0);
	assert(strcmp(out, not_synced) == 0);
	/* The help of "ntp" alone. */
	assert(run_ntp(NULL) != 0);
	assert(strcmp(out, "ntp - Network time synchronization\n"
			   "Subcommands:\n"
			   "  status  : Show time synchronization status\n") == 0);

	/* Synchronized, with one exchange. */
	gateway.s_addr = htonl(0x0a2a0001);
	assert(resolve_server() == 0);
	{
		struct time_offset_sample sample;

		reset_net();
		server_offset_us = 1794614400LL * 1000000 - test_now_us;
		assert(exchange(FAKE_FD, &sample) == 0);
		realtime_us = 0;
		record(&sample);
	}
	atomic_set(&queries, 3);
	atomic_set(&timeouts, 1);
	atomic_set(&rejected, 1);
	atomic_set(&last_error, 0);
	test_now_us += 58000;
	assert(run_ntp("status") == 0);
	snprintf(expected, sizeof(expected),
		 "state: synchronized\n"
		 "  server:                     10.42.0.1:123 (default gateway)\n"
		 "  interval_ms:                2000\n");
	assert(strncmp(out, expected, strlen(expected)) == 0);
	{
		const char *labels[] = {
			"  unix_time:                  ", "  last_reply_ms_ago:          58\n",
			"exchanges:\n", "  queries:                    3\n",
			"  replies:                    1\n", "  timeouts:                   1\n",
			"  rejected:                   1\n", "  last_error:                 0\n",
			"round_trip:\n", "  last_us:                    ",
			"  min_us:                     ", "  in_use_us:                  ",
			"clock:\n", "  rate_valid:                 0\n",
			"  rate_ppb:                   0\n", "  last_residual_us:           0\n",
			"  slew_ppb:                   0\n", "  smoothed_minus_estimate_us: ",
			"  steps:                      ", "  realtime_clock_steps:       1\n",
		};
		const char *p = out + strlen(expected);

		for (size_t i = 0; i < ARRAY_SIZE(labels); i++) {
			const char *q = strstr(p, labels[i]);

			assert(q != NULL);
			p = q + strlen(labels[i]);
		}
	}
}

int main(void)
{
	test_ntp_time();
	test_exchange_ok();
	test_exchange_late_reply();
	test_exchange_timeout();
	test_socket();
	test_recv_waits();
	test_exchange_errors();
	test_exchange_rejected();
	test_estimate_age();
	test_realtime_forward_only();
	test_wifi_events();
	test_resolve_server();
	test_status_output();
	printf("time_sync: all tests passed\n");
	return 0;
}
