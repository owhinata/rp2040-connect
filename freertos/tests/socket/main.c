/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's sockets on the FreeRTOS POSIX port:
 * the zsock_*() calls of the layer's socket part (common/compat/net_offload:
 * subsys/net/lib/sockets, lib/os/zvfs) on the NINA's Zephyr binding
 * (common/drivers/nina/port/zephyr/nina_socket.c, registered with
 * NET_SOCKET_OFFLOAD_REGISTER(), with NINA_OS_DNS_OFFLOAD) on the shared
 * NINA core (common/drivers/nina/nina_sock.c, nina_dns.c) and the address
 * conversions (common/compat/libc_net/inet.c), as the board builds them
 * (common/compat/port/pico_sdk/compat.cmake, common/pico_sdk/drivers/nina/nina.cmake),
 * on a fake module (nina_proto_request(), here: the firmware's sockets as a small
 * model that logs each command) and a fake clock (nina_os.h, and the
 * binding's k_uptime_get() and k_sleep(), wrapped here with -Wl,--wrap: the
 * time moves only with the sleeps and when a test moves it; each sleep also
 * sleeps for real, so that the other tasks run).  The core's own behaviour
 * (each command's parameters and replies, the deadlines of one pass) is
 * common/tests/nina_sock.c's, the layer's alone freertos/tests/sockets';
 * this test covers what the binding on the layer gives the build:
 *
 * - the descriptors: the lowest free number, ten sockets at most (EMFILE
 *   from the binding; the layer's table has 16), a closed or out-of-range
 *   one (EBADF, POLLNVAL), reuse after close, the firmware socket made on
 *   first use;
 * - the Zephyr build's wait loops and their sleeps (the socket events
 *   cannot start here: nina_os_event_start() fails): connect (every 20 ms,
 *   also on a non-blocking socket, the lwIP error, ETIMEDOUT), TCP send
 *   (1 ms between passes, EAGAIN at the deadline, EPIPE), receive (every
 *   20 ms until data or SO_RCVTIMEO; a poll on every pass, also after
 *   zsock_poll() found data), poll (every 20 ms until the timeout);
 * - non-blocking: O_NONBLOCK (ZVFS_O_NONBLOCK) by zsock_fcntl() and MSG_DONTWAIT: one
 *   command, EAGAIN, no sleep;
 * - poll: a failed poll of the module is POLLERR on that socket (the call
 *   does not fail);
 * - the options: SO_RCVTIMEO's checks, TCP_NODELAY accepted on TCP only,
 *   the rest ENOPROTOOPT; getsockopt(), getsockname() EOPNOTSUPP,
 *   shutdown() ENOTSUP; zsock_fcntl()'s commands;
 * - accept (never waits; the peer's address and its length; a blocking
 *   descriptor), sendmsg (the vectors joined, the refusals), datagrams
 *   (the sender's address and its length);
 * - the descriptor's lock: a close and a poll both wait for a receive in
 *   progress on another task (Zephyr's VTABLE_CALL(), zvfs_close() and the
 *   POLL_PREPARE of zvfs_poll() take it);
 * - name resolution through zsock_getaddrinfo() (EAI_FAIL until the
 *   binding's resolver is registered; then the module's commands, a numeric
 *   address without them, Zephyr's DNS_EAI_* values, the results on the
 *   binding's 2048-byte k_heap: EAI_MEMORY when it is full, the room back
 *   after zsock_freeaddrinfo());
 * - the constants of the network headers against Zephyr's values.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#undef NDEBUG
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/fdtable.h>

#include "nina_dns.h"
#include "nina_os.h"
#include "nina_proto.h"
#include "nina_sock.h"
#include "nina_wait.h"
#include "priorities.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 32768
#define HELPER_STACK 32768

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* The fake clock and sleeps (nina_os.h) ---------------------------------*/

static int64_t fake_ms = 1000;
static int sleeps;
static int32_t sleep_total_ms, sleep_last_ms;
static K_MUTEX_DEFINE(test_nina_lock);
static K_MUTEX_DEFINE(test_dns_lock);

int nina_os_lock(void)
{
	return k_mutex_lock(&test_nina_lock, K_FOREVER);
}

void nina_os_unlock(void)
{
	(void)k_mutex_unlock(&test_nina_lock);
}

void nina_os_dns_lock(void)
{
	(void)k_mutex_lock(&test_dns_lock, K_FOREVER);
}

void nina_os_dns_unlock(void)
{
	(void)k_mutex_unlock(&test_dns_lock);
}

void nina_os_sleep_us(uint32_t us)
{
	(void)us;
	assert(0); /* only the SPI link sleeps in microseconds */
}

void nina_os_sleep_ms(uint32_t ms)
{
	sleeps++;
	sleep_total_ms += (int32_t)ms;
	sleep_last_ms = (int32_t)ms;
	fake_ms += ms;
	k_msleep((int32_t)ms);
}

int64_t nina_os_uptime_ms(void)
{
	return fake_ms;
}

/* The binding's own clock and sleeps (k_uptime_get(), k_sleep() of the
 * layer): the same fake clock and count.
 */
int32_t __real_k_sleep(k_timeout_t timeout);

int64_t __wrap_k_uptime_get(void)
{
	return fake_ms;
}

int32_t __wrap_k_sleep(k_timeout_t timeout)
{
	int32_t ms = (int32_t)(timeout.ticks * Z_USEC_PER_TICK / 1000);

	sleeps++;
	sleep_total_ms += ms;
	sleep_last_ms = ms;
	fake_ms += ms;
	return __real_k_sleep(timeout);
}

/* The socket events cannot start here, so the waits are the sleeps above
 * (nina_wait.c); the other two are then never called (nina_os.h).  The
 * waits on the events are common/tests/nina_wait.c's and the Zephyr
 * build's tests/nina_socket.c's.
 */
int nina_os_event_start(void)
{
	return -ENOTSUP;
}

uint32_t nina_os_event_seq(void)
{
	assert(0);
	return 0;
}

void nina_os_event_wait(uint32_t seq, uint32_t ms)
{
	(void)seq;
	(void)ms;
	assert(0);
}

/* The fake module ---------------------------------------------------------*/

#define FW_MAX 10

struct dgram {
	uint8_t ip[4];
	uint16_t port;
	uint8_t data[64];
	size_t len;
};

struct fw_sock {
	bool open;
	bool udp;
	bool nonblocking;
	bool listening;
	bool connected;
	int connect_polls; /* polls that find the handshake in progress; -1: all */
	int lwip_error;    /* its SO_ERROR */
	bool error;        /* POLL_ERROR */
	bool poll_fail;    /* POLL_FAIL */
	uint8_t data[1024];
	size_t len;
	bool eof;
	int zero_writes; /* sends that take nothing; -1: all */
	int later_polls; /* polls before "later" becomes the data */
	const char *later;
	int advance_on_poll, advance_on_send;
	struct dgram dgram[4];
	int ndgram;
	uint8_t sent[4096];
	size_t sent_len;
	int pending; /* a client to accept: 1 */
	uint8_t peer_ip[4];
	uint16_t peer_port;
};

static struct fw_sock fw[FW_MAX];
static char cmd_log[4096];
static uint8_t close_reply = 1;
static uint8_t dns_answer = 1;
static uint8_t dns_ip[4] = {192, 0, 2, 7};
static char dns_name[256];

static void log_cmd(const char *name)
{
	size_t n = strlen(cmd_log);

	assert(n + strlen(name) + 2 < sizeof(cmd_log));
	snprintf(cmd_log + n, sizeof(cmd_log) - n, "%s%s", n ? " " : "", name);
}

static void log_expect(const char *expected)
{
	if (strcmp(cmd_log, expected) != 0) {
		fprintf(stderr, "commands: \"%s\"\nexpected: \"%s\"\n", cmd_log, expected);
		assert(0);
	}
	cmd_log[0] = '\0';
}

static void sleeps_expect(int count, int32_t total)
{
	if (sleeps != count || sleep_total_ms != total) {
		fprintf(stderr, "sleeps %d (%d ms), expected %d (%d ms)\n", sleeps,
			(int)sleep_total_ms, count, (int)total);
		assert(0);
	}
	sleeps = 0;
	sleep_total_ms = 0;
}

/* The firmware gives its sockets from the top, so that a descriptor (the
 * core's number) is not the firmware's. */
static int fw_alloc(bool udp)
{
	for (int i = FW_MAX - 1; i >= 0; i--) {
		if (!fw[i].open) {
			memset(&fw[i], 0, sizeof(fw[i]));
			fw[i].open = true;
			fw[i].udp = udp;
			return i;
		}
	}
	return -1;
}

/* The binding numbers its sockets as the layer its descriptors, from the
 * lowest free one, so with only NINA sockets open a descriptor is its
 * socket's number in the core's table.
 */
static struct fw_sock *fw_of(int fd)
{
	struct nina_sock *s = nina_sock_get(fd);

	assert(s && s->id != NINA_NO_SOCKET && s->id < FW_MAX && fw[s->id].open);
	return &fw[s->id];
}

static void reply(uint8_t *out, size_t cap, size_t *out_len, const void *data, size_t len)
{
	assert(len <= cap);
	memcpy(out, data, len);
	*out_len = len;
}

int nina_proto_request(uint8_t cmd, const struct nina_proto_param *p, size_t nparams,
		       bool wide_params, bool wide_response, uint8_t *out, size_t cap,
		       size_t *out_len)
{
	struct fw_sock *s = NULL;
	uint8_t b[8];

	(void)wide_params;
	(void)wide_response;
	if (cmd >= NINA_CMD_SOCKET_SOCKET && cmd != NINA_CMD_SOCKET_SOCKET) {
		assert(nparams >= 1 && p[0].len == 1 && p[0].data[0] < FW_MAX);
		s = &fw[p[0].data[0]];
		assert(s->open);
	}
	switch (cmd) {
	case NINA_CMD_SOCKET_SOCKET: {
		int id;

		log_cmd("socket");
		assert(nparams == 2);
		id = fw_alloc(p[0].data[0] == 2);
		assert(p[1].data[0] == (p[0].data[0] == 2 ? 17 : 6));
		b[0] = id < 0 ? NINA_NO_SOCKET : (uint8_t)id;
		reply(out, cap, out_len, b, 1);
		return 0;
	}
	case NINA_CMD_SOCKET_IOCTL:
		log_cmd("ioctl");
		s->nonblocking = true;
		reply(out, cap, out_len, "\0\0\0\0", 4);
		return 0;
	case NINA_CMD_SOCKET_CLOSE:
		log_cmd("close");
		s->open = false;
		reply(out, cap, out_len, &close_reply, 1);
		return 0;
	case NINA_CMD_SOCKET_BIND:
		log_cmd("bind");
		reply(out, cap, out_len, "\1", 1);
		return 0;
	case NINA_CMD_SOCKET_LISTEN:
		log_cmd("listen");
		s->listening = true;
		reply(out, cap, out_len, "\1", 1);
		return 0;
	case NINA_CMD_SOCKET_ACCEPT: {
		int id;

		log_cmd("accept");
		memset(b, 0, sizeof(b));
		b[0] = NINA_NO_SOCKET;
		if (s->pending) {
			id = fw_alloc(false);
			assert(id >= 0);
			fw[id].connected = true;
			s->pending = 0;
			b[0] = (uint8_t)id;
			memcpy(b + 1, s->peer_ip, 4);
			b[5] = s->peer_port & 0xff;
			b[6] = s->peer_port >> 8;
		}
		reply(out, cap, out_len, b, 7);
		return 0;
	}
	case NINA_CMD_SOCKET_CONNECT:
		log_cmd("connect");
		assert(nparams == 3 && p[1].len == 4 && p[2].len == 2);
		reply(out, cap, out_len, "\0", 1);
		return 0;
	case NINA_CMD_SOCKET_POLL: {
		uint8_t state = 0;

		log_cmd("poll");
		if (s->poll_fail) {
			state = NINA_POLL_FAIL;
		} else {
			if (!s->udp && !s->listening && !s->connected && s->connect_polls >= 0) {
				if (s->connect_polls == 0) {
					s->connected = true;
				} else {
					s->connect_polls--;
				}
			}
			if (s->later && s->later_polls-- == 0) {
				s->len = strlen(s->later);
				memcpy(s->data, s->later, s->len);
				s->later = NULL;
			}
			if (s->len || s->eof || s->ndgram || s->pending) {
				state |= NINA_POLL_READ;
			}
			if (s->udp || s->connected) {
				state |= NINA_POLL_WRITE;
			}
			if (s->error) {
				state |= NINA_POLL_ERROR;
			}
		}
		fake_ms += s->advance_on_poll;
		reply(out, cap, out_len, &state, 1);
		return 0;
	}
	case NINA_CMD_SOCKET_GETSOCKOPT:
		log_cmd("getsockopt");
		b[0] = s->lwip_error & 0xff;
		b[1] = (s->lwip_error >> 8) & 0xff;
		b[2] = 0;
		b[3] = 0;
		reply(out, cap, out_len, b, 4);
		return 0;
	case NINA_CMD_SOCKET_SEND: {
		size_t written = p[1].len;

		log_cmd("send");
		if (s->zero_writes) {
			written = 0;
			if (s->zero_writes > 0) {
				s->zero_writes--;
			}
		} else {
			assert(s->sent_len + written <= sizeof(s->sent));
			memcpy(s->sent + s->sent_len, p[1].data, written);
			s->sent_len += written;
		}
		fake_ms += s->advance_on_send;
		b[0] = written >> 8;
		b[1] = written & 0xff;
		reply(out, cap, out_len, b, 2);
		return 0;
	}
	case NINA_CMD_SOCKET_RECV: {
		size_t size = p[1].data[0] | (p[1].data[1] << 8), n = MIN(size, s->len);

		log_cmd("recv");
		assert(n <= cap);
		memcpy(out, s->data, n);
		memmove(s->data, s->data + n, s->len - n);
		s->len -= n;
		*out_len = n;
		return 0;
	}
	case NINA_CMD_SOCKET_SENDTO:
		log_cmd("sendto");
		assert(nparams == 4);
		assert(s->sent_len + p[3].len <= sizeof(s->sent));
		memcpy(s->sent + s->sent_len, p[3].data, p[3].len);
		s->sent_len += p[3].len;
		b[0] = p[3].len >> 8;
		b[1] = p[3].len & 0xff;
		reply(out, cap, out_len, b, 2);
		return 0;
	case NINA_CMD_SOCKET_RECVFROM: {
		size_t size = p[1].data[0] | (p[1].data[1] << 8), n;
		struct dgram d;

		log_cmd("recvfrom");
		assert(s->ndgram > 0);
		d = s->dgram[0];
		memmove(&s->dgram[0], &s->dgram[1], sizeof(d) * (size_t)(s->ndgram - 1));
		s->ndgram--;
		n = MIN(size, d.len);
		assert(6 + n <= cap);
		memcpy(out, d.ip, 4);
		out[4] = d.port & 0xff;
		out[5] = d.port >> 8;
		memcpy(out + 6, d.data, n);
		*out_len = 6 + n;
		return 0;
	}
	case 0x34:
		log_cmd("dns");
		assert(nparams == 1 && p[0].len < sizeof(dns_name));
		memcpy(dns_name, p[0].data, p[0].len);
		dns_name[p[0].len] = '\0';
		reply(out, cap, out_len, &dns_answer, 1);
		return 0;
	case 0x35:
		log_cmd("dns_result");
		reply(out, cap, out_len, dns_ip, 4);
		return 0;
	default:
		fprintf(stderr, "command 0x%02x\n", cmd);
		assert(0);
		return -EIO;
	}
}

/* Helpers -------------------------------------------------------------------*/

#define CHECK_FAIL(call, err)                                                                      \
	do {                                                                                       \
		errno = 0;                                                                         \
		if ((call) != -1 || errno != (err)) {                                              \
			fprintf(stderr, "%s:%d: %s: errno %d, expected %d\n", __FILE__, __LINE__,  \
				#call, errno, err);                                                \
			assert(0);                                                                 \
		}                                                                                  \
	} while (0)

static struct sockaddr_in addr_of(const char *ip, uint16_t port)
{
	struct sockaddr_in a;

	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons(port);
	assert(inet_pton(AF_INET, ip, &a.sin_addr) == 1);
	return a;
}

/* A connected TCP socket: the handshake done at the first poll. */
static int connected_tcp(void)
{
	struct sockaddr_in a = addr_of("10.42.0.1", 1883);
	int fd = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	assert(fd >= 0);
	assert(zsock_connect(fd, (struct sockaddr *)&a, sizeof(a)) == 0);
	log_expect("socket ioctl connect poll getsockopt");
	sleeps_expect(0, 0);
	return fd;
}

static void close_all(void)
{
	for (int fd = 0; fd < 16; fd++) {
		if (zvfs_get_fd_obj(fd, NULL, 0)) {
			assert(zsock_close(fd) == 0);
		}
	}
	for (int id = 0; id < NINA_SOCK_MAX; id++) {
		assert(!nina_sock_get(id));
	}
	cmd_log[0] = '\0';
	sleeps = 0;
	sleep_total_ms = 0;
	for (int i = 0; i < FW_MAX; i++) {
		assert(!fw[i].open);
	}
}

/* The constants ------------------------------------------------------------*/

static void test_constants(void)
{
	/* Zephyr's values (<zephyr/net/socket.h>, net_ip.h, dns_resolve.h),
	 * except AF_INET (Zephyr 1, Linux and NuttX 2). */
	_Static_assert(AF_INET == 2 && AF_UNSPEC == 0, "family");
	_Static_assert(SOCK_STREAM == 1 && SOCK_DGRAM == 2, "type");
	_Static_assert(IPPROTO_TCP == 6 && IPPROTO_UDP == 17, "protocol");
	_Static_assert(SOL_SOCKET == 1 && SO_RCVTIMEO == 20 && SO_ERROR == 4 &&
			       SO_BINDTODEVICE == 25,
		       "options");
	_Static_assert(TCP_NODELAY == 1, "TCP_NODELAY");
	_Static_assert(MSG_DONTWAIT == 0x40 && ZSOCK_MSG_DONTWAIT == MSG_DONTWAIT, "flags");
	_Static_assert(ZSOCK_POLLIN == 1 && ZSOCK_POLLOUT == 4 && ZSOCK_POLLERR == 8 &&
			       ZSOCK_POLLHUP == 0x10 && ZSOCK_POLLNVAL == 0x20,
		       "poll");
	_Static_assert(SHUT_RD == 0 && SHUT_WR == 1 && SHUT_RDWR == 2, "shutdown");
	_Static_assert(AI_NUMERICHOST == 4 && AI_ADDRCONFIG == 0x20 && AI_NUMERICSERV == 0x400,
		       "ai flags");
	_Static_assert(EAI_BADFLAGS == -1 && EAI_NONAME == -2 && EAI_AGAIN == -3 &&
			       EAI_FAIL == -4 && EAI_FAMILY == -6 && EAI_SOCKTYPE == -7 &&
			       EAI_SERVICE == -8 && EAI_MEMORY == -10,
		       "EAI");
	_Static_assert(INET_ADDRSTRLEN == 16, "INET_ADDRSTRLEN");
	_Static_assert(sizeof(struct sockaddr_in) == 16 && sizeof(struct sockaddr) == 16,
		       "sockaddr");
	assert(NINA_SOCK_MAX == 10 && NINA_POLL_INTERVAL_MS == 20 && NINA_SEND_RETRY_MS == 1 &&
	       NINA_TCP_MAX_IO == 3584);
	printf("ok  constants\n");
}

/* The descriptors ----------------------------------------------------------*/

static void test_descriptors(void)
{
	int fd;

	CHECK_FAIL(zsock_socket(AF_INET6, SOCK_STREAM, 0), EAFNOSUPPORT);
	CHECK_FAIL(zsock_socket(AF_INET, 3, 0), EAFNOSUPPORT);
	CHECK_FAIL(zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_UDP), EAFNOSUPPORT);
	CHECK_FAIL(zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_TCP), EAFNOSUPPORT);

	/* The lowest free number; the firmware socket comes with the first use. */
	for (int i = 0; i < NINA_SOCK_MAX; i++) {
		fd = zsock_socket(AF_INET, i % 2 ? SOCK_DGRAM : SOCK_STREAM, 0);
		assert(fd == i);
		assert(zsock_fcntl(fd, F_GETFL) == 0);
	}
	CHECK_FAIL(zsock_socket(AF_INET, SOCK_STREAM, 0), EMFILE);
	log_expect("");

	assert(zsock_close(3) == 0);
	assert(zsock_close(7) == 0);
	log_expect("");
	CHECK_FAIL(zsock_close(3), EBADF);
	assert(zsock_socket(AF_INET, SOCK_STREAM, 0) == 3);
	assert(zsock_socket(AF_INET, SOCK_DGRAM, 0) == 7);
	CHECK_FAIL(zsock_socket(AF_INET, SOCK_DGRAM, 0), EMFILE);
	close_all();

	/* Closed and out of range: EBADF for every call. */
	for (int i = 0; i < 4; i++) {
		static const int bad[] = {-1, 0, NINA_SOCK_MAX, 100};
		struct sockaddr_in a = addr_of("10.42.0.1", 1);
		socklen_t len = sizeof(a);
		struct msghdr msg = {0};
		char buf[4];
		int one = 1;

		fd = bad[i];
		CHECK_FAIL(zsock_close(fd), EBADF);
		CHECK_FAIL(zsock_bind(fd, (struct sockaddr *)&a, sizeof(a)), EBADF);
		CHECK_FAIL(zsock_connect(fd, (struct sockaddr *)&a, sizeof(a)), EBADF);
		CHECK_FAIL(zsock_listen(fd, 1), EBADF);
		CHECK_FAIL(zsock_accept(fd, NULL, NULL), EBADF);
		CHECK_FAIL(zsock_send(fd, "x", 1, 0), EBADF);
		CHECK_FAIL(zsock_sendto(fd, "x", 1, 0, (struct sockaddr *)&a, sizeof(a)), EBADF);
		CHECK_FAIL(zsock_sendmsg(fd, &msg, 0), EBADF);
		CHECK_FAIL(zsock_recv(fd, buf, sizeof(buf), 0), EBADF);
		CHECK_FAIL(zsock_recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&a, &len),
			   EBADF);
		CHECK_FAIL(zsock_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)),
			   EBADF);
		CHECK_FAIL(zsock_getsockopt(fd, SOL_SOCKET, SO_ERROR, &one, &len), EBADF);
		CHECK_FAIL(zsock_getsockname(fd, (struct sockaddr *)&a, &len), EBADF);
		CHECK_FAIL(zsock_shutdown(fd, SHUT_RDWR), EBADF);
		CHECK_FAIL(zsock_fcntl(fd, F_GETFL), EBADF);
	}
	log_expect("");

	/* A failed close in the module still frees the descriptor. */
	fd = connected_tcp();
	close_reply = 0;
	CHECK_FAIL(zsock_close(fd), EIO);
	close_reply = 1;
	log_expect("close");
	assert(!nina_sock_get(fd));
	assert(zsock_socket(AF_INET, SOCK_STREAM, 0) == fd);
	close_all();
	printf("ok  descriptors\n");
}

/* Options, fcntl and the calls the offload does not have --------------------*/

static void test_options(void)
{
	int tcp = zsock_socket(AF_INET, SOCK_STREAM, 0);
	int udp = zsock_socket(AF_INET, SOCK_DGRAM, 0);
	struct timeval tv = {0};
	socklen_t len = sizeof(int);
	int one = 1;
	char c;

	CHECK_FAIL(zsock_getsockopt(tcp, SOL_SOCKET, SO_ERROR, &one, &len), EOPNOTSUPP);
	CHECK_FAIL(zsock_getsockname(tcp, NULL, NULL), EOPNOTSUPP);
	CHECK_FAIL(zsock_shutdown(tcp, SHUT_RDWR), ENOTSUP);

	assert(zsock_setsockopt(tcp, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0);
	CHECK_FAIL(zsock_setsockopt(tcp, IPPROTO_TCP, TCP_NODELAY, &one, 1), EINVAL);
	CHECK_FAIL(zsock_setsockopt(tcp, IPPROTO_TCP, TCP_NODELAY, NULL, sizeof(one)), EINVAL);
	CHECK_FAIL(zsock_setsockopt(udp, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)), ENOPROTOOPT);
	CHECK_FAIL(zsock_setsockopt(tcp, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)), ENOPROTOOPT);

	CHECK_FAIL(zsock_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv) - 1), EINVAL);
	CHECK_FAIL(zsock_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, NULL, sizeof(tv)), EINVAL);
	tv.tv_sec = -1;
	CHECK_FAIL(zsock_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)), EINVAL);
	tv.tv_sec = 0;
	tv.tv_usec = 1000000;
	CHECK_FAIL(zsock_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)), EINVAL);
	tv.tv_usec = -1;
	CHECK_FAIL(zsock_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)), EINVAL);
	tv.tv_sec = INT32_MAX / 1000 + 1;
	tv.tv_usec = 0;
	CHECK_FAIL(zsock_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)), EDOM);
	tv.tv_sec = 0;
	tv.tv_usec = 1;
	assert(zsock_setsockopt(udp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);

	assert(zsock_fcntl(tcp, F_GETFL) == 0);
	assert(zsock_fcntl(tcp, F_SETFL, ZVFS_O_NONBLOCK) == 0);
	assert(zsock_fcntl(tcp, F_GETFL) == ZVFS_O_NONBLOCK);
	CHECK_FAIL(zsock_fcntl(tcp, F_SETFL, ZVFS_O_NONBLOCK | ZVFS_O_APPEND), EOPNOTSUPP);
	assert(zsock_fcntl(tcp, F_GETFL) == ZVFS_O_NONBLOCK);
	assert(zsock_fcntl(udp, F_GETFL) == 0);
	assert(zsock_fcntl(tcp, F_SETFL, 0) == 0);
	assert(zsock_fcntl(tcp, F_GETFL) == 0);
	CHECK_FAIL(zsock_fcntl(tcp, F_GETFD), ENOTSUP);

	/* The refusals of the core pass as errno, before any command. */
	CHECK_FAIL(zsock_connect(tcp, NULL, 0), EINVAL);
	CHECK_FAIL(zsock_recv(tcp, &c, 1, 0), ENOTCONN);
	CHECK_FAIL(zsock_send(tcp, &c, 1, 0), ENOTCONN);
	CHECK_FAIL(zsock_send(tcp, &c, 1, MSG_PEEK), EOPNOTSUPP);
	CHECK_FAIL(zsock_recvfrom(tcp, &c, 1, 0, NULL, &len), EOPNOTSUPP);
	log_expect("");

	/* A new socket on a number has none of the old one's settings. */
	tv.tv_sec = 5;
	tv.tv_usec = 0;
	assert(zsock_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);
	assert(zsock_fcntl(tcp, F_SETFL, ZVFS_O_NONBLOCK) == 0);
	assert(zsock_close(tcp) == 0);
	assert(zsock_socket(AF_INET, SOCK_STREAM, 0) == tcp);
	assert(zsock_fcntl(tcp, F_GETFL) == 0);
	close_all();
	printf("ok  options\n");
}

/* connect ---------------------------------------------------------------------*/

static void test_connect(void)
{
	struct sockaddr_in a = addr_of("10.42.0.1", 1883);
	struct fw_sock *f;
	char c;
	int fd;

	fd = connected_tcp();
	CHECK_FAIL(zsock_connect(fd, (struct sockaddr *)&a, sizeof(a)), EISCONN);
	close_all();

	/* The handshake takes three more polls: three sleeps of 20 ms.  A
	 * non-blocking socket waits as well, as in the Zephyr build. */
	fd = zsock_socket(AF_INET, SOCK_STREAM, 0);
	assert(zsock_fcntl(fd, F_SETFL, ZVFS_O_NONBLOCK) == 0);
	/* The firmware socket exists after the first command: make it now. */
	assert(zsock_bind(fd, (struct sockaddr *)&(struct sockaddr_in){.sin_family = AF_INET},
			  sizeof(struct sockaddr_in)) == 0);
	log_expect("socket ioctl bind");
	f = fw_of(fd);
	f->connect_polls = 3;
	assert(zsock_connect(fd, (struct sockaddr *)&a, sizeof(a)) == 0);
	log_expect("connect poll poll poll poll getsockopt");
	sleeps_expect(3, 60);
	assert(sleep_last_ms == NINA_POLL_INTERVAL_MS);
	close_all();

	/* Refused: the lwIP error 104 is ECONNRESET; the firmware socket is
	 * closed, the descriptor stays. */
	fd = zsock_socket(AF_INET, SOCK_STREAM, 0);
	assert(zsock_bind(fd, (struct sockaddr *)&(struct sockaddr_in){.sin_family = AF_INET},
			  sizeof(struct sockaddr_in)) == 0);
	f = fw_of(fd);
	f->connect_polls = -1;
	f->error = true;
	f->lwip_error = 104;
	cmd_log[0] = '\0';
	CHECK_FAIL(zsock_connect(fd, (struct sockaddr *)&a, sizeof(a)), ECONNRESET);
	log_expect("connect poll getsockopt close");
	sleeps_expect(0, 0);
	assert(nina_sock_get(fd) && nina_sock_get(fd)->id == NINA_NO_SOCKET);
	CHECK_FAIL(zsock_recv(fd, &c, 1, 0), ENOTCONN);
	close_all();

	/* No answer: ETIMEDOUT after NINA_CONNECT_WAIT_MS.  Each poll moves the
	 * clock by 2500 ms; the deadline is compared with the time taken before
	 * each check (the core's connect pass), so the fifth poll is the last. */
	fd = zsock_socket(AF_INET, SOCK_STREAM, 0);
	assert(zsock_bind(fd, (struct sockaddr *)&(struct sockaddr_in){.sin_family = AF_INET},
			  sizeof(struct sockaddr_in)) == 0);
	f = fw_of(fd);
	f->connect_polls = -1;
	f->advance_on_poll = 2500;
	cmd_log[0] = '\0';
	CHECK_FAIL(zsock_connect(fd, (struct sockaddr *)&a, sizeof(a)), ETIMEDOUT);
	log_expect("connect poll poll poll poll poll close");
	sleeps_expect(4, 80);
	close_all();

	/* UDP: the peer is kept, no wait. */
	fd = zsock_socket(AF_INET, SOCK_DGRAM, 0);
	assert(zsock_connect(fd, (struct sockaddr *)&a, sizeof(a)) == 0);
	log_expect("socket ioctl");
	close_all();
	printf("ok  connect\n");
}

/* send, sendmsg ----------------------------------------------------------------*/

static void test_send(void)
{
	static char big[5000];
	struct iovec iov[3] = {{"abc", 3}, {NULL, 0}, {"defg", 4}};
	struct msghdr msg = {.msg_iov = iov, .msg_iovlen = 3};
	struct sockaddr_in a = addr_of("10.42.0.1", 123);
	struct fw_sock *f;
	int fd;

	/* No room twice: the state check at the first, 1 ms between passes; a
	 * pass that continues polls first and sends again when writable. */
	fd = connected_tcp();
	f = fw_of(fd);
	f->zero_writes = 2;
	assert(zsock_send(fd, "0123456789", 10, 0) == 10);
	log_expect("send poll poll send poll send");
	sleeps_expect(2, 2);
	assert(sleep_last_ms == NINA_SEND_RETRY_MS);
	assert(f->sent_len == 10 && !memcmp(f->sent, "0123456789", 10));

	/* One command carries NINA_TCP_MAX_IO bytes at most. */
	f->sent_len = 0;
	memset(big, 'x', sizeof(big));
	assert(zsock_send(fd, big, sizeof(big), 0) == NINA_TCP_MAX_IO);
	assert(f->sent_len == NINA_TCP_MAX_IO);
	log_expect("send");

	/* Non-blocking (ZVFS_O_NONBLOCK or MSG_DONTWAIT): one command, EAGAIN. */
	f->zero_writes = 1;
	CHECK_FAIL(zsock_send(fd, "a", 1, MSG_DONTWAIT), EAGAIN);
	log_expect("send");
	assert(zsock_fcntl(fd, F_SETFL, ZVFS_O_NONBLOCK) == 0);
	f->zero_writes = 1;
	CHECK_FAIL(zsock_send(fd, "a", 1, 0), EAGAIN);
	f->zero_writes = 1;
	CHECK_FAIL(zsock_sendmsg(fd, &msg, 0), EAGAIN);
	log_expect("send send");
	sleeps_expect(0, 0);
	assert(zsock_fcntl(fd, F_SETFL, 0) == 0);

	/* The deadline: each send moves the clock by 3000 ms; the poll before
	 * each later pass (writable), and the state check after each send
	 * (100 ms gone by since the last); EAGAIN at the fourth. */
	f->zero_writes = -1;
	f->advance_on_send = 3000;
	CHECK_FAIL(zsock_send(fd, "a", 1, 0), EAGAIN);
	log_expect("send poll poll send poll poll send poll poll send poll");
	sleeps_expect(3, 3);
	f->advance_on_send = 0;

	/* The connection failed: EPIPE at the state check. */
	f->error = true;
	CHECK_FAIL(zsock_send(fd, "a", 1, 0), EPIPE);
	log_expect("send poll");
	f->error = false;
	f->zero_writes = 0;

	/* sendmsg: the vectors joined into one command. */
	f->sent_len = 0;
	assert(zsock_sendmsg(fd, &msg, 0) == 7);
	assert(f->sent_len == 7 && !memcmp(f->sent, "abcdefg", 7));
	log_expect("send");
	CHECK_FAIL(zsock_sendmsg(fd, NULL, 0), EINVAL);
	msg.msg_iov = NULL;
	CHECK_FAIL(zsock_sendmsg(fd, &msg, 0), EINVAL);
	msg.msg_iov = iov;
	msg.msg_name = &a;
	CHECK_FAIL(zsock_sendmsg(fd, &msg, 0), EOPNOTSUPP);
	msg.msg_name = NULL;
	msg.msg_controllen = 1;
	CHECK_FAIL(zsock_sendmsg(fd, &msg, 0), EOPNOTSUPP);
	msg.msg_controllen = 0;
	CHECK_FAIL(zsock_sendto(fd, "a", 1, 0, (struct sockaddr *)&a, sizeof(a)), EOPNOTSUPP);
	log_expect("");
	close_all();

	/* Datagrams. */
	fd = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	CHECK_FAIL(zsock_send(fd, "a", 1, 0), EDESTADDRREQ);
	CHECK_FAIL(zsock_sendto(fd, big, NINA_MAX_IO + 1, 0, (struct sockaddr *)&a, sizeof(a)),
		   EMSGSIZE);
	CHECK_FAIL(zsock_sendmsg(fd, &msg, 0), EOPNOTSUPP);
	log_expect("");
	assert(zsock_sendto(fd, "ntp", 3, 0, (struct sockaddr *)&a, sizeof(a)) == 3);
	log_expect("socket ioctl sendto");
	assert(fw_of(fd)->sent_len == 3);
	close_all();
	printf("ok  send\n");
}

/* recv, recvfrom ---------------------------------------------------------------*/

static void test_recv(void)
{
	struct sockaddr_in from;
	struct zsock_pollfd pfd;
	struct timeval tv = {0, 100000};
	socklen_t len;
	struct fw_sock *f;
	char buf[64];
	int fd;

	/* MSG_DONTWAIT: one poll, EAGAIN, no sleep. */
	fd = connected_tcp();
	f = fw_of(fd);
	CHECK_FAIL(zsock_recv(fd, buf, sizeof(buf), MSG_DONTWAIT), EAGAIN);
	log_expect("poll");
	sleeps_expect(0, 0);

	/* Blocking: a poll every 20 ms until the data is there (the fourth). */
	f->later = "hello";
	f->later_polls = 3;
	assert(zsock_recv(fd, buf, sizeof(buf), 0) == 5 && !memcmp(buf, "hello", 5));
	log_expect("poll poll poll poll recv");
	sleeps_expect(3, 60);
	assert(sleep_last_ms == NINA_POLL_INTERVAL_MS);

	/* ZVFS_O_NONBLOCK: one poll, EAGAIN, no sleep. */
	assert(zsock_fcntl(fd, F_SETFL, ZVFS_O_NONBLOCK) == 0);
	CHECK_FAIL(zsock_recv(fd, buf, sizeof(buf), 0), EAGAIN);
	log_expect("poll");
	sleeps_expect(0, 0);
	assert(zsock_fcntl(fd, F_SETFL, 0) == 0);

	/* SO_RCVTIMEO 100 ms: polls at 0, 20, .., 100 ms, then EAGAIN. */
	assert(zsock_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);
	CHECK_FAIL(zsock_recv(fd, buf, sizeof(buf), 0), EAGAIN);
	log_expect("poll poll poll poll poll poll");
	sleeps_expect(5, 100);

	/* A poll that found data does not stand for the receive's own poll
	 * (the Zephyr build polls on every pass). */
	memcpy(f->data, "abc", 3);
	f->len = 3;
	pfd.fd = fd;
	pfd.events = ZSOCK_POLLIN;
	assert(zsock_poll(&pfd, 1, 0) == 1 && pfd.revents == ZSOCK_POLLIN);
	assert(zsock_recv(fd, buf, 2, 0) == 2 && !memcmp(buf, "ab", 2));
	log_expect("poll poll recv");

	/* A TCP socket refuses an address. */
	len = sizeof(from);
	CHECK_FAIL(zsock_recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &len),
		   EOPNOTSUPP);
	log_expect("");

	/* The end of the stream: readable, nothing to read. */
	f->len = 0;
	f->eof = true;
	assert(zsock_recv(fd, buf, sizeof(buf), 0) == 0);
	log_expect("poll recv");
	close_all();

	/* Datagrams: the sender's address, cut to the caller's length. */
	fd = zsock_socket(AF_INET, SOCK_DGRAM, 0);
	assert(zsock_bind(fd,
			  (struct sockaddr *)&(struct sockaddr_in){.sin_family = AF_INET,
								   .sin_port = htons(4000)},
			  sizeof(struct sockaddr_in)) == 0);
	log_expect("socket ioctl bind");
	f = fw_of(fd);
	f->dgram[0] = (struct dgram){{10, 0, 0, 5}, 4242, "xyz", 3};
	f->dgram[1] = (struct dgram){{10, 0, 0, 6}, 4343, "uvw", 3};
	f->ndgram = 2;
	memset(&from, 0xa5, sizeof(from));
	len = sizeof(from);
	assert(zsock_recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &len) == 3);
	assert(!memcmp(buf, "xyz", 3) && len == sizeof(from));
	assert(from.sin_family == AF_INET && from.sin_port == htons(4242) &&
	       from.sin_addr.s_addr == addr_of("10.0.0.5", 0).sin_addr.s_addr);
	memset(&from, 0xa5, sizeof(from));
	len = 4;
	assert(zsock_recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &len) == 3);
	assert(len == sizeof(from) && from.sin_family == AF_INET && from.sin_port == htons(4343) &&
	       ((uint8_t *)&from)[4] == 0xa5);
	log_expect("poll recvfrom poll recvfrom");
	len = sizeof(from);
	CHECK_FAIL(zsock_recvfrom(fd, buf, sizeof(buf), 0, NULL, &len), EOPNOTSUPP);
	CHECK_FAIL(zsock_recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, NULL),
		   EOPNOTSUPP);
	log_expect("");

	/* Connected: a datagram from another sender is dropped in the pass. */
	{
		struct sockaddr_in peer = addr_of("10.0.0.6", 4343);

		assert(zsock_connect(fd, (struct sockaddr *)&peer, sizeof(peer)) == 0);
		f->dgram[0] = (struct dgram){{10, 0, 0, 5}, 4242, "no", 2};
		f->dgram[1] = (struct dgram){{10, 0, 0, 6}, 4343, "yes", 3};
		f->ndgram = 2;
		assert(zsock_recv(fd, buf, sizeof(buf), MSG_DONTWAIT) == 3);
		assert(!memcmp(buf, "yes", 3));
		log_expect("poll recvfrom poll recvfrom");
	}
	close_all();
	printf("ok  recv\n");
}

/* poll -------------------------------------------------------------------------*/

static void test_poll(void)
{
	struct zsock_pollfd p[6];
	struct fw_sock *f;
	int tcp, udp, fd;

	/* Without firmware sockets: no command. */
	tcp = zsock_socket(AF_INET, SOCK_STREAM, 0);
	udp = zsock_socket(AF_INET, SOCK_DGRAM, 0);
	p[0] = (struct zsock_pollfd){.fd = tcp, .events = ZSOCK_POLLIN, .revents = 7};
	p[1] = (struct zsock_pollfd){.fd = udp, .events = ZSOCK_POLLIN, .revents = 7};
	p[2] = (struct zsock_pollfd){.fd = udp, .events = ZSOCK_POLLOUT};
	p[3] = (struct zsock_pollfd){.fd = -1, .events = ZSOCK_POLLIN, .revents = 7};
	p[4] = (struct zsock_pollfd){.fd = 9, .events = ZSOCK_POLLIN};
	p[5] = (struct zsock_pollfd){.fd = 100, .events = 0};
	assert(zsock_poll(p, 6, -1) == 4);
	assert(p[0].revents == ZSOCK_POLLHUP && p[1].revents == 0 &&
	       p[2].revents == ZSOCK_POLLOUT && p[3].revents == 0 &&
	       p[4].revents == ZSOCK_POLLNVAL && p[5].revents == ZSOCK_POLLNVAL);
	log_expect("");
	sleeps_expect(0, 0);
	close_all();

	/* What was asked for, POLLERR always. */
	fd = connected_tcp();
	f = fw_of(fd);
	memcpy(f->data, "x", 1);
	f->len = 1;
	p[0] = (struct zsock_pollfd){.fd = fd, .events = ZSOCK_POLLOUT};
	assert(zsock_poll(p, 1, 0) == 1 && p[0].revents == ZSOCK_POLLOUT);
	p[0].events = ZSOCK_POLLIN | ZSOCK_POLLOUT;
	assert(zsock_poll(p, 1, 0) == 1 && p[0].revents == (ZSOCK_POLLIN | ZSOCK_POLLOUT));
	f->error = true;
	p[0].events = ZSOCK_POLLIN;
	assert(zsock_poll(p, 1, 0) == 1 && p[0].revents == (ZSOCK_POLLIN | ZSOCK_POLLERR));
	f->error = false;
	f->len = 0;
	log_expect("poll poll poll");

	/* The timeout: a pass every 20 ms, 0 when 50 ms have gone by. */
	assert(zsock_poll(p, 1, 50) == 0 && p[0].revents == 0);
	log_expect("poll poll poll poll");
	sleeps_expect(3, 60);

	/* Data at the third pass. */
	f->later = "y";
	f->later_polls = 2;
	assert(zsock_poll(p, 1, 1000) == 1 && p[0].revents == ZSOCK_POLLIN);
	log_expect("poll poll poll");
	sleeps_expect(2, 40);

	/* A failed poll of the module is POLLERR on that socket, at once; the
	 * call does not fail (the Zephyr build's binding). */
	f->poll_fail = true;
	p[0].events = ZSOCK_POLLIN;
	assert(zsock_poll(p, 1, 1000) == 1 && p[0].revents == ZSOCK_POLLERR);
	log_expect("poll");
	sleeps_expect(0, 0);
	f->poll_fail = false;
	close_all();
	printf("ok  poll\n");
}

/* accept -----------------------------------------------------------------------*/

static void test_accept(void)
{
	struct sockaddr_in any = {.sin_family = AF_INET, .sin_port = htons(23)};
	struct sockaddr_in peer, bad = addr_of("10.42.0.97", 23);
	socklen_t len;
	struct fw_sock *f;
	int listener, client;

	listener = zsock_socket(AF_INET, SOCK_STREAM, 0);
	CHECK_FAIL(zsock_bind(listener, (struct sockaddr *)&bad, sizeof(bad)), EADDRNOTAVAIL);
	CHECK_FAIL(zsock_accept(listener, NULL, NULL), EINVAL);
	assert(zsock_bind(listener, (struct sockaddr *)&any, sizeof(any)) == 0);
	assert(zsock_listen(listener, 1) == 0);
	log_expect("socket ioctl bind listen");
	assert(zsock_fcntl(listener, F_SETFL, ZVFS_O_NONBLOCK) == 0);

	len = sizeof(peer);
	CHECK_FAIL(zsock_accept(listener, (struct sockaddr *)&peer, NULL), EINVAL);
	CHECK_FAIL(zsock_accept(listener, NULL, &len), EINVAL);
	log_expect("");

	/* Nobody there: EAGAIN at once, blocking or not. */
	CHECK_FAIL(zsock_accept(listener, NULL, NULL), EAGAIN);
	assert(zsock_fcntl(listener, F_SETFL, 0) == 0);
	CHECK_FAIL(zsock_accept(listener, NULL, NULL), EAGAIN);
	log_expect("accept accept");
	sleeps_expect(0, 0);

	f = fw_of(listener);
	f->pending = 1;
	memcpy(f->peer_ip, (uint8_t[]){10, 42, 0, 1}, 4);
	f->peer_port = 51000;
	assert(zsock_fcntl(listener, F_SETFL, ZVFS_O_NONBLOCK) == 0);
	memset(&peer, 0, sizeof(peer));
	len = sizeof(peer);
	client = zsock_accept(listener, (struct sockaddr *)&peer, &len);
	assert(client == listener + 1);
	assert(len == sizeof(peer) && peer.sin_family == AF_INET && peer.sin_port == htons(51000) &&
	       peer.sin_addr.s_addr == addr_of("10.42.0.1", 0).sin_addr.s_addr);
	assert(zsock_fcntl(client, F_GETFL) == 0);
	log_expect("accept ioctl");

	/* The address cut to the caller's length. */
	f->pending = 1;
	memset(&peer, 0xa5, sizeof(peer));
	len = 2;
	assert(zsock_accept(listener, (struct sockaddr *)&peer, &len) == listener + 2);
	assert(len == sizeof(peer) && peer.sin_family == AF_INET && ((uint8_t *)&peer)[2] == 0xa5);
	log_expect("accept ioctl");
	close_all();
	printf("ok  accept\n");
}

/* The descriptor's lock --------------------------------------------------------*/

static volatile int helper_state;
static volatile int helper_errno;

static void receiver(void *arg)
{
	int fd = (int)(intptr_t)arg;
	char c;

	helper_state = 1;
	if (zsock_recv(fd, &c, 1, 0) == -1) {
		helper_errno = errno;
	}
	helper_state = 2;
	vTaskDelete(NULL);
}

/* A receive of 200 ms (ten passes) on FD in another task, under way. */
static void start_receiver(int fd)
{
	UBaseType_t mine = uxTaskPriorityGet(NULL);

	helper_state = 0;
	helper_errno = 0;
	assert(xTaskCreate(receiver, "receiver", HELPER_STACK / sizeof(StackType_t),
			   (void *)(intptr_t)fd, mine, NULL) == pdPASS);
	while (helper_state == 0) {
		k_msleep(1);
	}
	k_msleep(30);
	assert(helper_state == 1);
}

static void test_lock(void)
{
	struct timeval tv = {0, 200000};
	struct zsock_pollfd p;
	int fd = connected_tcp();

	assert(zsock_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);

	/* A poll waits for the receive (zvfs_poll() takes the descriptor's
	 * lock for its POLL_PREPARE), then polls. */
	start_receiver(fd);
	p = (struct zsock_pollfd){.fd = fd, .events = ZSOCK_POLLIN};
	assert(zsock_poll(&p, 1, 0) == 0);
	assert(helper_state == 2 && helper_errno == EAGAIN);
	cmd_log[0] = '\0';

	/* A close waits for the receive to end, then closes. */
	start_receiver(fd);
	assert(zsock_close(fd) == 0);
	assert(helper_state == 2 && helper_errno == EAGAIN);
	assert(!nina_sock_get(fd));
	assert(strstr(cmd_log, "close") && !strstr(strstr(cmd_log, "close"), "poll"));
	close_all();
	printf("ok  lock\n");
}

/* Name resolution ----------------------------------------------------------------*/

static void test_dns(void)
{
	struct zsock_addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_DGRAM};
	struct zsock_addrinfo *res = NULL, *all[64];
	struct sockaddr_in *in;
	int n, again;

	/* Before the binding registers its resolver (nina_dns_init(), from
	 * the interface's init): the layer has none. */
	assert(zsock_getaddrinfo("10.1.2.3", "1883", NULL, &res) == EAI_FAIL && !res);
	nina_dns_init();

	/* A numeric address: no command. */
	assert(zsock_getaddrinfo("10.1.2.3", "1883", NULL, &res) == 0 && res);
	in = (struct sockaddr_in *)res->ai_addr;
	assert(res->ai_family == AF_INET && res->ai_socktype == SOCK_STREAM &&
	       res->ai_protocol == IPPROTO_TCP && res->ai_addrlen == sizeof(*in));
	assert(in->sin_port == htons(1883) &&
	       in->sin_addr.s_addr == addr_of("10.1.2.3", 0).sin_addr.s_addr);
	zsock_freeaddrinfo(res);
	log_expect("");

	/* A name: the module's two commands. */
	res = NULL;
	assert(zsock_getaddrinfo("pool.ntp.org", NULL, &hints, &res) == 0 && res);
	in = (struct sockaddr_in *)res->ai_addr;
	assert(res->ai_socktype == SOCK_DGRAM && res->ai_protocol == IPPROTO_UDP);
	assert(in->sin_port == 0 && !memcmp(&in->sin_addr, dns_ip, 4));
	assert(!strcmp(dns_name, "pool.ntp.org"));
	zsock_freeaddrinfo(res);
	log_expect("dns dns_result");

	/* Not found; numeric only; bad flags: Zephyr's DNS_EAI_* values. */
	dns_answer = 0;
	assert(zsock_getaddrinfo("nowhere.invalid", NULL, NULL, &res) == EAI_NONAME && !res);
	dns_answer = 1;
	log_expect("dns");
	hints.ai_flags = AI_NUMERICHOST;
	assert(zsock_getaddrinfo("pool.ntp.org", NULL, &hints, &res) == -2);
	hints.ai_flags = AI_PASSIVE;
	assert(zsock_getaddrinfo("pool.ntp.org", NULL, &hints, &res) == -1);
	hints.ai_flags = 0;
	assert(zsock_getaddrinfo("10.1.2.3", "x", &hints, &res) == -8);
	log_expect("");

	/* The results come from the binding's 2048-byte k_heap: EAI_MEMORY
	 * when it is full, all of it back after they are freed. */
	for (n = 0; n < (int)ARRAY_SIZE(all); n++) {
		int ret = zsock_getaddrinfo("10.1.2.3", "1883", NULL, &all[n]);

		if (ret == EAI_MEMORY) {
			break;
		}
		assert(ret == 0);
	}
	assert(n > 10 && n < (int)ARRAY_SIZE(all) && n * 64 <= 2048);
	for (int i = 0; i < n; i++) {
		zsock_freeaddrinfo(all[i]);
	}
	for (again = 0; again < (int)ARRAY_SIZE(all); again++) {
		if (zsock_getaddrinfo("10.1.2.3", "1883", NULL, &all[again]) != 0) {
			break;
		}
	}
	assert(again == n);
	for (int i = 0; i < again; i++) {
		zsock_freeaddrinfo(all[i]);
	}
	log_expect("");
	printf("ok  dns\n");
}

static void runner(void *arg)
{
	(void)arg;
	assert(nina_wait_start_events() == -ENOTSUP && !nina_wait_events());
	test_constants();
	test_descriptors();
	test_options();
	test_connect();
	test_send();
	test_recv();
	test_poll();
	test_accept();
	test_lock();
	test_dns();
	fflush(stdout);
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL,
			APP_PRIO_FROM_ZEPHYR(9), NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
