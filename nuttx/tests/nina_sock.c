/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of nuttx/drivers/nina/nina_sock.c (the socket table and the
 * socket commands of the Zephyr build's drivers/nina/nina_socket.c) with a
 * fake nina_request() that checks each command and its parameters and
 * gives scripted answers, and a fake clock.
 *
 * Covers: which sockets are supported, the table of 10, the lazy firmware
 * socket (type and protocol, FIONBIO), bind, listen (backlog clamp),
 * accept (peer address and port byte order, -EAGAIN, -EMFILE), connect
 * (UDP; TCP in progress, each SO_ERROR mapping, the error flag, the
 * 10000 ms deadline, a failed poll; the firmware socket closed after a
 * failure), close, setsockopt, and what the event thread polls for and
 * reports (nina_sock_watch(), nina_sock_events()).
 */

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include "nina_os.h"
#include "nina_proto.h"
#include "nina_sock.h"

/* Fake clock --------------------------------------------------------------*/

static int64_t now_ms = 1000;

int64_t nina_os_uptime_ms(void)
{
	return now_ms;
}

/* Fake module -------------------------------------------------------------*/

struct step {
	uint8_t cmd;
	int ret;
	uint8_t reply[8];
	size_t reply_len;
};

static struct step script[32];
static size_t nsteps, next_step;
static uint8_t params[32][6][8];
static size_t param_len[32][6], nparams_seen[32];

static void expect(uint8_t cmd, int ret, const uint8_t *reply, size_t len)
{
	assert(nsteps < sizeof(script) / sizeof(script[0]));
	script[nsteps].cmd = cmd;
	script[nsteps].ret = ret;
	if (len) {
		memcpy(script[nsteps].reply, reply, len);
	}
	script[nsteps].reply_len = len;
	nsteps++;
}

static void expect1(uint8_t cmd, uint8_t value)
{
	expect(cmd, 0, &value, 1);
}

static void reset_script(void)
{
	nsteps = next_step = 0;
	memset(params, 0, sizeof(params));
	memset(param_len, 0, sizeof(param_len));
	memset(nparams_seen, 0, sizeof(nparams_seen));
}

static void done(void)
{
	assert(next_step == nsteps);
}

int nina_request(uint8_t cmd, const struct nina_param *p, size_t nparams, bool wide_params,
		 bool wide_response, uint8_t *out, size_t cap, size_t *out_len)
{
	struct step *s;

	assert(next_step < nsteps);
	s = &script[next_step];
	if (s->cmd != cmd) {
		fprintf(stderr, "step %zu: command 0x%02x, expected 0x%02x\n", next_step, cmd,
			s->cmd);
		assert(0);
	}
	/* The control commands use 8-bit lengths both ways. */
	assert(!wide_params && !wide_response);
	nparams_seen[next_step] = nparams;
	for (size_t i = 0; i < nparams; i++) {
		assert(p[i].len <= sizeof(params[0][0]));
		memcpy(params[next_step][i], p[i].data, p[i].len);
		param_len[next_step][i] = p[i].len;
	}
	next_step++;
	if (s->ret) {
		return s->ret;
	}
	assert(s->reply_len <= cap);
	memcpy(out, s->reply, s->reply_len);
	*out_len = s->reply_len;
	return 0;
}

/* Checks parameter I of step STEP. */
static void param_is(size_t step, size_t i, const void *data, size_t len)
{
	assert(i < nparams_seen[step]);
	assert(param_len[step][i] == len);
	assert(!memcmp(params[step][i], data, len));
}

static void param1(size_t step, size_t i, uint8_t value)
{
	param_is(step, i, &value, 1);
}

/* The two commands of a first use: socket (type, protocol) and FIONBIO. */
static void expect_open(uint8_t id)
{
	expect1(NINA_CMD_SOCKET, id);
	expect(NINA_CMD_IOCTL, 0, (const uint8_t[]){0, 0, 0, 0}, 4);
}

static void check_open(size_t step, bool udp, uint8_t id)
{
	static const uint8_t fionbio[4] = {0x7e, 0x66, 0x04, 0x80}, one[4] = {1, 0, 0, 0};

	param1(step, 0, udp ? 2 : 1);
	param1(step, 1, udp ? 17 : 6);
	param1(step + 1, 0, id);
	param_is(step + 1, 1, fionbio, 4);
	param_is(step + 1, 2, one, 4);
}

static struct sockaddr_in in4(const char *ip, uint16_t port)
{
	struct sockaddr_in a;

	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons(port);
	assert(inet_pton(AF_INET, ip, &a.sin_addr) == 1);
	return a;
}

static void close_all(void)
{
	for (int i = 0; i < NINA_SOCK_MAX; i++) {
		if (g_nina_sockets[i].used) {
			reset_script();
			if (g_nina_sockets[i].id != NINA_NO_SOCKET) {
				expect1(NINA_CMD_CLOSE, 1);
			}
			assert(nina_sock_close(i) == 0);
			done();
		}
	}
}

/* Tests -------------------------------------------------------------------*/

static void test_create(void)
{
	int ids[NINA_SOCK_MAX];

	reset_script();
	assert(nina_sock_supported(AF_INET, SOCK_STREAM, 0));
	assert(nina_sock_supported(AF_INET, SOCK_STREAM, IPPROTO_TCP));
	assert(nina_sock_supported(AF_INET, SOCK_DGRAM, 0));
	assert(nina_sock_supported(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
	assert(!nina_sock_supported(AF_INET, SOCK_STREAM, IPPROTO_UDP));
	assert(!nina_sock_supported(AF_INET, SOCK_DGRAM, IPPROTO_TCP));
	assert(!nina_sock_supported(AF_INET, SOCK_RAW, 0));
	assert(!nina_sock_supported(AF_INET, SOCK_DGRAM, IPPROTO_ICMP));
	assert(!nina_sock_supported(AF_INET6, SOCK_STREAM, 0));
	assert(nina_sock_create(AF_INET6, SOCK_STREAM, 0) == -EAFNOSUPPORT);
	assert(nina_sock_create(AF_INET, SOCK_RAW, 0) == -EAFNOSUPPORT);

	/* Ten entries, numbered from 0; no command to the module. */
	for (int i = 0; i < NINA_SOCK_MAX; i++) {
		ids[i] = nina_sock_create(AF_INET, i % 2 ? SOCK_DGRAM : SOCK_STREAM, 0);
		assert(ids[i] == i);
		assert(g_nina_sockets[i].id == NINA_NO_SOCKET);
		assert(g_nina_sockets[i].udp == (i % 2));
		assert(nina_sock_get(i) == &g_nina_sockets[i]);
	}
	assert(nina_sock_create(AF_INET, SOCK_STREAM, 0) == -EMFILE);
	done();
	assert(nina_sock_get(-1) == NULL && nina_sock_get(NINA_SOCK_MAX) == NULL);

	/* A closed entry is reused, cleared. */
	assert(nina_sock_close(3) == 0);
	assert(nina_sock_get(3) == NULL);
	assert(nina_sock_close(3) == -EBADF);
	g_nina_sockets[3].reported = 0xff;
	g_nina_sockets[3].connecting = true;
	assert(nina_sock_create(AF_INET, SOCK_STREAM, 0) == 3);
	assert(!g_nina_sockets[3].reported && !g_nina_sockets[3].connecting);
	done();
	close_all();
}

static void test_bind_listen(void)
{
	struct sockaddr_in any = in4("0.0.0.0", 23), some = in4("10.42.0.2", 23);
	uint16_t port = htons(23);
	int s = nina_sock_create(AF_INET, SOCK_STREAM, 0), u;

	reset_script();
	assert(nina_sock_bind(s, NULL, sizeof(any)) == -EINVAL);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any) - 1) == -EINVAL);
	any.sin_family = AF_INET6;
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == -EINVAL);
	any.sin_family = AF_INET;
	assert(nina_sock_bind(s, (struct sockaddr *)&some, sizeof(some)) == -EADDRNOTAVAIL);
	assert(nina_sock_listen(s, 5) == -EINVAL); /* not bound */
	done();

	/* First use: socket, FIONBIO, then bind with the port's two bytes. */
	reset_script();
	expect_open(4);
	expect1(NINA_CMD_BIND, 1);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == 0);
	done();
	check_open(0, false, 4);
	param1(2, 0, 4);
	param_is(2, 1, &port, 2);
	assert(g_nina_sockets[s].bound && g_nina_sockets[s].id == 4);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == -EINVAL);

	/* Listen: backlog clamped to 1..255, answer 1. */
	reset_script();
	expect1(NINA_CMD_LISTEN, 0);
	assert(nina_sock_listen(s, 0) == -EIO);
	expect1(NINA_CMD_LISTEN, 1);
	assert(nina_sock_listen(s, 0) == 0);
	done();
	param1(0, 0, 4);
	param1(0, 1, 1);
	param1(1, 1, 1);
	assert(g_nina_sockets[s].listening);
	assert(nina_sock_listen(s, 5) == -EINVAL);
	close_all();

	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	reset_script();
	expect_open(2);
	expect1(NINA_CMD_BIND, 1);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == 0);
	expect1(NINA_CMD_LISTEN, 1);
	assert(nina_sock_listen(s, 1000) == 0);
	done();
	param1(3, 1, 255);
	close_all();

	/* A UDP socket binds but does not listen. Bind failure: EADDRINUSE. */
	u = nina_sock_create(AF_INET, SOCK_DGRAM, 0);
	reset_script();
	expect_open(7);
	expect1(NINA_CMD_BIND, 0);
	assert(nina_sock_bind(u, (struct sockaddr *)&any, sizeof(any)) == -EADDRINUSE);
	done();
	check_open(0, true, 7);
	assert(!g_nina_sockets[u].bound && g_nina_sockets[u].id == 7);
	reset_script();
	expect1(NINA_CMD_BIND, 1);
	assert(nina_sock_bind(u, (struct sockaddr *)&any, sizeof(any)) == 0);
	assert(nina_sock_listen(u, 1) == -EINVAL);
	done();
	close_all();

	/* No firmware socket: ENFILE; a failed FIONBIO closes it again. */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	reset_script();
	expect1(NINA_CMD_SOCKET, NINA_NO_SOCKET);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == -ENFILE);
	expect(NINA_CMD_SOCKET, -ETIMEDOUT, NULL, 0);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == -ETIMEDOUT);
	expect1(NINA_CMD_SOCKET, 5);
	expect(NINA_CMD_IOCTL, 0, (const uint8_t[]){0, 0}, 2);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == -EIO);
	done();
	param1(4, 0, 5);
	assert(g_nina_sockets[s].id == NINA_NO_SOCKET && !g_nina_sockets[s].bound);
	close_all();
}

static int listening_socket(uint8_t id)
{
	struct sockaddr_in any = in4("0.0.0.0", 23);
	int s = nina_sock_create(AF_INET, SOCK_STREAM, 0);

	reset_script();
	expect_open(id);
	expect1(NINA_CMD_BIND, 1);
	expect1(NINA_CMD_LISTEN, 1);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == 0);
	assert(nina_sock_listen(s, 5) == 0);
	done();
	return s;
}

static void test_accept(void)
{
	/* reply: id, address, then the port's bytes swapped (see the source). */
	const uint8_t reply[7] = {6, 10, 42, 0, 1, 0xd4, 0x31};
	struct sockaddr_in peer;
	int s = nina_sock_create(AF_INET, SOCK_STREAM, 0), l, c;

	reset_script();
	assert(nina_sock_accept(s, &peer) == -EINVAL);
	assert(nina_sock_accept(9, &peer) == -EBADF);
	done();
	close_all();

	l = listening_socket(3);
	reset_script();
	expect(NINA_CMD_ACCEPT, 0, (const uint8_t[]){NINA_NO_SOCKET, 0, 0, 0, 0, 0, 0}, 7);
	assert(nina_sock_accept(l, &peer) == -EAGAIN);
	expect(NINA_CMD_ACCEPT, 0, reply, 6);
	assert(nina_sock_accept(l, &peer) == -EAGAIN);
	expect(NINA_CMD_ACCEPT, -EPROTO, NULL, 0);
	assert(nina_sock_accept(l, &peer) == -EPROTO);
	expect(NINA_CMD_ACCEPT, 0, reply, 7);
	expect(NINA_CMD_IOCTL, 0, (const uint8_t[]){0, 0, 0, 0}, 4);
	c = nina_sock_accept(l, &peer);
	done();
	param1(0, 0, 3);
	param1(4, 0, 6);
	assert(c == 1);
	assert(g_nina_sockets[c].used && g_nina_sockets[c].connected && g_nina_sockets[c].id == 6);
	assert(!g_nina_sockets[c].udp && !g_nina_sockets[c].reported);
	assert(peer.sin_family == AF_INET);
	assert(peer.sin_addr.s_addr == inet_addr("10.42.0.1"));
	/* The firmware's bytes 0xd4 0x31: port 0x31d4 = 12756. */
	assert(ntohs(peer.sin_port) == 0x31d4);

	/* FIONBIO fails: the new socket is closed and its entry freed. */
	reset_script();
	expect(NINA_CMD_ACCEPT, 0, reply, 7);
	expect(NINA_CMD_IOCTL, -ETIMEDOUT, NULL, 0);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_accept(l, &peer) == -ETIMEDOUT);
	done();
	assert(!g_nina_sockets[2].used);

	/* Table full: -EMFILE before asking the module. */
	for (int i = 2; i < NINA_SOCK_MAX; i++) {
		assert(nina_sock_create(AF_INET, SOCK_DGRAM, 0) == i);
	}
	reset_script();
	assert(nina_sock_accept(l, &peer) == -EMFILE);
	done();
	close_all();
}

static void connect_started(int s, uint8_t id, uint8_t first_state)
{
	struct sockaddr_in to = in4("10.42.0.1", 1883);

	reset_script();
	expect_open(id);
	expect1(NINA_CMD_CONNECT, 0);
	expect1(NINA_CMD_POLL, first_state);
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == 1);
	done();
	check_open(0, false, id);
	param1(2, 0, id);
	param_is(2, 1, &to.sin_addr, 4);
	param_is(2, 2, &to.sin_port, 2);
	param1(3, 0, id);
	assert(g_nina_sockets[s].connecting && !g_nina_sockets[s].connected);
	assert(g_nina_sockets[s].connect_deadline == now_ms + NINA_CONNECT_WAIT_MS);
	assert(nina_sock_watch(&g_nina_sockets[s]) == NINA_SOCK_WATCH_CONNECT);
}

/* SO_ERROR answer: little-endian int. */
static void expect_so_error(int value)
{
	uint8_t v[4] = {value & 0xff, (value >> 8) & 0xff, (value >> 16) & 0xff,
			(value >> 24) & 0xff};

	expect(NINA_CMD_GETSOCKOPT, 0, v, 4);
}

static void check_so_error_request(size_t step, uint8_t id)
{
	static const uint8_t name[4] = {0x07, 0x10, 0, 0};

	param1(step, 0, id);
	param_is(step, 1, name, 4);
	param1(step, 2, 4);
}

static void test_connect(void)
{
	struct sockaddr_in to = in4("10.42.0.1", 1883);
	static const struct {
		int lwip;
		int err;
	} map[] = {{104, ECONNRESET}, {113, ECONNABORTED}, {116, ETIMEDOUT},
		   {118, EHOSTUNREACH}, {111, ECONNREFUSED}, {1, ECONNREFUSED}};
	int s, u;

	/* Arguments. */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	reset_script();
	assert(nina_sock_connect(s, NULL, sizeof(to)) == -EINVAL);
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to) - 1) == -EINVAL);
	to.sin_family = AF_UNIX;
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == -EINVAL);
	to.sin_family = AF_INET;
	assert(nina_sock_connect_check(s, now_ms) == -EINVAL);
	done();

	/* Connected at the first check: writable and SO_ERROR 0. */
	reset_script();
	expect_open(1);
	expect1(NINA_CMD_CONNECT, 0);
	expect1(NINA_CMD_POLL, NINA_POLL_WRITE);
	expect_so_error(0);
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == 0);
	done();
	check_so_error_request(4, 1);
	assert(g_nina_sockets[s].connected && !g_nina_sockets[s].connecting);
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == -EISCONN);
	close_all();

	/* In progress, then connected on a later pass. */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	connect_started(s, 2, 0);
	reset_script();
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_connect_check(s, now_ms + 20) == 1);
	expect1(NINA_CMD_POLL, NINA_POLL_WRITE | NINA_POLL_READ);
	expect_so_error(0);
	assert(nina_sock_connect_check(s, now_ms + 40) == 0);
	done();
	assert(g_nina_sockets[s].connected && !g_nina_sockets[s].connecting);
	assert(g_nina_sockets[s].id == 2);
	close_all();

	/* Each SO_ERROR value: the errno, the firmware socket closed. */
	for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
		s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
		connect_started(s, 3, 0);
		reset_script();
		expect1(NINA_CMD_POLL, NINA_POLL_WRITE);
		expect_so_error(map[i].lwip);
		expect1(NINA_CMD_CLOSE, 1);
		assert(nina_sock_connect_check(s, now_ms + 20) == -map[i].err);
		done();
		param1(2, 0, 3);
		assert(g_nina_sockets[s].used && g_nina_sockets[s].id == NINA_NO_SOCKET);
		assert(!g_nina_sockets[s].connecting && !g_nina_sockets[s].connected);
		assert(nina_sock_watch(&g_nina_sockets[s]) == 0);
		close_all();
	}

	/* The error flag with SO_ERROR 0: ECONNREFUSED. A failed SO_ERROR read. */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	connect_started(s, 3, 0);
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_ERROR);
	expect_so_error(0);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect_check(s, now_ms + 20) == -ECONNREFUSED);
	done();
	/* The socket connects again with a new firmware socket. */
	connect_started(s, 4, 0);
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_ERROR);
	expect(NINA_CMD_GETSOCKOPT, 0, (const uint8_t[]){0, 0}, 2);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect_check(s, now_ms + 20) == -EIO);
	done();
	close_all();

	/* A failure keeps the usrsock request to answer (connect_xid). */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	connect_started(s, 5, 0);
	g_nina_sockets[s].connect_xid = 1234;
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_ERROR);
	expect_so_error(116);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect_check(s, now_ms + 20) == -ETIMEDOUT);
	done();
	assert(g_nina_sockets[s].connect_xid == 1234 && !g_nina_sockets[s].connecting);
	close_all();
	assert(g_nina_sockets[s].connect_xid == 0);

	/* Deadline: 10000 ms after the connect command; the poll comes first. */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	connect_started(s, 5, 0);
	reset_script();
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_connect_check(s, now_ms + NINA_CONNECT_WAIT_MS - 1) == 1);
	expect1(NINA_CMD_POLL, NINA_POLL_WRITE);
	expect_so_error(0);
	assert(nina_sock_connect_check(s, now_ms + NINA_CONNECT_WAIT_MS) == 0);
	done();
	close_all();
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	connect_started(s, 5, 0);
	reset_script();
	expect1(NINA_CMD_POLL, 0);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect_check(s, now_ms + NINA_CONNECT_WAIT_MS) == -ETIMEDOUT);
	done();
	assert(g_nina_sockets[s].id == NINA_NO_SOCKET);

	/* A failed poll (no answer byte, or the firmware's failure bit). */
	connect_started(s, 6, 0);
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_FAIL | NINA_POLL_WRITE);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect_check(s, now_ms + 20) == -EIO);
	done();
	reset_script();
	expect_open(6);
	expect1(NINA_CMD_CONNECT, 0);
	expect(NINA_CMD_POLL, 0, NULL, 0);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == -EIO);
	done();

	/* The connect command: no answer byte is EPROTO, an SPI error is
	 * returned; both close the firmware socket. */
	reset_script();
	expect_open(7);
	expect(NINA_CMD_CONNECT, 0, NULL, 0);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == -EPROTO);
	expect_open(7);
	expect(NINA_CMD_CONNECT, -ETIMEDOUT, NULL, 0);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == -ETIMEDOUT);
	done();
	close_all();

	/* A listening socket does not connect. */
	s = listening_socket(1);
	reset_script();
	assert(nina_sock_connect(s, (struct sockaddr *)&to, sizeof(to)) == -EINVAL);
	done();
	close_all();

	/* UDP: the firmware socket and the peer, no connect command; again. */
	u = nina_sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	reset_script();
	expect_open(8);
	assert(nina_sock_connect(u, (struct sockaddr *)&to, sizeof(to)) == 0);
	done();
	check_open(0, true, 8);
	assert(g_nina_sockets[u].connected && !g_nina_sockets[u].connecting);
	assert(!memcmp(&g_nina_sockets[u].peer, &to, sizeof(to)));
	to = in4("10.42.0.1", 123);
	reset_script();
	assert(nina_sock_connect(u, (struct sockaddr *)&to, sizeof(to)) == 0);
	done();
	assert(g_nina_sockets[u].peer.sin_port == htons(123));
	close_all();
}

static void test_close(void)
{
	struct sockaddr_in any = in4("0.0.0.0", 5001);
	int s = nina_sock_create(AF_INET, SOCK_STREAM, 0);

	reset_script();
	expect_open(9);
	expect1(NINA_CMD_BIND, 1);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == 0);
	expect1(NINA_CMD_CLOSE, 0);
	assert(nina_sock_close(s) == -EIO);
	done();
	param1(3, 0, 9);
	/* Freed even so. */
	assert(nina_sock_get(s) == NULL && g_nina_sockets[s].id == NINA_NO_SOCKET);

	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	reset_script();
	expect_open(9);
	expect1(NINA_CMD_BIND, 1);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == 0);
	expect(NINA_CMD_CLOSE, -ETIMEDOUT, NULL, 0);
	assert(nina_sock_close(s) == -ETIMEDOUT);
	done();
	assert(nina_sock_get(s) == NULL);
}

static void test_setsockopt(void)
{
	int one = 1, s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	int u = nina_sock_create(AF_INET, SOCK_DGRAM, 0);
	char c = 1;

	reset_script();
	assert(nina_sock_setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0);
	assert(nina_sock_setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &c, 1) == -EINVAL);
	assert(nina_sock_setsockopt(s, IPPROTO_TCP, TCP_NODELAY, NULL, sizeof(one)) == -EINVAL);
	assert(nina_sock_setsockopt(u, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) ==
	       -ENOPROTOOPT);
	assert(nina_sock_setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) ==
	       -ENOPROTOOPT);
	assert(nina_sock_setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one)) ==
	       -ENOPROTOOPT);
	assert(nina_sock_setsockopt(s, IPPROTO_IP, IP_TOS, &one, sizeof(one)) == -ENOPROTOOPT);
	done();
	close_all();
}

static void test_watch_events(void)
{
	struct sockaddr_in any = in4("0.0.0.0", 123);
	struct nina_sock *t, *u, *l;
	int s;

	/* No firmware socket: nothing to poll. */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	t = nina_sock_get(s);
	assert(nina_sock_watch(t) == 0);
	assert(nina_sock_events(t, 0, 0xff) == 0);
	close_all();

	/* A listening socket: a connection is READ; errors are not reported. */
	l = nina_sock_get(listening_socket(2));
	assert(nina_sock_watch(l) == NINA_SOCK_EV_READ);
	assert(nina_sock_events(l, 0, 0) == 0);
	assert(nina_sock_events(l, 0, NINA_POLL_READ | NINA_POLL_WRITE) == NINA_SOCK_EV_READ);
	assert(nina_sock_events(l, 0, NINA_POLL_ERROR) == 0);
	assert(nina_sock_events(l, -EIO, 0) == 0);
	l->reported = NINA_SOCK_EV_READ;
	assert(nina_sock_watch(l) == 0);
	assert(nina_sock_events(l, 0, NINA_POLL_READ) == 0);
	close_all();

	/* A connected TCP socket: READ, WRITE; an error or a failed poll is
	 * CLOSED, after which nothing more is polled. */
	assert(listening_socket(2) == 0);
	reset_script();
	expect(NINA_CMD_ACCEPT, 0, (const uint8_t[]){5, 10, 42, 0, 1, 0, 80}, 7);
	expect(NINA_CMD_IOCTL, 0, (const uint8_t[]){0, 0, 0, 0}, 4);
	{
		struct sockaddr_in peer;

		s = nina_sock_accept(0, &peer);
	}
	done();
	t = nina_sock_get(s);
	assert(nina_sock_watch(t) == (NINA_SOCK_EV_READ | NINA_SOCK_EV_WRITE));
	t->reported = NINA_SOCK_EV_WRITE;
	assert(nina_sock_watch(t) == NINA_SOCK_EV_READ);
	assert(nina_sock_events(t, 0, NINA_POLL_WRITE) == 0);
	assert(nina_sock_events(t, 0, NINA_POLL_READ | NINA_POLL_WRITE) == NINA_SOCK_EV_READ);
	assert(nina_sock_events(t, 0, NINA_POLL_READ | NINA_POLL_ERROR) ==
	       (NINA_SOCK_EV_READ | NINA_SOCK_EV_CLOSED));
	assert(nina_sock_events(t, -EIO, 0) == NINA_SOCK_EV_CLOSED);
	assert(nina_sock_events(t, -ETIMEDOUT, 0) == NINA_SOCK_EV_CLOSED);
	t->reported = NINA_SOCK_EV_READ | NINA_SOCK_EV_WRITE;
	assert(nina_sock_watch(t) == 0);
	assert(nina_sock_events(t, 0, NINA_POLL_ERROR) == 0);
	t->reported = NINA_SOCK_EV_CLOSED;
	assert(nina_sock_watch(t) == 0);
	assert(nina_sock_events(t, -EIO, NINA_POLL_READ) == 0);
	close_all();

	/* A UDP socket with a firmware socket: READ only; errors ignored. */
	s = nina_sock_create(AF_INET, SOCK_DGRAM, 0);
	u = nina_sock_get(s);
	assert(nina_sock_watch(u) == 0);
	reset_script();
	expect_open(1);
	expect1(NINA_CMD_BIND, 1);
	assert(nina_sock_bind(s, (struct sockaddr *)&any, sizeof(any)) == 0);
	done();
	u->reported = NINA_SOCK_EV_WRITE;
	assert(nina_sock_watch(u) == NINA_SOCK_EV_READ);
	assert(nina_sock_events(u, 0, NINA_POLL_WRITE | NINA_POLL_ERROR) == 0);
	assert(nina_sock_events(u, -EIO, 0) == 0);
	assert(nina_sock_events(u, 0, NINA_POLL_READ) == NINA_SOCK_EV_READ);

	/* Closing the firmware socket (failed connect) keeps what usrsock holds. */
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	connect_started(s, 3, 0);
	assert(nina_sock_events(nina_sock_get(s), 0, NINA_POLL_WRITE) == 0);
	g_nina_sockets[s].reported = NINA_SOCK_EV_WRITE;
	reset_script();
	expect1(NINA_CMD_POLL, 0);
	expect1(NINA_CMD_CLOSE, 1);
	assert(nina_sock_connect_check(s, now_ms + NINA_CONNECT_WAIT_MS) == -ETIMEDOUT);
	done();
	assert(g_nina_sockets[s].reported == NINA_SOCK_EV_WRITE);
	close_all();
}

int main(void)
{
	assert(NINA_SOCK_MAX == 10 && NINA_MAX_IO == 512 && NINA_TCP_MAX_IO == 3584);
	assert(NINA_CONNECT_WAIT_MS == 10000 && NINA_POLL_INTERVAL_MS == 20);
	assert(NINA_SEND_WAIT_MS == 10000 && NINA_SEND_STATE_INTERVAL_MS == 100);
	test_create();
	test_bind_listen();
	test_accept();
	test_connect();
	test_close();
	test_setsockopt();
	test_watch_events();
	puts("nina_sock: ok");
	return 0;
}
