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
 *
 * The data calls (nina_socket.c:374-659): TCP send (the 3584-byte limit,
 * the answer checks, MSG_DONTWAIT, the passes of a waiting send with its
 * 100 ms state checks and 10000 ms deadline, a new call against a
 * continued one), sendmsg (vectors joined into one command, its refusals),
 * UDP send (address checks, the connected peer, the 512-byte limit, a
 * datagram not taken), TCP and UDP receive (poll first, the size
 * parameter, the sender's port byte order, the connected peer's filter,
 * an empty datagram), the poll finding that stands for the next receive's
 * first poll, and FIONREAD.
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

#define MAX_STEPS 32
#define MAX_PARAM NINA_TCP_MAX_IO

struct step {
	uint8_t cmd;
	int ret;
	uint8_t reply[6 + NINA_MAX_IO + 8];
	size_t reply_len;
};

static struct step script[MAX_STEPS];
static size_t nsteps, next_step;
static uint8_t params[MAX_STEPS][6][MAX_PARAM];
static size_t param_len[MAX_STEPS][6], nparams_seen[MAX_STEPS], caps[MAX_STEPS];

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
	/* The data commands use 16-bit lengths for the data they carry
	 * (parameters of send, the reply of receive); the others 8-bit
	 * lengths both ways. */
	assert(wide_params == (cmd == NINA_CMD_SEND || cmd == NINA_CMD_SENDTO));
	assert(wide_response == (cmd == NINA_CMD_RECV || cmd == NINA_CMD_RECVFROM));
	nparams_seen[next_step] = nparams;
	caps[next_step] = cap;
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

/* Data calls ----------------------------------------------------------------*/

/* A connected TCP socket with firmware socket ID (as after accept). */
static int tcp_connected(uint8_t id)
{
	int s = nina_sock_create(AF_INET, SOCK_STREAM, 0);

	assert(s >= 0);
	g_nina_sockets[s].id = id;
	g_nina_sockets[s].connected = true;
	return s;
}

static void expect_sent(unsigned int n)
{
	expect(NINA_CMD_SEND, 0, (const uint8_t[]){n >> 8, n & 0xff}, 2);
}

static uint8_t data[4096];

static void test_send_tcp(void)
{
	bool retry = true;
	int s;

	for (size_t i = 0; i < sizeof(data); i++) {
		data[i] = (uint8_t)(i * 7 + 1);
	}

	/* Refusals before the module is asked. */
	reset_script();
	assert(nina_sock_send(9, data, 1, 0, NULL, 0, &retry) == -EBADF && !retry);
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	assert(nina_sock_send(s, data, 1, 0, NULL, 0, &retry) == -ENOTCONN);
	close_all();
	s = tcp_connected(3);
	{
		struct sockaddr_in to = in4("10.42.0.1", 80);

		assert(nina_sock_send(s, data, 1, 0, (struct sockaddr *)&to, sizeof(to),
				      &retry) == -EOPNOTSUPP);
	}
	assert(nina_sock_send(s, data, 1, MSG_OOB, NULL, 0, &retry) == -EOPNOTSUPP);
	assert(nina_sock_send(s, data, 1, MSG_PEEK, NULL, 0, &retry) == -EOPNOTSUPP);
	assert(nina_sock_send(s, data, 0, 0, NULL, 0, &retry) == 0);
	assert(nina_sock_send(s, NULL, 5, 0, NULL, 0, &retry) == -EINVAL);
	done();

	/* One command: the id, then the data with a 16-bit length. */
	reset_script();
	expect_sent(100);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == 100 && !retry);
	done();
	param1(0, 0, 3);
	param_is(0, 1, data, 100);
	/* At most NINA_TCP_MAX_IO bytes per call; a partial write is returned. */
	reset_script();
	expect_sent(NINA_TCP_MAX_IO);
	expect_sent(1000);
	assert(nina_sock_send(s, data, 4000, 0, NULL, 0, &retry) == NINA_TCP_MAX_IO);
	assert(nina_sock_send(s, data, 4000, MSG_DONTWAIT, NULL, 0, &retry) == 1000);
	done();
	param_is(0, 1, data, NINA_TCP_MAX_IO);
	param_is(1, 1, data, NINA_TCP_MAX_IO);
	/* Bad answers: more than asked, not two bytes; an SPI error. */
	reset_script();
	expect_sent(101);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == -EPROTO);
	expect(NINA_CMD_SEND, 0, (const uint8_t[]){0}, 1);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == -EPROTO);
	expect(NINA_CMD_SEND, -ETIMEDOUT, NULL, 0);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == -ETIMEDOUT);
	done();

	/* Nothing sent, the call does not wait: EAGAIN at once, no poll. */
	reset_script();
	expect_sent(0);
	assert(nina_sock_send(s, data, 100, MSG_DONTWAIT, NULL, 0, &retry) == -EAGAIN);
	assert(!retry && !g_nina_sockets[s].send_waiting);
	done();
	assert(nina_sock_watch(&g_nina_sockets[s]) ==
	       (NINA_SOCK_EV_READ | NINA_SOCK_EV_WRITE));

	/* Nothing sent, the call waits: the state is checked at the first
	 * pass and then every 100 ms; the next pass after NINA_SEND_RETRY_MS. */
	reset_script();
	expect_sent(0);
	expect1(NINA_CMD_POLL, NINA_POLL_WRITE);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == -EAGAIN && retry);
	done();
	param1(1, 0, 3);
	assert(g_nina_sockets[s].send_waiting);
	assert(g_nina_sockets[s].send_deadline == now_ms + NINA_SEND_WAIT_MS);
	assert(g_nina_sockets[s].send_next_state_check == now_ms + NINA_SEND_STATE_INTERVAL_MS);
	/* Not polled for room while it waits: its passes are timed. */
	assert(nina_sock_watch(&g_nina_sockets[s]) == NINA_SOCK_EV_READ);
	now_ms += 1;
	reset_script();
	expect_sent(0);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == -EAGAIN && retry);
	now_ms += 98;
	expect_sent(0);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == -EAGAIN && retry);
	now_ms += 1;
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == -EAGAIN && retry);
	done();
	assert(g_nina_sockets[s].send_deadline == now_ms - 100 + NINA_SEND_WAIT_MS);
	/* Room again: the count, the wait over. */
	reset_script();
	expect_sent(60);
	assert(nina_sock_send(s, data, 100, 0, NULL, 0, &retry) == 60 && !retry);
	done();
	assert(!g_nina_sockets[s].send_waiting);

	/* The deadline: EAGAIN without another pass (10000 ms after the first). */
	reset_script();
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_send(s, data, 50, 0, NULL, 0, &retry) == -EAGAIN && retry);
	now_ms += NINA_SEND_WAIT_MS - 1;
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_send(s, data, 50, 0, NULL, 0, &retry) == -EAGAIN && retry);
	now_ms += 1;
	expect_sent(0);
	assert(nina_sock_send(s, data, 50, 0, NULL, 0, &retry) == -EAGAIN && !retry);
	done();
	assert(!g_nina_sockets[s].send_waiting);

	/* A new call (other data) starts its own deadline and state check. */
	reset_script();
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_send(s, data, 50, 0, NULL, 0, &retry) == -EAGAIN && retry);
	now_ms += 5000;
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_send(s, data + 1, 50, 0, NULL, 0, &retry) == -EAGAIN && retry);
	done();
	assert(g_nina_sockets[s].send_deadline == now_ms + NINA_SEND_WAIT_MS);

	/* The error flag: EPIPE; a failed poll: its error. */
	reset_script();
	expect_sent(0);
	expect1(NINA_CMD_POLL, NINA_POLL_ERROR | NINA_POLL_WRITE);
	assert(nina_sock_send(s, data, 20, 0, NULL, 0, &retry) == -EPIPE && !retry);
	expect_sent(0);
	expect1(NINA_CMD_POLL, NINA_POLL_FAIL);
	assert(nina_sock_send(s, data, 20, 0, NULL, 0, &retry) == -EIO);
	expect_sent(0);
	expect(NINA_CMD_POLL, -ETIMEDOUT, NULL, 0);
	assert(nina_sock_send(s, data, 20, 0, NULL, 0, &retry) == -ETIMEDOUT);
	done();
	assert(!g_nina_sockets[s].send_waiting);

	/* Closing ends a wait. */
	reset_script();
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_send(s, data, 20, 0, NULL, 0, &retry) == -EAGAIN && retry);
	done();
	close_all();
	assert(!g_nina_sockets[s].send_waiting);
}

static void test_sendmsg(void)
{
	uint8_t head[5] = {0x30, 0x80, 0x13, 0, 7};
	struct iovec iov[3];
	bool retry;
	int s, u;

	s = tcp_connected(4);
	iov[0] = (struct iovec){head, sizeof(head)};
	iov[1] = (struct iovec){data, 2448};
	iov[2] = (struct iovec){data + 3000, 10};

	/* All vectors in one command. */
	reset_script();
	expect_sent(5 + 2448 + 10);
	assert(nina_sock_sendmsg(s, iov, 3, false, 0, &retry) == 5 + 2448 + 10);
	done();
	assert(param_len[0][1] == 5 + 2448 + 10);
	assert(!memcmp(params[0][1], head, 5));
	assert(!memcmp(params[0][1] + 5, data, 2448));
	assert(!memcmp(params[0][1] + 5 + 2448, data + 3000, 10));

	/* Up to NINA_TCP_MAX_IO bytes; the caller advances by what was sent. */
	iov[1].iov_len = 4000;
	reset_script();
	expect_sent(NINA_TCP_MAX_IO);
	assert(nina_sock_sendmsg(s, iov, 3, false, 0, &retry) == NINA_TCP_MAX_IO);
	done();
	assert(param_len[0][1] == NINA_TCP_MAX_IO);
	assert(!memcmp(params[0][1] + 5, data, NINA_TCP_MAX_IO - 5));

	/* A waiting sendmsg continues with the same vectors. */
	iov[1].iov_len = 100;
	reset_script();
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_sendmsg(s, iov, 3, false, 0, &retry) == -EAGAIN && retry);
	now_ms += NINA_SEND_STATE_INTERVAL_MS;
	expect_sent(0);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_sendmsg(s, iov, 3, false, 0, &retry) == -EAGAIN && retry);
	done();
	assert(g_nina_sockets[s].send_deadline ==
	       now_ms - NINA_SEND_STATE_INTERVAL_MS + NINA_SEND_WAIT_MS);
	reset_script();
	expect_sent(115);
	assert(nina_sock_sendmsg(s, iov, 3, false, 0, &retry) == 115 && !retry);
	done();

	/* Refusals: an address, a vector without data, nothing to send. */
	reset_script();
	assert(nina_sock_sendmsg(s, iov, 3, true, 0, &retry) == -EOPNOTSUPP);
	iov[1].iov_base = NULL;
	assert(nina_sock_sendmsg(s, iov, 3, false, 0, &retry) == -EFAULT);
	iov[1].iov_len = 0;
	expect_sent(15);
	assert(nina_sock_sendmsg(s, iov, 3, false, 0, &retry) == 15);
	assert(nina_sock_sendmsg(s, iov, 0, false, 0, &retry) == 0);
	assert(nina_sock_sendmsg(s, NULL, 1, false, 0, &retry) == -EINVAL);
	assert(nina_sock_sendmsg(9, iov, 1, false, 0, &retry) == -EBADF);
	u = nina_sock_create(AF_INET, SOCK_DGRAM, 0);
	assert(nina_sock_sendmsg(u, iov, 1, false, 0, &retry) == -EOPNOTSUPP);
	done();
	close_all();
}

static void test_send_udp(void)
{
	struct sockaddr_in to = in4("10.42.0.1", 123), peer = in4("10.42.0.1", 1883);
	bool retry = true;
	int u;

	u = nina_sock_create(AF_INET, SOCK_DGRAM, 0);
	reset_script();
	assert(nina_sock_send(u, data, 48, 0, NULL, 0, &retry) == -EDESTADDRREQ);
	assert(nina_sock_send(u, data, NINA_MAX_IO + 1, 0, (struct sockaddr *)&to, sizeof(to),
			      &retry) == -EMSGSIZE);
	assert(nina_sock_send(u, data, NINA_MAX_IO + 1, MSG_OOB, (struct sockaddr *)&to,
			      sizeof(to), &retry) == -EMSGSIZE);
	assert(nina_sock_send(u, data, 48, MSG_OOB, (struct sockaddr *)&to, sizeof(to),
			      &retry) == -EOPNOTSUPP);
	assert(nina_sock_send(u, NULL, 48, 0, (struct sockaddr *)&to, sizeof(to), &retry) ==
	       -EINVAL);
	assert(nina_sock_send(u, data, 48, 0, (struct sockaddr *)&to, sizeof(to) - 1, &retry) ==
	       -EINVAL);
	to.sin_family = AF_UNIX;
	assert(nina_sock_send(u, data, 48, 0, (struct sockaddr *)&to, sizeof(to), &retry) ==
	       -EINVAL);
	to.sin_family = AF_INET;
	done();

	/* First use opens the firmware socket; id, address, port, data. */
	reset_script();
	expect_open(2);
	expect(NINA_CMD_SENDTO, 0, (const uint8_t[]){0, 48}, 2);
	assert(nina_sock_send(u, data, 48, 0, (struct sockaddr *)&to, sizeof(to), &retry) == 48);
	assert(!retry);
	done();
	check_open(0, true, 2);
	param1(2, 0, 2);
	param_is(2, 1, &to.sin_addr, 4);
	param_is(2, 2, &to.sin_port, 2);
	param_is(2, 3, data, 48);
	/* The full 512 bytes, MSG_DONTWAIT accepted. */
	reset_script();
	expect(NINA_CMD_SENDTO, 0, (const uint8_t[]){NINA_MAX_IO >> 8, NINA_MAX_IO & 0xff}, 2);
	assert(nina_sock_send(u, data, NINA_MAX_IO, MSG_DONTWAIT, (struct sockaddr *)&to,
			      sizeof(to), &retry) == NINA_MAX_IO);
	/* Not taken (any failure is zero bytes): EAGAIN, never a wait. */
	expect(NINA_CMD_SENDTO, 0, (const uint8_t[]){0, 0}, 2);
	assert(nina_sock_send(u, data, 48, 0, (struct sockaddr *)&to, sizeof(to), &retry) ==
	       -EAGAIN && !retry);
	expect(NINA_CMD_SENDTO, 0, (const uint8_t[]){0}, 1);
	assert(nina_sock_send(u, data, 48, 0, (struct sockaddr *)&to, sizeof(to), &retry) ==
	       -EPROTO);
	expect(NINA_CMD_SENDTO, -ETIMEDOUT, NULL, 0);
	assert(nina_sock_send(u, data, 48, 0, (struct sockaddr *)&to, sizeof(to), &retry) ==
	       -ETIMEDOUT);
	/* An empty datagram. */
	expect(NINA_CMD_SENDTO, 0, (const uint8_t[]){0, 0}, 2);
	assert(nina_sock_send(u, NULL, 0, 0, (struct sockaddr *)&to, sizeof(to), &retry) == 0);
	done();
	assert(param_len[4][3] == 0);

	/* Connected: the peer when no address is given. */
	reset_script();
	assert(nina_sock_connect(u, (struct sockaddr *)&peer, sizeof(peer)) == 0);
	expect(NINA_CMD_SENDTO, 0, (const uint8_t[]){0, 4}, 2);
	assert(nina_sock_send(u, data, 4, 0, NULL, 0, &retry) == 4);
	done();
	param_is(0, 1, &peer.sin_addr, 4);
	param_is(0, 2, &peer.sin_port, 2);
	close_all();
}

static void expect_received(const uint8_t *bytes, size_t n)
{
	expect(NINA_CMD_RECV, 0, bytes, n);
}

static void test_recv_tcp(void)
{
	struct sockaddr_in from;
	uint8_t buf[600];
	int s, t;

	reset_script();
	assert(nina_sock_recv(9, buf, 10, 0, NULL) == -EBADF);
	t = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	assert(nina_sock_recv(t, buf, 10, 0, NULL) == -ENOTCONN);
	close_all();
	s = tcp_connected(5);
	assert(nina_sock_recv(s, buf, 10, 0, &from) == -EOPNOTSUPP);
	assert(nina_sock_recv(s, buf, 10, MSG_PEEK, NULL) == -EOPNOTSUPP);
	assert(nina_sock_recv(s, buf, 0, 0, NULL) == 0);
	assert(nina_sock_recv(s, NULL, 10, 0, NULL) == -EINVAL);
	done();

	/* Poll first: nothing there is EAGAIN (the caller waits or not). */
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_WRITE);
	assert(nina_sock_recv(s, buf, 10, 0, NULL) == -EAGAIN);
	expect1(NINA_CMD_POLL, NINA_POLL_FAIL);
	assert(nina_sock_recv(s, buf, 10, MSG_DONTWAIT, NULL) == -EIO);
	/* Data: the size as two bytes, low first; at most 512. */
	expect1(NINA_CMD_POLL, NINA_POLL_READ | NINA_POLL_WRITE);
	expect_received(data, 7);
	assert(nina_sock_recv(s, buf, 600, 0, NULL) == 7);
	assert(!memcmp(buf, data, 7));
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_received(data, 300);
	assert(nina_sock_recv(s, buf, 300, 0, NULL) == 300);
	done();
	param1(3, 0, 5);
	param_is(3, 1, (const uint8_t[]){0x00, 0x02}, 2);
	assert(caps[3] == NINA_MAX_IO);
	param_is(5, 1, (const uint8_t[]){0x2c, 0x01}, 2);
	assert(caps[5] == 300);
	/* Readable with nothing to read: the end of the stream (0). */
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_received(NULL, 0);
	assert(nina_sock_recv(s, buf, 10, 0, NULL) == 0);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect(NINA_CMD_RECV, -ETIMEDOUT, NULL, 0);
	assert(nina_sock_recv(s, buf, 10, 0, NULL) == -ETIMEDOUT);
	done();

	/* A poll that found data (the event thread's, FIONREAD) stands for the
	 * next receive's first poll, once. */
	reset_script();
	{
		uint8_t state;

		expect1(NINA_CMD_POLL, NINA_POLL_READ);
		assert(nina_sock_poll_state(&g_nina_sockets[s], &state) == 0);
		assert(g_nina_sockets[s].read_hint);
		expect_received(data, 3);
		assert(nina_sock_recv(s, buf, 10, 0, NULL) == 3);
		assert(!g_nina_sockets[s].read_hint);
		expect1(NINA_CMD_POLL, 0);
		assert(nina_sock_recv(s, buf, 10, 0, NULL) == -EAGAIN);
		expect1(NINA_CMD_POLL, NINA_POLL_READ);
		assert(nina_sock_fionread(s) == 1);
		expect_received(data, 2);
		assert(nina_sock_recv(s, buf, 10, 0, NULL) == 2);
		/* A poll without data clears it. */
		expect1(NINA_CMD_POLL, NINA_POLL_READ);
		assert(nina_sock_fionread(s) == 1);
		expect1(NINA_CMD_POLL, 0);
		assert(nina_sock_poll_state(&g_nina_sockets[s], &state) == 0);
		expect1(NINA_CMD_POLL, 0);
		assert(nina_sock_recv(s, buf, 10, 0, NULL) == -EAGAIN);
	}
	done();
	close_all();
}

/* A datagram answer: address, port (its bytes as the firmware sends them),
 * then the data. */
static void expect_datagram(const char *ip, uint16_t port, const uint8_t *bytes, size_t n)
{
	uint8_t reply[6 + 64];
	struct in_addr a;

	assert(n <= 64);
	assert(inet_pton(AF_INET, ip, &a) == 1);
	memcpy(reply, &a, 4);
	reply[4] = port >> 8;
	reply[5] = port & 0xff;
	if (n) {
		memcpy(reply + 6, bytes, n);
	}
	expect(NINA_CMD_RECVFROM, 0, reply, 6 + n);
}

static void test_recv_udp(void)
{
	struct sockaddr_in from, peer = in4("10.42.0.1", 123);
	uint8_t buf[600];
	int u;

	u = nina_sock_create(AF_INET, SOCK_DGRAM, 0);
	reset_script();
	assert(nina_sock_recv(u, buf, 10, MSG_PEEK, NULL) == -EOPNOTSUPP);
	assert(nina_sock_recv(u, NULL, 10, 0, NULL) == -EINVAL);
	done();
	/* First use opens the firmware socket; then poll first. */
	reset_script();
	expect_open(6);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_recv(u, buf, 48, 0, NULL) == -EAGAIN);
	done();
	/* A datagram: its sender (port bytes as in the accept reply), the
	 * data; the size parameter is the room asked for, at most 512. */
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_datagram("10.42.0.1", 0x007b, data, 48);
	memset(&from, 0xaa, sizeof(from));
	assert(nina_sock_recv(u, buf, 600, 0, &from) == 48);
	done();
	param1(1, 0, 6);
	param_is(1, 1, (const uint8_t[]){0x00, 0x02}, 2);
	assert(caps[1] == 6 + NINA_MAX_IO);
	assert(!memcmp(buf, data, 48));
	assert(from.sin_family == AF_INET && from.sin_addr.s_addr == inet_addr("10.42.0.1"));
	assert(ntohs(from.sin_port) == 0x7b00);
	/* A short buffer gets the start of the datagram; zero room asks for 1
	 * byte, and an empty datagram is 0. */
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_datagram("10.42.0.1", 123, data, 10);
	assert(nina_sock_recv(u, buf, 10, 0, NULL) == 10);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_datagram("10.42.0.1", 123, data, 1);
	assert(nina_sock_recv(u, buf, 0, 0, NULL) == 0);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_datagram("10.42.0.1", 123, NULL, 0);
	assert(nina_sock_recv(u, buf, 48, 0, NULL) == 0);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect(NINA_CMD_RECVFROM, 0, data, 5);
	assert(nina_sock_recv(u, buf, 48, 0, NULL) == -EPROTO);
	done();
	param_is(1, 1, (const uint8_t[]){10, 0}, 2);
	assert(caps[1] == 16);
	param_is(3, 1, (const uint8_t[]){1, 0}, 2);
	assert(caps[3] == 7);

	/* Connected: datagrams from others are dropped and the loop polls
	 * again; the peer's comes through. */
	reset_script();
	assert(nina_sock_connect(u, (struct sockaddr *)&peer, sizeof(peer)) == 0);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	/* peer.sin_port in the firmware's byte order: 123 in network order. */
	expect_datagram("10.42.0.9", 0x7b00, data, 4);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_datagram("10.42.0.1", 0x7c00, data, 4);
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_recv(u, buf, 48, MSG_DONTWAIT, NULL) == -EAGAIN);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	expect_datagram("10.42.0.1", 0x7b00, data + 100, 48);
	assert(nina_sock_recv(u, buf, 48, 0, &from) == 48);
	done();
	assert(!memcmp(buf, data + 100, 48));
	assert(from.sin_port == htons(123));

	/* FIONREAD's finding stands for the first poll: no poll before the
	 * datagram (the time synchronization's 1 ms polling). */
	reset_script();
	expect1(NINA_CMD_POLL, NINA_POLL_READ | NINA_POLL_WRITE);
	assert(nina_sock_fionread(u) == 1);
	expect_datagram("10.42.0.1", 0x7b00, data, 48);
	assert(nina_sock_recv(u, buf, 48, MSG_DONTWAIT, NULL) == 48);
	expect1(NINA_CMD_POLL, NINA_POLL_WRITE);
	assert(nina_sock_fionread(u) == 0);
	done();
	close_all();
}

static void test_fionread(void)
{
	int s, u;

	reset_script();
	assert(nina_sock_fionread(9) == -EBADF);
	s = nina_sock_create(AF_INET, SOCK_STREAM, 0);
	assert(nina_sock_fionread(s) == -ENOTCONN);
	/* No firmware socket yet: nothing can be there; the module is not
	 * asked and no socket is made. */
	u = nina_sock_create(AF_INET, SOCK_DGRAM, 0);
	assert(nina_sock_fionread(u) == 0);
	assert(g_nina_sockets[u].id == NINA_NO_SOCKET);
	done();
	close_all();
	s = tcp_connected(7);
	reset_script();
	expect1(NINA_CMD_POLL, 0);
	assert(nina_sock_fionread(s) == 0);
	expect1(NINA_CMD_POLL, NINA_POLL_READ);
	assert(nina_sock_fionread(s) == 1);
	expect1(NINA_CMD_POLL, NINA_POLL_FAIL | NINA_POLL_READ);
	assert(nina_sock_fionread(s) == -EIO);
	expect(NINA_CMD_POLL, -ETIMEDOUT, NULL, 0);
	assert(nina_sock_fionread(s) == -ETIMEDOUT);
	done();
	param1(0, 0, 7);
	close_all();
}

int main(void)
{
	assert(NINA_SOCK_MAX == 10 && NINA_MAX_IO == 512 && NINA_TCP_MAX_IO == 3584);
	assert(NINA_CONNECT_WAIT_MS == 10000 && NINA_POLL_INTERVAL_MS == 20);
	assert(NINA_SEND_WAIT_MS == 10000 && NINA_SEND_STATE_INTERVAL_MS == 100);
	assert(NINA_SEND_RETRY_MS == 1);
	test_create();
	test_bind_listen();
	test_accept();
	test_connect();
	test_close();
	test_setsockopt();
	test_watch_events();
	test_send_tcp();
	test_sendmsg();
	test_send_udp();
	test_recv_tcp();
	test_recv_udp();
	test_fionread();
	puts("nina_sock: ok");
	return 0;
}
