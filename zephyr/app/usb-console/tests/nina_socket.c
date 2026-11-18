/* SPDX-License-Identifier: MIT */
/* Host test of this build's socket offload (drivers/nina/nina_socket.c) on
 * the shared core (common/drivers/nina/nina_sock.c, nina_dns.c), as
 * CMakeLists.txt builds them (NINA_OS_DNS_OFFLOAD), on fakes of the Zephyr
 * calls (tests/socket_fakes: the descriptor table, mutexes, the clock and
 * sleeps) and of the module (nina_request() here: the firmware's sockets as
 * a small model that counts each command).  The test calls the offload's
 * vtable as Zephyr's socket layer does.  The core's own commands and
 * replies are common/tests/nina_sock.c's; this covers what the offload
 * adds, against the former drivers/nina/nina_socket.c:
 *
 * - descriptors: EAFNOSUPPORT before one is reserved, EMFILE after ten
 *   (the descriptor given back), the table full (-1 from zvfs), the
 *   firmware socket made on first use, CMD_CLOSE only when it exists;
 * - the wait loops and their sleeps: connect (20 ms, also non-blocking,
 *   the lwIP error, ETIMEDOUT at 10 s), TCP send (1 ms between passes, one
 *   state check per 100 ms, EPIPE), receive (20 ms until data or
 *   SO_RCVTIMEO; a poll on every pass, also after poll() found data), poll
 *   (20 ms until the timeout); no lock of the table across a sleep;
 * - O_NONBLOCK and MSG_DONTWAIT: no sleep;
 * - setsockopt(), the fcntl, accept (EINVAL before a descriptor is
 *   reserved), sendmsg (vectors joined, refusals), datagrams (sender,
 *   filter of a connected socket), poll() (POLLHUP, POLLNVAL, other
 *   descriptors);
 * - the resolver registered as the offload's, its results on the heap.
 *
 * The waits between passes are common/drivers/nina/nina_wait.c, linked in:
 * without its events (the 3.0.2 firmware) they are the sleeps above.
 * Built a second time with NINA_OS_EVENTS and the events started (fakes of
 * nina_os_event_*() here): each loop waits for the IRQ line's edge after
 * the count it noted before its pass (an edge during the pass's commands
 * ends the wait at once), at most 100 ms and not past its deadline, with
 * no sleep; a send waits for the edge only after a pass that polled
 * without sending; a poll() with another kind of descriptor waits at most
 * 20 ms.
 */
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "socket_test.h"

#include "nina_dns.h"
#include "nina_os.h"
#include "nina_proto.h"
#include "nina_sock.h"
#include "nina_sockopt.h"
#include "nina_wait.h"

extern struct k_mutex socket_lock, sendmsg_lock;
extern const struct fake_offload fake_offload_nina;

/* Clock, sleeps, mutexes, heap. */
static int64_t now_ms;
static unsigned int sleeps;
static int64_t slept_ms;

/* A close by another thread that comes first while the next caller waits
 * for the table lock: the descriptor's close runs when the lock is taken,
 * the descriptor stays in zvfs's table until the test frees it (zvfs_close()
 * frees it after the close). */
static int close_at_lock = -1;
static void close_now(int fd);

int k_mutex_lock(struct k_mutex *lock, k_timeout_t timeout)
{
	assert(timeout.ms == -1);
	if (lock == &socket_lock && close_at_lock >= 0) {
		int fd = close_at_lock;

		close_at_lock = -1;
		close_now(fd);
	}
	lock->held++;
	return 0;
}

int k_mutex_unlock(struct k_mutex *lock)
{
	assert(lock->held > 0);
	lock->held--;
	return 0;
}

int32_t k_sleep(k_timeout_t timeout)
{
	/* Never with the table locked. */
	assert(!socket_lock.held);
	sleeps++;
	slept_ms += timeout.ms;
	now_ms += timeout.ms;
	return 0;
}

int64_t k_uptime_get(void)
{
	return now_ms;
}

static int heap_blocks;

void *k_heap_alloc(struct k_heap *heap, size_t bytes, k_timeout_t timeout)
{
	(void)heap;
	assert(timeout.ms == 0);
	heap_blocks++;
	return malloc(bytes);
}

void k_heap_free(struct k_heap *heap, void *mem)
{
	(void)heap;
	heap_blocks--;
	free(mem);
}

void nina_os_dns_lock(void)
{
}

void nina_os_dns_unlock(void)
{
}

int64_t nina_os_uptime_ms(void)
{
	return now_ms;
}

/* nina_wait.c's sleep: nina_os.c's k_sleep(K_MSEC()). */
void nina_os_sleep_ms(uint32_t ms)
{
	(void)k_sleep(K_MSEC(ms));
}

#ifdef NINA_OS_EVENTS
/* The IRQ line: the edge count; an edge during the next CMD_POLL (after
 * its answer: the event the poll armed comes at once) or when a wait
 * starts, which also makes the socket EDGE_SOCKET writable or gives it
 * EDGE_DATA. */
static uint32_t edges;
static unsigned int event_waits;
static uint32_t event_wait_last_ms;
static int64_t event_waited_ms;
static bool edge_in_poll, edge_in_wait;
static int edge_socket = -1;
static const char *edge_data;

int nina_os_event_start(void)
{
	return 0;
}

uint32_t nina_os_event_seq(void)
{
	return edges;
}

static void edge(void);

void nina_os_event_wait(uint32_t seq, uint32_t ms)
{
	assert(!socket_lock.held);
	event_waits++;
	event_wait_last_ms = ms;
	if (edges != seq) {
		return;
	}
	if (edge_in_wait) {
		edge_in_wait = false;
		edge();
		return;
	}
	now_ms += ms;
	event_waited_ms += ms;
}
#endif

static const struct socket_dns_offload *dns_registered;

void socket_offload_dns_register(const struct socket_dns_offload *ops)
{
	dns_registered = ops;
}

/* The descriptor table. */
#define FDS 16
static struct {
	bool used;
	void *obj;
	const struct fd_op_vtable *vtable;
} table[FDS];
static const struct fd_op_vtable native_vtable;
static int native_polls;

int zvfs_reserve_fd(void)
{
	for (int i = 0; i < FDS; i++) {
		if (!table[i].used) {
			table[i].used = true;
			table[i].obj = NULL;
			table[i].vtable = NULL;
			return i;
		}
	}
	errno = ENFILE;
	return -1;
}

void zvfs_free_fd(int fd)
{
	assert(table[fd].used);
	table[fd].used = false;
}

void zvfs_finalize_typed_fd(int fd, void *obj, const struct fd_op_vtable *vtable, uint32_t mode)
{
	assert(table[fd].used && !table[fd].obj && mode == ZVFS_MODE_IFSOCK);
	table[fd].obj = obj;
	table[fd].vtable = vtable;
}

void *zvfs_get_fd_obj_and_vtable(int fd, const struct fd_op_vtable **vtable, struct k_mutex **lock)
{
	assert(!lock);
	if (fd < 0 || fd >= FDS || !table[fd].used || !table[fd].obj) {
		return NULL;
	}
	*vtable = table[fd].vtable;
	return table[fd].obj;
}

int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
	assert(nfds == 1 && timeout == 0);
	native_polls++;
	fds[0].revents = fds[0].events & POLLIN;
	return fds[0].revents ? 1 : 0;
}

static void close_now(int fd)
{
	assert(table[fd].vtable->close(table[fd].obj) == 0);
}

static int used_fds(void)
{
	int n = 0;

	for (int i = 0; i < FDS; i++) {
		n += table[i].used;
	}
	return n;
}

/* The module: the firmware's sockets. */
struct fw {
	bool open, udp;
	int polls_until_write; /* -1: never writable */
	uint8_t extra_state;   /* POLL_ERROR, ... */
	int so_error;
	int zero_sends; /* SENDs that take nothing */
	uint8_t rx[64];
	size_t rx_len;
	uint8_t from[6]; /* a datagram's sender: address, port (network order) */
	bool poll_fail;  /* the firmware's select() fails: POLL_FAIL */
};
static struct fw fw[10];
/* What the next firmware socket starts with (the socket command). */
static struct fw fw_next;
static unsigned int count[256];
static int pending_client = -1;
/* The next POLL fails on the SPI link with this (0: it does not). */
static int poll_link_error;
static uint8_t last_send[4096];
static size_t last_send_len;

static int next_fw(void)
{
	for (int i = 0; i < 10; i++) {
		if (!fw[i].open) {
			return i;
		}
	}
	return 0xff;
}

int nina_request(uint8_t cmd, const struct nina_param *p, size_t nparams, bool wide_params,
		 bool wide_response, uint8_t *out, size_t cap, size_t *out_len)
{
	struct fw *f = nparams && cmd >= 0x71 ? &fw[p[0].data[0]] : NULL;
	size_t n = 0;

	(void)wide_params;
	(void)wide_response;
	count[cmd]++;
	if (cmd >= 0x70 && cmd <= 0x7e) {
		assert(socket_lock.held);
	}
	if (f) {
		assert(p[0].data[0] < 10 && f->open);
	}
	switch (cmd) {
	case 0x70: { /* socket */
		int id = next_fw();

		if (id != 0xff) {
			fw[id] = fw_next;
			fw[id].open = true;
			fw[id].udp = p[0].data[0] == 2;
		}
		out[n++] = (uint8_t)id;
		break;
	}
	case 0x7b: /* ioctl FIONBIO */
		memset(out, 0, 4);
		n = 4;
		break;
	case 0x71: /* close */
		f->open = false;
		out[n++] = 1;
		break;
	case 0x73: /* bind */
	case 0x74: /* listen */
		out[n++] = 1;
		break;
	case 0x75: { /* accept */
		uint8_t reply[7] = {0xff, 192, 168, 1, 2, 0x12, 0x34};

		if (pending_client >= 0) {
			reply[0] = (uint8_t)pending_client;
			memset(&fw[pending_client], 0, sizeof(fw[0]));
			fw[pending_client].open = true;
			pending_client = -1;
		}
		memcpy(out, reply, 7);
		n = 7;
		break;
	}
	case 0x76: /* connect */
		out[n++] = 0;
		break;
	case 0x7c: { /* poll */
		uint8_t state = f->extra_state;

#ifdef NINA_OS_EVENTS
		if (edge_in_poll) {
			/* The answer is the state before the edge. */
			edge_in_poll = false;
			out[n++] = state | (f->polls_until_write == 0 ? 0x02 : 0) |
				   (f->rx_len ? 0x01 : 0);
			edge();
			break;
		}
#endif
		if (poll_link_error) {
			int ret = poll_link_error;

			poll_link_error = 0;
			return ret;
		}
		if (f->poll_fail) {
			state |= 0x80;
		}
		if (f->polls_until_write == 0) {
			state |= 0x02;
		} else if (f->polls_until_write > 0) {
			f->polls_until_write--;
		}
		if (f->rx_len) {
			state |= 0x01;
		}
		out[n++] = state;
		break;
	}
	case 0x7e: /* getsockopt SO_ERROR */
		out[0] = (uint8_t)f->so_error;
		out[1] = out[2] = out[3] = 0;
		n = 4;
		break;
	case 0x77: { /* send */
		size_t len = p[1].len;

		if (f->zero_sends > 0) {
			f->zero_sends--;
			len = 0;
		} else {
			memcpy(last_send, p[1].data, p[1].len);
			last_send_len = p[1].len;
		}
		out[0] = (uint8_t)(len >> 8);
		out[1] = (uint8_t)len;
		n = 2;
		break;
	}
	case 0x78: { /* recv */
		size_t len = p[1].data[0] | (p[1].data[1] << 8);

		n = MIN(len, f->rx_len);
		memcpy(out, f->rx, n);
		f->rx_len = 0;
		break;
	}
	case 0x79: /* sendto */
		out[0] = (uint8_t)(p[3].len >> 8);
		out[1] = (uint8_t)p[3].len;
		n = 2;
		break;
	case 0x7a: /* recvfrom */
		memcpy(out, f->from, 4);
		out[4] = f->from[5];
		out[5] = f->from[4];
		memcpy(out + 6, f->rx, f->rx_len);
		n = 6 + f->rx_len;
		f->rx_len = 0;
		break;
	default:
		assert(0);
	}
	assert(n <= cap);
	*out_len = n;
	return 0;
}

/* Zephyr's socket layer: the calls through the vtable. */
static const struct socket_op_vtable *ops(int fd, void **obj)
{
	assert(table[fd].used && table[fd].obj);
	*obj = table[fd].obj;
	return (const struct socket_op_vtable *)table[fd].vtable;
}

#define CALL(fn, fd, ...)                                                                          \
	({                                                                                         \
		void *obj_;                                                                        \
		const struct socket_op_vtable *v_ = ops((fd), &obj_);                              \
		v_->fn(obj_, __VA_ARGS__);                                                         \
	})

static int z_socket(int type)
{
	return fake_offload_nina.create(AF_INET, type, 0);
}

static int z_close(int fd)
{
	void *obj;
	int ret = ops(fd, &obj)->fd_vtable.close(obj);

	zvfs_free_fd(fd);
	return ret;
}

static int z_ioctl(int fd, unsigned int request, ...)
{
	void *obj;
	va_list args;
	int ret;

	va_start(args, request);
	ret = ops(fd, &obj)->fd_vtable.ioctl(obj, request, args);
	va_end(args);
	return ret;
}

/* zvfs_poll(): ZFD_IOCTL_POLL_PREPARE answers -EXDEV, so the offloaded
 * poll gets all descriptors. */
static int z_poll(struct pollfd *pfds, int nfds, int timeout)
{
	assert(z_ioctl(pfds[0].fd, ZFD_IOCTL_POLL_PREPARE, NULL, NULL, NULL) == -EXDEV);
	return z_ioctl(pfds[0].fd, ZFD_IOCTL_POLL_OFFLOAD, pfds, nfds, timeout);
}

static struct sockaddr_in addr_of(uint8_t last, uint16_t port)
{
	struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(port)};

	a.sin_addr.s_addr = htonl(0x0a000000u | last);
	return a;
}

static void reset_counts(void)
{
	memset(count, 0, sizeof(count));
	sleeps = 0;
	slept_ms = 0;
#ifdef NINA_OS_EVENTS
	event_waits = 0;
	event_waited_ms = 0;
#endif
}

/* A connected TCP socket on firmware socket *ID. */
static int connected_tcp(int *id)
{
	struct sockaddr_in a = addr_of(1, 1883);
	int fd = z_socket(SOCK_STREAM);

	assert(fd >= 0);
	*id = next_fw();
	/* The first poll after the connect command finds it writable. */
	memset(&fw_next, 0, sizeof(fw_next));
	assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == 0);
	return fd;
}

static void test_descriptors(void)
{
	int fds[11], before = used_fds(), id;

	errno = 0;
	assert(fake_offload_nina.create(AF_INET6, SOCK_STREAM, 0) == -1 && errno == EAFNOSUPPORT);
	assert(fake_offload_nina.create(AF_INET, SOCK_RAW, 0) == -1 && errno == EAFNOSUPPORT);
	assert(used_fds() == before);
	assert(fake_offload_nina.supported(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
	assert(!fake_offload_nina.supported(AF_INET, SOCK_DGRAM, IPPROTO_TCP));

	reset_counts();
	for (int i = 0; i < 10; i++) {
		fds[i] = z_socket(i % 2 ? SOCK_DGRAM : SOCK_STREAM);
		assert(fds[i] >= 0);
	}
	/* No command until first use. */
	assert(count[0x70] == 0);
	errno = 0;
	assert(z_socket(SOCK_STREAM) == -1 && errno == EMFILE);
	assert(used_fds() == before + 10);
	for (int i = 0; i < 10; i++) {
		assert(z_close(fds[i]) == 0);
	}
	assert(count[0x71] == 0 && used_fds() == before);

	/* The descriptor table full: zvfs's -1 (ENFILE), no entry taken. */
	for (int i = 0; i < FDS; i++) {
		fds[0] = zvfs_reserve_fd();
	}
	errno = 0;
	assert(z_socket(SOCK_STREAM) == -1 && errno == ENFILE);
	for (int i = 0; i < FDS; i++) {
		zvfs_free_fd(i);
	}
	for (int i = 0; i < NINA_SOCK_MAX; i++) {
		assert(!g_nina_sockets[i].used);
	}

	/* CMD_CLOSE once the firmware socket exists. */
	fds[0] = connected_tcp(&id);
	reset_counts();
	assert(z_close(fds[0]) == 0 && count[0x71] == 1 && !fw[id].open);
}

static void test_connect(void)
{
	struct sockaddr_in a = addr_of(3, 80);
	int fd, id;

	/* Writable after three more polls: three sleeps of 20 ms, also on a
	 * non-blocking socket. */
	for (int nonblocking = 0; nonblocking < 2; nonblocking++) {
		fd = z_socket(SOCK_STREAM);
		if (nonblocking) {
			assert(z_ioctl(fd, ZVFS_F_SETFL, ZVFS_O_NONBLOCK) == 0);
		}
		id = next_fw();
		memset(&fw_next, 0, sizeof(fw_next));
		fw_next.polls_until_write = 3;
		reset_counts();
		assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == 0);
		assert(count[0x70] == 1 && count[0x7b] == 1 && count[0x76] == 1);
		assert(count[0x7c] == 4 && count[0x7e] == 1 && sleeps == 3 && slept_ms == 60);
		assert(z_ioctl(fd, ZVFS_F_GETFL) == (nonblocking ? ZVFS_O_NONBLOCK : 0));
		errno = 0;
		assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == -1 &&
		       errno == EISCONN);
		assert(z_close(fd) == 0 && !fw[id].open);
	}

	/* The lwIP error: ECONNREFUSED for one not mapped, ECONNABORTED for
	 * 113; the firmware socket closed each time. */
	fd = z_socket(SOCK_STREAM);
	memset(&fw_next, 0, sizeof(fw_next));
	fw_next.so_error = 111;
	reset_counts();
	errno = 0;
	assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == -1 && errno == ECONNREFUSED);
	assert(count[0x71] == 1);
	fw_next.so_error = 113;
	errno = 0;
	assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == -1 && errno == ECONNABORTED);

	/* Never writable: ETIMEDOUT after 10 s of 20 ms sleeps. */
	memset(&fw_next, 0, sizeof(fw_next));
	fw_next.polls_until_write = -1;
	reset_counts();
	errno = 0;
	assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == -1 && errno == ETIMEDOUT);
	assert(slept_ms >= 10000 && slept_ms <= 10020 && slept_ms == 20 * (int64_t)sleeps);
	assert(count[0x71] == 1);
	assert(z_close(fd) == 0);
	memset(&fw_next, 0, sizeof(fw_next));

	/* A bad address: EINVAL, nothing sent. */
	fd = z_socket(SOCK_STREAM);
	a.sin_family = AF_INET6;
	reset_counts();
	errno = 0;
	assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == -1 && errno == EINVAL);
	assert(count[0x70] == 0);
	assert(z_close(fd) == 0);
}

static void test_send(void)
{
	uint8_t data[5000];
	int fd, id;

	for (size_t i = 0; i < sizeof(data); i++) {
		data[i] = (uint8_t)i;
	}
	fd = connected_tcp(&id);

	/* Two sends without room: two sleeps of 1 ms, the state check of the
	 * first pass, and a poll before each pass that continues (writable:
	 * the data goes again). */
	fw[id].zero_sends = 2;
	reset_counts();
	assert(CALL(sendto, fd, data, 100, 0, NULL, 0) == 100);
	assert(count[0x77] == 3 && count[0x7c] == 3 && sleeps == 2 && slept_ms == 2);
	assert(last_send_len == 100 && !memcmp(last_send, data, 100));

	/* Not writable for three polls: the data is not sent again until it
	 * is (1 ms between the passes, as before). */
	fw[id].zero_sends = 1;
	fw[id].polls_until_write = 3;
	reset_counts();
	assert(CALL(sendto, fd, data, 10, 0, NULL, 0) == 10);
	assert(count[0x77] == 2 && count[0x7c] == 4 && sleeps == 3 && slept_ms == 3);

	/* At most CONFIG_NINA_TCP_MAX_IO (3584) bytes in one command. */
	assert(CALL(sendto, fd, data, sizeof(data), 0, NULL, 0) == 3584);
	assert(last_send_len == 3584);

	/* MSG_DONTWAIT and O_NONBLOCK: EAGAIN at once, no sleep. */
	fw[id].zero_sends = 1;
	reset_counts();
	errno = 0;
	assert(CALL(sendto, fd, data, 10, MSG_DONTWAIT, NULL, 0) == -1 && errno == EAGAIN);
	assert(sleeps == 0 && count[0x7c] == 0);
	assert(z_ioctl(fd, ZVFS_F_SETFL, ZVFS_O_NONBLOCK) == 0);
	fw[id].zero_sends = 1;
	errno = 0;
	assert(CALL(sendto, fd, data, 10, 0, NULL, 0) == -1 && errno == EAGAIN);
	assert(sleeps == 0);
	assert(z_ioctl(fd, ZVFS_F_SETFL, 0) == 0);

	/* No room and the socket's error: EPIPE. */
	fw[id].zero_sends = 1;
	fw[id].extra_state = 0x04;
	errno = 0;
	assert(CALL(sendto, fd, data, 10, 0, NULL, 0) == -1 && errno == EPIPE);
	fw[id].extra_state = 0;

	/* No room until the 10 s deadline: EAGAIN. */
	fw[id].zero_sends = 1000000;
	reset_counts();
	errno = 0;
	assert(CALL(sendto, fd, data, 10, 0, NULL, 0) == -1 && errno == EAGAIN);
	/* Writable all along: a poll and the data at each pass. */
	assert(slept_ms >= 9999 && slept_ms <= 10001 && count[0x7c] >= 9999 &&
	       count[0x7c] <= 10001 && count[0x77] == count[0x7c]);
	fw[id].zero_sends = 0;

	/* Refusals: other flags, an address on TCP. */
	errno = 0;
	assert(CALL(sendto, fd, data, 10, MSG_PEEK, NULL, 0) == -1 && errno == EOPNOTSUPP);
	{
		struct sockaddr_in a = addr_of(9, 9);

		errno = 0;
		assert(CALL(sendto, fd, data, 10, 0, (struct sockaddr *)&a, sizeof(a)) == -1 &&
		       errno == EOPNOTSUPP);
	}
	/* write() is send() without flags. */
	reset_counts();
	assert(table[fd].vtable->write(table[fd].obj, data, 7) == 7 && count[0x77] == 1);

	/* sendmsg: the vectors joined into one command. */
	{
		struct iovec iov[2] = {{data, 3}, {data + 100, 4}};
		struct msghdr msg = {.msg_iov = iov, .msg_iovlen = 2};

		fw[id].zero_sends = 1;
		reset_counts();
		assert(CALL(sendmsg, fd, &msg, 0) == 7);
		assert(count[0x77] == 2 && count[0x7c] == 2 && sleeps == 1 && !sendmsg_lock.held);
		assert(last_send_len == 7 && !memcmp(last_send, data, 3) &&
		       !memcmp(last_send + 3, data + 100, 4));
		errno = 0;
		assert(CALL(sendmsg, fd, NULL, 0) == -1 && errno == EINVAL);
		msg.msg_name = &msg;
		errno = 0;
		assert(CALL(sendmsg, fd, &msg, 0) == -1 && errno == EOPNOTSUPP);
	}
	assert(z_close(fd) == 0);
}

static void test_recv(void)
{
	struct zsock_timeval tv = {.tv_sec = 0, .tv_usec = 100000};
	struct sockaddr_in from;
	socklen_t fromlen = sizeof(from);
	uint8_t buf[600];
	int fd, id;

	fd = connected_tcp(&id);

	/* Nothing within SO_RCVTIMEO (100 ms): polls at 0, 20, ... 100 ms. */
	assert(CALL(setsockopt, fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);
	reset_counts();
	errno = 0;
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, NULL, NULL) == -1 && errno == EAGAIN);
	assert(count[0x7c] == 6 && sleeps == 5 && count[0x78] == 0);

	/* Data: one poll, one read of at most 512 bytes. */
	memcpy(fw[id].rx, "hello", 5);
	fw[id].rx_len = 5;
	reset_counts();
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, NULL, NULL) == 5);
	assert(!memcmp(buf, "hello", 5) && count[0x7c] == 1 && count[0x78] == 1 && sleeps == 0);

	/* After poll() found data, the receive polls again (no read_hint). */
	fw[id].rx_len = 3;
	{
		struct pollfd p = {.fd = fd, .events = POLLIN};

		assert(z_poll(&p, 1, 0) == 1 && p.revents == POLLIN);
	}
	reset_counts();
	assert(table[fd].vtable->read(table[fd].obj, buf, sizeof(buf)) == 3);
	assert(count[0x7c] == 1 && count[0x78] == 1);

	/* MSG_DONTWAIT, O_NONBLOCK: one poll, EAGAIN, no sleep. */
	reset_counts();
	errno = 0;
	assert(CALL(recvfrom, fd, buf, sizeof(buf), MSG_DONTWAIT, NULL, NULL) == -1 &&
	       errno == EAGAIN);
	assert(z_ioctl(fd, ZVFS_F_SETFL, ZVFS_O_NONBLOCK) == 0);
	errno = 0;
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, NULL, NULL) == -1 && errno == EAGAIN);
	assert(count[0x7c] == 2 && sleeps == 0);

	/* TCP takes no address; an address without its length: EOPNOTSUPP. */
	errno = 0;
	assert(CALL(recvfrom, fd, buf, 1, 0, (struct sockaddr *)&from, &fromlen) == -1 &&
	       errno == EOPNOTSUPP);
	errno = 0;
	assert(CALL(recvfrom, fd, buf, 1, 0, (struct sockaddr *)&from, NULL) == -1 &&
	       errno == EOPNOTSUPP);
	errno = 0;
	assert(CALL(recvfrom, fd, buf, 1, MSG_PEEK, NULL, NULL) == -1 && errno == EOPNOTSUPP);
	assert(z_close(fd) == 0);

	/* An unconnected TCP socket: ENOTCONN. */
	fd = z_socket(SOCK_STREAM);
	errno = 0;
	assert(CALL(recvfrom, fd, buf, 1, 0, NULL, NULL) == -1 && errno == ENOTCONN);
	assert(z_close(fd) == 0);
}

static void test_datagrams(void)
{
	struct sockaddr_in peer = addr_of(1, 123), from;
	socklen_t fromlen = sizeof(from);
	uint8_t buf[64];
	int fd, id;

	fd = z_socket(SOCK_DGRAM);
	id = next_fw();
	memset(&fw_next, 0, sizeof(fw_next));
	reset_counts();
	assert(CALL(sendto, fd, "q", 1, 0, (struct sockaddr *)&peer, sizeof(peer)) == 1);
	assert(count[0x70] == 1 && count[0x79] == 1);

	/* A datagram and its sender. */
	memcpy(fw[id].from, &peer.sin_addr, 4);
	memcpy(fw[id].from + 4, &peer.sin_port, 2);
	memcpy(fw[id].rx, "abc", 3);
	fw[id].rx_len = 3;
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromlen) == 3);
	assert(!memcmp(buf, "abc", 3) && fromlen == sizeof(from));
	assert(from.sin_family == AF_INET && from.sin_addr.s_addr == peer.sin_addr.s_addr &&
	       from.sin_port == peer.sin_port);

	/* Connected: a datagram from another sender is dropped, without a
	 * sleep, and the receive goes on. */
	assert(CALL(connect, fd, (struct sockaddr *)&peer, sizeof(peer)) == 0);
	fw[id].from[3] = 99;
	fw[id].rx_len = 3;
	reset_counts();
	errno = 0;
	assert(CALL(recvfrom, fd, buf, sizeof(buf), MSG_DONTWAIT, NULL, NULL) == -1 &&
	       errno == EAGAIN);
	assert(count[0x7a] == 1 && count[0x7c] == 2 && sleeps == 0);
	errno = 0;
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, NULL) == -1 &&
	       errno == EOPNOTSUPP);
	assert(z_close(fd) == 0);
}

static void test_poll(void)
{
	struct pollfd p[4];
	int tcp, udp, conn, id, native;

	tcp = z_socket(SOCK_STREAM);
	udp = z_socket(SOCK_DGRAM);
	conn = connected_tcp(&id);
	native = zvfs_reserve_fd();
	table[native].obj = &native_polls;
	table[native].vtable = &native_vtable;

	/* A TCP socket the firmware does not have: POLLHUP alone; such a UDP
	 * socket: POLLOUT if asked; a connected one: as the module says. */
	p[0] = (struct pollfd){.fd = tcp, .events = POLLIN};
	p[1] = (struct pollfd){.fd = udp, .events = POLLIN | POLLOUT};
	p[2] = (struct pollfd){.fd = conn, .events = POLLIN};
	p[3] = (struct pollfd){.fd = -1, .events = POLLIN};
	fw[id].polls_until_write = 0;
	reset_counts();
	assert(z_poll(p, 4, 0) == 2);
	assert(p[0].revents == POLLHUP && p[1].revents == POLLOUT && p[2].revents == 0 &&
	       p[3].revents == 0 && count[0x7c] == 1 && sleeps == 0);

	/* Nothing: again every 20 ms until the timeout (50 ms). */
	p[0] = (struct pollfd){.fd = udp, .events = POLLIN};
	p[1] = (struct pollfd){.fd = conn, .events = POLLIN};
	reset_counts();
	assert(z_poll(p, 2, 50) == 0);
	assert(sleeps == 3 && count[0x7c] == 4);

	/* The module's error; another kind of descriptor; a closed one. */
	fw[id].extra_state = 0x04;
	p[0] = (struct pollfd){.fd = conn, .events = POLLIN};
	p[1] = (struct pollfd){.fd = native, .events = POLLIN};
	p[2] = (struct pollfd){.fd = 15, .events = POLLIN};
	native_polls = 0;
	assert(z_poll(p, 3, 0) == 3);
	assert(p[0].revents == POLLERR && p[1].revents == POLLIN && p[2].revents == POLLNVAL);
	assert(native_polls == 1);
	fw[id].extra_state = 0;

	/* The module's poll fails for one socket (its select() or the SPI
	 * link): POLLERR on it, even if not asked for, the others answered,
	 * no wait; the call does not fail. */
	fw[id].poll_fail = true;
	p[0] = (struct pollfd){.fd = conn, .events = POLLIN};
	p[1] = (struct pollfd){.fd = tcp, .events = POLLIN};
	reset_counts();
	assert(z_poll(p, 2, -1) == 2);
	assert(p[0].revents == POLLERR && p[1].revents == POLLHUP && sleeps == 0);
	fw[id].poll_fail = false;
	poll_link_error = -ETIMEDOUT;
	p[0] = (struct pollfd){.fd = conn, .events = 0};
	p[1] = (struct pollfd){.fd = udp, .events = POLLIN};
	assert(z_poll(p, 2, 100) == 1);
	assert(p[0].revents == POLLERR && p[1].revents == 0 && sleeps == 0);

	/* A socket closed by another thread while the poll waits for the
	 * table lock (its descriptor not freed yet): POLLNVAL, the call goes
	 * on (Zephyr's socket service thread stops for good on -1). */
	close_at_lock = conn;
	p[0] = (struct pollfd){.fd = conn, .events = POLLIN};
	p[1] = (struct pollfd){.fd = udp, .events = POLLIN};
	reset_counts();
	assert(z_poll(p, 2, 1000) == 1);
	assert(p[0].revents == POLLNVAL && p[1].revents == 0 && close_at_lock == -1);
	assert(count[0x71] == 1 && !fw[id].open && sleeps == 0);
	zvfs_free_fd(conn);

	zvfs_free_fd(native);
	assert(z_close(tcp) == 0 && z_close(udp) == 0);
}

static void test_options(void)
{
	struct zsock_timeval tv;
	int one = 1, tcp = z_socket(SOCK_STREAM), udp = z_socket(SOCK_DGRAM);

	tv = (struct zsock_timeval){.tv_sec = 0, .tv_usec = 1000000};
	errno = 0;
	assert(CALL(setsockopt, tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == -1 &&
	       errno == EINVAL);
	tv = (struct zsock_timeval){.tv_sec = 3000000, .tv_usec = 0};
	errno = 0;
	assert(CALL(setsockopt, tcp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == -1 &&
	       errno == EDOM);
	errno = 0;
	assert(CALL(setsockopt, tcp, SOL_SOCKET, SO_RCVTIMEO, NULL, sizeof(tv)) == -1 &&
	       errno == EINVAL);
	tv = (struct zsock_timeval){.tv_sec = 1, .tv_usec = 1};
	assert(CALL(setsockopt, udp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);
	assert(fake_offload_nina.create && g_nina_sockets[0].used);

	assert(CALL(setsockopt, tcp, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0);
	errno = 0;
	assert(CALL(setsockopt, tcp, IPPROTO_TCP, TCP_NODELAY, &one, 1) == -1 && errno == EINVAL);
	errno = 0;
	assert(CALL(setsockopt, udp, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == -1 &&
	       errno == ENOPROTOOPT);
	errno = 0;
	assert(CALL(setsockopt, tcp, NINA_SOL, NINA_SO_CALLER_POLLS, &one, sizeof(one)) == -1 &&
	       errno == ENOPROTOOPT);
	errno = 0;
	assert(CALL(setsockopt, tcp, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == -1 &&
	       errno == ENOPROTOOPT);

	assert(z_ioctl(tcp, ZVFS_F_GETFL) == 0);
	errno = 0;
	assert(z_ioctl(tcp, ZVFS_F_SETFL, ZVFS_O_NONBLOCK | 1) == -1 && errno == EOPNOTSUPP);
	errno = 0;
	assert(z_ioctl(tcp, 99) == -1 && errno == ENOTSUP);
	assert(z_close(tcp) == 0 && z_close(udp) == 0);
}

static void test_accept(void)
{
	struct sockaddr_in any = {.sin_family = AF_INET, .sin_port = htons(23)}, peer;
	socklen_t len = sizeof(peer);
	int fd, client, before, id;

	fd = z_socket(SOCK_STREAM);
	before = used_fds();
	/* Not listening: EINVAL before a descriptor is reserved. */
	errno = 0;
	assert(CALL(accept, fd, (struct sockaddr *)&peer, &len) == -1 && errno == EINVAL);
	assert(used_fds() == before);
	/* Even with the descriptor table full (not ENFILE). */
	{
		int reserved[FDS], n = 0;

		while ((reserved[n] = zvfs_reserve_fd()) >= 0) {
			n++;
		}
		errno = 0;
		assert(CALL(accept, fd, (struct sockaddr *)&peer, &len) == -1 && errno == EINVAL);
		while (n > 0) {
			zvfs_free_fd(reserved[--n]);
		}
	}
	assert(CALL(bind, fd, (struct sockaddr *)&any, sizeof(any)) == 0);
	assert(CALL(listen, fd, 1) == 0);
	errno = 0;
	assert(CALL(accept, fd, (struct sockaddr *)&peer, NULL) == -1 && errno == EINVAL);

	/* Nobody waiting: EAGAIN at once, the descriptor given back. */
	reset_counts();
	errno = 0;
	assert(CALL(accept, fd, (struct sockaddr *)&peer, &len) == -1 && errno == EAGAIN);
	assert(used_fds() == before && sleeps == 0);

	/* A client: a new blocking descriptor, the peer's address. */
	id = 7;
	pending_client = id;
	assert(z_ioctl(fd, ZVFS_F_SETFL, ZVFS_O_NONBLOCK) == 0);
	client = CALL(accept, fd, (struct sockaddr *)&peer, &len);
	assert(client >= 0 && client != fd && len == sizeof(peer));
	/* The firmware sends its network-order port's bytes swapped (0x12,
	 * 0x34 for port 0x3412). */
	assert(peer.sin_family == AF_INET && peer.sin_port == htons(0x3412) &&
	       peer.sin_addr.s_addr == htonl(0xc0a80102));
	assert(z_ioctl(client, ZVFS_F_GETFL) == 0);
	reset_counts();
	assert(z_close(client) == 0 && count[0x71] == 1 && !fw[id].open);
	assert(z_close(fd) == 0);
}

#ifdef NINA_OS_EVENTS
static void edge(void)
{
	edges++;
	if (edge_socket >= 0) {
		fw[edge_socket].polls_until_write = 0;
		if (edge_data) {
			fw[edge_socket].rx_len = strlen(edge_data);
			memcpy(fw[edge_socket].rx, edge_data, fw[edge_socket].rx_len);
		}
	}
}

static void test_events(void)
{
	struct zsock_timeval tv = {.tv_sec = 0, .tv_usec = 250000};
	struct sockaddr_in a = addr_of(3, 80);
	uint8_t buf[16], data[100] = {0};
	struct pollfd p[2];
	int fd, id, native;

	assert(nina_wait_start_events() == 0 && nina_wait_events());

	/* A receive whose data comes right after its poll said none (the edge
	 * during the pass): the wait ends at once, no time, no sleep. */
	fd = connected_tcp(&id);
	edge_socket = id;
	edge_data = "late";
	edge_in_poll = true;
	reset_counts();
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, NULL, NULL) == 4 && !memcmp(buf, "late", 4));
	assert(count[0x7c] == 2 && event_waits == 1 && event_waited_ms == 0 && sleeps == 0);
	/* Data at the edge in the wait: one wait, then the read. */
	edge_data = "edge";
	edge_in_wait = true;
	reset_counts();
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, NULL, NULL) == 4 && !memcmp(buf, "edge", 4));
	assert(count[0x7c] == 2 && event_waits == 1 && event_waited_ms == 0 && sleeps == 0);
	/* No edge: 100 ms waits, the last one cut at SO_RCVTIMEO (250 ms). */
	edge_data = NULL;
	assert(CALL(setsockopt, fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);
	reset_counts();
	errno = 0;
	assert(CALL(recvfrom, fd, buf, sizeof(buf), 0, NULL, NULL) == -1 && errno == EAGAIN);
	assert(event_waits == 3 && event_wait_last_ms == 50 && event_waited_ms == 250);
	assert(count[0x7c] == 4 && sleeps == 0);

	/* A send without room: after the pass that sent, 1 ms as before; after
	 * the pass that polled only, the edge (the socket writable then). */
	fw[id].zero_sends = 1;
	fw[id].polls_until_write = -1;
	edge_in_wait = true;
	reset_counts();
	assert(CALL(sendto, fd, data, 10, 0, NULL, 0) == 10);
	assert(count[0x77] == 2 && count[0x7c] == 3 && sleeps == 1 && slept_ms == 1);
	assert(event_waits == 1 && event_waited_ms == 0);
	/* Never writable: 100 ms waits up to the 10 s deadline, no data sent
	 * again. */
	fw[id].zero_sends = 1;
	fw[id].polls_until_write = -1;
	reset_counts();
	errno = 0;
	assert(CALL(sendto, fd, data, 10, 0, NULL, 0) == -1 && errno == EAGAIN);
	assert(count[0x77] == 1 && sleeps == 1 && slept_ms + event_waited_ms == 10000);
	assert(event_waits >= 99 && event_waits <= 101 && count[0x7c] == event_waits + 2);
	fw[id].polls_until_write = 0;
	assert(z_close(fd) == 0);

	/* A connect whose handshake ends right after the first poll: no wait. */
	fd = z_socket(SOCK_STREAM);
	id = next_fw();
	memset(&fw_next, 0, sizeof(fw_next));
	fw_next.polls_until_write = -1;
	edge_socket = id;
	edge_in_poll = true;
	reset_counts();
	assert(CALL(connect, fd, (struct sockaddr *)&a, sizeof(a)) == 0);
	assert(count[0x7c] == 2 && event_waits == 1 && event_waited_ms == 0 && sleeps == 0);

	/* poll(): the edge, or 100 ms; with another kind of descriptor, at
	 * most 20 ms per wait. */
	fw[id].polls_until_write = -1;
	edge_socket = -1;
	p[0] = (struct pollfd){.fd = fd, .events = POLLIN};
	reset_counts();
	assert(z_poll(p, 1, 150) == 0);
	assert(event_waits == 2 && event_wait_last_ms == 50 && event_waited_ms == 150);
	native = zvfs_reserve_fd();
	table[native].obj = &native_polls;
	table[native].vtable = &native_vtable;
	p[1] = (struct pollfd){.fd = native, .events = POLLOUT};
	reset_counts();
	assert(z_poll(p, 2, 50) == 0);
	assert(event_waits == 3 && event_wait_last_ms == 10 && event_waited_ms == 50);
	zvfs_free_fd(native);
	assert(z_close(fd) == 0);
	edge_socket = -1;

	/* A connected socket closed by another thread while a poll without a
	 * timeout waits for its edge (zperf's "download stop" while the
	 * socket service thread polls): the next pass sees POLLNVAL and the
	 * poll returns; no lock is held across the wait. */
	fd = connected_tcp(&id);
	fw[id].polls_until_write = -1;
	p[0] = (struct pollfd){.fd = fd, .events = POLLIN};
	close_at_lock = fd;
	reset_counts();
	assert(z_poll(p, 1, -1) == 1);
	assert(p[0].revents == POLLNVAL && close_at_lock == -1 && !fw[id].open);
	assert(!socket_lock.held && event_waits <= 1);
	zvfs_free_fd(fd);
}
#endif

static void test_dns(void)
{
	struct addrinfo *res;

	dns_registered = NULL;
	nina_dns_init();
	assert(dns_registered && dns_registered->getaddrinfo == nina_getaddrinfo &&
	       dns_registered->freeaddrinfo == nina_freeaddrinfo);
	assert(dns_registered->getaddrinfo("10.1.2.3", "1883", NULL, &res) == 0);
	assert(heap_blocks == 1 && res->ai_family == AF_INET);
	assert(((struct sockaddr_in *)res->ai_addr)->sin_port == htons(1883));
	dns_registered->freeaddrinfo(res);
	assert(heap_blocks == 0);
}

int main(void)
{
	test_descriptors();
	test_connect();
	test_send();
	test_recv();
	test_datagrams();
	test_poll();
	test_options();
	test_accept();
#ifdef NINA_OS_EVENTS
	test_events();
#endif
	test_dns();
	for (int i = 0; i < NINA_SOCK_MAX; i++) {
		assert(!g_nina_sockets[i].used);
	}
	assert(!socket_lock.held && !sendmsg_lock.held);
	puts("NINA socket offload: tests passed");
	return 0;
}
