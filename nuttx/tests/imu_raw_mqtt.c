/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the MQTT transport (nuttx/apps/imu/imu_raw_mqtt.c, the copy
 * of the Zephyr build's src/imu_raw_mqtt.c) with the port's MQTT client
 * (mqtt_min.c), its socket layer (mqtt_min_sock.c) and the compatibility
 * layer's zsock_*() calls (<zephyr/net/socket.h>), on a fake socket, clock
 * and network interface.  The sources are included, so the static
 * functions are tested directly:
 *
 * - imu_raw_mqtt_start(): the checks (-EINVAL, -EBUSY), the topics
 *   imu/<device>/raw, status and info, the client ID imu-<device>, the
 *   thread imu_raw_mqtt_tid (Zephyr priority 11 = NuttX 107, 2048 bytes).
 * - open_connection(): -ENETUNREACH without a default gateway (no socket),
 *   the configured address or the gateway, the port; the socket calls in
 *   order (socket, connect, send of CONNECT with keep-alive 60 and no
 *   clean session), the CONNACK wait: one FIONREAD (a poll of the module)
 *   and a poll() that does not wait, then a poll() of the rest of the 5000
 *   ms; every read that does not wait asks the module first (FIONREAD);
 *   a CONNACK that arrives in two pieces; -ETIMEDOUT, -ECONNREFUSED.
 * - serve_connection(): info (QoS 1, retain, message ID 1), then status
 *   (QoS 0), then the batch (QoS 0), each one PUBLISH in one sendmsg()
 *   call with the header and the body as two vectors; status every 1000
 *   ms, input every 100 ms, 5 ms sleeps while the queue is empty; a failed
 *   publish discards the batch (dropped) and ends the connection.
 * - service_input(): POLLHUP and a failed poll of the module end the
 *   connection with -EIO; a PINGRESP is read.
 * - imu_raw_mqtt_submit() while disconnected: dropped and -EAGAIN.
 * - The counters of imu_raw_mqtt_get_stats().
 */
#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>

/* The fake socket calls that the files under test (and the layer's inline
 * zsock_*()) call, and the test's sleep.
 */
#define socket(f, t, p) fake_socket(f, t, p)
#define connect(s, a, l) fake_connect(s, a, l)
#define send(s, b, l, f) fake_send(s, b, l, f)
#define sendmsg(s, m, f) fake_sendmsg(s, m, f)
#define recv(s, b, l, f) fake_recv(s, b, l, f)
#define close(s) fake_close(s)
#define ioctl(fd, req, ...) fake_ioctl(fd, req, __VA_ARGS__)
#define poll(fds, n, t) fake_poll(fds, n, t)
#define k_sleep test_sleep

int fake_socket(int family, int type, int protocol);
int fake_connect(int sock, const struct sockaddr *addr, socklen_t len);
ssize_t fake_send(int sock, const void *buf, size_t len, int flags);
ssize_t fake_sendmsg(int sock, const struct msghdr *msg, int flags);
ssize_t fake_recv(int sock, void *buf, size_t len, int flags);
int fake_close(int sock);
int fake_ioctl(int fd, unsigned long req, ...);
int fake_poll(struct pollfd *fds, nfds_t n, int timeout);

#include <zephyr/kernel.h>
int32_t test_sleep(k_timeout_t timeout);

#include "../apps/imu/imu_raw_mqtt.c"
#include "../apps/imu/mqtt_min_sock.c"

#include "imu_raw_batch.h"
#include "zephyr_kernel_host.h"

/* The clock ---------------------------------------------------------------*/

static uint64_t now_us = 1000000;
static uint64_t cycles;
static int64_t stop_at_ms = -1;  /* test_sleep() clears running from then */
static int sleeps;

uint64_t k_cycle_get_64(void)
{
	return cycles;
}

uint32_t k_cycle_get_32(void)
{
	return (uint32_t)cycles;
}

static void set_time_us(uint64_t us)
{
	now_us = us;
	test_time_set_us(us);
}

int32_t test_sleep(k_timeout_t timeout)
{
	sleeps++;
	set_time_us(now_us + (uint64_t)(timeout.ticks + 1) * Z_USEC_PER_TICK);
	if (stop_at_ms >= 0 && k_uptime_get() >= stop_at_ms) {
		atomic_clear(&running);
	}
	return 0;
}

/* The network interface ---------------------------------------------------*/

static struct net_if iface;
static struct in_addr gateway;
static bool have_iface = true;

struct net_if *net_if_get_default(void)
{
	return have_iface ? &iface : NULL;
}

struct in_addr net_if_ipv4_get_gw(struct net_if *i)
{
	assert(i == &iface);
	return gateway;
}

/* The fake socket ---------------------------------------------------------*/

#define FD 9

enum ev { EV_SOCKET, EV_CONNECT, EV_SEND, EV_SENDMSG, EV_RECV, EV_CLOSE, EV_IOCTL, EV_POLL };

struct event {
	enum ev type;
	long arg;       /* send/recv length, poll timeout, iovec count */
	int flags;
};

static struct {
	bool open;
	struct sockaddr_in addr;
	int family, type, protocol;
	struct event ev[4096];
	int nev;
	uint8_t tx[65536];
	size_t tx_len;
	/* Each sendmsg(): where its bytes start in tx, its length, the vector
	 * lengths. */
	size_t msg_start[64], msg_len[64], msg_iov0[64], msg_iov1[64];
	int nmsg;
	int sendmsg_errno;
	uint64_t send_us;       /* cycles one sendmsg() takes */
	uint8_t rx[512];
	size_t rx_len, rx_pos, seg_end[16];
	int segs, seg;
	bool hup;
	int ioctl_errno;
	int connect_errno;
} fk;

static void fake_reset(void)
{
	memset(&fk, 0, sizeof(fk));
}

static void record(enum ev type, long arg, int flags)
{
	assert(fk.nev < (int)(sizeof(fk.ev) / sizeof(fk.ev[0])));
	fk.ev[fk.nev++] = (struct event){type, arg, flags};
}

/* Bytes the broker sends, in one piece each call. */
static void arrive(const void *data, size_t len)
{
	memcpy(&fk.rx[fk.rx_len], data, len);
	fk.rx_len += len;
	fk.seg_end[fk.segs++] = fk.rx_len;
}

static bool readable(void)
{
	return fk.rx_pos < fk.rx_len;
}

int fake_socket(int family, int type, int protocol)
{
	record(EV_SOCKET, 0, 0);
	assert(!fk.open);
	fk.open = true;
	fk.family = family;
	fk.type = type;
	fk.protocol = protocol;
	return FD;
}

int fake_connect(int sock, const struct sockaddr *addr, socklen_t len)
{
	assert(sock == FD && fk.open && len == sizeof(struct sockaddr_in));
	record(EV_CONNECT, 0, 0);
	memcpy(&fk.addr, addr, sizeof(fk.addr));
	if (fk.connect_errno) {
		errno = fk.connect_errno;
		return -1;
	}
	return 0;
}

ssize_t fake_send(int sock, const void *buf, size_t len, int flags)
{
	assert(sock == FD && fk.open && flags == 0);
	record(EV_SEND, (long)len, flags);
	memcpy(&fk.tx[fk.tx_len], buf, len);
	fk.tx_len += len;
	return (ssize_t)len;
}

ssize_t fake_sendmsg(int sock, const struct msghdr *msg, int flags)
{
	size_t total = 0;

	assert(sock == FD && fk.open && flags == 0);
	record(EV_SENDMSG, (long)msg->msg_iovlen, flags);
	cycles += fk.send_us;
	if (fk.sendmsg_errno) {
		errno = fk.sendmsg_errno;
		return -1;
	}
	assert(fk.nmsg < 64);
	fk.msg_start[fk.nmsg] = fk.tx_len;
	fk.msg_iov0[fk.nmsg] = msg->msg_iov[0].iov_len;
	fk.msg_iov1[fk.nmsg] = msg->msg_iovlen > 1 ? msg->msg_iov[1].iov_len : 0;
	for (size_t i = 0; i < (size_t)msg->msg_iovlen; i++) {
		memcpy(&fk.tx[fk.tx_len], msg->msg_iov[i].iov_base, msg->msg_iov[i].iov_len);
		fk.tx_len += msg->msg_iov[i].iov_len;
		total += msg->msg_iov[i].iov_len;
	}
	fk.msg_len[fk.nmsg++] = total;
	return (ssize_t)total;
}

ssize_t fake_recv(int sock, void *buf, size_t len, int flags)
{
	size_t n;

	assert(sock == FD && fk.open);
	record(EV_RECV, (long)len, flags);
	if (!readable()) {
		errno = EAGAIN;
		return -1;
	}
	while (fk.seg < fk.segs && fk.seg_end[fk.seg] <= fk.rx_pos) {
		fk.seg++;
	}
	n = fk.seg_end[fk.seg] - fk.rx_pos;
	n = n < len ? n : len;
	memcpy(buf, &fk.rx[fk.rx_pos], n);
	fk.rx_pos += n;
	return (ssize_t)n;
}

int fake_close(int sock)
{
	assert(sock == FD && fk.open);
	record(EV_CLOSE, 0, 0);
	fk.open = false;
	return 0;
}

int fake_ioctl(int fd, unsigned long req, ...)
{
	va_list ap;
	int *avail;

	assert(fd == FD && fk.open && req == FIONREAD);
	va_start(ap, req);
	avail = va_arg(ap, int *);
	va_end(ap);
	record(EV_IOCTL, 0, 0);
	if (fk.ioctl_errno) {
		errno = fk.ioctl_errno;
		return -1;
	}
	*avail = readable() ? 1 : 0;
	return 0;
}

int fake_poll(struct pollfd *fds, nfds_t n, int timeout)
{
	assert(n == 1 && fds[0].fd == FD && fds[0].events == POLLIN);
	record(EV_POLL, timeout, 0);
	fds[0].revents = (readable() ? POLLIN : 0) | (fk.hup ? POLLHUP : 0);
	if (!fds[0].revents && timeout > 0) {
		set_time_us(now_us + (uint64_t)timeout * 1000);
	}
	return fds[0].revents ? 1 : 0;
}

/* Checks ------------------------------------------------------------------*/

static void expect(int from, const enum ev *types, int n)
{
	assert(fk.nev - from == n);
	for (int i = 0; i < n; i++) {
		assert(fk.ev[from + i].type == types[i]);
	}
}

/* The PUBLISH of sendmsg() call I: its fixed header, topic, and the start
 * of the body. */
struct pub {
	uint8_t first;
	char topic[80];
	uint16_t message_id;
	const uint8_t *payload;
	size_t payload_len;
};

static struct pub publish_of(int i)
{
	const uint8_t *p = &fk.tx[fk.msg_start[i]];
	size_t pos = 1, remaining = 0, shift = 0, n, header;
	struct pub out = {0};

	out.first = p[0];
	do {
		remaining |= (size_t)(p[pos] & 0x7f) << shift;
		shift += 7;
	} while (p[pos++] & 0x80);
	header = pos;
	assert(header + remaining == fk.msg_len[i]);
	n = ((size_t)p[pos] << 8) | p[pos + 1];
	pos += 2;
	memcpy(out.topic, &p[pos], n);
	pos += n;
	if (out.first & 0x06) {
		out.message_id = (uint16_t)((p[pos] << 8) | p[pos + 1]);
		pos += 2;
	}
	/* The header vector ends where the payload starts. */
	assert(fk.msg_iov0[i] == pos);
	out.payload = &p[pos];
	out.payload_len = fk.msg_len[i] - pos;
	assert(fk.msg_iov1[i] == out.payload_len);
	return out;
}

static void start(const char *device)
{
	struct imu_raw_mqtt_config cfg = {.port = 18830};

	strcpy(cfg.device_id, device);
	memset(cfg.session_id, 0xab, sizeof(cfg.session_id));
	assert(imu_raw_mqtt_start(&cfg) == 0);
	assert(k_sem_take(&raw_start, K_NO_WAIT) == 0);
}

static void finish(void)
{
	/* What raw_mqtt_thread() does after a connection. */
	set_connected(false);
	(void)mqtt_abort(&client);
	atomic_clear(&running);
	atomic_clear(&worker_active);
	assert(!fk.open);
}

static const uint8_t connack_ok[] = {0x20, 0x02, 0x00, 0x00};

static void connect_ok(void)
{
	fake_reset();
	arrive(connack_ok, 2);
	arrive(connack_ok + 2, 2);
	assert(open_connection() == 0);
	set_connected(true);
	atomic_inc(&connects);
}

static size_t make_batch(uint8_t *payload, size_t cap)
{
	struct imu_raw_metadata m = {.accel_first_index = 10, .gyro_first_index = 20};
	const struct lsm6dso_fifo_sample samples[] = {
		{LSM6DSO_FIFO_ACCEL, {1, 2, 3}}, {LSM6DSO_FIFO_GYRO, {4, 5, 6}},
		{LSM6DSO_FIFO_ACCEL, {7, 8, 9}},
	};
	size_t length;

	assert(imu_raw_encode(payload, cap, &length, &m, samples, 3) == 0);
	return length;
}

int main(void)
{
	static uint8_t batch[IMU_RAW_MAX_SIZE];
	struct imu_raw_mqtt_config cfg = {.port = 18830};
	struct imu_raw_mqtt_stats st;
	const struct z_thread_static *t;
	size_t batch_len = make_batch(batch, sizeof(batch));
	int from;

	set_time_us(now_us);
	gateway.s_addr = htonl(0x0a2a0001); /* 10.42.0.1 */

	/* The thread: K_THREAD_DEFINE(imu_raw_mqtt_tid, 2048, ..., 11, 0, 0). */
	assert(z_thread_start_imu_raw_mqtt_tid() == 0);
	t = test_thread(0);
	assert(t && !strcmp(t->name, "imu_raw_mqtt_tid") && t->prio == 107 &&
	       t->stack_size == 2048 && t->entry == raw_mqtt_thread);

	/* Start: the checks, the topics and the client ID. */
	assert(imu_raw_mqtt_start(&cfg) == -EINVAL); /* no device */
	strcpy(cfg.device_id, "a.b");
	assert(imu_raw_mqtt_start(&cfg) == -EINVAL);
	strcpy(cfg.device_id, "rp2040-x_1");
	cfg.port = 0;
	assert(imu_raw_mqtt_start(&cfg) == -EINVAL);
	cfg.port = 18830;
	assert(imu_raw_mqtt_start(&cfg) == 0);
	assert(imu_raw_mqtt_start(&cfg) == -EBUSY);
	assert(!strcmp(topic, "imu/rp2040-x_1/raw") && !strcmp(status_topic, "imu/rp2040-x_1/status") &&
	       !strcmp(info_topic, "imu/rp2040-x_1/info") && !strcmp(client_id, "imu-rp2040-x_1"));
	assert(atomic_get(&running) && k_sem_take(&raw_start, K_NO_WAIT) == 0);

	/* Disconnected: a batch is dropped and counted. */
	assert(imu_raw_mqtt_submit(NULL, batch, batch_len) == -EAGAIN);
	imu_raw_mqtt_get_stats(&st);
	assert(st.dropped_batches == 1 && st.dropped_accel == 2 && st.dropped_gyro == 1 &&
	       st.lock_wait_count == 1 && st.via_gateway && st.port == 18830);

	/* No default gateway: no socket. */
	fake_reset();
	gateway.s_addr = 0;
	assert(open_connection() == -ENETUNREACH && fk.nev == 0);
	have_iface = false;
	assert(open_connection() == -ENETUNREACH && fk.nev == 0);
	have_iface = true;
	gateway.s_addr = htonl(0x0a2a0001);

	/* A connection: the calls in order.  The CONNACK comes in two pieces;
	 * every read that does not wait asks the module first. */
	connect_ok();
	{
		static const enum ev seq[] = {EV_SOCKET, EV_CONNECT, EV_SEND, EV_IOCTL, EV_POLL,
					      EV_IOCTL, EV_RECV, EV_IOCTL, EV_RECV};
		expect(0, seq, 9);
	}
	assert(fk.family == AF_INET && fk.type == SOCK_STREAM && fk.protocol == IPPROTO_TCP);
	assert(fk.addr.sin_family == AF_INET && fk.addr.sin_port == htons(18830) &&
	       fk.addr.sin_addr.s_addr == htonl(0x0a2a0001));
	assert(fk.ev[4].arg == 0);                          /* poll() without waiting */
	assert(fk.ev[6].arg == 2 && fk.ev[6].flags == MSG_DONTWAIT); /* fixed header */
	assert(fk.ev[8].arg == 2 && fk.ev[8].flags == MSG_DONTWAIT);
	/* CONNECT: MQTT 3.1.1, no clean session, keep-alive 60, the client ID. */
	{
		static const uint8_t connect_pkt[] = {
			0x10, 26, 0, 4, 'M', 'Q', 'T', 'T', 4, 0x00, 0, 60, 0, 14,
			'i', 'm', 'u', '-', 'r', 'p', '2', '0', '4', '0', '-', 'x', '_', '1'};
		assert(fk.tx_len == sizeof(connect_pkt) && !memcmp(fk.tx, connect_pkt, fk.tx_len));
	}
	assert(atomic_get(&connected));
	imu_raw_mqtt_get_stats(&st);
	assert(st.connected && st.address.s_addr == htonl(0x0a2a0001));

	/* Serving: info, status, batch, in that order, one sendmsg() each. */
	{
		struct imu_raw_info info = {.accel_odr_hz = 6667, .gyro_odr_hz = 6667,
					    .accel_ug_per_lsb = 244, .gyro_udps_per_lsb = 70000,
					    .fifo_watermark = 128, .timestamp_decimation = 8};
		struct imu_raw_status status = {.state = IMU_RAW_STATE_MEASURING};
		struct pub p;

		imu_raw_mqtt_set_info(&info);
		imu_raw_mqtt_report(&status);
		assert(imu_raw_mqtt_submit(NULL, batch, batch_len) == 0);
		from = fk.nev;
		fk.nmsg = 0;
		fk.send_us = 1234;
		sleeps = 0;
		stop_at_ms = 0; /* stop at the first sleep */
		assert(serve_connection() == 0);
		assert(fk.nmsg == 3 && sleeps == 1);
		{
			static const enum ev seq[] = {EV_SENDMSG, EV_SENDMSG, EV_SENDMSG, EV_IOCTL,
						      EV_POLL};
			expect(from, seq, 5);
		}
		for (int i = 0; i < 3; i++) {
			assert(fk.ev[from + i].arg == 2); /* header and body: two vectors */
		}
		assert(fk.ev[from + 4].arg == 0);
		p = publish_of(0);
		assert(p.first == 0x33 && !strcmp(p.topic, "imu/rp2040-x_1/info") &&
		       p.message_id == 1 && !memcmp(p.payload, "{\"v\":1,", 7));
		p = publish_of(1);
		assert(p.first == 0x30 && !strcmp(p.topic, "imu/rp2040-x_1/status") &&
		       strstr((const char *)p.payload, "\"state\":\"measuring\""));
		p = publish_of(2);
		assert(p.first == 0x30 && !strcmp(p.topic, "imu/rp2040-x_1/raw") &&
		       p.payload_len == batch_len && !memcmp(p.payload, batch, batch_len));
		imu_raw_mqtt_get_stats(&st);
		assert(st.published == 1 && st.status_published == 1 && st.info_published == 1 &&
		       st.publish_max_us == 1234 && st.publish_total_us == 1234 &&
		       st.publish_bytes == batch_len && st.queue_high == 1);
	}

	/* Status every 1000 ms, input every 100 ms, 5.1 ms sleeps. */
	{
		int64_t t0 = k_uptime_get();
		int status_msgs = 0, polls = 0;

		atomic_set(&running, 1);
		fk.nmsg = 0;
		from = fk.nev;
		stop_at_ms = t0 + 2050;
		assert(serve_connection() == 0);
		for (int i = from; i < fk.nev; i++) {
			polls += fk.ev[i].type == EV_IOCTL;
		}
		for (int i = 0; i < fk.nmsg; i++) {
			status_msgs += !strcmp(publish_of(i).topic, "imu/rp2040-x_1/status");
		}
		/* A new serve_connection() (a reconnection) sends the info again
		 * and the status at once, then the status at +1000 and +2000. */
		assert(fk.nmsg == 4 && status_msgs == 3 &&
		       !strcmp(publish_of(0).topic, "imu/rp2040-x_1/info"));
		assert(polls >= 20 && polls <= 22);
	}

	/* A PINGRESP is read; a hung up socket ends the connection. */
	{
		static const uint8_t pingresp[] = {0xd0, 0x00};

		arrive(pingresp, 2);
		assert(service_input() == 0 && !readable());
		fk.hup = true;
		assert(service_input() == -EIO);
		fk.hup = false;
		fk.ioctl_errno = EIO;
		assert(service_input() == -EIO);
		fk.ioctl_errno = 0;
	}

	/* A failed publish: the batch is discarded (dropped), the connection
	 * ends with the error. */
	{
		imu_raw_mqtt_get_stats(&st);
		uint64_t dropped = st.dropped_batches;

		atomic_set(&running, 1);
		assert(imu_raw_mqtt_submit(NULL, batch, batch_len) == 0);
		fk.sendmsg_errno = EAGAIN;
		stop_at_ms = -1;
		assert(serve_connection() == -EAGAIN);
		imu_raw_mqtt_get_stats(&st);
		/* The info, published first, failed; the batch is still queued. */
		assert(st.queued == 1 && st.dropped_batches == dropped && st.published == 1);
		fk.sendmsg_errno = 0;
	}
	finish();

	/* Reconnect, then the batch publish fails: dropped. */
	start("rp2040");
	connect_ok();
	assert(imu_raw_mqtt_submit(NULL, batch, batch_len) == 0);
	{
		struct imu_raw_status status = {.state = IMU_RAW_STATE_MEASURING};

		imu_raw_mqtt_report(&status);
		stop_at_ms = -1;
		assert(status_is_forced() && publish_status() == 0 && !status_is_forced());
	}
	fk.sendmsg_errno = ECONNRESET;
	{
		int taken;

		taken = imu_raw_queue_take(&queue, &outgoing);
		assert(taken == 0 && publish_batch() == -ECONNRESET);
	}
	imu_raw_mqtt_get_stats(&st);
	assert(st.dropped_batches == 1 && st.published == 0 && st.queued == 0);
	fk.sendmsg_errno = 0;
	finish();

	/* No CONNACK within 5000 ms: -ETIMEDOUT; poll() waits for the rest. */
	start("rp2040");
	fake_reset();
	assert(open_connection() == -ETIMEDOUT);
	{
		static const enum ev seq[] = {EV_SOCKET, EV_CONNECT, EV_SEND, EV_IOCTL, EV_POLL,
					      EV_POLL};
		expect(0, seq, 6);
		assert(fk.ev[4].arg == 0 && fk.ev[5].arg == CONNACK_WAIT_MS);
	}
	finish();

	/* Refused (return code 5): -ECONNREFUSED. */
	start("rp2040");
	fake_reset();
	{
		static const uint8_t refused[] = {0x20, 0x02, 0x00, 0x05};

		arrive(refused, 4);
	}
	assert(open_connection() == -ECONNREFUSED && !atomic_get(&connected));
	finish();

	/* A fixed address is used instead of the gateway. */
	cfg.address.s_addr = htonl(0x0a2a0063);
	strcpy(cfg.device_id, "rp2040");
	assert(imu_raw_mqtt_start(&cfg) == 0);
	connect_ok();
	assert(fk.addr.sin_addr.s_addr == htonl(0x0a2a0063));
	imu_raw_mqtt_get_stats(&st);
	assert(!st.via_gateway);
	/* Stop: nothing queued, so it does not wait. */
	imu_raw_mqtt_stop();
	assert(!atomic_get(&running) && !atomic_get(&connected));
	(void)mqtt_abort(&client);
	assert(!fk.open);

	printf("imu_raw_mqtt: ok\n");
	return 0;
}
