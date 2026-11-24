/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's MQTT transport on the FreeRTOS POSIX
 * port: the shared common/imu/imu_raw_mqtt.c with its JSON messages
 * (imu_raw_status.c), the port's MQTT client (mqtt_min.c) and its socket
 * layer (mqtt_min_sock.c), the queue and the packets (imu_raw_queue.c,
 * imu_raw_packet.c), with the prelude, as common/pico_sdk/drivers/imu/imu.cmake
 * builds them, on the build's interface (freertos/drivers/compat/
 * zephyr_net_if.c) and the layer's thread, mutex, semaphore, sleeps and
 * clock, with fake zsock_*() calls (here) that behave as the build's NINA
 * sockets do where the transport depends on it (one descriptor from 0, a
 * blocking send that waits while the module has no room, a read that does
 * not wait, a poll every 20 ms, POLLERR for a failed connection) and a fake
 * broker that decodes what the board sends.  The shared code's details (the
 * client's bytes on Zephyr's own library, the socket calls in order, the
 * CONNACK wait as the Zephyr build's poll) are tested by common/tests
 * (mqtt_min) and nuttx/tests/imu_raw_mqtt.c; this test is about the
 * FreeRTOS build's thread and boundary and the behaviour of the whole:
 *
 * - The thread: "imu_raw_mqtt_tid" at Zephyr's 11 (APP_PRIO_IMU_RAW_MQTT,
 *   FreeRTOS 9: below the acquisition's 11 and 10, above the shell's 6),
 *   started by K_THREAD_DEFINE_START(); it waits for imu_raw_mqtt_start().
 * - The start's checks (-EINVAL; -EBUSY while running).
 * - Before the network is up (no default gateway): no socket, a failure
 *   every RECONNECT_DELAY_MS (1 s), batches dropped with -EAGAIN.
 * - With the gateway (10.42.0.1): the connection to its port, CONNECT
 *   (MQTT 3.1.1, client ID imu-<device>, keep-alive 60, no clean session),
 *   then info (QoS 1, retained, message ID 1, PUBACK), status (QoS 0) and
 *   the batches (QoS 0) on imu/<device>/info, status and raw, each PUBLISH
 *   in one sendmsg() with its header and body as two vectors; the info and
 *   status bodies are imu_raw_info_encode()'s and imu_raw_status_encode()'s;
 *   status every 1000 ms, the input polled every 100 ms; the statistics.
 * - Congestion (the module takes nothing for 1.5 s while batches come every
 *   28 ms): the queue fills (queue_high 8), the rest is dropped with
 *   -EAGAIN and counted in dropped_batches, dropped_accel and dropped_gyro;
 *   the connection stays and everything queued is sent afterwards.
 * - A failed connection (POLLERR): -EIO, the queued batches dropped, the
 *   socket closed, a new connection 1 s later with info sent again.
 * - CONNACK refused (-ECONNREFUSED), CONNACK in two pieces, no CONNACK
 *   (-ETIMEDOUT after 5000 ms), a refused TCP connection.
 * - "imu stop": what is queued is sent first (up to 500 ms), then the
 *   socket is closed and the next start is accepted; a stop while the
 *   module takes nothing gives up after 500 ms.
 * - A fixed server (imu.host), another device name: its address and
 *   topics; the idle status carries the session ID.
 *
 * The test port's tick is 1 ms (tests/compat/FreeRTOSConfig.h).  It waits
 * for the real intervals: about 20 s.  Built and run by
 * freertos/script/host_tests.sh.
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
#include <zephyr/net/socket.h>

#include "imu_raw_mqtt.h"
#include "imu_raw_packet.h"
#include "imu_raw_status.h"
#include "priorities.h"
#include "zephyr_compat.h"
#include "zephyr_os_port.h"

#define RUNNER_STACK   16384
#define PRODUCER_STACK 16384

/* The fake module's waits, as the build's NINA sockets
 * (common/drivers/nina/internal/nina_sock.h, internal/nina_wait.h).
 */
#define POLL_INTERVAL_MS 20
#define SEND_RETRY_MS    1
#define SEND_WAIT_MS     10000

#define MAX_MESSAGES 1024
#define JSON_MAX     800

_Thread_local int z_host_isr_depth;

void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

extern k_tid_t imu_raw_mqtt_tid;

static void say(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	fflush(stdout);
}

static void fail(const char *what)
{
	say("FAIL: %s\n", what);
	fflush(stdout);
	_exit(1);
}

#define check(cond)                                                                                \
	do {                                                                                       \
		if (!(cond)) {                                                                     \
			say("%s:%d: ", __FILE__, __LINE__);                                        \
			fail(#cond);                                                               \
		}                                                                                  \
	} while (0)

static int64_t now_ms(void)
{
	return (int64_t)(rp2040_timer_us() / 1000u);
}

static void sleep_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

#define wait_for(cond, ms)                                                                         \
	do {                                                                                       \
		int64_t _end = now_ms() + (ms);                                                    \
		while (!(cond)) {                                                                  \
			if (now_ms() > _end) {                                                     \
				say("%s:%d: ", __FILE__, __LINE__);                                \
				fail("timeout: " #cond);                                           \
			}                                                                          \
			sleep_ms(5);                                                               \
		}                                                                                  \
	} while (0)

/* The log (common/compat/include/app_log.h): not used by the files under test. */
void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	(void)level;
	(void)module;
	(void)func;
	(void)fmt;
	(void)ap;
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	(void)level;
	(void)module;
	(void)func;
	(void)fmt;
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

/* The fake broker: what the board sent, decoded ----------------------------*/

enum connack_mode {
	CONNACK_OK,
	CONNACK_SPLIT, /* the 4 bytes come 2 at a time */
	CONNACK_REFUSED,
	CONNACK_NONE,
};

struct message {
	uint8_t type; /* the fixed header's first byte, high nibble */
	uint8_t qos;
	bool retain;
	uint16_t id;
	char topic[64];
	uint32_t length;
	uint64_t seq; /* of a batch */
	char json[JSON_MAX + 1];
	int64_t at_ms;
	int iovlen; /* of the sendmsg() that carried it; 0 for a send() */
	int connection;
};

static struct {
	/* The socket (the module's: one at a time here). */
	bool open;
	int sockets_made, closes, connects_tried;
	struct sockaddr_in peer;
	bool refuse_tcp;
	volatile bool stall;  /* the module takes nothing */
	volatile bool failed; /* the connection failed (POLLERR) */
	enum connack_mode connack;
	/* Board -> broker, not yet decoded; broker -> board. */
	uint8_t in[8192];
	size_t in_len;
	uint8_t out[256];
	size_t out_len, out_pos;
	size_t recv_max;
	int cur_iovlen;
	/* Counters. */
	int sends, sendmsgs, polls_now, pingreqs, pubacks_sent;
	/* CONNECT */
	char client_id[64];
	uint16_t keepalive;
	uint8_t connect_flags, level;
	char protocol[8];
	/* PUBLISH */
	struct message msgs[MAX_MESSAGES];
	int nmsgs;
} br;

static uint16_t be16(const uint8_t *p)
{
	return (uint16_t)(p[0] << 8 | p[1]);
}

static uint64_t le64(const uint8_t *p)
{
	uint64_t v = 0;

	for (int i = 7; i >= 0; i--) {
		v = v << 8 | p[i];
	}
	return v;
}

static void reply(const uint8_t *data, size_t len)
{
	check(br.out_len + len <= sizeof(br.out));
	memcpy(br.out + br.out_len, data, len);
	br.out_len += len;
}

static void decode_connect(const uint8_t *p, size_t len)
{
	uint16_t n = be16(p);
	const uint8_t *q;

	check(n < sizeof(br.protocol) && len >= 2u + n + 4u + 2u);
	memcpy(br.protocol, p + 2, n);
	br.protocol[n] = '\0';
	q = p + 2 + n;
	br.level = q[0];
	br.connect_flags = q[1];
	br.keepalive = be16(q + 2);
	n = be16(q + 4);
	check(n < sizeof(br.client_id));
	memcpy(br.client_id, q + 6, n);
	br.client_id[n] = '\0';
	switch (br.connack) {
	case CONNACK_OK:
	case CONNACK_SPLIT:
		reply((const uint8_t[]){0x20, 0x02, 0x00, 0x00}, 4);
		break;
	case CONNACK_REFUSED:
		reply((const uint8_t[]){0x20, 0x02, 0x00, 0x05}, 4);
		break;
	case CONNACK_NONE:
		break;
	}
	br.recv_max = br.connack == CONNACK_SPLIT ? 2 : SIZE_MAX;
}

static void decode_publish(uint8_t first, const uint8_t *p, size_t len)
{
	struct message *m;
	uint16_t n = be16(p);
	size_t at;

	check(br.nmsgs < MAX_MESSAGES);
	m = &br.msgs[br.nmsgs++];
	memset(m, 0, sizeof(*m));
	m->type = first >> 4;
	m->qos = (first >> 1) & 3;
	m->retain = first & 1;
	check(n < sizeof(m->topic));
	memcpy(m->topic, p + 2, n);
	at = 2u + n;
	if (m->qos) {
		m->id = be16(p + at);
		at += 2;
		reply((const uint8_t[]){0x40, 0x02, (uint8_t)(m->id >> 8), (uint8_t)m->id}, 4);
		br.pubacks_sent++;
	}
	m->length = (uint32_t)(len - at);
	if (strstr(m->topic, "/raw")) {
		check(m->length >= IMU_RAW_HEADER_SIZE && memcmp(p + at, "IMU1", 4) == 0);
		m->seq = le64(p + at + 24);
	} else {
		check(m->length <= JSON_MAX);
		memcpy(m->json, p + at, m->length);
	}
	m->at_ms = now_ms();
	m->iovlen = br.cur_iovlen;
	m->connection = br.sockets_made;
}

/* Decodes the complete packets that the board has sent. */
static void broker_take(const uint8_t *data, size_t len)
{
	size_t used = 0;

	check(br.in_len + len <= sizeof(br.in));
	memcpy(br.in + br.in_len, data, len);
	br.in_len += len;
	for (;;) {
		size_t rem = 0, hdr = 1;
		int shift = 0;

		if (br.in_len - used < 2) {
			break;
		}
		do {
			check(hdr < 5);
			rem |= (size_t)(br.in[used + hdr] & 0x7f) << shift;
			shift += 7;
		} while (br.in[used + hdr++] & 0x80 && used + hdr < br.in_len);
		if (br.in_len - used < hdr + rem) {
			break;
		}
		switch (br.in[used] & 0xf0) {
		case 0x10:
			decode_connect(br.in + used + hdr, rem);
			break;
		case 0x30:
			decode_publish(br.in[used], br.in + used + hdr, rem);
			break;
		case 0xc0:
			br.pingreqs++;
			reply((const uint8_t[]){0xd0, 0x00}, 2);
			break;
		default:
			say("broker: packet 0x%02x\n", br.in[used]);
			fail("unexpected packet");
		}
		used += hdr + rem;
	}
	memmove(br.in, br.in + used, br.in_len - used);
	br.in_len -= used;
}

/* The fake zsock_*() calls -------------------------------------------------*/

static int err(int e)
{
	errno = e;
	return -1;
}

static bool valid(int sock)
{
	return sock == 0 && br.open;
}

int zsock_socket(int family, int type, int proto)
{
	check(family == AF_INET && type == SOCK_STREAM && (proto == IPPROTO_TCP || proto == 0));
	if (br.open) {
		return err(EMFILE);
	}
	br.open = true;
	br.sockets_made++;
	br.failed = false;
	br.in_len = 0;
	br.out_len = br.out_pos = 0;
	br.recv_max = SIZE_MAX;
	return 0;
}

int zsock_close(int sock)
{
	if (!valid(sock)) {
		return err(EBADF);
	}
	br.open = false;
	br.closes++;
	return 0;
}

int zsock_connect(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	if (!valid(sock)) {
		return err(EBADF);
	}
	check(addrlen == sizeof(struct sockaddr_in));
	memcpy(&br.peer, addr, sizeof(br.peer));
	br.connects_tried++;
	if (br.refuse_tcp) {
		return err(ECONNREFUSED);
	}
	return 0;
}

/* A blocking send: tried again every SEND_RETRY_MS while the module has no
 * room, up to SEND_WAIT_MS (EAGAIN), as the build's NINA sockets.
 */
static ssize_t module_send(const uint8_t *data, size_t len, int flags)
{
	int64_t deadline = now_ms() + SEND_WAIT_MS;

	for (;;) {
		if (br.failed) {
			return err(EPIPE);
		}
		if (!br.stall) {
			break;
		}
		if ((flags & MSG_DONTWAIT) || now_ms() >= deadline) {
			return err(EAGAIN);
		}
		sleep_ms(SEND_RETRY_MS);
	}
	broker_take(data, len);
	return (ssize_t)len;
}

ssize_t zsock_sendto(int sock, const void *buf, size_t len, int flags,
		     const struct sockaddr *dest_addr, socklen_t addrlen)
{
	check(dest_addr == NULL && addrlen == 0);
	if (!valid(sock)) {
		return err(EBADF);
	}
	br.sends++;
	br.cur_iovlen = 0;
	return module_send(buf, len, flags);
}

ssize_t zsock_sendmsg(int sock, const struct msghdr *msg, int flags)
{
	static uint8_t joined[3584];
	size_t len = 0;

	if (!valid(sock)) {
		return err(EBADF);
	}
	check(msg->msg_name == NULL && msg->msg_controllen == 0);
	/* The build's NINA sockets join the vectors into one command. */
	for (size_t i = 0; i < (size_t)msg->msg_iovlen; i++) {
		check(len + msg->msg_iov[i].iov_len <= sizeof(joined));
		memcpy(joined + len, msg->msg_iov[i].iov_base, msg->msg_iov[i].iov_len);
		len += msg->msg_iov[i].iov_len;
	}
	br.sendmsgs++;
	br.cur_iovlen = (int)msg->msg_iovlen;
	return module_send(joined, len, flags);
}

ssize_t zsock_recvfrom(int sock, void *buf, size_t max_len, int flags, struct sockaddr *src_addr,
		       socklen_t *addrlen)
{
	check(src_addr == NULL && addrlen == NULL);
	for (;;) {
		size_t n;

		if (!valid(sock)) {
			return err(EBADF);
		}
		if (br.failed) {
			return 0; /* the NINA reports a failed connection as the end */
		}
		n = br.out_len - br.out_pos;
		if (n) {
			n = MIN(n, MIN(max_len, br.recv_max));
			memcpy(buf, br.out + br.out_pos, n);
			br.out_pos += n;
			if (br.out_pos == br.out_len) {
				br.out_pos = br.out_len = 0;
			}
			return (ssize_t)n;
		}
		if (flags & MSG_DONTWAIT) {
			return err(EAGAIN);
		}
		sleep_ms(POLL_INTERVAL_MS);
	}
}

int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
	int64_t deadline = timeout < 0 ? INT64_MAX : now_ms() + timeout;

	check(nfds == 1);
	if (timeout == 0) {
		br.polls_now++;
	}
	for (;;) {
		fds[0].revents = 0;
		if (!valid(fds[0].fd)) {
			fds[0].revents = ZSOCK_POLLNVAL;
		} else if (br.failed) {
			fds[0].revents = ZSOCK_POLLERR;
		} else if (br.out_len > br.out_pos) {
			fds[0].revents = fds[0].events & ZSOCK_POLLIN;
		}
		if (fds[0].revents) {
			return 1;
		}
		if (now_ms() >= deadline) {
			return 0;
		}
		sleep_ms(POLL_INTERVAL_MS);
	}
}

/* The parts of the build that the transport uses ---------------------------*/

static struct net_if iface;

static void config_init(struct imu_raw_mqtt_config *cfg, const char *device, uint16_t port)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->port = port;
	strcpy(cfg->device_id, device);
	/* The chip ID (hwinfo, tests/imu) and the stored counter 7. */
	memcpy(cfg->session_id, (const uint8_t[]){0x50, 0x35, 0x33, 0x74, 0x8a, 0x8c, 0x92, 0x18},
	       8);
	cfg->session_id[11] = 7;
}

/* A batch as imu_raw_batch.c makes them: COUNT words, accel and gyro in
 * turn.
 */
static uint8_t batch[IMU_RAW_MAX_SIZE];
static size_t batch_len;
static uint64_t batch_seq;

#define BATCH_WORDS 380
#define BATCH_ACCEL (BATCH_WORDS / 2)
#define BATCH_GYRO  (BATCH_WORDS - BATCH_ACCEL)

static int submit(void)
{
	static struct lsm6dso_fifo_sample samples[BATCH_WORDS];
	struct imu_raw_metadata m = {
		.batch_seq = batch_seq,
		.accel_first_index = batch_seq * BATCH_ACCEL,
		.gyro_first_index = batch_seq * BATCH_GYRO,
		.epoch_id = 1,
	};

	for (int i = 0; i < BATCH_WORDS; i++) {
		samples[i].kind = (i & 1) ? LSM6DSO_FIFO_GYRO : LSM6DSO_FIFO_ACCEL;
		samples[i].xyz[0] = (int16_t)i;
	}
	check(imu_raw_encode(batch, sizeof(batch), &batch_len, &m, samples, BATCH_WORDS) == 0);
	batch_seq++;
	return imu_raw_mqtt_submit(NULL, batch, batch_len);
}

static void stats(struct imu_raw_mqtt_stats *s)
{
	imu_raw_mqtt_get_stats(s);
}

static int count(const char *suffix, int from)
{
	int n = 0;

	for (int i = from; i < br.nmsgs; i++) {
		size_t a = strlen(br.msgs[i].topic), b = strlen(suffix);

		n += a >= b && strcmp(br.msgs[i].topic + a - b, suffix) == 0;
	}
	return n;
}

static int find(const char *topic, int from)
{
	for (int i = from; i < br.nmsgs; i++) {
		if (strcmp(br.msgs[i].topic, topic) == 0) {
			return i;
		}
	}
	return -1;
}

static int last(const char *topic)
{
	for (int i = br.nmsgs - 1; i >= 0; i--) {
		if (strcmp(br.msgs[i].topic, topic) == 0) {
			return i;
		}
	}
	fail("no such message");
	return -1;
}

/* The producer: the acquisition (imu_stream_tid), a batch every 28 ms
 * (about 35 per second at 6667 Hz).
 */
static volatile int produce, produced, refused;

static void producer(void *arg)
{
	(void)arg;
	for (;;) {
		if (produce > 0) {
			int rc = submit();

			check(rc == 0 || rc == -EAGAIN);
			produced++;
			refused += rc == -EAGAIN;
			produce--;
		}
		sleep_ms(produce > 0 ? 28 : 2);
	}
}

static const struct imu_raw_info info = {
	.epoch_id = 1,
	.accel_odr_hz = 6667,
	.gyro_odr_hz = 6667,
	.accel_ug_per_lsb = 244,
	.gyro_udps_per_lsb = 70000,
	.fifo_watermark = 128,
	.timestamp_decimation = 1,
	.freq_fine = -3,
};

static void test_thread(void)
{
	struct imu_raw_mqtt_config cfg;

	check(K_THREAD_DEFINE_START(imu_raw_mqtt_tid) == 0);
	check(imu_raw_mqtt_tid != NULL);
	check(strcmp(pcTaskGetName(imu_raw_mqtt_tid), "imu_raw_mqtt_tid") == 0);
	check(uxTaskPriorityGet(imu_raw_mqtt_tid) == APP_PRIO_IMU_RAW_MQTT &&
	      APP_PRIO_IMU_RAW_MQTT == APP_PRIO_FROM_ZEPHYR(11) && APP_PRIO_IMU_RAW_MQTT == 9);
	check(APP_PRIO_IMU_RAW_MQTT < APP_PRIO_IMU_FIFO_WORKER &&
	      APP_PRIO_IMU_RAW_MQTT < APP_PRIO_IMU_STREAM &&
	      APP_PRIO_IMU_RAW_MQTT < APP_PRIO_TIME_SYNC && APP_PRIO_IMU_RAW_MQTT > APP_PRIO_SHELL);
	check(APP_STACK_IMU_RAW_MQTT == 2048);
	sleep_ms(50);
	check(br.sockets_made == 0); /* waits for the start */

	config_init(&cfg, "rp2040", 0);
	check(imu_raw_mqtt_start(&cfg) == -EINVAL);
	config_init(&cfg, "", 18830);
	check(imu_raw_mqtt_start(&cfg) == -EINVAL);
	config_init(&cfg, "imu/x", 18830);
	check(imu_raw_mqtt_start(&cfg) == -EINVAL);
	sleep_ms(50);
	check(br.sockets_made == 0);
	say("ok  thread, start checks\n");
}

static void test_no_network(void)
{
	struct imu_raw_mqtt_config cfg;
	struct imu_raw_mqtt_stats s;
	struct imu_raw_status status = {.state = IMU_RAW_STATE_MEASURING, .epoch_id = 1};

	/* "imu start" with imu.host unset: the default gateway. */
	config_init(&cfg, "rp2040", 18830);
	check(imu_raw_mqtt_start(&cfg) == 0);
	check(imu_raw_mqtt_start(&cfg) == -EBUSY);
	imu_raw_mqtt_set_info(&info);
	imu_raw_mqtt_report(&status);
	sleep_ms(1500);
	stats(&s);
	check(s.running && !s.connected && s.via_gateway && s.address.s_addr == 0 &&
	      s.port == 18830);
	check(s.failures == 2 && s.connects == 0 && br.sockets_made == 0); /* at 0 and 1 s */
	check(submit() == -EAGAIN);
	stats(&s);
	check(s.dropped_batches == 1 && s.dropped_accel == BATCH_ACCEL &&
	      s.dropped_gyro == BATCH_GYRO && s.queued == 0);
	say("ok  no network (-ENETUNREACH every 1 s, batches dropped)\n");
}

static void test_connection(void)
{
	static const struct net_in_addr gateway = {.s_addr = 0x01002a0a}; /* 10.42.0.1 */
	struct imu_raw_mqtt_stats s;
	char json[768];
	int64_t t0;
	int i, polls, n, first_raw;

	/* The interface as the NINA driver leaves it on a connection. */
	z_net_if_register(&iface);
	net_if_ipv4_set_gw(&iface, &gateway);
	net_if_dormant_off(&iface);
	wait_for((imu_raw_mqtt_get_stats(&s), s.connected), 1500);

	check(br.sockets_made == 1 && br.connects_tried == 1);
	check(br.peer.sin_family == AF_INET && br.peer.sin_port == htons(18830) &&
	      br.peer.sin_addr.s_addr == gateway.s_addr);
	check(strcmp(br.protocol, "MQTT") == 0 && br.level == 4 && br.connect_flags == 0 &&
	      br.keepalive == 60 && strcmp(br.client_id, "imu-rp2040") == 0);
	stats(&s);
	check(s.connects == 1 && s.failures == 2 && s.address.s_addr == gateway.s_addr);

	/* info, then status, then the batches. */
	wait_for(br.nmsgs >= 2, 500);
	check(strcmp(br.msgs[0].topic, "imu/rp2040/info") == 0 && br.msgs[0].qos == 1 &&
	      br.msgs[0].retain && br.msgs[0].id == 1 && br.msgs[0].iovlen == 2);
	n = imu_raw_info_encode(json, sizeof(json), &info, "rp2040");
	check(n > 0 && br.msgs[0].length == (uint32_t)n && memcmp(br.msgs[0].json, json, n) == 0);
	check(strcmp(br.msgs[1].topic, "imu/rp2040/status") == 0 && br.msgs[1].qos == 0 &&
	      !br.msgs[1].retain && br.msgs[1].iovlen == 2);
	check(strstr(br.msgs[1].json, "\"state\":\"measuring\"") &&
	      strstr(br.msgs[1].json, "\"connects\":1") &&
	      strstr(br.msgs[1].json, "\"failures\":2") &&
	      strstr(br.msgs[1].json, "\"dropped_batches\":1"));
	{
		struct imu_raw_status expect = {.state = IMU_RAW_STATE_MEASURING, .epoch_id = 1};

		expect.dropped_batches = 1;
		expect.dropped_accel = BATCH_ACCEL;
		expect.dropped_gyro = BATCH_GYRO;
		expect.connects = 1;
		expect.failures = 2;
		n = imu_raw_status_encode(json, sizeof(json), &expect);
		check(n > 0 && br.msgs[1].length == (uint32_t)n &&
		      memcmp(br.msgs[1].json, json, n) == 0);
	}
	wait_for(br.pubacks_sent == 1 && br.out_len == 0, 500); /* PUBACK read */

	/* Ten batches, 28 ms apart. */
	first_raw = br.nmsgs;
	produce = 10;
	wait_for((imu_raw_mqtt_get_stats(&s), s.published == 10), 2000);
	check(count("/raw", 0) == 10);
	for (i = first_raw, n = 0; i < br.nmsgs; i++) {
		if (strcmp(br.msgs[i].topic, "imu/rp2040/raw") != 0) {
			continue;
		}
		check(br.msgs[i].qos == 0 && !br.msgs[i].retain && br.msgs[i].iovlen == 2 &&
		      br.msgs[i].length == batch_len && br.msgs[i].seq == (uint64_t)(1 + n));
		n++;
	}
	stats(&s);
	check(s.dropped_batches == 1 && s.queued == 0 && s.queue_high >= 1 && s.queue_high <= 2);
	check(s.publish_bytes == 10 * batch_len && s.publish_max_us > 0 &&
	      s.publish_total_us >= s.publish_max_us && s.lock_wait_count == 11);

	/* status every 1000 ms, the input every 100 ms. */
	t0 = now_ms();
	n = count("/status", 0);
	polls = br.polls_now;
	sleep_ms(2100);
	check(count("/status", 0) - n == 2);
	polls = br.polls_now - polls;
	check(polls >= 18 && polls <= 23);
	for (i = find("imu/rp2040/status", 2); i >= 0; i = find("imu/rp2040/status", i + 1)) {
		int j = find("imu/rp2040/status", i + 1);

		if (j >= 0) {
			int64_t d = br.msgs[j].at_ms - br.msgs[i].at_ms;

			check(d >= 1000 && d <= 1100);
		}
	}
	check(br.sends == 1); /* CONNECT; the PUBLISHes go by sendmsg() */
	(void)t0;
	say("ok  connection (CONNECT, info, status, batches, intervals)\n");
}

static void test_congestion(void)
{
	struct imu_raw_mqtt_stats s, before;
	int sent_before = count("/raw", 0), refused_before = refused;
	int n, from;

	stats(&before);
	br.stall = true;
	produce = 54; /* 1.5 s */
	wait_for(produce == 0, 3000);
	stats(&s);
	n = refused - refused_before;
	check(s.connected && s.queued == 8 && s.queue_high == 8);
	/* One batch is in the send that waits, eight in the queue. */
	/* (Eight and a status in the send that waits, if one was due.) */
	check(n == 54 - 9 || n == 54 - 8);
	check(s.dropped_batches - before.dropped_batches == (uint64_t)n &&
	      s.dropped_accel - before.dropped_accel == (uint64_t)n * BATCH_ACCEL &&
	      s.dropped_gyro - before.dropped_gyro == (uint64_t)n * BATCH_GYRO);
	br.stall = false;
	wait_for((imu_raw_mqtt_get_stats(&s), s.queued == 0), 2000);
	wait_for(count("/raw", 0) - sent_before == 54 - n, 1000);
	stats(&s);
	check(s.connects == 1 && s.published == before.published + (uint32_t)(54 - n) &&
	      s.connected);
	/* The next status reports it. */
	from = br.nmsgs;
	wait_for(find("imu/rp2040/status", from) >= 0, 1500);
	check(strstr(br.msgs[find("imu/rp2040/status", from)].json, "\"queue_high\":8"));
	say("ok  congestion (queue full at 8, %d of 54 dropped and counted, recovered)\n", n);
}

static void test_failure(void)
{
	struct imu_raw_mqtt_stats s, before;
	int closes = br.closes, n;

	stats(&before);
	/* Batches queued while the module takes nothing, then the connection
	 * fails: -EIO from the next input poll, the queue dropped.
	 */
	br.stall = true;
	produce = 4;
	wait_for(produce == 0, 1000);
	br.failed = true;
	br.stall = false;
	wait_for((imu_raw_mqtt_get_stats(&s), !s.connected), 500);
	wait_for(br.closes == closes + 1, 500);
	stats(&s);
	check(s.failures == before.failures + 1);
	/* One was in the send (EPIPE ends it as well), the others queued. */
	check(s.dropped_batches - before.dropped_batches == 4 && s.queued == 0);

	/* A new connection 1 s later, info first again. */
	n = br.nmsgs;
	wait_for((imu_raw_mqtt_get_stats(&s), s.connected), 1500);
	check(br.sockets_made == 2);
	stats(&s);
	check(s.connects == 2);
	wait_for(br.nmsgs >= n + 2, 500);
	check(strcmp(br.msgs[n].topic, "imu/rp2040/info") == 0 && br.msgs[n].id == 2);
	check(strcmp(br.msgs[n + 1].topic, "imu/rp2040/status") == 0 &&
	      strstr(br.msgs[n + 1].json, "\"connects\":2"));
	say("ok  failed connection (-EIO, queue dropped, reconnected, info again)\n");
}

static void test_connack(void)
{
	struct imu_raw_mqtt_stats s, before;
	int64_t t0;
	int made;

	/* Refused by the broker. */
	stats(&before);
	br.connack = CONNACK_REFUSED;
	br.failed = true;
	wait_for(br.sockets_made == 3 && !br.open, 2000);
	stats(&s);
	check(!s.connected && s.connects == before.connects);
	check(s.failures == before.failures + 2); /* -EIO, then -ECONNREFUSED */

	/* In two pieces: accepted. */
	br.connack = CONNACK_SPLIT;
	wait_for((imu_raw_mqtt_get_stats(&s), s.connected), 1500);
	check(br.sockets_made == 4 && s.connects == before.connects + 1);

	/* None: -ETIMEDOUT after 5000 ms. */
	br.connack = CONNACK_NONE;
	br.failed = true;
	made = br.sockets_made;
	wait_for(br.sockets_made == made + 1, 2000);
	t0 = now_ms();
	wait_for(!br.open, 6000);
	check(now_ms() - t0 >= 4900);
	stats(&s);
	check(!s.connected && s.failures == before.failures + 4);

	/* A refused TCP connection. */
	br.connack = CONNACK_OK;
	br.refuse_tcp = true;
	made = br.sockets_made;
	wait_for(br.sockets_made == made + 1 && !br.open, 2000);
	stats(&s);
	check(!s.connected && s.failures == before.failures + 5);
	br.refuse_tcp = false;
	wait_for((imu_raw_mqtt_get_stats(&s), s.connected), 1500);
	say("ok  CONNACK refused, in pieces, missing (5 s), TCP refused\n");
}

static void test_stop(void)
{
	struct imu_raw_mqtt_stats s, before;
	struct imu_raw_mqtt_config cfg;
	struct imu_raw_status status = {.state = IMU_RAW_STATE_STOPPED, .epoch_id = 1};
	int64_t t0;
	int closes = br.closes, rc;

	/* Queued batches are sent before the stop ends the connection. */
	stats(&before);
	br.stall = true;
	for (int i = 0; i < 3; i++) {
		check(submit() == 0);
	}
	imu_raw_mqtt_report(&status);
	br.stall = false;
	t0 = now_ms();
	imu_raw_mqtt_stop();
	check(now_ms() - t0 < 500);
	stats(&s);
	check(!s.running && !s.connected && s.published == before.published + 3 &&
	      s.dropped_batches == before.dropped_batches);
	check(strstr(br.msgs[last("imu/rp2040/status")].json, "\"state\":\"stopped\""));
	wait_for(br.closes == closes + 1, 500);

	/* The next start, once the thread has noticed. */
	config_init(&cfg, "rp2040", 18830);
	wait_for((rc = imu_raw_mqtt_start(&cfg)) != -EBUSY, 1000);
	check(rc == 0);
	wait_for((imu_raw_mqtt_get_stats(&s), s.connected), 1500);
	check(s.connects == 1 && s.failures == 0 && s.published == 0 && s.dropped_batches == 0);

	/* A stop while the module takes nothing: gives up after 500 ms. */
	br.stall = true;
	for (int i = 0; i < 3; i++) {
		check(submit() == 0);
	}
	sleep_ms(10);
	t0 = now_ms();
	imu_raw_mqtt_stop();
	check(now_ms() - t0 >= 490 && now_ms() - t0 < 700);
	stats(&s);
	check(!s.running && !s.connected && s.dropped_batches == 2); /* one is in the send */
	check(imu_raw_mqtt_start(&cfg) == -EBUSY);                   /* still in the send */
	br.stall = false;
	wait_for((rc = imu_raw_mqtt_start(&cfg)) != -EBUSY, 1000);
	check(rc == 0);
	wait_for((imu_raw_mqtt_get_stats(&s), s.connected), 1500);
	imu_raw_mqtt_stop();
	say("ok  stop (queue sent first; 500 ms limit)\n");
}

static void test_fixed_server(void)
{
	struct imu_raw_mqtt_config cfg;
	struct imu_raw_mqtt_stats s;
	struct imu_raw_status idle = {.state = IMU_RAW_STATE_IDLE};
	int rc, n;

	config_init(&cfg, "board-2_b", 1883);
	check(inet_pton(AF_INET, "192.168.1.5", &cfg.address) == 1);
	/* The transport's thread is above this task: it connects inside the
	 * start.  The state reported before the start (the start clears it) is
	 * idle.
	 */
	n = br.nmsgs;
	wait_for((rc = imu_raw_mqtt_start(&cfg)) != -EBUSY, 1000);
	check(rc == 0);
	imu_raw_mqtt_report(&idle);
	wait_for((imu_raw_mqtt_get_stats(&s), s.connected), 1500);
	check(!s.via_gateway && s.address.s_addr == cfg.address.s_addr && s.port == 1883);
	check(br.peer.sin_addr.s_addr == cfg.address.s_addr && br.peer.sin_port == htons(1883));
	check(strcmp(br.client_id, "imu-board-2_b") == 0);
	wait_for(find("imu/board-2_b/status", n) >= 0, 500);
	n = find("imu/board-2_b/status", n);
	check(strstr(br.msgs[n].json, "\"state\":\"idle\"") &&
	      strstr(br.msgs[n].json, "\"session\":\"503533748a8c92180000000700000000\"") &&
	      strstr(br.msgs[n].json, "\"uptime_ms\":"));
	check(find("imu/board-2_b/info", 0) < 0); /* no info this session */
	check(submit() == 0);
	wait_for(find("imu/board-2_b/raw", n) >= 0, 500);
	imu_raw_mqtt_stop();
	say("ok  fixed server, another device (topics, client ID, idle status)\n");
}

static void runner(void *arg)
{
	(void)arg;
	/* "imu start", "imu stop" and "imu status" run in the shell's task. */
	test_thread();
	test_no_network();
	test_connection();
	test_congestion();
	test_failure();
	test_connack();
	test_stop();
	test_fixed_server();
	say("imu_raw_mqtt: ok\n");
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
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, APP_PRIO_SHELL,
			NULL) != pdPASS ||
	    xTaskCreate(producer, "producer", PRODUCER_STACK / sizeof(StackType_t), NULL,
			APP_PRIO_IMU_STREAM, NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
