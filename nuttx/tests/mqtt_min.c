/* SPDX-License-Identifier: Apache-2.0 */
/* The minimal MQTT client (apps/imu/mqtt_min.c) on a fake socket: the bytes
 * it sends (CONNECT, PUBLISH QoS 0/1/2, PINGREQ, DISCONNECT, remaining
 * lengths), the CONNACK codes, packets that arrive in pieces, malformed
 * packets, short writes, socket errors, keep-alive, PUBACK and the order
 * of the events.  The expected values follow MQTT 3.1.1 (OASIS standard,
 * sections 2 and 3) and Zephyr's library.
 *
 * script/host_tests.sh builds this file twice: with the port's copy, and,
 * when the Zephyr tree is there, with Zephyr's own library
 * (the .c files of zephyr/zephyr/subsys/net/lib/mqtt on the shims of
 * tests/mqtt_ref),
 * so every expectation here is also checked against Zephyr's code.
 */
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>

#include "mqtt_min.h"
#include "zephyr_kernel_host.h"

/* The fake socket --------------------------------------------------------*/

#define FAKE_FD 7
#define MAX_SEGMENTS 64
#define MAX_CALLS 256

static struct {
	int sockets, connects, closes;
	int family, type, protocol;
	struct sockaddr_in addr;
	socklen_t addrlen;
	bool open;
	int socket_errno, connect_errno;
	/* Sent bytes, the calls and the iovec lengths of each sendmsg(). */
	uint8_t tx[40000];
	size_t tx_len;
	int sends, sendmsgs;
	size_t send_max;        /* most bytes one call takes (0: all) */
	int fail_call;          /* the send/sendmsg call (1-based) that fails */
	int fail_errno;
	size_t msg_iovlen[MAX_CALLS];
	size_t msg_iov0[MAX_CALLS], msg_iov1[MAX_CALLS];
	/* Received bytes, in the segments in which they "arrive". */
	uint8_t rx[4096];
	size_t rx_len, rx_pos;
	size_t seg_end[MAX_SEGMENTS];
	int segs, seg;
	bool rx_closed;
	int recv_errno;
	int recvs;
	size_t recv_asked[MAX_CALLS];
	int recv_flags[MAX_CALLS];
} fk;

static void fake_reset(void)
{
	memset(&fk, 0, sizeof(fk));
}

int mqtt_min_sock_socket(int family, int type, int protocol)
{
	fk.sockets++;
	fk.family = family;
	fk.type = type;
	fk.protocol = protocol;
	if (fk.socket_errno) {
		errno = fk.socket_errno;
		return -1;
	}
	assert(!fk.open);
	fk.open = true;
	return FAKE_FD;
}

int mqtt_min_sock_connect(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	assert(sock == FAKE_FD && fk.open);
	fk.connects++;
	fk.addrlen = addrlen;
	memcpy(&fk.addr, addr, sizeof(fk.addr));
	if (fk.connect_errno) {
		errno = fk.connect_errno;
		return -1;
	}
	return 0;
}

static bool send_fails(void)
{
	if (fk.fail_call && fk.sends + fk.sendmsgs == fk.fail_call) {
		errno = fk.fail_errno;
		return true;
	}
	return false;
}

ssize_t mqtt_min_sock_send(int sock, const void *buf, size_t len, int flags)
{
	assert(sock == FAKE_FD && fk.open && flags == 0 && len > 0);
	fk.sends++;
	if (send_fails()) {
		return -1;
	}
	if (fk.send_max && len > fk.send_max) {
		len = fk.send_max;
	}
	assert(fk.tx_len + len <= sizeof(fk.tx));
	memcpy(&fk.tx[fk.tx_len], buf, len);
	fk.tx_len += len;
	return len;
}

ssize_t mqtt_min_sock_sendmsg(int sock, const struct msghdr *msg, int flags)
{
	size_t taken = 0;

	assert(sock == FAKE_FD && fk.open && flags == 0);
	assert(fk.sendmsgs < MAX_CALLS);
	fk.msg_iovlen[fk.sendmsgs] = msg->msg_iovlen;
	fk.msg_iov0[fk.sendmsgs] = msg->msg_iovlen > 0 ? msg->msg_iov[0].iov_len : 0;
	fk.msg_iov1[fk.sendmsgs] = msg->msg_iovlen > 1 ? msg->msg_iov[1].iov_len : 0;
	fk.sendmsgs++;
	if (send_fails()) {
		return -1;
	}
	for (size_t i = 0; i < (size_t)msg->msg_iovlen; i++) {
		size_t n = msg->msg_iov[i].iov_len;

		if (fk.send_max && taken + n > fk.send_max) {
			n = fk.send_max - taken;
		}
		assert(fk.tx_len + n <= sizeof(fk.tx));
		memcpy(&fk.tx[fk.tx_len], msg->msg_iov[i].iov_base, n);
		fk.tx_len += n;
		taken += n;
	}
	return taken;
}

ssize_t mqtt_min_sock_recv(int sock, void *buf, size_t len, int flags)
{
	size_t n;

	assert(sock == FAKE_FD && fk.open && len > 0);
	assert(fk.recvs < MAX_CALLS);
	fk.recv_asked[fk.recvs] = len;
	fk.recv_flags[fk.recvs] = flags;
	fk.recvs++;
	if (fk.recv_errno) {
		errno = fk.recv_errno;
		return -1;
	}
	while (fk.seg < fk.segs && fk.rx_pos == fk.seg_end[fk.seg]) {
		fk.seg++;
	}
	if (fk.seg == fk.segs) {
		if (fk.rx_closed) {
			return 0;
		}
		assert(flags & MSG_DONTWAIT); /* a blocking read would hang */
		errno = EAGAIN;
		return -1;
	}
	n = fk.seg_end[fk.seg] - fk.rx_pos;
	if (n > len) {
		n = len;
	}
	memcpy(buf, &fk.rx[fk.rx_pos], n);
	fk.rx_pos += n;
	return n;
}

int mqtt_min_sock_close(int sock)
{
	assert(sock == FAKE_FD && fk.open);
	fk.open = false;
	fk.closes++;
	return 0;
}

/* One segment of received bytes. */
static void rx_push(const uint8_t *data, size_t len)
{
	assert(fk.segs < MAX_SEGMENTS && fk.rx_len + len <= sizeof(fk.rx));
	memcpy(&fk.rx[fk.rx_len], data, len);
	fk.rx_len += len;
	fk.seg_end[fk.segs++] = fk.rx_len;
}

#define RX(...)                                                   \
	do {                                                      \
		static const uint8_t bytes_[] = {__VA_ARGS__};    \
		rx_push(bytes_, sizeof(bytes_));                  \
	} while (0)

static void tx_expect(const uint8_t *data, size_t len)
{
	if (fk.tx_len != len || memcmp(fk.tx, data, len) != 0) {
		fprintf(stderr, "sent %zu bytes, expected %zu:", fk.tx_len, len);
		for (size_t i = 0; i < fk.tx_len && i < 64; i++) {
			fprintf(stderr, " %02x", fk.tx[i]);
		}
		fprintf(stderr, "\n");
		assert(0);
	}
	fk.tx_len = 0;
}

#define TX_EXPECT(...)                                            \
	do {                                                      \
		static const uint8_t bytes_[] = {__VA_ARGS__};    \
		tx_expect(bytes_, sizeof(bytes_));                \
	} while (0)

/* The events -------------------------------------------------------------*/

#define MAX_EVENTS 16

static struct mqtt_evt events[MAX_EVENTS];
static int event_count;

static void on_event(struct mqtt_client *client, const struct mqtt_evt *evt)
{
	assert(client != NULL);
	assert(event_count < MAX_EVENTS);
	events[event_count++] = *evt;
}

static void events_reset(void)
{
	event_count = 0;
	memset(events, 0, sizeof(events));
}

static void expect_event(int i, enum mqtt_evt_type type, int result)
{
	assert(i < event_count);
	assert(events[i].type == type);
	assert(events[i].result == result);
}

/* The client as imu_raw_mqtt.c sets it up --------------------------------*/

static struct mqtt_client client;
static struct sockaddr_in broker;
static uint8_t rx_buf[512], tx_buf[512];
static const char client_id[] = "imu-dev1";

static void setup(void)
{
	fake_reset();
	events_reset();
	broker.sin_family = AF_INET;
	broker.sin_port = htons(18830);
	broker.sin_addr.s_addr = htonl(0x0a2a0001); /* 10.42.0.1 */
	mqtt_client_init(&client);
	client.broker = (struct sockaddr *)&broker;
	client.evt_cb = on_event;
	client.client_id.utf8 = (const uint8_t *)client_id;
	client.client_id.size = strlen(client_id);
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.transport.type = MQTT_TRANSPORT_NON_SECURE;
	client.rx_buf = rx_buf;
	client.rx_buf_size = sizeof(rx_buf);
	client.tx_buf = tx_buf;
	client.tx_buf_size = sizeof(tx_buf);
}

/* CONNECT of "imu-dev1", MQTT 3.1.1, flags 0 (no clean session, will, user
 * or password), keep-alive 60 s (3.1.2): remaining length 10 + 2 + 8.
 */
#define CONNECT_BYTES                                                       \
	0x10, 0x14, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x00, 0x00, 0x3c, \
	0x00, 0x08, 'i', 'm', 'u', '-', 'd', 'e', 'v', '1'

/* Connected: CONNECT sent, CONNACK accepted. */
static void connect_ok(void)
{
	setup();
	assert(mqtt_connect(&client) == 0);
	TX_EXPECT(CONNECT_BYTES);
	RX(0x20, 0x02, 0x00, 0x00);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_CONNACK, 0);
	events_reset();
	fk.recvs = 0;
	fk.sends = 0;
	fk.sendmsgs = 0;
}

/* Tests ------------------------------------------------------------------*/

static void test_init_defaults(void)
{
	struct mqtt_client c;

	memset(&c, 0xa5, sizeof(c));
	mqtt_client_init(&c);
	assert(c.protocol_version == MQTT_VERSION_3_1_1);
	assert(c.keepalive == 60);
	assert(c.clean_session == 0);
	assert(c.internal.state == 0 && c.internal.last_activity == 0);
	assert(c.evt_cb == NULL && c.broker == NULL && c.will_topic == NULL);
	assert(c.user_name == NULL && c.password == NULL && c.unacked_ping == 0);
	assert(c.transport.type == MQTT_TRANSPORT_NON_SECURE);
}

static void test_connect(void)
{
	test_time_set_us(5000000);
	setup();
	assert(mqtt_connect(&client) == 0);
	assert(fk.sockets == 1 && fk.family == AF_INET && fk.type == SOCK_STREAM &&
	       fk.protocol == IPPROTO_TCP);
	assert(fk.connects == 1 && fk.addrlen == sizeof(struct sockaddr_in));
	assert(fk.addr.sin_port == htons(18830) && fk.addr.sin_addr.s_addr == htonl(0x0a2a0001));
	assert(client.transport.tcp.sock == FAKE_FD);
	assert(fk.sends == 1 && fk.sendmsgs == 0); /* one write */
	TX_EXPECT(CONNECT_BYTES);
	assert(client.internal.last_activity == 5000);
	assert(event_count == 0);

	/* Before CONNACK: no PUBLISH, PING or DISCONNECT. */
	{
		struct mqtt_publish_param p = {0};

		p.message.topic.topic.utf8 = (const uint8_t *)"t";
		p.message.topic.topic.size = 1;
		assert(mqtt_publish(&client, &p) == -ENOTCONN);
		assert(mqtt_ping(&client) == -ENOTCONN);
		assert(mqtt_disconnect(&client, NULL) == -ENOTCONN);
		assert(fk.tx_len == 0 && fk.open);
	}

	/* Nothing has arrived: 0, no event; reads do not wait. */
	assert(mqtt_input(&client) == 0);
	assert(event_count == 0);
	assert(fk.recvs == 1 && fk.recv_asked[0] == 2 && (fk.recv_flags[0] & MSG_DONTWAIT));

	/* CONNACK accepted, session present. */
	RX(0x20, 0x02, 0x01, 0x00);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_CONNACK, 0);
	assert(events[0].param.connack.return_code == MQTT_CONNECTION_ACCEPTED);
	assert(events[0].param.connack.session_present_flag == 1);
	assert(fk.open && fk.closes == 0);
}

/* The CONNECT fields that imu_raw_mqtt.c does not use: clean session,
 * user name, password, will (3.1.2.3-3.1.3).
 */
static void test_connect_options(void)
{
	struct mqtt_utf8 user = MQTT_UTF8_LITERAL("u");
	struct mqtt_utf8 pass = MQTT_UTF8_LITERAL("pw");
	struct mqtt_utf8 will_msg = MQTT_UTF8_LITERAL("bye");
	struct mqtt_topic will = {.topic = MQTT_UTF8_LITERAL("w"), .qos = 1};

	setup();
	client.clean_session = 1;
	client.keepalive = 0x1234;
	client.user_name = &user;
	client.password = &pass;
	assert(mqtt_connect(&client) == 0);
	TX_EXPECT(0x10, 0x1b, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0xc2, 0x12, 0x34,
		  0x00, 0x08, 'i', 'm', 'u', '-', 'd', 'e', 'v', '1',
		  0x00, 0x01, 'u', 0x00, 0x02, 'p', 'w');
	assert(mqtt_abort(&client) == 0);

	setup();
	client.will_topic = &will;
	client.will_retain = 1;
	assert(mqtt_connect(&client) == 0);
	TX_EXPECT(0x10, 0x19, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x2c, 0x00, 0x3c,
		  0x00, 0x08, 'i', 'm', 'u', '-', 'd', 'e', 'v', '1',
		  0x00, 0x01, 'w', 0x00, 0x00);
	assert(mqtt_abort(&client) == 0);

	setup();
	client.will_topic = &will;
	client.will_message = &will_msg;
	assert(mqtt_connect(&client) == 0);
	TX_EXPECT(0x10, 0x1c, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x0c, 0x00, 0x3c,
		  0x00, 0x08, 'i', 'm', 'u', '-', 'd', 'e', 'v', '1',
		  0x00, 0x01, 'w', 0x00, 0x03, 'b', 'y', 'e');
	assert(mqtt_abort(&client) == 0);

	/* MQTT 3.1: protocol name "MQIsdp", level 3. */
	setup();
	client.protocol_version = MQTT_VERSION_3_1_0;
	assert(mqtt_connect(&client) == 0);
	TX_EXPECT(0x10, 0x16, 0x00, 0x06, 'M', 'Q', 'I', 's', 'd', 'p', 0x03, 0x00, 0x00, 0x3c,
		  0x00, 0x08, 'i', 'm', 'u', '-', 'd', 'e', 'v', '1');
	assert(mqtt_abort(&client) == 0);
}

/* CONNACK return codes 1-5 (3.2.2.3): CONNACK with the code, then
 * DISCONNECT with -ECONNREFUSED; the socket is closed.
 */
static void test_connack_refused(void)
{
	for (uint8_t code = 1; code <= 6; code++) {
		uint8_t connack[] = {0x20, 0x02, 0x00, code};

		setup();
		assert(mqtt_connect(&client) == 0);
		rx_push(connack, sizeof(connack));
		assert(mqtt_input(&client) == -ECONNREFUSED);
		assert(event_count == 2);
		expect_event(0, MQTT_EVT_CONNACK, code);
		assert(events[0].param.connack.return_code == code);
		expect_event(1, MQTT_EVT_DISCONNECT, -ECONNREFUSED);
		assert(!fk.open && fk.closes == 1);
		assert(client.internal.state == 0);
		/* Closed: no input, abort does nothing. */
		assert(mqtt_input(&client) == -ENOTCONN);
		assert(mqtt_abort(&client) == 0);
		assert(event_count == 2 && fk.closes == 1);
	}
}

/* Packets that arrive in pieces: the client reads exactly what it needs
 * (2 bytes of fixed header, then one byte at a time while the remaining
 * length continues, then the rest), keeps the partial packet and returns 0.
 */
static void test_split_receive(void)
{
	static uint8_t suback[3 + 130];

	setup();
	assert(mqtt_connect(&client) == 0);
	fk.tx_len = 0;
	RX(0x20);
	RX(0x02);
	RX(0x00);
	RX(0x00);
	fk.recvs = 0;
	/* 0x20 of the 2 bytes asked for; then 0x02 and 0x00 of the 2 bytes
	 * of the rest; then the last byte.
	 */
	assert(mqtt_input(&client) == 0 && event_count == 0);
	assert(client.internal.rx_buf_datalen == 1);
	assert(mqtt_input(&client) == 0 && event_count == 0);
	assert(client.internal.rx_buf_datalen == 3);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_CONNACK, 0);
	assert(fk.recvs == 4 && fk.recv_asked[0] == 2 && fk.recv_asked[1] == 1 &&
	       fk.recv_asked[2] == 2 && fk.recv_asked[3] == 1);
	assert(client.internal.rx_buf_datalen == 0);
	events_reset();

	/* Two packets in one segment: one per call. */
	fk.recvs = 0;
	RX(0x40, 0x02, 0x00, 0x07, 0xd0, 0x00);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_PUBACK, 0);
	assert(events[0].param.puback.message_id == 7);
	assert(fk.recvs == 2 && fk.recv_asked[0] == 2 && fk.recv_asked[1] == 2);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 2);
	expect_event(1, MQTT_EVT_PINGRESP, 0);
	events_reset();

	/* A two-byte remaining length (130) split after its first byte. */
	suback[0] = 0x90;
	suback[1] = 0x82;
	suback[2] = 0x01;
	suback[3] = 0x12;
	suback[4] = 0x34;
	for (int i = 5; i < (int)sizeof(suback); i++) {
		suback[i] = i & 1 ? 0x80 : 0x01;
	}
	fk.recvs = 0;
	rx_push(suback, 2);
	rx_push(&suback[2], 40);
	rx_push(&suback[42], sizeof(suback) - 42);
	assert(mqtt_input(&client) == 0 && event_count == 0);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_SUBACK, 0);
	assert(events[0].param.suback.message_id == 0x1234);
	assert(events[0].param.suback.return_codes.len == 128);
	assert(events[0].param.suback.return_codes.data[0] == 0x80);
	assert(fk.recvs == 4 && fk.recv_asked[0] == 2 && fk.recv_asked[1] == 1 &&
	       fk.recv_asked[2] == 130 && fk.recv_asked[3] == 91);
	assert(fk.open);
}

/* Malformed or unexpected packets. */
static void test_malformed(void)
{
	/* A remaining length of more than 4 bytes (2.2.3). */
	connect_ok();
	RX(0x30, 0xff, 0xff, 0xff, 0xff, 0x7f);
	assert(mqtt_input(&client) == -EINVAL);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, -EINVAL);
	assert(!fk.open && fk.closes == 1);

	/* A short CONNACK: CONNACK with the error, then DISCONNECT. */
	setup();
	assert(mqtt_connect(&client) == 0);
	RX(0x20, 0x01, 0x00);
	assert(mqtt_input(&client) == -EINVAL);
	assert(event_count == 2);
	expect_event(0, MQTT_EVT_CONNACK, -EINVAL);
	expect_event(1, MQTT_EVT_DISCONNECT, -EINVAL);
	assert(!fk.open);

	/* A short PUBACK. */
	connect_ok();
	RX(0x40, 0x01, 0x00);
	assert(mqtt_input(&client) == -EINVAL);
	assert(event_count == 2);
	expect_event(0, MQTT_EVT_PUBACK, -EINVAL);
	expect_event(1, MQTT_EVT_DISCONNECT, -EINVAL);

	/* A packet larger than the receive buffer (3 + 639 > 512). */
	connect_ok();
	RX(0x40, 0xff, 0x04);
	assert(mqtt_input(&client) == -ENOMEM);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, -ENOMEM);

	/* The broker closes the connection. */
	connect_ok();
	fk.rx_closed = true;
	assert(mqtt_input(&client) == -ENOTCONN);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, -ENOTCONN);
	assert(!fk.open);

	/* A receive error. */
	connect_ok();
	fk.recv_errno = ECONNRESET;
	assert(mqtt_input(&client) == -ECONNRESET);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, -ECONNRESET);

	/* Unknown types (a CONNECT, type 0) are skipped without an event. */
	connect_ok();
	RX(0x10, 0x00);
	RX(0x00, 0x01, 0x55);
	RX(0xd0, 0x00);
	assert(mqtt_input(&client) == 0 && event_count == 0);
	assert(mqtt_input(&client) == 0 && event_count == 0);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_PINGRESP, 0);
	assert(fk.open);

	/* A PUBACK with more bytes than 3.1.1 has (MQTT 5.0's reason code) is
	 * accepted.
	 */
	connect_ok();
	RX(0x40, 0x03, 0x00, 0x09, 0x10);
	assert(mqtt_input(&client) == 0);
	expect_event(0, MQTT_EVT_PUBACK, 0);
	assert(events[0].param.puback.message_id == 9 && fk.open);

	/* A PUBLISH whose remaining length is shorter than its topic. */
	connect_ok();
	RX(0x30, 0x01, 0x00, 0x00);
	assert(mqtt_input(&client) == -EINVAL);
	assert(event_count == 2);
	expect_event(0, MQTT_EVT_PUBLISH, -EINVAL);
	expect_event(1, MQTT_EVT_DISCONNECT, -EINVAL);
}

/* A PUBLISH from the broker (not subscribed, but possible): the event
 * carries the payload length; until the payload is read, mqtt_input()
 * returns -EBUSY and the connection stays open.
 */
static void test_publish_received(void)
{
	uint8_t payload[8];

	connect_ok();
	RX(0x33, 0x09, 0x00, 0x02, 'a', 'b', 0x00, 0x05, 'x', 'y', 'z');
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_PUBLISH, 0);
	assert(events[0].param.publish.message.topic.qos == 1);
	assert(events[0].param.publish.retain_flag == 1);
	assert(events[0].param.publish.message_id == 5);
	assert(events[0].param.publish.message.topic.topic.size == 2);
	assert(events[0].param.publish.message.payload.len == 3);
	assert(mqtt_input(&client) == -EBUSY);
	assert(event_count == 1 && fk.open);
	assert(mqtt_read_publish_payload(&client, payload, sizeof(payload)) == 3);
	assert(memcmp(payload, "xyz", 3) == 0);
	assert(mqtt_read_publish_payload(&client, payload, sizeof(payload)) == 0);
	RX(0xd0, 0x00);
	assert(mqtt_input(&client) == 0);
	expect_event(1, MQTT_EVT_PINGRESP, 0);
}

static void publish(enum mqtt_qos qos, uint16_t id, bool retain, bool dup, const char *topic,
		    uint8_t *payload, size_t len, int expect)
{
	struct mqtt_publish_param p = {0};

	p.message.topic.topic.utf8 = (const uint8_t *)topic;
	p.message.topic.topic.size = strlen(topic);
	p.message.topic.qos = qos;
	p.message.payload.data = payload;
	p.message.payload.len = len;
	p.message_id = id;
	p.retain_flag = retain;
	p.dup_flag = dup;
	assert(mqtt_publish(&client, &p) == expect);
}

/* PUBLISH (3.3): fixed header flags, topic, packet identifier for QoS > 0,
 * payload; the header from the TX buffer and the payload from the caller
 * in one sendmsg() of two iovecs.
 */
static void test_publish(void)
{
	static uint8_t big[16384];
	static const struct {
		size_t len;
		uint8_t rl[4];
		size_t rl_bytes;
	} lengths[] = {
		{127 - 3, {0x7f}, 1},
		{128 - 3, {0x80, 0x01}, 2},
		{16383 - 3, {0xff, 0x7f}, 2},
		{16384 - 3, {0x80, 0x80, 0x01}, 3},
	};
	uint8_t raw[2448];

	for (size_t i = 0; i < sizeof(raw); i++) {
		raw[i] = i * 7;
	}
	for (size_t i = 0; i < sizeof(big); i++) {
		big[i] = i * 13;
	}

	/* A raw batch, QoS 0: remaining length 2 + 12 + 2448 = 2462. */
	connect_ok();
	test_time_set_us(9000000);
	publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "imu/dev1/raw", raw, sizeof(raw), 0);
	assert(fk.sendmsgs == 1 && fk.sends == 0);
	assert(fk.msg_iovlen[0] == 2 && fk.msg_iov0[0] == 17 && fk.msg_iov1[0] == sizeof(raw));
	{
		static const uint8_t head[] = {0x30, 0x9e, 0x13, 0x00, 0x0c, 'i', 'm', 'u', '/', 'd',
					       'e', 'v', '1', '/', 'r', 'a', 'w'};

		assert(fk.tx_len == sizeof(head) + sizeof(raw));
		assert(memcmp(fk.tx, head, sizeof(head)) == 0);
		assert(memcmp(&fk.tx[sizeof(head)], raw, sizeof(raw)) == 0);
		fk.tx_len = 0;
	}
	assert(client.internal.last_activity == 9000);

	/* info, QoS 1, retained, message ID 1. */
	publish(MQTT_QOS_1_AT_LEAST_ONCE, 1, true, false, "imu/dev1/info", (uint8_t *)"{}", 2, 0);
	TX_EXPECT(0x33, 0x13, 0x00, 0x0d, 'i', 'm', 'u', '/', 'd', 'e', 'v', '1', '/', 'i', 'n',
		  'f', 'o', 0x00, 0x01, '{', '}');
	/* status, QoS 0, not retained, empty payload. */
	publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "s", NULL, 0, 0);
	TX_EXPECT(0x30, 0x03, 0x00, 0x01, 's');
	/* DUP with QoS 1, and QoS 2. */
	publish(MQTT_QOS_1_AT_LEAST_ONCE, 0xabcd, false, true, "t", (uint8_t *)"x", 1, 0);
	TX_EXPECT(0x3a, 0x06, 0x00, 0x01, 't', 0xab, 0xcd, 'x');
	publish(MQTT_QOS_2_EXACTLY_ONCE, 2, false, false, "t", (uint8_t *)"x", 1, 0);
	TX_EXPECT(0x34, 0x06, 0x00, 0x01, 't', 0x00, 0x02, 'x');
	/* QoS > 0 without a message ID: -EINVAL, nothing sent, still open. */
	publish(MQTT_QOS_1_AT_LEAST_ONCE, 0, false, false, "t", (uint8_t *)"x", 1, -EINVAL);
	assert(fk.tx_len == 0 && fk.open && event_count == 0);
	/* A topic that does not fit the TX buffer: -ENOMEM. */
	{
		static char long_topic[600];

		memset(long_topic, 'a', sizeof(long_topic) - 1);
		publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, long_topic, NULL, 0, -ENOMEM);
		assert(fk.tx_len == 0 && fk.open && event_count == 0);
	}

	/* Remaining length encodings (2.2.3): topic "t" (3 bytes). */
	for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++) {
		publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "t", big, lengths[i].len, 0);
		assert(fk.tx[0] == 0x30);
		assert(memcmp(&fk.tx[1], lengths[i].rl, lengths[i].rl_bytes) == 0);
		assert(fk.tx[1 + lengths[i].rl_bytes] == 0 && fk.tx[2 + lengths[i].rl_bytes] == 1);
		assert(fk.tx[3 + lengths[i].rl_bytes] == 't');
		assert(fk.tx_len == 4 + lengths[i].rl_bytes + lengths[i].len);
		assert(memcmp(&fk.tx[4 + lengths[i].rl_bytes], big, lengths[i].len) == 0);
		fk.tx_len = 0;
	}
	assert(event_count == 0 && fk.closes == 0);
}

/* PUBACK of a QoS 1 PUBLISH (3.4). */
static void test_puback(void)
{
	connect_ok();
	publish(MQTT_QOS_1_AT_LEAST_ONCE, 0x0102, true, false, "i", (uint8_t *)"{}", 2, 0);
	TX_EXPECT(0x33, 0x07, 0x00, 0x01, 'i', 0x01, 0x02, '{', '}');
	RX(0x40, 0x02, 0x01, 0x02);
	assert(mqtt_input(&client) == 0);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_PUBACK, 0);
	assert(events[0].param.puback.message_id == 0x0102);
	/* The client sends nothing back. */
	assert(fk.tx_len == 0 && fk.open);
}

/* Short writes: the rest is sent in further calls, the stream is the same. */
static void test_short_writes(void)
{
	uint8_t payload[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

	setup();
	fk.send_max = 3;
	assert(mqtt_connect(&client) == 0);
	assert(fk.sends == 8); /* 22 bytes, 3 at a time */
	TX_EXPECT(CONNECT_BYTES);
	RX(0x20, 0x02, 0x00, 0x00);
	assert(mqtt_input(&client) == 0);
	events_reset();

	/* 7 bytes of header and 10 of payload, 4 at a time: the iovecs move
	 * on across their boundary.
	 */
	fk.send_max = 4;
	publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "abc", payload, sizeof(payload), 0);
	assert(fk.sendmsgs == 5);
	assert(fk.msg_iov0[0] == 7 && fk.msg_iov1[0] == 10);
	assert(fk.msg_iov0[1] == 3 && fk.msg_iov1[1] == 10);
	assert(fk.msg_iov0[2] == 0 && fk.msg_iov1[2] == 9);
	assert(fk.msg_iov0[3] == 0 && fk.msg_iov1[3] == 5);
	assert(fk.msg_iov0[4] == 0 && fk.msg_iov1[4] == 1);
	TX_EXPECT(0x30, 0x0f, 0x00, 0x03, 'a', 'b', 'c', 0, 1, 2, 3, 4, 5, 6, 7, 8, 9);
	assert(event_count == 0 && fk.open);
}

/* Socket errors on the way out. */
static void test_send_errors(void)
{
	/* socket() fails: nothing to close. */
	setup();
	fk.socket_errno = EMFILE;
	assert(mqtt_connect(&client) == -EMFILE);
	assert(fk.connects == 0 && fk.closes == 0 && event_count == 0);
	assert(client.internal.state == 0);

	/* connect() fails: the socket is closed, no event. */
	setup();
	fk.connect_errno = ECONNREFUSED;
	assert(mqtt_connect(&client) == -ECONNREFUSED);
	assert(fk.closes == 1 && !fk.open && event_count == 0 && fk.tx_len == 0);
	assert(client.internal.state == 0);

	/* Sending CONNECT fails: closed, no event. */
	setup();
	fk.fail_call = 1;
	fk.fail_errno = EPIPE;
	assert(mqtt_connect(&client) == -EPIPE);
	assert(fk.closes == 1 && !fk.open && event_count == 0);
	assert(mqtt_input(&client) == -ENOTCONN);

	/* CONNECT does not fit the TX buffer: closed, no event. */
	setup();
	client.tx_buf_size = 20;
	assert(mqtt_connect(&client) == -ENOMEM);
	assert(fk.closes == 1 && fk.tx_len == 0 && event_count == 0);

	/* No buffers: nothing is opened. */
	setup();
	client.rx_buf = NULL;
	assert(mqtt_connect(&client) == -ENOMEM);
	assert(fk.sockets == 0);
	setup();
	client.client_id.utf8 = NULL;
	assert(mqtt_connect(&client) == -EINVAL);
	assert(fk.sockets == 0);

	/* PUBLISH fails (after a short write): DISCONNECT with the error. */
	connect_ok();
	fk.send_max = 5;
	fk.fail_call = 2;
	fk.fail_errno = ECONNRESET;
	publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "imu/dev1/raw", (uint8_t *)"0123456789",
		10, -ECONNRESET);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, -ECONNRESET);
	assert(!fk.open && fk.closes == 1);
	publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "t", NULL, 0, -ENOTCONN);
	assert(mqtt_abort(&client) == 0 && event_count == 1);
}

/* Keep-alive (3.1.2.10): PINGREQ when keepalive seconds have passed since
 * the last transmission; PINGRESP counts down unacked_ping.
 */
static void test_keepalive(void)
{
	test_time_set_us(1000000);
	connect_ok();
	assert(client.internal.last_activity == 1000);
	test_time_set_us((1000 + 59999) * 1000ULL + 999);
	assert(mqtt_keepalive_time_left(&client) == 1);
	assert(mqtt_live(&client) == -EAGAIN);
	assert(fk.tx_len == 0);
	test_time_set_us((1000 + 60000) * 1000ULL);
	assert(mqtt_keepalive_time_left(&client) == 0);
	assert(mqtt_live(&client) == 0);
	TX_EXPECT(0xc0, 0x00);
	assert(client.unacked_ping == 1);
	assert(client.internal.last_activity == 61000);
	assert(mqtt_keepalive_time_left(&client) == 60000);
	assert(mqtt_live(&client) == -EAGAIN);
	RX(0xd0, 0x00);
	assert(mqtt_input(&client) == 0);
	expect_event(0, MQTT_EVT_PINGRESP, 0);
	assert(client.unacked_ping == 0);
	/* An unexpected PINGRESP leaves it at 0. */
	RX(0xd0, 0x00);
	assert(mqtt_input(&client) == 0);
	expect_event(1, MQTT_EVT_PINGRESP, 0);
	assert(client.unacked_ping == 0);

	/* A PUBLISH restarts the interval; receiving does not. */
	test_time_set_us(100000 * 1000ULL);
	publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "t", NULL, 0, 0);
	fk.tx_len = 0;
	test_time_set_us((100000 + 59999) * 1000ULL);
	RX(0xd0, 0x00);
	assert(mqtt_input(&client) == 0);
	assert(mqtt_live(&client) == -EAGAIN);
	test_time_set_us((100000 + 60000) * 1000ULL);
	assert(mqtt_live(&client) == 0);
	TX_EXPECT(0xc0, 0x00);

	/* The millisecond clock wraps at 2^32. */
	test_time_set_us((0x100000000ULL - 1000) * 1000);
	publish(MQTT_QOS_0_AT_MOST_ONCE, 0, false, false, "t", NULL, 0, 0);
	fk.tx_len = 0;
	assert(client.internal.last_activity == 0xfffffc18);
	test_time_set_us((0x100000000ULL + 58999) * 1000);
	assert(mqtt_live(&client) == -EAGAIN);
	test_time_set_us((0x100000000ULL + 59000) * 1000);
	assert(mqtt_live(&client) == 0);
	TX_EXPECT(0xc0, 0x00);

	/* A PINGREQ that fails: the error, DISCONNECT, and the count still
	 * goes up (as in Zephyr's mqtt_ping()).
	 */
	events_reset();
	client.unacked_ping = 0;
	test_time_set_us((0x100000000ULL + 200000) * 1000);
	fk.fail_call = fk.sends + fk.sendmsgs + 1;
	fk.fail_errno = EPIPE;
	assert(mqtt_live(&client) == -EPIPE);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, -EPIPE);
	assert(client.unacked_ping == 1 && !fk.open);

	/* Before CONNACK, a due PINGREQ is -ENOTCONN (not -EAGAIN). */
	test_time_set_us(0);
	setup();
	assert(mqtt_connect(&client) == 0);
	fk.tx_len = 0;
	test_time_set_us(60000 * 1000ULL);
	assert(mqtt_live(&client) == -ENOTCONN);
	assert(fk.tx_len == 0 && client.unacked_ping == 0);

	/* keepalive 0: never. */
	test_time_set_us(0);
	setup();
	client.keepalive = 0;
	assert(mqtt_connect(&client) == 0);
	fk.tx_len = 0;
	RX(0x20, 0x02, 0x00, 0x00);
	assert(mqtt_input(&client) == 0);
	test_time_set_us(1000000 * 1000ULL);
	assert(mqtt_live(&client) == -EAGAIN);
	assert(mqtt_keepalive_time_left(&client) == -1);
	assert(fk.tx_len == 0);
}

/* DISCONNECT (3.14) and abort. */
static void test_disconnect_abort(void)
{
	connect_ok();
	assert(mqtt_disconnect(&client, NULL) == 0);
	TX_EXPECT(0xe0, 0x00);
	assert(event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, 0);
	assert(!fk.open && fk.closes == 1);
	assert(mqtt_abort(&client) == 0 && event_count == 1);

	/* Abort: no DISCONNECT packet, event with -ECONNABORTED, once. */
	connect_ok();
	assert(mqtt_abort(&client) == 0);
	assert(fk.tx_len == 0 && event_count == 1);
	expect_event(0, MQTT_EVT_DISCONNECT, -ECONNABORTED);
	assert(!fk.open && fk.closes == 1);
	assert(mqtt_abort(&client) == 0 && event_count == 1);
	assert(mqtt_input(&client) == -ENOTCONN);

	/* Abort before CONNACK also closes (state TCP_CONNECTED). */
	setup();
	assert(mqtt_connect(&client) == 0);
	assert(mqtt_abort(&client) == 0);
	expect_event(0, MQTT_EVT_DISCONNECT, -ECONNABORTED);
	assert(!fk.open);

	/* Abort of a client that never connected: nothing (imu_raw_mqtt.c
	 * calls it after every failed attempt).
	 */
	setup();
	assert(mqtt_abort(&client) == 0);
	assert(event_count == 0 && fk.closes == 0);

	/* Reconnecting with the same client after a disconnection. */
	connect_ok();
	assert(mqtt_abort(&client) == 0);
	fk.tx_len = 0;
	fk.rx_len = fk.rx_pos = 0;
	fk.segs = fk.seg = 0;
	assert(mqtt_connect(&client) == 0);
	TX_EXPECT(CONNECT_BYTES);
	assert(fk.sockets == 2 && fk.open);
}

int main(void)
{
	test_init_defaults();
	test_connect();
	test_connect_options();
	test_connack_refused();
	test_split_receive();
	test_malformed();
	test_publish_received();
	test_publish();
	test_puback();
	test_short_writes();
	test_send_errors();
	test_keepalive();
	test_disconnect_abort();
	printf("mqtt_min: all tests passed\n");
	return 0;
}
