/* SPDX-License-Identifier: MIT */
/* A minimal iperf3 for the port's "zperf" command (<zephyr/net/zperf.h>):
 * the TCP tests of the iperf3 protocol, both ways, on the zsock_*() calls
 * of the compatibility layer.  Written for the FreeRTOS build in place of
 * Zephyr's zperf (subsys/net/lib/zperf: about 5,400 lines with a JSON
 * library, a work queue and the socket service), with its behaviour where
 * the port has the feature.  The Zephyr build uses Zephyr's zperf.
 *
 * The protocol (iperf3 3.x; checked against iperf 3.16 on the host,
 * freertos/tests/telnet).  A test is a TCP control connection and one TCP
 * data connection to the server's port.  The client opens the control
 * connection and sends a cookie of 36 characters and a NUL; the server then
 * moves the test on with single signed bytes, the states below.  After
 * PARAM_EXCHANGE the client sends its parameters, JSON with a 32-bit
 * big-endian length in front; after CREATE_STREAMS it opens the data
 * connection and sends the cookie again; after TEST_START and TEST_RUNNING
 * it sends data until its time is up and then TEST_END; after
 * EXCHANGE_RESULTS each side sends its results (JSON, the same framing),
 * the client first; DISPLAY_RESULTS from the server and IPERF_DONE from
 * the client end the test.  A server that is busy answers a new connection
 * with ACCESS_DENIED; one that refuses the parameters answers SERVER_ERROR
 * with an error number and an errno (32 bits each).
 *
 * What the port does (as Zephyr's zperf with CONFIG_NET_ZPERF_IPERF3,
 * CONFIG_NET_IPV4 only, CONFIG_NET_ZPERF_LOG_LEVEL 0: no log):
 *
 * - zperf_tcp_upload(): the client, in the calling thread (the shell
 *   session that runs "zperf tcp upload"), one test at a time (-EBUSY).  It
 *   sends blocks of the requested size (at most ZPERF_PACKET_SIZE_MAX)
 *   with blocking sends and yields after each, as Zephyr's send_tcp();
 *   waits for the server's states with a 10 s receive timeout (-ETIMEDOUT).
 *   The parameters say "tcp", the time in whole seconds rounded up, one
 *   stream, the block size, pacing_timer 1000, "nodelay" when asked, and
 *   client_version "zperf-1.1".
 * - zperf_tcp_download(): the server, in the thread zperf_server_tid
 *   (Zephyr's net_socket_service thread: priority 14, 2048 bytes; the
 *   application starts it with K_THREAD_DEFINE_START()), polling its
 *   sockets (zsock_poll(), at most 100 ms between checks of a stop) and
 *   reading them without waiting.  One test at a time; other connections
 *   are denied.  It refuses (SERVER_ERROR, "not implemented") UDP, SCTP,
 *   reverse and bidirectional tests, omitted seconds and more than one
 *   stream.  A test whose client sends nothing for 30 s is dropped
 *   (CONFIG_NET_ZPERF_IPERF3_IDLE_TIMEOUT).  The listening socket is bound
 *   to the given address or any (the NINA takes only any) with a backlog of
 *   2; SO_REUSEADDR is asked for and its failure ignored, as in Zephyr.
 * - Results: the CPU load is reported as 0, the retransmissions as unknown
 *   (-1), the stream's time from 0 to the measured duration; the peer's
 *   results are read for the bytes and the duration only.
 *
 * Not done: UDP (Zephyr's UDP server also needs zsock_getsockname(), which
 * the NINA's sockets do not have), IPv6, the asynchronous upload ("-a") and
 * its periodic reports ("-i"), "zperf jobs", "setip", "upload2".
 */

#ifndef COMMON_COMPAT_INTERNAL_ZPERF_IPERF3_H
#define COMMON_COMPAT_INTERNAL_ZPERF_IPERF3_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ZPERF_IPERF3_DEFAULT_PORT  5201
#define ZPERF_IPERF3_COOKIE_SIZE   37 /* 36 characters and a NUL */
#define ZPERF_IPERF3_JSON_LEN_SIZE 4
/* CONFIG_NET_ZPERF_IPERF3_JSON_MAX_LEN */
#define ZPERF_IPERF3_JSON_MAX_LEN  1024
/* CONFIG_NET_ZPERF_MAX_PACKET_SIZE: the largest block an upload sends. */
#define ZPERF_PACKET_SIZE_MAX      1064

enum zperf_iperf3_state {
	ZPERF_IPERF3_TEST_START = 1,
	ZPERF_IPERF3_TEST_RUNNING = 2,
	ZPERF_IPERF3_TEST_END = 4,
	ZPERF_IPERF3_PARAM_EXCHANGE = 9,
	ZPERF_IPERF3_CREATE_STREAMS = 10,
	ZPERF_IPERF3_SERVER_TERMINATE = 11,
	ZPERF_IPERF3_CLIENT_TERMINATE = 12,
	ZPERF_IPERF3_EXCHANGE_RESULTS = 13,
	ZPERF_IPERF3_DISPLAY_RESULTS = 14,
	ZPERF_IPERF3_IPERF_DONE = 16,
	ZPERF_IPERF3_ACCESS_DENIED = -1,
	ZPERF_IPERF3_SERVER_ERROR = -2,
};

/* The error numbers of SERVER_ERROR that the port sends (iperf3's i_errno). */
#define ZPERF_IPERF3_IEUNIMP      13  /* not implemented */
#define ZPERF_IPERF3_IERECVPARAMS 114 /* unable to receive parameters */

/* The parameters a server acts on. */
struct zperf_iperf3_params {
	bool tcp;
	bool udp;
	bool sctp;
	bool reverse;
	bool bidirectional;
	int32_t omit;
	int32_t time;
	int32_t parallel;
	int32_t len;
};

/* One side's results of the stream. */
struct zperf_iperf3_stream_stats {
	uint64_t bytes;
	uint64_t packets;
	uint64_t errors;
	uint64_t jitter_us;
	uint64_t duration_us;
};

/* JSON: the value of KEY in the object JSON (LEN bytes, braces included),
 * as its raw text; 0, -ENOENT, or -EINVAL for text that is not an object.
 */
int zperf_iperf3_json_find(const char *json, size_t len, const char *key, const char **value,
			   size_t *value_len);

/* The client's parameters; 0 or -EINVAL (not an object, a value of the
 * wrong type).  Keys that are missing keep 0 or false.
 */
int zperf_iperf3_params_decode(const char *json, size_t len, struct zperf_iperf3_params *params);

/* The client's parameters with the length in front, for a TCP test of
 * TIME_S seconds and LEN-byte blocks: the bytes written to BUF, or
 * -ENOSPC.
 */
int zperf_iperf3_params_encode(int32_t time_s, uint32_t len, bool nodelay, uint8_t *buf,
			       size_t size);

/* The results of one stream with the length in front (SENDER: the side
 * that sent the data): the bytes written to BUF, or -ENOSPC.
 */
int zperf_iperf3_results_encode(const struct zperf_iperf3_stream_stats *stats, bool sender,
				uint8_t *buf, size_t size);

/* The first stream of the peer's results; 0 or -EINVAL. */
int zperf_iperf3_results_decode(const char *json, size_t len,
				struct zperf_iperf3_stream_stats *stats);

/* Seconds as a JSON number ("12.345678"), and back (a JSON number, with an
 * exponent too, rounded to the microsecond): the length or 0, or -EINVAL,
 * -ERANGE, -ENOSPC.
 */
int zperf_iperf3_fmt_seconds(char *buf, size_t size, uint64_t usec);
int zperf_iperf3_parse_seconds(const char *str, size_t len, uint64_t *usec);

#endif /* COMMON_COMPAT_INTERNAL_ZPERF_IPERF3_H */
