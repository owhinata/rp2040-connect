/* SPDX-License-Identifier: MIT */
/* A minimal iperf3 for the port's "zperf" command (zperf_iperf3.h,
 * <zephyr/net/zperf.h>): JSON, the TCP client and the TCP server.  Part of
 * the small Zephyr compatibility layer of the port, its OS-independent part
 * (common/compat); built by the FreeRTOS build only
 * (common/compat/port/pico_sdk/zperf.cmake).
 *
 * The buffers are taken from malloc() only while they are in use: the
 * client's for one upload, the server's from zperf_tcp_download() until the
 * server stops.  When there is no room, the call fails with -ENOMEM.
 */

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/zperf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "zperf_iperf3.h"

/* How long the client waits for the server on the control connection. */
#define CTRL_TIMEOUT_MS 10000

#define JSON_BUF_SIZE (ZPERF_IPERF3_JSON_LEN_SIZE + ZPERF_IPERF3_JSON_MAX_LEN + 1)

#define CLIENT_VERSION "zperf-1.1"

/* The server: the longest it waits in zsock_poll() before it looks at a
 * stop request again, the reads of the data stream per turn, the reads that
 * take what is queued at the end of a test, the size of each read
 * (CONFIG_NET_ZPERF_TCP_RECEIVER_BUF_SIZE) and the time a test may stay
 * silent (CONFIG_NET_ZPERF_IPERF3_IDLE_TIMEOUT).
 */
#define SERVER_POLL_MS    100
#define DATA_READ_BUDGET  8
#define DRAIN_READ_LIMIT  1024
#define DATA_BUF_SIZE     1500
#define IDLE_TIMEOUT_MS   30000
#define SERVER_ERROR_SIZE 9

/* "12345678901.123456" */
#define SECONDS_STR_SIZE (10 + 1 + 6 + 1)

#define USEC_PER_SEC_U64 1000000ULL

static uint64_t now_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

/* JSON ------------------------------------------------------------------*/

static const char *skip_space(const char *p, const char *end)
{
	while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
		p++;
	}
	return p;
}

/* Past the string that starts at P (its '"'), or NULL. */
static const char *skip_string(const char *p, const char *end)
{
	for (p++; p < end; p++) {
		if (*p == '\\') {
			p++;
		} else if (*p == '"') {
			return p + 1;
		}
	}
	return NULL;
}

/* Past the value that starts at P, or NULL. */
static const char *skip_value(const char *p, const char *end)
{
	const char *start = p;
	int depth = 0;

	if (p >= end) {
		return NULL;
	}
	if (*p == '"') {
		return skip_string(p, end);
	}
	if (*p != '{' && *p != '[') {
		while (p < end && *p != ',' && *p != '}' && *p != ']' && *p != ' ' && *p != '\t' &&
		       *p != '\n' && *p != '\r') {
			p++;
		}
		return p > start ? p : NULL;
	}
	while (p < end) {
		if (*p == '"') {
			p = skip_string(p, end);
			if (p == NULL) {
				return NULL;
			}
			continue;
		}
		if (*p == '{' || *p == '[') {
			depth++;
		} else if (*p == '}' || *p == ']') {
			if (--depth == 0) {
				return p + 1;
			}
		}
		p++;
	}
	return NULL;
}

int zperf_iperf3_json_find(const char *json, size_t len, const char *key, const char **value,
			   size_t *value_len)
{
	const char *end = json + len;
	const char *p = skip_space(json, end);
	size_t key_len = strlen(key);

	if (p >= end || *p != '{') {
		return -EINVAL;
	}
	p = skip_space(p + 1, end);
	if (p < end && *p == '}') {
		return -ENOENT;
	}
	for (;;) {
		const char *name;
		const char *name_end;
		const char *v;
		bool match;

		if (p >= end || *p != '"') {
			return -EINVAL;
		}
		name = p + 1;
		name_end = skip_string(p, end);
		if (name_end == NULL) {
			return -EINVAL;
		}
		match = (size_t)(name_end - 1 - name) == key_len && memcmp(name, key, key_len) == 0;
		p = skip_space(name_end, end);
		if (p >= end || *p != ':') {
			return -EINVAL;
		}
		v = skip_space(p + 1, end);
		p = skip_value(v, end);
		if (p == NULL) {
			return -EINVAL;
		}
		if (match) {
			*value = v;
			*value_len = (size_t)(p - v);
			return 0;
		}
		p = skip_space(p, end);
		if (p < end && *p == ',') {
			p = skip_space(p + 1, end);
			continue;
		}
		return (p < end && *p == '}') ? -ENOENT : -EINVAL;
	}
}

static int token_bool(const char *v, size_t n, bool *out)
{
	if (n == 4 && memcmp(v, "true", 4) == 0) {
		*out = true;
	} else if (n == 5 && memcmp(v, "false", 5) == 0) {
		*out = false;
	} else {
		return -EINVAL;
	}
	return 0;
}

/* An integer token (no fraction, no exponent). */
static int token_int64(const char *v, size_t n, bool is_signed, int64_t *s, uint64_t *u)
{
	char text[24];
	char *endp;

	if (n == 0 || n >= sizeof(text) || (!is_signed && v[0] == '-')) {
		return -EINVAL;
	}
	memcpy(text, v, n);
	text[n] = '\0';
	errno = 0;
	if (is_signed) {
		*s = strtoll(text, &endp, 10);
	} else {
		*u = strtoull(text, &endp, 10);
	}
	return (errno != 0 || *endp != '\0') ? -EINVAL : 0;
}

static int find_bool(const char *json, size_t len, const char *key, bool *out)
{
	const char *v;
	size_t n;
	int ret = zperf_iperf3_json_find(json, len, key, &v, &n);

	if (ret == -ENOENT) {
		return 0;
	}
	return ret < 0 ? ret : token_bool(v, n, out);
}

static int find_int32(const char *json, size_t len, const char *key, int32_t *out)
{
	const char *v;
	size_t n;
	int64_t value;
	int ret = zperf_iperf3_json_find(json, len, key, &v, &n);

	if (ret == -ENOENT) {
		return 0;
	}
	if (ret < 0 || token_int64(v, n, true, &value, NULL) < 0 || value < INT32_MIN ||
	    value > INT32_MAX) {
		return -EINVAL;
	}
	*out = (int32_t)value;
	return 0;
}

static int find_uint64(const char *json, size_t len, const char *key, uint64_t *out)
{
	const char *v;
	size_t n;
	int ret = zperf_iperf3_json_find(json, len, key, &v, &n);

	if (ret == -ENOENT) {
		return 0;
	}
	return ret < 0 ? ret : token_int64(v, n, false, NULL, out);
}

static int find_seconds(const char *json, size_t len, const char *key, uint64_t *usec)
{
	const char *v;
	size_t n;
	int ret = zperf_iperf3_json_find(json, len, key, &v, &n);

	if (ret == -ENOENT) {
		*usec = 0;
		return 0;
	}
	return ret < 0 ? ret : zperf_iperf3_parse_seconds(v, n, usec);
}

int zperf_iperf3_params_decode(const char *json, size_t len, struct zperf_iperf3_params *params)
{
	const char *v;
	size_t n;

	memset(params, 0, sizeof(*params));
	if (zperf_iperf3_json_find(json, len, "", &v, &n) == -EINVAL) {
		return -EINVAL;
	}
	if (find_bool(json, len, "tcp", &params->tcp) < 0 ||
	    find_bool(json, len, "udp", &params->udp) < 0 ||
	    find_bool(json, len, "sctp", &params->sctp) < 0 ||
	    find_bool(json, len, "reverse", &params->reverse) < 0 ||
	    find_bool(json, len, "bidirectional", &params->bidirectional) < 0 ||
	    find_int32(json, len, "omit", &params->omit) < 0 ||
	    find_int32(json, len, "time", &params->time) < 0 ||
	    find_int32(json, len, "parallel", &params->parallel) < 0 ||
	    find_int32(json, len, "len", &params->len) < 0) {
		return -EINVAL;
	}
	return 0;
}

/* The length in front of the JSON text that snprintf() wrote after it. */
static int frame(int json_len, uint8_t *buf, size_t size)
{
	if (json_len < 0 || (size_t)json_len > ZPERF_IPERF3_JSON_MAX_LEN ||
	    (size_t)json_len + ZPERF_IPERF3_JSON_LEN_SIZE >= size) {
		return -ENOSPC;
	}
	sys_put_be32((uint32_t)json_len, buf);
	return ZPERF_IPERF3_JSON_LEN_SIZE + json_len;
}

int zperf_iperf3_params_encode(int32_t time_s, uint32_t len, bool nodelay, uint8_t *buf,
			       size_t size)
{
	int n;

	if (size <= ZPERF_IPERF3_JSON_LEN_SIZE) {
		return -ENOSPC;
	}
	/* The keys in the order iperf3 sends them; "udp", "bandwidth" and
	 * "TOS" are left out (servers take a key that is there as set).
	 */
	n = snprintf((char *)buf + ZPERF_IPERF3_JSON_LEN_SIZE, size - ZPERF_IPERF3_JSON_LEN_SIZE,
		     "{\"tcp\":true,\"omit\":0,\"time\":%ld,\"num\":0,\"blockcount\":0,%s"
		     "\"parallel\":1,\"len\":%lu,\"pacing_timer\":1000,"
		     "\"client_version\":\"" CLIENT_VERSION "\"}",
		     (long)time_s, nodelay ? "\"nodelay\":true," : "", (unsigned long)len);
	return frame(n, buf, size);
}

int zperf_iperf3_results_encode(const struct zperf_iperf3_stream_stats *stats, bool sender,
				uint8_t *buf, size_t size)
{
	char jitter[SECONDS_STR_SIZE];
	char end_time[SECONDS_STR_SIZE];
	int n;

	if (size <= ZPERF_IPERF3_JSON_LEN_SIZE ||
	    zperf_iperf3_fmt_seconds(jitter, sizeof(jitter), stats->jitter_us) < 0 ||
	    zperf_iperf3_fmt_seconds(end_time, sizeof(end_time), stats->duration_us) < 0) {
		return -ENOSPC;
	}
	/* No CPU load is measured and no retransmission count is known: a
	 * sender says it has none to give, a receiver that it does not apply.
	 */
	n = snprintf((char *)buf + ZPERF_IPERF3_JSON_LEN_SIZE, size - ZPERF_IPERF3_JSON_LEN_SIZE,
		     "{\"cpu_util_total\":0,\"cpu_util_user\":0,\"cpu_util_system\":0,"
		     "\"sender_has_retransmits\":%d,\"streams\":[{\"id\":1,\"bytes\":%llu,"
		     "\"retransmits\":-1,\"jitter\":%s,\"errors\":%llu,\"packets\":%llu,"
		     "\"start_time\":0,\"end_time\":%s}]}",
		     sender ? 0 : -1, (unsigned long long)stats->bytes, jitter,
		     (unsigned long long)stats->errors, (unsigned long long)stats->packets,
		     end_time);
	return frame(n, buf, size);
}

int zperf_iperf3_results_decode(const char *json, size_t len,
				struct zperf_iperf3_stream_stats *stats)
{
	const char *v;
	const char *end;
	const char *elem;
	const char *elem_end;
	uint64_t start_us;
	uint64_t end_us;
	size_t n;

	if (zperf_iperf3_json_find(json, len, "streams", &v, &n) < 0 || n < 2 || v[0] != '[') {
		return -EINVAL;
	}
	end = v + n;
	elem = skip_space(v + 1, end);
	if (elem >= end || *elem != '{') {
		return -EINVAL;
	}
	elem_end = skip_value(elem, end);
	if (elem_end == NULL) {
		return -EINVAL;
	}
	n = (size_t)(elem_end - elem);
	memset(stats, 0, sizeof(*stats));
	if (find_uint64(elem, n, "bytes", &stats->bytes) < 0 ||
	    find_uint64(elem, n, "packets", &stats->packets) < 0 ||
	    find_uint64(elem, n, "errors", &stats->errors) < 0 ||
	    find_seconds(elem, n, "jitter", &stats->jitter_us) < 0 ||
	    find_seconds(elem, n, "start_time", &start_us) < 0 ||
	    find_seconds(elem, n, "end_time", &end_us) < 0) {
		return -EINVAL;
	}
	stats->duration_us = end_us > start_us ? end_us - start_us : 0;
	return 0;
}

int zperf_iperf3_fmt_seconds(char *buf, size_t size, uint64_t usec)
{
	uint64_t secs = usec / USEC_PER_SEC_U64;
	int n;

	if (secs > UINT32_MAX) {
		return -ERANGE;
	}
	n = snprintf(buf, size, "%lu.%06lu", (unsigned long)secs,
		     (unsigned long)(usec % USEC_PER_SEC_U64));
	if (n < 0 || (size_t)n >= size) {
		return -ENOSPC;
	}
	return n;
}

/* Digits past this many are dropped (far below a microsecond). */
#define MANTISSA_MAX 100000000000000000ULL

int zperf_iperf3_parse_seconds(const char *str, size_t len, uint64_t *usec)
{
	uint64_t mantissa = 0;
	uint64_t divisor = 1;
	int exp10 = 6; /* the value is mantissa * 10^exp10 microseconds */
	bool digits = false;
	size_t i = 0;

	if (i < len && str[i] == '-') {
		return -ERANGE;
	}
	for (; i < len && isdigit((unsigned char)str[i]); i++) {
		digits = true;
		if (mantissa < MANTISSA_MAX) {
			mantissa = mantissa * 10 + (uint64_t)(str[i] - '0');
		} else {
			exp10++;
		}
	}
	if (i < len && str[i] == '.') {
		for (i++; i < len && isdigit((unsigned char)str[i]); i++) {
			digits = true;
			if (mantissa < MANTISSA_MAX) {
				mantissa = mantissa * 10 + (uint64_t)(str[i] - '0');
				exp10--;
			}
		}
	}
	if (!digits) {
		return -EINVAL;
	}
	if (i < len && (str[i] == 'e' || str[i] == 'E')) {
		bool negative = false;
		bool exp_digits = false;
		int exponent = 0;

		i++;
		if (i < len && (str[i] == '+' || str[i] == '-')) {
			negative = str[i] == '-';
			i++;
		}
		for (; i < len && isdigit((unsigned char)str[i]); i++) {
			exp_digits = true;
			if (exponent < 1000) {
				exponent = exponent * 10 + (str[i] - '0');
			}
		}
		if (!exp_digits) {
			return -EINVAL;
		}
		exp10 += negative ? -exponent : exponent;
	}
	if (i != len) {
		return -EINVAL;
	}
	if (mantissa == 0) {
		*usec = 0;
		return 0;
	}
	for (; exp10 > 0; exp10--) {
		if (mantissa > UINT64_MAX / 10) {
			return -ERANGE;
		}
		mantissa *= 10;
	}
	if (exp10 < -19) {
		*usec = 0;
		return 0;
	}
	for (; exp10 < 0; exp10++) {
		divisor *= 10;
	}
	/* To the nearest microsecond. */
	*usec = mantissa / divisor + ((mantissa % divisor) * 2 >= divisor ? 1 : 0);
	return 0;
}

/* The control connection ------------------------------------------------*/

static int send_all(int sock, const void *buf, size_t len, int flags)
{
	const uint8_t *pos = buf;

	while (len > 0) {
		ssize_t ret = zsock_send(sock, pos, len, flags);

		if (ret < 0) {
			return -errno;
		}
		pos += ret;
		len -= (size_t)ret;
	}
	return 0;
}

static int recv_all(int sock, void *buf, size_t len)
{
	uint8_t *pos = buf;

	while (len > 0) {
		ssize_t ret = zsock_recv(sock, pos, len, 0);

		if (ret < 0) {
			return -errno;
		}
		if (ret == 0) {
			return -ECONNRESET;
		}
		pos += ret;
		len -= (size_t)ret;
	}
	return 0;
}

static int send_state(int sock, int8_t state, int flags)
{
	return send_all(sock, &state, sizeof(state), flags);
}

/* The client -------------------------------------------------------------*/

struct send_stats {
	uint64_t bytes;
	uint32_t packets;
	uint32_t errors;
	uint32_t packet_size;
	uint64_t duration_us;
};

/* The client's buffers, from malloc() for one upload. */
struct upload_buf {
	uint8_t sample_packet[ZPERF_PACKET_SIZE_MAX];
	uint8_t json[JSON_BUF_SIZE];
};

/* One upload at a time. */
static K_MUTEX_DEFINE(upload_lock);

/* A receive timeout comes as EAGAIN. */
static int map_recv_error(int ret)
{
	return (ret == -EAGAIN || ret == -EWOULDBLOCK) ? -ETIMEDOUT : ret;
}

static int ctrl_expect_state(int ctrl, int8_t expected)
{
	uint8_t reply[SERVER_ERROR_SIZE - 1];
	int8_t state;
	int32_t error;
	int ret;

	ret = recv_all(ctrl, &state, sizeof(state));
	if (ret < 0) {
		return map_recv_error(ret);
	}
	if (state == expected) {
		return 0;
	}
	switch (state) {
	case ZPERF_IPERF3_ACCESS_DENIED:
		return -EBUSY;
	case ZPERF_IPERF3_SERVER_ERROR:
		ret = recv_all(ctrl, reply, sizeof(reply));
		if (ret < 0) {
			return map_recv_error(ret);
		}
		error = (int32_t)sys_get_be32(reply);
		return error == ZPERF_IPERF3_IEUNIMP ? -ENOTSUP : -EPROTO;
	case ZPERF_IPERF3_SERVER_TERMINATE:
		return -ECONNABORTED;
	default:
		return -EPROTO;
	}
}

static int set_rcvtimeo(int sock, uint32_t ms)
{
	struct zsock_timeval tv = {
		.tv_sec = ms / 1000,
		.tv_usec = (ms % 1000) * 1000,
	};

	return zsock_setsockopt(sock, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &tv, sizeof(tv)) < 0
		       ? -errno
		       : 0;
}

/* zperf_prepare_upload_sock() for TCP over IPv4.  The type of service is
 * not asked for: the NINA's sockets have no IP_TOS (Zephyr only warns in
 * its log, which the build keeps off).
 */
static int open_upload_sock(const struct sockaddr_in *peer, int nodelay)
{
	int sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	int ret;

	if (sock < 0) {
		return -errno;
	}
	if (nodelay && zsock_setsockopt(sock, IPPROTO_TCP, ZSOCK_TCP_NODELAY, &nodelay,
					sizeof(nodelay)) != 0) {
		ret = -errno;
		goto error;
	}
	if (zsock_connect(sock, (const struct sockaddr *)peer, sizeof(*peer)) < 0) {
		ret = -errno;
		goto error;
	}
	return sock;

error:
	zsock_close(sock);
	return ret;
}

/* 36 characters of "a".."z", "2".."7" (iperf3's set) and a NUL. */
static void make_cookie(char cookie[ZPERF_IPERF3_COOKIE_SIZE])
{
	static const char chars[] = "abcdefghijklmnopqrstuvwxyz234567";
	static uint64_t state;

	if (state == 0) {
		state = k_cycle_get_64() ^ 0x9e3779b97f4a7c15ULL;
	}
	for (size_t i = 0; i < ZPERF_IPERF3_COOKIE_SIZE - 1; i++) {
		/* xorshift64 */
		state ^= state << 13;
		state ^= state >> 7;
		state ^= state << 17;
		cookie[i] = chars[state % (sizeof(chars) - 1)];
	}
	cookie[ZPERF_IPERF3_COOKIE_SIZE - 1] = '\0';
}

static int send_block(int sock, const uint8_t *buf, size_t len, size_t *sent)
{
	*sent = 0;
	while (*sent < len) {
		ssize_t ret = zsock_send(sock, buf + *sent, len - *sent, 0);

		if (ret < 0) {
			return -errno;
		}
		*sent += (size_t)ret;
	}
	return 0;
}

static int send_tcp(int sock, const struct zperf_upload_params *param, uint32_t packet_size,
		    struct send_stats *stats, uint8_t *sample_packet)
{
	int64_t end_ms = k_uptime_get() + param->duration_ms;
	uint64_t start = now_us();
	size_t sent;
	int ret;

	memset(sample_packet, 'z', ZPERF_PACKET_SIZE_MAX);
	do {
		ret = send_block(sock, sample_packet, packet_size, &sent);
		/* The server counts every byte, those of a failed block too. */
		stats->bytes += sent;
		if (ret == -ENOMEM) {
			stats->errors++;
		} else if (ret < 0) {
			stats->errors++;
			return ret;
		} else {
			stats->packets++;
		}
		k_sleep(K_NO_WAIT);
	} while (k_uptime_get() < end_ms);

	stats->packet_size = packet_size;
	stats->duration_us = now_us() - start;
	return 0;
}

static int exchange_results(int ctrl, const struct send_stats *sent,
			    struct zperf_iperf3_stream_stats *server, uint8_t *client_json)
{
	struct zperf_iperf3_stream_stats ours = {
		.bytes = sent->bytes,
		.packets = sent->packets,
		.duration_us = sent->duration_us,
	};
	uint32_t len;
	int ret;

	ret = zperf_iperf3_results_encode(&ours, true, client_json, JSON_BUF_SIZE);
	if (ret < 0) {
		return ret;
	}
	ret = send_all(ctrl, client_json, (size_t)ret, 0);
	if (ret < 0) {
		return ret;
	}
	ret = recv_all(ctrl, client_json, ZPERF_IPERF3_JSON_LEN_SIZE);
	if (ret < 0) {
		return map_recv_error(ret);
	}
	len = sys_get_be32(client_json);
	if (len == 0 || len > ZPERF_IPERF3_JSON_MAX_LEN) {
		return -EMSGSIZE;
	}
	ret = recv_all(ctrl, client_json, len);
	if (ret < 0) {
		return map_recv_error(ret);
	}
	return zperf_iperf3_results_decode((const char *)client_json, len, server);
}

int zperf_tcp_upload(const struct zperf_upload_params *param, struct zperf_results *result)
{
	struct zperf_iperf3_stream_stats server = {0};
	struct send_stats sent = {0};
	char cookie[ZPERF_IPERF3_COOKIE_SIZE];
	struct upload_buf *buf;
	uint32_t packet_size;
	int stream = -1;
	int ctrl;
	int ret;

	if (param == NULL || result == NULL || param->peer_addr.sin_family != AF_INET) {
		return -EINVAL;
	}
	if (k_mutex_lock(&upload_lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}
	buf = malloc(sizeof(*buf));
	if (buf == NULL) {
		k_mutex_unlock(&upload_lock);
		return -ENOMEM;
	}

	/* Small messages that should not wait for more. */
	ctrl = open_upload_sock(&param->peer_addr, 1);
	if (ctrl < 0) {
		free(buf);
		k_mutex_unlock(&upload_lock);
		return ctrl;
	}
	ret = set_rcvtimeo(ctrl, CTRL_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	make_cookie(cookie);
	ret = send_all(ctrl, cookie, sizeof(cookie), 0);
	if (ret < 0) {
		goto out;
	}
	ret = ctrl_expect_state(ctrl, ZPERF_IPERF3_PARAM_EXCHANGE);
	if (ret < 0) {
		goto out;
	}

	packet_size = CLAMP(param->packet_size, 1U, (uint32_t)ZPERF_PACKET_SIZE_MAX);
	ret = zperf_iperf3_params_encode((int32_t)DIV_ROUND_UP(param->duration_ms, 1000U),
					 packet_size, param->options.tcp_nodelay != 0, buf->json,
					 sizeof(buf->json));
	if (ret < 0) {
		goto out;
	}
	ret = send_all(ctrl, buf->json, (size_t)ret, 0);
	if (ret < 0) {
		goto out;
	}
	ret = ctrl_expect_state(ctrl, ZPERF_IPERF3_CREATE_STREAMS);
	if (ret < 0) {
		goto out;
	}

	stream = open_upload_sock(&param->peer_addr, param->options.tcp_nodelay);
	if (stream < 0) {
		ret = stream;
		goto out;
	}
	/* A TCP stream names its test with the cookie. */
	ret = send_all(stream, cookie, sizeof(cookie), 0);
	if (ret < 0) {
		goto out;
	}
	ret = ctrl_expect_state(ctrl, ZPERF_IPERF3_TEST_START);
	if (ret < 0) {
		goto out;
	}
	ret = ctrl_expect_state(ctrl, ZPERF_IPERF3_TEST_RUNNING);
	if (ret < 0) {
		goto out;
	}

	ret = send_tcp(stream, param, packet_size, &sent, buf->sample_packet);
	if (ret < 0) {
		(void)send_state(ctrl, ZPERF_IPERF3_CLIENT_TERMINATE, 0);
		goto out;
	}
	ret = send_state(ctrl, ZPERF_IPERF3_TEST_END, 0);
	if (ret < 0) {
		goto out;
	}
	ret = ctrl_expect_state(ctrl, ZPERF_IPERF3_EXCHANGE_RESULTS);
	if (ret < 0) {
		goto out;
	}
	ret = exchange_results(ctrl, &sent, &server, buf->json);
	if (ret < 0) {
		goto out;
	}
	ret = ctrl_expect_state(ctrl, ZPERF_IPERF3_DISPLAY_RESULTS);
	if (ret < 0) {
		goto out;
	}
	(void)send_state(ctrl, ZPERF_IPERF3_IPERF_DONE, 0);

	memset(result, 0, sizeof(*result));
	result->nb_packets_sent = sent.packets;
	result->nb_packets_errors = sent.errors;
	result->client_time_in_us = sent.duration_us;
	result->packet_size = sent.packet_size;
	result->time_in_us = server.duration_us;
	result->total_len = sent.bytes;

out:
	if (stream >= 0) {
		zsock_close(stream);
	}
	zsock_close(ctrl);
	free(buf);
	k_mutex_unlock(&upload_lock);
	return ret;
}

/* The server -------------------------------------------------------------*/

enum srv_state {
	ST_IDLE,           /* no test; a new connection becomes its control */
	ST_COOKIE,         /* reading the cookie */
	ST_PARAMS_LEN,     /* reading the parameters */
	ST_PARAMS,
	ST_CREATE_STREAMS, /* waiting for the data connection and its cookie */
	ST_RUNNING,        /* reading the data until TEST_END */
	ST_RESULTS_LEN,    /* reading the client's results */
	ST_RESULTS,
	ST_DONE, /* results sent; waiting for the client to go */
};

/* The server's buffers, from malloc() while it runs. */
struct srv_buf {
	uint8_t json[JSON_BUF_SIZE];
	uint8_t data[DATA_BUF_SIZE];
};

struct srv_test {
	enum srv_state state;
	bool started;
	char cookie[ZPERF_IPERF3_COOKIE_SIZE];
	/* The control message being read. */
	uint8_t *rx_buf;
	size_t rx_want;
	size_t rx_have;
	uint8_t len_buf[ZPERF_IPERF3_JSON_LEN_SIZE];
	uint8_t state_byte;
	/* The data connection's cookie. */
	char data_cookie[ZPERF_IPERF3_COOKIE_SIZE];
	size_t data_cookie_have;
	/* The data connection after TEST_END: no longer read, closed with the
	 * test (the client closes its end first).
	 */
	int held_data;
	uint64_t start_us;
	uint64_t end_us;
	uint64_t bytes;
};

static struct {
	/* Under srv_lock.  enabled: started by zperf_tcp_download() until the
	 * thread has stopped it; stop: zperf_tcp_download_stop() asks the
	 * thread to.  The sockets are opened by zperf_tcp_download() while the
	 * thread waits and otherwise used and closed by the thread only.  buf:
	 * taken by zperf_tcp_download(), returned by the thread when it stops
	 * the server.
	 */
	bool enabled;
	bool stop;
	zperf_callback cb;
	void *user_data;
	int listen_fd;
	int ctrl_fd;
	int data_fd;
	int64_t last_activity;
	struct srv_buf *buf;
	struct srv_test test;
} srv = {
	.listen_fd = -1,
	.ctrl_fd = -1,
	.data_fd = -1,
	.test = {.held_data = -1},
};

static K_MUTEX_DEFINE(srv_lock);
static K_SEM_DEFINE(srv_wake, 0, 1);
static K_SEM_DEFINE(srv_stopped, 0, 1);

static void close_fd(int *fd)
{
	if (*fd >= 0) {
		zsock_close(*fd);
		*fd = -1;
	}
}

static void notify(enum zperf_status status, struct zperf_results *results)
{
	if (srv.cb != NULL) {
		srv.cb(status, results, srv.user_data);
	}
}

static void test_clear(void)
{
	close_fd(&srv.test.held_data);
	memset(&srv.test, 0, sizeof(srv.test));
	srv.test.state = ST_IDLE;
	srv.test.held_data = -1;
}

static void test_reset(void)
{
	close_fd(&srv.ctrl_fd);
	close_fd(&srv.data_fd);
	test_clear();
}

static void test_abort(void)
{
	bool started = srv.test.started;

	test_reset();
	if (started) {
		notify(ZPERF_SESSION_ERROR, NULL);
	}
}

static void ctrl_expect(void *buf, size_t len)
{
	srv.test.rx_buf = buf;
	srv.test.rx_want = len;
	srv.test.rx_have = 0;
}

static int ctrl_send_state(int8_t state)
{
	return send_state(srv.ctrl_fd, state, ZSOCK_MSG_DONTWAIT);
}

static void ctrl_server_error(int32_t error)
{
	uint8_t msg[SERVER_ERROR_SIZE];

	msg[0] = (uint8_t)ZPERF_IPERF3_SERVER_ERROR;
	sys_put_be32((uint32_t)error, &msg[1]);
	sys_put_be32(0, &msg[5]);
	(void)send_all(srv.ctrl_fd, msg, sizeof(msg), ZSOCK_MSG_DONTWAIT);
	test_abort();
}

static void deny(int sock)
{
	int8_t state = ZPERF_IPERF3_ACCESS_DENIED;

	(void)zsock_send(sock, &state, sizeof(state), ZSOCK_MSG_DONTWAIT);
	zsock_close(sock);
}

static int set_nonblocking(int sock)
{
	if (zsock_fcntl(sock, F_SETFL, O_NONBLOCK) < 0) {
		int ret = -errno;

		zsock_close(sock);
		return ret;
	}
	return sock;
}

static void start_test(void)
{
	if (ctrl_send_state(ZPERF_IPERF3_TEST_START) < 0 ||
	    ctrl_send_state(ZPERF_IPERF3_TEST_RUNNING) < 0) {
		test_abort();
		return;
	}
	srv.test.state = ST_RUNNING;
	srv.test.start_us = now_us();
	srv.test.started = true;
	notify(ZPERF_SESSION_STARTED, NULL);
}

static int32_t check_params(const struct zperf_iperf3_params *params)
{
	if (params->reverse || params->bidirectional || params->omit > 0 || params->sctp ||
	    params->parallel > 1 || params->udp || !params->tcp) {
		return ZPERF_IPERF3_IEUNIMP;
	}
	return 0;
}

static void on_params(size_t len)
{
	struct zperf_iperf3_params params;
	int32_t error;

	if (zperf_iperf3_params_decode((const char *)srv.buf->json, len, &params) < 0) {
		ctrl_server_error(ZPERF_IPERF3_IERECVPARAMS);
		return;
	}
	error = check_params(&params);
	if (error != 0) {
		ctrl_server_error(error);
		return;
	}
	srv.test.state = ST_CREATE_STREAMS;
	if (ctrl_send_state(ZPERF_IPERF3_CREATE_STREAMS) < 0) {
		test_abort();
	}
}

/* Reads the data connection once (at most DATA_READ_BUDGET reads); true if
 * it may have more.
 */
static bool on_data(void)
{
	int sock = srv.data_fd;
	ssize_t ret;

	if (srv.test.state == ST_CREATE_STREAMS) {
		/* Exactly the cookie; data may follow it. */
		ret = zsock_recv(sock, srv.test.data_cookie + srv.test.data_cookie_have,
				 ZPERF_IPERF3_COOKIE_SIZE - srv.test.data_cookie_have,
				 ZSOCK_MSG_DONTWAIT);
		if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			return false;
		}
		if (ret <= 0) {
			close_fd(&srv.data_fd);
			return false;
		}
		srv.test.data_cookie_have += (size_t)ret;
		if (srv.test.data_cookie_have < ZPERF_IPERF3_COOKIE_SIZE) {
			return false;
		}
		if (memcmp(srv.test.data_cookie, srv.test.cookie, ZPERF_IPERF3_COOKIE_SIZE - 1) !=
		    0) {
			srv.data_fd = -1;
			deny(sock);
			srv.test.data_cookie_have = 0;
			return false;
		}
		start_test();
		return false;
	}

	for (int i = 0; i < DATA_READ_BUDGET; i++) {
		ret = zsock_recv(sock, srv.buf->data, sizeof(srv.buf->data), ZSOCK_MSG_DONTWAIT);
		if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			return false;
		}
		if (ret <= 0) {
			/* The stream ended before TEST_END: wait for the
			 * control connection without it.
			 */
			close_fd(&srv.data_fd);
			return false;
		}
		srv.test.bytes += (size_t)ret;
	}
	return true;
}

static void on_test_end(void)
{
	srv.test.end_us = now_us();
	/* What the client sent before TEST_END belongs to the test. */
	for (int i = 0; i < DRAIN_READ_LIMIT / DATA_READ_BUDGET; i++) {
		if (srv.test.state != ST_RUNNING || srv.data_fd < 0 || !on_data()) {
			break;
		}
	}
	srv.test.held_data = srv.data_fd;
	srv.data_fd = -1;
	srv.test.state = ST_RESULTS_LEN;
	ctrl_expect(srv.test.len_buf, sizeof(srv.test.len_buf));
	if (ctrl_send_state(ZPERF_IPERF3_EXCHANGE_RESULTS) < 0) {
		test_abort();
	}
}

static void on_client_results(size_t len)
{
	struct zperf_iperf3_stream_stats client = {0};
	struct zperf_iperf3_stream_stats stats = {0};
	struct zperf_results results = {0};
	uint64_t duration_us = srv.test.end_us - srv.test.start_us;
	int ret;

	/* Only kept for the report: a client that sends something odd still
	 * gets the server's results.
	 */
	(void)zperf_iperf3_results_decode((const char *)srv.buf->json, len, &client);

	stats.duration_us = duration_us;
	stats.bytes = srv.test.bytes;
	results.time_in_us = duration_us;
	results.client_time_in_us = client.duration_us;
	results.nb_packets_sent = (uint32_t)client.packets;
	results.total_len = srv.test.bytes;

	ret = zperf_iperf3_results_encode(&stats, false, srv.buf->json, sizeof(srv.buf->json));
	if (ret < 0) {
		test_abort();
		return;
	}
	if (send_all(srv.ctrl_fd, srv.buf->json, (size_t)ret, ZSOCK_MSG_DONTWAIT) < 0 ||
	    ctrl_send_state(ZPERF_IPERF3_DISPLAY_RESULTS) < 0) {
		test_abort();
		return;
	}
	srv.test.state = ST_DONE;
	srv.test.started = false;
	notify(ZPERF_SESSION_FINISHED, &results);
}

static void on_ctrl_message(size_t len)
{
	uint32_t json_len;

	switch (srv.test.state) {
	case ST_COOKIE:
		srv.test.cookie[ZPERF_IPERF3_COOKIE_SIZE - 1] = '\0';
		srv.test.state = ST_PARAMS_LEN;
		ctrl_expect(srv.test.len_buf, sizeof(srv.test.len_buf));
		if (ctrl_send_state(ZPERF_IPERF3_PARAM_EXCHANGE) < 0) {
			test_abort();
		}
		break;
	case ST_PARAMS_LEN:
	case ST_RESULTS_LEN:
		json_len = sys_get_be32(srv.test.len_buf);
		if (json_len == 0 || json_len > ZPERF_IPERF3_JSON_MAX_LEN) {
			if (srv.test.state == ST_PARAMS_LEN) {
				ctrl_server_error(ZPERF_IPERF3_IERECVPARAMS);
			} else {
				test_abort();
			}
			break;
		}
		srv.test.state = srv.test.state == ST_PARAMS_LEN ? ST_PARAMS : ST_RESULTS;
		ctrl_expect(srv.buf->json, json_len);
		break;
	case ST_PARAMS:
		on_params(len);
		break;
	case ST_RESULTS:
		on_client_results(len);
		break;
	case ST_CREATE_STREAMS:
	case ST_RUNNING:
	case ST_DONE:
		/* A state byte. */
		if (srv.test.state == ST_RUNNING &&
		    srv.test.state_byte == ZPERF_IPERF3_TEST_END) {
			on_test_end();
		} else if (srv.test.state == ST_DONE &&
			   srv.test.state_byte == (uint8_t)ZPERF_IPERF3_IPERF_DONE) {
			/* The client closes next. */
		} else {
			test_abort();
		}
		break;
	default:
		test_abort();
		break;
	}
}

static void on_ctrl(void)
{
	/* Until the socket runs dry: several messages may be queued. */
	while (srv.test.state != ST_IDLE) {
		size_t len;
		ssize_t ret;

		if (srv.test.rx_want == 0) {
			/* Between messages the client sends only states. */
			ctrl_expect(&srv.test.state_byte, sizeof(srv.test.state_byte));
		}
		ret = zsock_recv(srv.ctrl_fd, srv.test.rx_buf + srv.test.rx_have,
				 srv.test.rx_want - srv.test.rx_have, ZSOCK_MSG_DONTWAIT);
		if (ret < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				test_abort();
			}
			break;
		}
		if (ret == 0) {
			/* A client that has its results may simply go. */
			if (srv.test.state == ST_DONE) {
				test_reset();
			} else {
				test_abort();
			}
			break;
		}
		srv.test.rx_have += (size_t)ret;
		if (srv.test.rx_have < srv.test.rx_want) {
			continue;
		}
		len = srv.test.rx_want;
		srv.test.rx_want = 0;
		on_ctrl_message(len);
	}
}

static void on_accept(void)
{
	for (;;) {
		struct sockaddr_in peer;
		socklen_t peer_len = sizeof(peer);
		int sock = zsock_accept(srv.listen_fd, (struct sockaddr *)&peer, &peer_len);

		if (sock < 0) {
			break;
		}
		/* A test whose results are out is over. */
		if (srv.test.state == ST_DONE) {
			test_reset();
		}
		if (srv.test.state == ST_IDLE && srv.ctrl_fd < 0) {
			sock = set_nonblocking(sock);
			if (sock < 0) {
				continue;
			}
			srv.ctrl_fd = sock;
			test_clear();
			srv.test.state = ST_COOKIE;
			srv.last_activity = k_uptime_get();
			ctrl_expect(srv.test.cookie, ZPERF_IPERF3_COOKIE_SIZE);
		} else if (srv.test.state == ST_CREATE_STREAMS && srv.data_fd < 0) {
			sock = set_nonblocking(sock);
			if (sock < 0) {
				continue;
			}
			srv.data_fd = sock;
			srv.test.data_cookie_have = 0;
		} else {
			/* Busy, as iperf3 answers a second client. */
			deny(sock);
		}
	}
}

/* The thread's turn after zsock_poll() on FDS (listening, control, data;
 * the descriptors as they were before the call).
 */
static void server_events(const struct zsock_pollfd *fds)
{
	short ev;

	ev = fds[0].revents;
	if (fds[0].fd >= 0 && fds[0].fd == srv.listen_fd && ev != 0) {
		if (ev & (ZSOCK_POLLERR | ZSOCK_POLLNVAL)) {
			/* Stop polling it, or the error comes back for ever. */
			close_fd(&srv.listen_fd);
			test_abort();
			return;
		}
		on_accept();
	}
	ev = fds[1].revents;
	if (fds[1].fd >= 0 && fds[1].fd == srv.ctrl_fd && ev != 0) {
		srv.last_activity = k_uptime_get();
		if (ev & (ZSOCK_POLLERR | ZSOCK_POLLNVAL)) {
			test_abort();
		} else if (ev & ZSOCK_POLLIN) {
			/* A hang-up is seen by reading the end of the stream. */
			on_ctrl();
		}
	}
	ev = fds[2].revents;
	if (fds[2].fd >= 0 && fds[2].fd == srv.data_fd && ev != 0) {
		srv.last_activity = k_uptime_get();
		if (ev & (ZSOCK_POLLERR | ZSOCK_POLLNVAL)) {
			close_fd(&srv.data_fd);
		} else if (ev & ZSOCK_POLLIN) {
			(void)on_data();
		}
	}
}

static void server_stop(void)
{
	if (srv.test.state != ST_IDLE && srv.test.state != ST_DONE && srv.ctrl_fd >= 0) {
		(void)ctrl_send_state(ZPERF_IPERF3_SERVER_TERMINATE);
	}
	test_abort();
	close_fd(&srv.listen_fd);
	free(srv.buf);
	srv.buf = NULL;
	srv.cb = NULL;
	srv.user_data = NULL;
}

static void server_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		(void)k_sem_take(&srv_wake, K_FOREVER);
		for (;;) {
			struct zsock_pollfd fds[3];
			int ret;

			k_mutex_lock(&srv_lock, K_FOREVER);
			if (srv.stop || !srv.enabled) {
				server_stop();
				srv.enabled = false;
				srv.stop = false;
				k_mutex_unlock(&srv_lock);
				k_sem_give(&srv_stopped);
				break;
			}
			fds[0] = (struct zsock_pollfd){.fd = srv.listen_fd, .events = ZSOCK_POLLIN};
			fds[1] = (struct zsock_pollfd){.fd = srv.ctrl_fd, .events = ZSOCK_POLLIN};
			fds[2] = (struct zsock_pollfd){.fd = srv.data_fd, .events = ZSOCK_POLLIN};
			k_mutex_unlock(&srv_lock);

			ret = zsock_poll(fds, ARRAY_SIZE(fds), SERVER_POLL_MS);

			k_mutex_lock(&srv_lock, K_FOREVER);
			if (ret > 0) {
				server_events(fds);
			}
			if (srv.test.state != ST_IDLE &&
			    k_uptime_get() - srv.last_activity > IDLE_TIMEOUT_MS) {
				test_abort();
			}
			k_mutex_unlock(&srv_lock);
			if (ret < 0) {
				/* The module did not answer: do not spin. */
				k_msleep(SERVER_POLL_MS);
			}
		}
	}
}

/* Zephyr's net_socket_service thread, which runs Zephyr's iperf3 server:
 * priority 14 (CONFIG_NET_SOCKETS_SERVICE_THREAD_PRIO 15, which Zephyr
 * lowers to the lowest application priority), 2048 bytes
 * (CONFIG_NET_SOCKETS_SERVICE_STACK_SIZE).  Started by the application.
 */
K_THREAD_DEFINE(zperf_server_tid, 2048, server_thread, NULL, NULL, NULL, 14, 0, 0);

static int server_open(const struct zperf_download_params *param)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(param->port),
	};
	int on = 1;
	int sock;
	int ret;

	if (param->addr.sin_family == AF_INET) {
		addr.sin_addr = param->addr.sin_addr;
	}
	sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		return -errno;
	}
	/* The connections of the previous test may still be closing. */
	(void)zsock_setsockopt(sock, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR, &on, sizeof(on));
	/* Room for a control connection and its data stream. */
	if (zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(sock, 2) < 0) {
		ret = -errno;
		zsock_close(sock);
		return ret;
	}
	sock = set_nonblocking(sock);
	if (sock < 0) {
		return sock;
	}
	srv.listen_fd = sock;
	return 0;
}

int zperf_tcp_download(const struct zperf_download_params *param, zperf_callback callback,
		       void *user_data)
{
	int ret = 0;

	if (param == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&srv_lock, K_FOREVER);
	if (srv.enabled) {
		ret = -EALREADY;
		goto out;
	}
	srv.buf = malloc(sizeof(*srv.buf));
	if (srv.buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	ret = server_open(param);
	if (ret < 0) {
		free(srv.buf);
		srv.buf = NULL;
		goto out;
	}
	test_clear();
	srv.cb = callback;
	srv.user_data = user_data;
	srv.enabled = true;
	srv.stop = false;
	k_sem_give(&srv_wake);
out:
	k_mutex_unlock(&srv_lock);
	return ret;
}

int zperf_tcp_download_stop(void)
{
	k_mutex_lock(&srv_lock, K_FOREVER);
	if (!srv.enabled || srv.stop) {
		k_mutex_unlock(&srv_lock);
		return -EALREADY;
	}
	srv.stop = true;
	k_mutex_unlock(&srv_lock);
	/* The thread ends the test, closes the sockets and answers within
	 * SERVER_POLL_MS and the turn in progress.
	 */
	(void)k_sem_take(&srv_stopped, K_FOREVER);
	return 0;
}
