/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/net/zperf.h> (Zephyr v4.5.0) that the port's
 * "zperf" command uses: TCP uploads and the TCP server of the iperf3
 * protocol (common/compat/zperf_iperf3.c, the port's own minimal iperf3;
 * the command is common/compat/zephyr_zperf_shell.c).  IPv4 only, as the
 * Zephyr build (CONFIG_NET_IPV4 without CONFIG_NET_IPV6); no UDP, no
 * asynchronous uploads, no data loader.  The Zephyr build uses Zephyr's
 * own zperf, not this file.
 *
 * Part of the small Zephyr compatibility layer of the port, its
 * OS-independent part (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_NET_ZPERF_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_NET_ZPERF_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/net/socket.h>

enum zperf_status {
	ZPERF_SESSION_STARTED,
	ZPERF_SESSION_PERIODIC_RESULT,
	ZPERF_SESSION_FINISHED,
	ZPERF_SESSION_ERROR
};

struct zperf_upload_params {
	struct sockaddr_in peer_addr; /* IPv4 */
	uint32_t duration_ms;
	uint32_t rate_kbps;
	uint16_t packet_size;
	struct {
		uint8_t tos;
		int tcp_nodelay;
	} options;
};

struct zperf_download_params {
	uint16_t port;
	struct sockaddr_in addr; /* sin_family 0: any address */
};

struct zperf_results {
	uint32_t nb_packets_sent;
	uint32_t nb_packets_rcvd;
	uint32_t nb_packets_lost;
	uint32_t nb_packets_outorder;
	uint64_t total_len;
	uint64_t time_in_us;
	uint32_t jitter_in_us;
	uint64_t client_time_in_us;
	uint32_t packet_size;
	uint32_t nb_packets_errors;
	bool is_multicast;
};

typedef void (*zperf_callback)(enum zperf_status status, struct zperf_results *result,
			       void *user_data);

/* A whole iperf3 test against an iperf3 server ("iperf3 -s"), in the
 * calling thread: 0 and RESULT, or a negative errno.
 */
int zperf_tcp_upload(const struct zperf_upload_params *param, struct zperf_results *result);

/* Starts the iperf3 server (TCP) on PARAM's port; CALLBACK reports each
 * test from the server's thread.  -EALREADY when it runs, another negative
 * errno when the listening socket cannot be opened.
 */
int zperf_tcp_download(const struct zperf_download_params *param, zperf_callback callback,
		       void *user_data);

/* Stops it (ending a test in progress); -EALREADY when it does not run. */
int zperf_tcp_download_stop(void);

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_NET_ZPERF_H */
