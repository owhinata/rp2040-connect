/* SPDX-License-Identifier: Apache-2.0 */
/* The "zperf" command of the Zephyr build (Zephyr v4.5.0, iperf3 protocol)
 * for the ports without Zephyr's zperf: the part of
 * subsys/net/lib/zperf/zperf_shell.c for TCP over IPv4, "zperf tcp upload",
 * "zperf tcp download", "zperf tcp download stop" and "zperf version", with
 * their syntax, messages and return values, on the port's minimal iperf3
 * (common/compat/zperf_iperf3.c, <zephyr/net/zperf.h> of the layer).
 *
 * Changes from Zephyr's file (as the Zephyr build has it: CONFIG_NET_IPV4
 * without IPv6, CONFIG_WIFI, no CONFIG_ZPERF_SESSION_PER_THREAD, no
 * CONFIG_NET_CONTEXT_PRIORITY, no CONFIG_NET_ZPERF_RAW_TX):
 * - only TCP: "zperf udp", "zperf jobs", "zperf setip", "zperf connectap" and
 *   "zperf tcp upload2" are not here;
 * - "zperf tcp upload" has no "-a" (asynchronous upload) and "-i" (its
 *   periodic reports): they are "Unrecognized argument"; "-S" (type of
 *   service) is taken and has no effect (Zephyr only warns in its log, which
 *   the build keeps off, when the socket refuses IP_TOS, as the NINA's do);
 *   "-I" (interface) is taken and has no effect (there is one interface);
 *   the help lists what is left;
 * - the addresses are parsed and printed with inet_pton() and inet_ntop()
 *   (Zephyr: net_addr_pton(), net_ipaddr_parse(), net_sprint_ipv4_addr());
 *   "zperf tcp download <port> <host>" takes an address without a port;
 * - the download's callback prints to the session that started the server
 *   from the server's thread, as in Zephyr;
 * - integer format specifiers that match newlib's types, and a cast in
 *   parse_number() for -Wsign-compare.
 * Part of the small Zephyr compatibility layer of the port, its
 * OS-independent part (common/compat); built by the FreeRTOS build only.
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/zperf.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "zperf_iperf3.h"

#ifndef USEC_PER_MSEC
#define USEC_PER_MSEC 1000U
#endif
#ifndef MSEC_PER_SEC
#define MSEC_PER_SEC 1000U
#endif
#ifndef USEC_PER_SEC
#define USEC_PER_SEC 1000000U
#endif

/* zperf_internal.h */
#define DEF_PORT                 ZPERF_IPERF3_DEFAULT_PORT
#define DEF_PORT_STR             "5201"
#define ZPERF_PROTOCOL           "iperf3"
#define DEF_DURATION_SECONDS     1
#define DEF_DURATION_SECONDS_STR "1"
#define DEF_PACKET_SIZE          256
#define DEF_PACKET_SIZE_STR      "256"
#define DEF_RATE_KBPS            10
#define ZPERF_VERSION            "1.1"

static const char *CONFIG = "unified wifi ipv4";

static const uint32_t TIME_US[] = {60 * 1000 * 1000, 1000 * 1000, 1000, 0};
static const char *TIME_US_UNIT[] = {"m", "s", "ms", "us"};
static const uint32_t KBPS[] = {1000, 0};
static const char *KBPS_UNIT[] = {"Mbps", "Kbps"};
static const uint32_t K[] = {1000 * 1000, 1000, 0};
static const char *K_UNIT[] = {"M", "K", ""};

static void print_number(const struct shell *sh, uint32_t value, const uint32_t *divisor_arr,
			 const char **units)
{
	const char **unit;
	const uint32_t *divisor;
	uint32_t dec, radix;

	unit = units;
	divisor = divisor_arr;

	while (value < *divisor) {
		divisor++;
		unit++;
	}

	if (*divisor != 0U) {
		radix = value / *divisor;
		dec = (value % *divisor) * 100U / *divisor;
		shell_fprintf(sh, SHELL_NORMAL, "%lu.%s%lu %s", (unsigned long)radix,
			      (dec < 10) ? "0" : "", (unsigned long)dec, *unit);
	} else {
		shell_fprintf(sh, SHELL_NORMAL, "%lu %s", (unsigned long)value, *unit);
	}
}

static void print_number_64(const struct shell *sh, uint64_t value, const uint32_t *divisor_arr,
			    const char **units)
{
	const char **unit;
	const uint32_t *divisor;
	uint32_t dec;
	uint64_t radix;

	unit = units;
	divisor = divisor_arr;

	while (value < *divisor) {
		divisor++;
		unit++;
	}

	if (*divisor != 0U) {
		radix = value / *divisor;
		dec = (value % *divisor) * 100U / *divisor;
		shell_fprintf(sh, SHELL_NORMAL, "%llu.%s%lu %s", (unsigned long long)radix,
			      (dec < 10) ? "0" : "", (unsigned long)dec, *unit);
	} else {
		shell_fprintf(sh, SHELL_NORMAL, "%llu %s", (unsigned long long)value, *unit);
	}
}

static long parse_number(const char *string, const uint32_t *divisor_arr, const char **units)
{
	const char **unit;
	const uint32_t *divisor;
	char *suffix;
	long dec;
	int cmp;

	dec = strtoul(string, &suffix, 10);
	unit = units;
	divisor = divisor_arr;

	do {
		cmp = strncasecmp(suffix, *unit++, 1);
	} while (cmp != 0 && *++divisor != 0U);

	return (*divisor == 0U) ? dec : dec * (long)*divisor;
}

static int parse_ipv4_addr(const struct shell *sh, char *host, char *port,
			   struct sockaddr_in *addr)
{
	if (!host) {
		return -EINVAL;
	}

	if (zsock_inet_pton(AF_INET, host, &addr->sin_addr) != 1) {
		return -EDESTADDRREQ;
	}

	addr->sin_port = htons(strtoul(port, NULL, 10));
	if (!addr->sin_port) {
		shell_fprintf(sh, SHELL_WARNING, "Invalid port %s\n", port);
		return -EINVAL;
	}

	return 0;
}

static int zperf_bind_host(const struct shell *sh, size_t argc, char *argv[],
			   struct zperf_download_params *param)
{
	/* Parse options */
	if (argc >= 2) {
		param->port = strtoul(argv[1], NULL, 10);
	} else {
		param->port = DEF_PORT;
	}

	if (argc >= 3) {
		char *addr_str = argv[2];

		memset(&param->addr, 0, sizeof(param->addr));
		if (zsock_inet_pton(AF_INET, addr_str, &param->addr.sin_addr) != 1) {
			shell_fprintf(sh, SHELL_WARNING, "Cannot parse address \"%s\"\n", addr_str);
			return -EINVAL;
		}
		param->addr.sin_family = AF_INET;
	}

	return 0;
}

/*
 * parse download options with '-'
 * return < 0 if parse error
 * return 0 if no '-' options
 * return > 0 num of argc we parsed
 * and following parse starts from this num
 */
static int shell_cmd_download(const struct shell *sh, size_t argc, char *argv[],
			      struct zperf_download_params *param)
{
	int opt_cnt = 0;
	size_t i;

	ARG_UNUSED(param);

	for (i = 1; i < argc; ++i) {
		if (*argv[i] != '-') {
			break;
		}

		switch (argv[i][1]) {
		case 'I':
			/* The one interface: the name is not kept. */
			i++;
			if (i >= argc) {
				shell_fprintf(sh, SHELL_WARNING, "-I <interface name>\n");
				return -ENOEXEC;
			}

			opt_cnt += 2;
			break;

		default:
			shell_fprintf(sh, SHELL_WARNING, "Unrecognized argument: %s\n", argv[i]);
			return -ENOEXEC;
		}
	}

	return opt_cnt;
}

static void shell_tcp_upload_print_stats(const struct shell *sh, struct zperf_results *results)
{
	uint64_t client_rate_in_kbps;

	shell_fprintf(sh, SHELL_NORMAL, "-\nUpload completed!\n");

	if (results->client_time_in_us != 0U) {
		client_rate_in_kbps =
			(uint32_t)(((uint64_t)results->nb_packets_sent *
				    (uint64_t)results->packet_size * (uint64_t)8 *
				    (uint64_t)USEC_PER_SEC) /
				   (results->client_time_in_us * 1000U));
	} else {
		client_rate_in_kbps = 0U;
	}

	shell_fprintf(sh, SHELL_NORMAL, "Duration:\t\t");
	print_number_64(sh, results->client_time_in_us, TIME_US, TIME_US_UNIT);
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	shell_fprintf(sh, SHELL_NORMAL, "Num packets:\t\t%lu\n",
		      (unsigned long)results->nb_packets_sent);
	shell_fprintf(sh, SHELL_NORMAL, "Num errors:\t\t%lu (retry or fail)\n",
		      (unsigned long)results->nb_packets_errors);
	shell_fprintf(sh, SHELL_NORMAL, "Rate:\t\t\t");
	print_number(sh, client_rate_in_kbps, KBPS, KBPS_UNIT);
	shell_fprintf(sh, SHELL_NORMAL, "\n");
}

static int execute_upload(const struct shell *sh, const struct zperf_upload_params *param)
{
	struct zperf_results results = {0};
	int ret;

	shell_fprintf(sh, SHELL_NORMAL, "Duration:\t");
	print_number_64(sh, (uint64_t)param->duration_ms * USEC_PER_MSEC, TIME_US, TIME_US_UNIT);
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	shell_fprintf(sh, SHELL_NORMAL, "Packet size:\t%u bytes\n", param->packet_size);
	shell_fprintf(sh, SHELL_NORMAL, "Rate:\t\t");
	print_number(sh, param->rate_kbps, KBPS, KBPS_UNIT);
	shell_fprintf(sh, SHELL_NORMAL, "\n");

	shell_fprintf(sh, SHELL_NORMAL, "Starting...\n");

	ret = zperf_tcp_upload(param, &results);
	if (ret < 0) {
		shell_fprintf(sh, SHELL_ERROR, "TCP upload failed (%d)\n", ret);
		return ret;
	}

	shell_tcp_upload_print_stats(sh, &results);

	return 0;
}

static int parse_arg(size_t *i, size_t argc, char *argv[])
{
	int res = -1;
	const char *str = argv[*i] + 2;
	char *endptr;

	if (*str == 0) {
		if (*i + 1 >= argc) {
			return -1;
		}

		*i += 1;
		str = argv[*i];
	}

	errno = 0;
	if (strncmp(str, "0x", 2) == 0) {
		res = strtol(str, &endptr, 16);
	} else {
		res = strtol(str, &endptr, 10);
	}

	if (errno || (endptr == str)) {
		return -1;
	}

	return res;
}

static int cmd_tcp_upload(const struct shell *sh, size_t argc, char *argv[])
{
	struct zperf_upload_params param = {0};
	struct sockaddr_in ipv4 = {.sin_family = AF_INET};
	char *port_str;
	int start = 0;
	size_t opt_cnt = 0;
	int ret;
	char addr_str[INET_ADDRSTRLEN];

	/* Parse options */
	for (size_t i = 1; i < argc; ++i) {
		if (*argv[i] != '-') {
			break;
		}

		switch (argv[i][1]) {
		case 'S': {
			int tos = parse_arg(&i, argc, argv);

			if (tos < 0 || tos > UINT8_MAX) {
				shell_fprintf(sh, SHELL_WARNING, "Parse error: %s\n", argv[i]);
				return -ENOEXEC;
			}

			param.options.tos = tos;
			opt_cnt += 2;
			break;
		}

		case 'n':
			param.options.tcp_nodelay = 1;
			opt_cnt += 1;
			break;

		case 'I':
			/* The one interface: the name is not kept. */
			i++;
			if (i >= argc) {
				shell_fprintf(sh, SHELL_WARNING, "-I <interface name>\n");
				return -ENOEXEC;
			}

			opt_cnt += 2;
			break;

		default:
			shell_fprintf(sh, SHELL_WARNING, "Unrecognized argument: %s\n", argv[i]);
			return -ENOEXEC;
		}
	}

	start += opt_cnt;
	argc -= opt_cnt;

	if (argc < 2) {
		shell_fprintf(sh, SHELL_WARNING, "Not enough parameters.\n");
		shell_help(sh);
		return -ENOEXEC;
	}

	if (argc > 2) {
		port_str = argv[start + 2];
		shell_fprintf(sh, SHELL_NORMAL, "Remote port is %s\n", port_str);
	} else {
		port_str = DEF_PORT_STR;
	}

	ret = parse_ipv4_addr(sh, argv[start + 1], port_str, &ipv4);
	if (ret == -EDESTADDRREQ) {
		shell_fprintf(sh, SHELL_WARNING, "Invalid IPv4 address %s\n", argv[start + 1]);
	}
	if (ret < 0) {
		shell_fprintf(sh, SHELL_WARNING,
			      "Please specify the IP address of the "
			      "remote server.\n");
		return -ENOEXEC;
	}

	shell_fprintf(sh, SHELL_NORMAL, "Connecting to %s\n",
		      zsock_inet_ntop(AF_INET, &ipv4.sin_addr, addr_str, sizeof(addr_str)));

	memcpy(&param.peer_addr, &ipv4, sizeof(ipv4));

	if (argc > 3) {
		param.duration_ms = MSEC_PER_SEC * strtoul(argv[start + 3], NULL, 10);
	} else {
		param.duration_ms = MSEC_PER_SEC * DEF_DURATION_SECONDS;
	}

	if (argc > 4) {
		param.packet_size = parse_number(argv[start + 4], K, K_UNIT);
	} else {
		param.packet_size = DEF_PACKET_SIZE;
	}

	if (argc > 5) {
		param.rate_kbps = (parse_number(argv[start + 5], K, K_UNIT) + 999) / 1000;
		shell_fprintf(sh, SHELL_WARNING, "TCP upload will ignore <baud rate> argument\n");
	} else {
		param.rate_kbps = DEF_RATE_KBPS;
	}

	return execute_upload(sh, &param);
}

static int cmd_tcp(const struct shell *sh, size_t argc, char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_help(sh);
	return -ENOEXEC;
}

static void tcp_session_cb(enum zperf_status status, struct zperf_results *result,
			   void *user_data)
{
	const struct shell *sh = user_data;

	switch (status) {
	case ZPERF_SESSION_STARTED:
		shell_fprintf(sh, SHELL_NORMAL, "New TCP session started.\n");
		break;

	case ZPERF_SESSION_FINISHED: {
		uint32_t rate_in_kbps;

		/* Compute baud rate */
		if (result->time_in_us != 0U) {
			rate_in_kbps = (uint32_t)((result->total_len * 8ULL * USEC_PER_SEC) /
						  (result->time_in_us * 1000ULL));
		} else {
			rate_in_kbps = 0U;
		}

		shell_fprintf(sh, SHELL_NORMAL, "TCP session ended\n");

		shell_fprintf(sh, SHELL_NORMAL, " Duration:\t\t");
		print_number_64(sh, result->time_in_us, TIME_US, TIME_US_UNIT);
		shell_fprintf(sh, SHELL_NORMAL, "\n");

		shell_fprintf(sh, SHELL_NORMAL, " rate:\t\t\t");
		print_number(sh, rate_in_kbps, KBPS, KBPS_UNIT);
		shell_fprintf(sh, SHELL_NORMAL, "\n");

		break;
	}

	case ZPERF_SESSION_ERROR:
		shell_fprintf(sh, SHELL_ERROR, "TCP session error.\n");
		break;

	default:
		break;
	}
}

static int cmd_tcp_download_stop(const struct shell *sh, size_t argc, char *argv[])
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = zperf_tcp_download_stop();
	if (ret < 0) {
		shell_fprintf(sh, SHELL_WARNING, "TCP server not running!\n");
		return -ENOEXEC;
	}

	shell_fprintf(sh, SHELL_NORMAL, "TCP server stopped\n");

	return 0;
}

static int cmd_tcp_download(const struct shell *sh, size_t argc, char *argv[])
{
	struct zperf_download_params param = {0};
	int ret;
	int start;

	start = shell_cmd_download(sh, argc, argv, &param);
	if (start < 0) {
		shell_fprintf(sh, SHELL_WARNING, "Unable to parse option.\n");
		return -ENOEXEC;
	}

	ret = zperf_bind_host(sh, argc - start, &argv[start], &param);
	if (ret < 0) {
		shell_fprintf(sh, SHELL_WARNING, "Unable to bind host.\n");
		shell_help(sh);
		return -ENOEXEC;
	}

	ret = zperf_tcp_download(&param, tcp_session_cb, (void *)sh);
	if (ret == -EALREADY) {
		shell_fprintf(sh, SHELL_WARNING, "TCP server already started!\n");
		return -ENOEXEC;
	} else if (ret < 0) {
		shell_fprintf(sh, SHELL_ERROR, "Failed to start TCP server!\n");
		return -ENOEXEC;
	}

	shell_fprintf(sh, SHELL_NORMAL, "TCP server started on port %u\n", param.port);

	return 0;
}

static int cmd_version(const struct shell *sh, size_t argc, char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_fprintf(sh, SHELL_NORMAL, "Version: %s\nProtocol: %s\nConfig: %s\n", ZPERF_VERSION,
		      ZPERF_PROTOCOL, CONFIG);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(zperf_cmd_tcp_download,
	SHELL_CMD(stop, NULL, "Stop TCP server\n", cmd_tcp_download_stop),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(zperf_cmd_tcp,
	SHELL_CMD(upload, NULL,
		  "[<options>] <dest ip> <dest port> <duration> <packet size>[K]\n"
		  "<options>     command options (optional): [-S tos -n]\n"
		  "<dest ip>     IP destination\n"
		  "<dest port>   port destination\n"
		  "<duration>    of the test in seconds "
		  "(default " DEF_DURATION_SECONDS_STR ")\n"
		  "<packet size> in byte or kilobyte "
		  "(with suffix K) "
		  "(default " DEF_PACKET_SIZE_STR ")\n"
		  "Available options:\n"
		  "-S tos: Specify IPv4/6 type of service\n"
		  "-n: Disable Nagle's algorithm\n"
		  "Example: tcp upload 192.0.2.2 1111 1 1K\n",
		  cmd_tcp_upload),
	SHELL_CMD(download, &zperf_cmd_tcp_download,
		  "[<port>]:  Server port to listen on/connect to\n"
		  "[<host>]:  Bind to <host>, an interface address\n"
		  "Example: tcp download " DEF_PORT_STR " 192.168.0.1\n",
		  cmd_tcp_download),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(zperf_commands,
	SHELL_CMD(tcp, &zperf_cmd_tcp,
		  "Upload/Download TCP data",
		  cmd_tcp),
	SHELL_CMD(version, NULL,
		  "Zperf version",
		  cmd_version),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(zperf, &zperf_commands, "Zperf commands", NULL);
