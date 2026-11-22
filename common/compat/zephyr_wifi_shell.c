/* SPDX-License-Identifier: Apache-2.0 */
/* The "wifi" command of the Zephyr build (Zephyr v4.5.0) for the ports
 * without Zephyr's network shell: the part of subsys/net/l2/wifi/wifi_shell.c
 * that the NINA-W102 driver (common/drivers/nina/nina_wifi.c through its
 * Zephyr binding, common/drivers/nina/port/zephyr/nina_net.c) can serve,
 * "wifi connect", "wifi disconnect", "wifi scan" and "wifi status", with
 * their messages, return values and net_mgmt event output, and the root
 * command "wifi" with those and "wifi cred" (the copy of Zephyr's
 * wifi_credentials_shell.c, common/compat/subsys/net/lib/
 * wifi_credentials, registered there with SHELL_SUBCMD_ADD()).  Zephyr's
 * other subcommands (ap, twt, ps, reg_domain, statistics, ...) are not
 * here: the driver has none of their operations, so in the Zephyr build
 * they only fail.
 *
 * Only the options the driver can serve are kept: "wifi connect" has
 * -s/--ssid, -p/--passphrase and -k/--key-mgmt with 0 (None) and
 * 1 (WPA2-PSK), the only types the module joins with (nina_wifi.c's
 * nina_wifi_connect() refuses the others), and Zephyr's --help (and its -h,
 * which takes an argument there); "wifi scan", "wifi disconnect" and
 * "wifi status" have none.  Zephyr's other options (channel, band, PBKDF2,
 * MFP, BSSID, timeout, the enterprise ones, 802.11r, SSID protection, the
 * scan's type, bands, dwell times, SSIDs, count and channels, and -i/--iface
 * of the one interface) are gone from the help and the parsing: the driver
 * ignores them or refuses the request, so in the Zephyr build they are
 * accepted and then do nothing or fail.  Here they are errors ("Invalid
 * option", the argument count check of the shell), as is another -k.
 *
 * Changes from Zephyr's file besides the options:
 * - the subcommand set of "wifi" is listed here (Zephyr's
 *   SHELL_SUBCMD_SET_CREATE(wifi_commands, (wifi)) collects the
 *   SHELL_SUBCMD_ADD() entries of all sources, sorted by name; see
 *   <zephyr/shell/shell.h> of the layer);
 * - the WPA3 enterprise type and WEP key type of the status are Zephyr's
 *   "none" (the driver reports neither), printed as the empty strings they
 *   are;
 * - get_iface(): the one interface, Wi-Fi's station;
 * - printk() of "Connection request failed with error" is LOG_ERR() (the
 *   Zephyr build's printk() goes to its log, CONFIG_LOG_PRINTK, so the
 *   message is in "dmesg" in both, here with the log's prefix);
 * - the scan's event callback is removed before it is added again: Zephyr's
 *   net_mgmt_add_event_callback() does that itself, the layer's does not
 *   (common/compat/zephyr_net_mgmt.c);
 * - the helpers it needs from Zephyr's wifi_mgmt.c (wifi_state_txt(),
 *   wifi_mode_txt(), wifi_link_mode_txt(), wifi_mfp_txt(),
 *   wifi_conn_status_txt(), wifi_disconn_reason_txt()), net/ip/utils.c
 *   (net_sprint_ll_addr_buf()) and shell_utils.c (shell_strtol()) are
 *   static here;
 * - integer format specifiers that match newlib's types.
 * The output goes to the session that ran the command (context.sh), also
 * from the "net_mgmt" thread when the result events arrive, as in Zephyr.
 * Part of the small Zephyr compatibility layer of the port, its
 * OS-independent part (common/compat).
 */

/*
 * Copyright (c) 2018 Intel Corporation
 * Copyright 2024 NXP
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_getopt.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(net_wifi_shell, LOG_LEVEL_INF);

/* subsys/net/lib/shell/net_shell_private.h (sh is never NULL here: the
 * commands run on a session, and the events print to the session of the
 * last command).
 */
#define PR(fmt, ...)         shell_fprintf_normal(sh, fmt, ##__VA_ARGS__)
#define PR_ERROR(fmt, ...)   shell_fprintf_error(sh, fmt, ##__VA_ARGS__)
#define PR_INFO(fmt, ...)    shell_fprintf_info(sh, fmt, ##__VA_ARGS__)
#define PR_WARNING(fmt, ...) shell_fprintf_warn(sh, fmt, ##__VA_ARGS__)

/* <zephyr/net/wifi_mgmt.h> */
enum wifi_conn_status {
	WIFI_STATUS_CONN_SUCCESS = 0,
	WIFI_STATUS_CONN_FAIL,
	WIFI_STATUS_CONN_WRONG_PASSWORD,
	WIFI_STATUS_CONN_TIMEOUT,
	WIFI_STATUS_CONN_AP_NOT_FOUND,
	WIFI_STATUS_CONN_AUTH_REJECT,
	WIFI_STATUS_CONN_ASSOC_REJECT,
};

enum wifi_disconn_reason {
	WIFI_REASON_DISCONN_SUCCESS = 0,
	WIFI_REASON_DISCONN_UNSPECIFIED,
	WIFI_REASON_DISCONN_USER_REQUEST,
	WIFI_REASON_DISCONN_AP_LEAVING,
	WIFI_REASON_DISCONN_INACTIVITY,
};

/* subsys/net/l2/wifi/wifi_mgmt.c ------------------------------------------*/

static const char *wifi_mfp_txt(enum wifi_mfp_options mfp)
{
	switch (mfp) {
	case WIFI_MFP_DISABLE:
		return "Disable";
	case WIFI_MFP_OPTIONAL:
		return "Optional";
	case WIFI_MFP_REQUIRED:
		return "Required";
	case WIFI_MFP_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

static const char *wifi_state_txt(enum wifi_iface_state state)
{
	switch (state) {
	case WIFI_STATE_DISCONNECTED:
		return "DISCONNECTED";
	case WIFI_STATE_INACTIVE:
		return "INACTIVE";
	case WIFI_STATE_INTERFACE_DISABLED:
		return "INTERFACE_DISABLED";
	case WIFI_STATE_SCANNING:
		return "SCANNING";
	case WIFI_STATE_AUTHENTICATING:
		return "AUTHENTICATING";
	case WIFI_STATE_ASSOCIATING:
		return "ASSOCIATING";
	case WIFI_STATE_ASSOCIATED:
		return "ASSOCIATED";
	case WIFI_STATE_4WAY_HANDSHAKE:
		return "4WAY_HANDSHAKE";
	case WIFI_STATE_GROUP_HANDSHAKE:
		return "GROUP_HANDSHAKE";
	case WIFI_STATE_COMPLETED:
		return "COMPLETED";
	case WIFI_STATE_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

static const char *wifi_mode_txt(enum wifi_iface_mode mode)
{
	switch (mode) {
	case WIFI_MODE_INFRA:
		return "STATION";
	case WIFI_MODE_IBSS:
		return "ADHOC";
	case WIFI_MODE_AP:
		return "ACCESS POINT";
	case WIFI_MODE_P2P_GO:
		return "P2P GROUP OWNER";
	case WIFI_MODE_P2P_GROUP_FORMATION:
		return "P2P GROUP FORMATION";
	case WIFI_MODE_MESH:
		return "MESH";
	case WIFI_MODE_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

static const char *wifi_link_mode_txt(enum wifi_link_mode link_mode)
{
	switch (link_mode) {
	case WIFI_0:
		return "WIFI 0 (802.11)";
	case WIFI_1:
		return "WIFI 1 (802.11b)";
	case WIFI_2:
		return "WIFI 2 (802.11a)";
	case WIFI_3:
		return "WIFI 3 (802.11g)";
	case WIFI_4:
		return "WIFI 4 (802.11n/HT)";
	case WIFI_5:
		return "WIFI 5 (802.11ac/VHT)";
	case WIFI_6:
		return "WIFI 6 (802.11ax/HE)";
	case WIFI_6E:
		return "WIFI 6E (802.11ax 6GHz/HE)";
	case WIFI_7:
		return "WIFI 7 (802.11be/EHT)";
	case WIFI_LINK_MODE_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

static const char *wifi_conn_status_txt(enum wifi_conn_status status)
{
	switch (status) {
	case WIFI_STATUS_CONN_SUCCESS:
		return "Connection successful";
	case WIFI_STATUS_CONN_FAIL:
		return "Connection failed";
	case WIFI_STATUS_CONN_WRONG_PASSWORD:
		return "Wrong password";
	case WIFI_STATUS_CONN_TIMEOUT:
		return "Connection timeout";
	case WIFI_STATUS_CONN_AP_NOT_FOUND:
		return "AP not found";
	case WIFI_STATUS_CONN_AUTH_REJECT:
		return "Authentication rejected";
	case WIFI_STATUS_CONN_ASSOC_REJECT:
		return "Association rejected";
	default:
		return "UNKNOWN";
	}
}

static const char *wifi_disconn_reason_txt(enum wifi_disconn_reason reason)
{
	switch (reason) {
	case WIFI_REASON_DISCONN_SUCCESS:
		return "Success";
	case WIFI_REASON_DISCONN_UNSPECIFIED:
		return "Unspecified";
	case WIFI_REASON_DISCONN_USER_REQUEST:
		return "User request";
	case WIFI_REASON_DISCONN_AP_LEAVING:
		return "AP leaving";
	case WIFI_REASON_DISCONN_INACTIVITY:
		return "Inactivity";
	default:
		return "UNKNOWN";
	}
}

/* subsys/net/ip/utils.c ----------------------------------------------------*/

static char *net_byte_to_hex(char *ptr, uint8_t byte, char base, bool pad)
{
	uint8_t high = (byte >> 4) & 0x0f;
	uint8_t low = byte & 0x0f;

	if (pad || (high > 0)) {
		*ptr++ = (high < 10) ? (char)(high + '0') : (char)(high - 10 + base);
	}

	*ptr++ = (low < 10) ? (char)(low + '0') : (char)(low - 10 + base);

	*ptr = '\0';

	return ptr;
}

static char *net_sprint_ll_addr_buf(const uint8_t *ll, uint8_t ll_len, char *buf, int buflen)
{
	uint8_t i, len, blen;
	char *ptr = buf;

	if (ll == NULL) {
		return "<unknown>";
	}

	switch (ll_len) {
	case 8:
		len = 8U;
		break;
	case 6:
		len = 6U;
		break;
	case 2:
		len = 2U;
		break;
	default:
		len = 6U;
		break;
	}

	for (i = 0U, blen = buflen; i < len && blen > 0; i++) {
		ptr = net_byte_to_hex(ptr, (char)ll[i], 'A', true);
		*ptr++ = ':';
		blen -= 3U;
	}

	if (!(ptr - buf)) {
		return NULL;
	}

	*(ptr - 1) = '\0';
	return buf;
}

/* subsys/shell/shell_utils.c -----------------------------------------------*/

static long shell_strtol(const char *str, int base, int *err)
{
	long val;
	char *endptr;

	errno = 0;
	val = strtol(str, &endptr, base);
	if (errno == ERANGE) {
		*err = -ERANGE;
		return 0;
	} else if (errno || endptr == str || *endptr) {
		*err = -EINVAL;
		return 0;
	}

	return val;
}

/* subsys/net/l2/wifi/wifi_shell.c ------------------------------------------*/

#define WIFI_SHELL_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

#define WIFI_SHELL_SCAN_EVENTS (NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE)

static struct {
	const struct shell *sh;
	uint32_t scan_result;

	union {
		struct {
			uint8_t connecting: 1;
			uint8_t disconnecting: 1;
			uint8_t _unused: 6;
		};
		uint8_t all;
	};
} context;

static struct net_mgmt_event_callback wifi_shell_mgmt_cb;
static struct net_mgmt_event_callback wifi_shell_scan_cb;

/* The station, the one interface (Zephyr: the one -i names, else the
 * station).
 */
static struct net_if *get_iface(void)
{
	struct net_if *iface = net_if_get_wifi_sta();

	if (iface == NULL) {
		LOG_ERR("No default interface found for type: %s", "STA");
		return NULL;
	}

	return iface;
}

static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
	uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];
	const struct shell *sh = context.sh;
	uint8_t ssid_print[WIFI_SSID_MAX_LEN + 1];

	context.scan_result++;

	if (context.scan_result == 1U) {
		PR("\n%-4s | %-32s %-5s | %-13s | %-4s | %-20s | %-17s | %-8s\n", "Num", "SSID",
		   "(len)", "Chan (Band)", "RSSI", "Security", "BSSID", "MFP");
	}

	strncpy((char *)ssid_print, (const char *)entry->ssid, sizeof(ssid_print) - 1);
	ssid_print[sizeof(ssid_print) - 1] = '\0';

	PR("%-4d | %-32s %-5u | %-4u (%-6s) | %-4d | %-20s | %-17s | %-8s\n",
	   (int)context.scan_result, ssid_print, entry->ssid_length, entry->channel,
	   wifi_band_txt(entry->band), entry->rssi,
	   (entry->security == WIFI_SECURITY_TYPE_EAP ? "WPA2 Enterprise"
						      : wifi_security_txt(entry->security)),
	   ((entry->mac_length) ? net_sprint_ll_addr_buf(entry->mac, WIFI_MAC_ADDR_LEN,
							 (char *)mac_string_buf,
							 sizeof(mac_string_buf))
				: ""),
	   wifi_mfp_txt(entry->mfp));
}

static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;
	const struct shell *sh = context.sh;

	if (status->status) {
		PR_WARNING("Scan request failed (%d)\n", status->status);
	} else {
		PR("Scan request done\n");
	}

	net_mgmt_del_event_callback(&wifi_shell_scan_cb);

	context.scan_result = 0U;
}

static void print_ieee80211_codes(const struct shell *sh, struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = cb->info;

	/* Not every producer of these events fills in the codes */
	if (cb->info_length < sizeof(struct wifi_status)) {
		return;
	}

	if (status->status_code != 0) {
		PR("IEEE 802.11 status code %u\n", status->status_code);
	}

	if (status->reason_code != 0) {
		PR("IEEE 802.11 reason code %u\n", status->reason_code);
	}
}

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;
	const struct shell *sh = context.sh;
	int st = status->status;

	if (st) {
		if (st < 0) {
			/* Errno values are negative, try to map to
			 * wifi status values.
			 */
			if (st == -ETIMEDOUT) {
				st = WIFI_STATUS_CONN_TIMEOUT;
			}
		}

		PR_WARNING("Connection request failed (%s/%d)\n", wifi_conn_status_txt(st), st);
		print_ieee80211_codes(sh, cb);
	} else {
		PR("Connected\n");
	}

	context.connecting = false;
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;
	const struct shell *sh = context.sh;
	int st = status->status;

	if (context.disconnecting) {
		if (st == WIFI_REASON_DISCONN_SUCCESS || st == WIFI_REASON_DISCONN_USER_REQUEST) {
			PR("Disconnection request done\n");
		} else {
			PR_WARNING("Disconnection request failed (%s/%d)\n",
				   wifi_disconn_reason_txt(st), st);
		}
		context.disconnecting = false;
	} else {
		if (st && st != WIFI_REASON_DISCONN_UNSPECIFIED) {
			PR("Disconnected (%s/%d)\n", wifi_disconn_reason_txt(st), st);
		} else {
			PR("Disconnected\n");
		}
	}

	print_ieee80211_codes(sh, cb);
}

/* Zephyr prints the events with context.sh, which is NULL before the first
 * command (printk() then; the port has no such output): nothing is printed
 * until a wifi command has run on a session.
 */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				    struct net_if *iface)
{
	ARG_UNUSED(iface);

	if (context.sh == NULL) {
		return;
	}

	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		handle_wifi_disconnect_result(cb);
		break;
	default:
		break;
	}
}

static void wifi_mgmt_scan_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
					 struct net_if *iface)
{
	ARG_UNUSED(iface);

	if (context.sh == NULL) {
		return;
	}

	switch (mgmt_event) {
	case NET_EVENT_WIFI_SCAN_RESULT:
		handle_wifi_scan_result(cb);
		break;
	case NET_EVENT_WIFI_SCAN_DONE:
		handle_wifi_scan_done(cb);
		break;
	default:
		break;
	}
}

static int __wifi_args_to_params(const struct shell *sh, size_t argc, char *argv[],
				 struct wifi_connect_req_params *params)
{
	int opt;
	int opt_index = 0;
	struct sys_getopt_state *state;
	static const struct sys_getopt_option long_options[] = {
		{"ssid", sys_getopt_required_argument, 0, 's'},
		{"passphrase", sys_getopt_required_argument, 0, 'p'},
		{"key-mgmt", sys_getopt_required_argument, 0, 'k'},
		{"help", sys_getopt_no_argument, 0, 'h'},
		{0, 0, 0, 0}};
	bool secure_connection = false;
	int ret = 0;
	long val;

	/* Defaults */
	params->band = WIFI_FREQ_BAND_UNKNOWN;
	params->channel = WIFI_CHANNEL_ANY;
	params->security = WIFI_SECURITY_TYPE_NONE;
	params->mfp = WIFI_MFP_OPTIONAL;

	while ((opt = sys_getopt_long(argc, argv, "s:p:k:h", long_options, &opt_index)) != -1) {
		state = sys_getopt_state_get();
		switch (opt) {
		case 's':
			params->ssid = (const uint8_t *)state->optarg;
			params->ssid_length = strlen(state->optarg);
			if (strlen(state->optarg) > WIFI_SSID_MAX_LEN) {
				PR_WARNING("SSID too long (max %d characters)\n",
					   WIFI_SSID_MAX_LEN);
				return -EINVAL;
			}
			break;
		case 'k':
			/* The types the driver joins with: 0 and 1. */
			val = shell_strtol(state->optarg, 10, &ret);
			if (ret ||
			    (val != WIFI_SECURITY_TYPE_NONE && val != WIFI_SECURITY_TYPE_PSK)) {
				PR_ERROR("Invalid key management type: %s\n", state->optarg);
				return -EINVAL;
			}
			params->security = val;
			if (params->security) {
				secure_connection = true;
			}
			break;
		case 'p':
			params->psk = (const uint8_t *)state->optarg;
			params->psk_length = strlen(state->optarg);
			break;
		case 'h':
			/* The caller prints the help. */
			return -ENOEXEC;
		default:
			/* optopt is 0 for an unknown long option. */
			if (state->optopt != 0) {
				PR_ERROR("Invalid option %c\n", state->optopt);
			} else {
				PR_ERROR("Invalid option %s\n", argv[state->optind - 1]);
			}
			return -EINVAL;
		}
	}

	if (params->psk && !secure_connection) {
		PR_WARNING("Passphrase provided without security configuration\n");
	}

	if (!params->ssid) {
		PR_ERROR("SSID not provided\n");
		return -EINVAL;
	}

	return 0;
}

static int cmd_wifi_connect(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = get_iface();
	struct wifi_connect_req_params cnx_params = {0};
	int ret;

	context.sh = sh;
	if (__wifi_args_to_params(sh, argc, argv, &cnx_params)) {
		shell_help(sh);
		return -ENOEXEC;
	}

	context.connecting = true;
	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params,
		       sizeof(struct wifi_connect_req_params));
	if (ret) {
		LOG_ERR("Connection request failed with error: %d", ret);
		context.connecting = false;
		return -ENOEXEC;
	}

	PR("Connection requested\n");

	return 0;
}

static int cmd_wifi_disconnect(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = get_iface();
	int status;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	context.disconnecting = true;
	context.sh = sh;

	status = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);

	if (status) {
		context.disconnecting = false;

		if (status == -EALREADY) {
			PR_INFO("Already disconnected\n");
		} else {
			PR_WARNING("Disconnect request failed: %d\n", status);
			return -ENOEXEC;
		}
	} else {
		PR("Disconnect requested\n");
	}

	return 0;
}

static int cmd_wifi_scan(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = get_iface();
	struct wifi_scan_params params = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	context.sh = sh;

	/* Zephyr's net_mgmt_add_event_callback() removes it first. */
	net_mgmt_del_event_callback(&wifi_shell_scan_cb);
	net_mgmt_add_event_callback(&wifi_shell_scan_cb);

	if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(params))) {
		PR_WARNING("Scan request failed\n");
		return -ENOEXEC;
	}

	PR("Scan requested\n");

	return 0;
}

static int cmd_wifi_status(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = get_iface();
	struct wifi_iface_status status = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	context.sh = sh;

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
		     sizeof(struct wifi_iface_status))) {
		PR_WARNING("Status request failed\n");

		return -ENOEXEC;
	}

	PR("Status: successful\n");
	PR("==================\n");
	PR("State: %s\n", wifi_state_txt(status.state));

	if (status.state >= WIFI_STATE_ASSOCIATED) {
		uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

		PR("Interface Mode: %s\n", wifi_mode_txt(status.iface_mode));
		PR("Link Mode: %s\n", wifi_link_mode_txt(status.link_mode));
		PR("SSID: %.32s\n", status.ssid);
		PR("BSSID: %s\n",
		   net_sprint_ll_addr_buf((const uint8_t *)status.bssid, WIFI_MAC_ADDR_LEN,
					  (char *)mac_string_buf, sizeof(mac_string_buf)));
		PR("Band: %s\n", wifi_band_txt(status.band));
		PR("Channel: %d\n", (int)status.channel);
		/* Zephyr's "%s %s%s" with the WPA3 enterprise and WEP key types,
		 * both "none" here: empty strings.
		 */
		PR("Security: %s %s%s\n", "", wifi_security_txt(status.security), "");
		PR("MFP: %s\n", wifi_mfp_txt(status.mfp));
		if (status.iface_mode == WIFI_MODE_INFRA) {
			PR("RSSI: %d\n", status.rssi);
		}
		PR("Beacon Interval: %d\n", status.beacon_interval);
		PR("DTIM: %d\n", status.dtim_period);
		PR("TWT: %s\n", status.twt_capable ? "Supported" : "Not supported");
		PR("Current PHY TX rate (Mbps) : %.1f\n", (double)status.current_phy_tx_rate);
	}

	return 0;
}

SHELL_SUBCMD_ADD((wifi), connect, NULL,
		 SHELL_HELP("Connect to a Wi-Fi AP",
			    "<-s --ssid \"<SSID>\">: SSID to connect\n"
			    "[-p, --passphrase]: Passphrase (valid only for secure SSIDs)\n"
			    "[-k, --key-mgmt]: Key Management type (valid only for secure SSIDs)\n"
			    "0:None, 1:WPA2-PSK"),
		 cmd_wifi_connect, 2, 5);

SHELL_SUBCMD_ADD((wifi), disconnect, NULL, SHELL_HELP("Disconnect from the Wi-Fi AP", NULL),
		 cmd_wifi_disconnect, 1, 0);

SHELL_SUBCMD_ADD((wifi), scan, NULL, SHELL_HELP("Scan for Wi-Fi APs", NULL), cmd_wifi_scan, 1, 0);

SHELL_SUBCMD_ADD((wifi), status, NULL, SHELL_HELP("Status of the Wi-Fi interface", NULL),
		 cmd_wifi_status, 1, 0);

/* SHELL_SUBCMD_SET_CREATE(wifi_commands, (wifi)): the subcommands, sorted
 * by name as Zephyr's linker section sorts them.  "cred" is the copy of
 * wifi_credentials_shell.c's (with the stored credentials,
 * CONFIG_WIFI_CREDENTIALS).
 */
#ifdef CONFIG_WIFI_CREDENTIALS
extern const struct shell_static_entry z_shell_subcmd_wifi_cred;
#endif

static const struct shell_static_entry *const wifi_subcmds[] = {
	&z_shell_subcmd_wifi_connect,
#ifdef CONFIG_WIFI_CREDENTIALS
	&z_shell_subcmd_wifi_cred,
#endif
	&z_shell_subcmd_wifi_disconnect,
	&z_shell_subcmd_wifi_scan,
	&z_shell_subcmd_wifi_status,
};

static void wifi_commands_get(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(wifi_subcmds)) {
		*entry = *wifi_subcmds[idx];
	} else {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(wifi_commands, wifi_commands_get);

SHELL_CMD_REGISTER(wifi, &wifi_commands, "Wi-Fi commands", NULL);

/* wifi_shell_init(), Zephyr's SYS_INIT(APPLICATION): the OS calls it at
 * boot, after the network's initialization and before the shell runs.
 */
int wifi_shell_init(void)
{
	context.sh = NULL;
	context.all = 0U;
	context.scan_result = 0U;

	net_mgmt_init_event_callback(&wifi_shell_mgmt_cb, wifi_mgmt_event_handler,
				     WIFI_SHELL_MGMT_EVENTS);

	net_mgmt_add_event_callback(&wifi_shell_mgmt_cb);

	net_mgmt_init_event_callback(&wifi_shell_scan_cb, wifi_mgmt_scan_event_handler,
				     WIFI_SHELL_SCAN_EVENTS);

	return 0;
}
