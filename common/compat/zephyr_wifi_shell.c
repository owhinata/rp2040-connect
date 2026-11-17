/* SPDX-License-Identifier: Apache-2.0 */
/* The "wifi" command of the Zephyr build (Zephyr v4.5.0) for the ports
 * without Zephyr's network shell: the part of subsys/net/l2/wifi/wifi_shell.c
 * that the NINA-W102 driver (common/drivers/nina/nina_net.c) can serve,
 * "wifi connect", "wifi disconnect", "wifi scan" and "wifi status", with
 * their syntax, help, messages, return values and net_mgmt event output,
 * and the root command "wifi" with those and "wifi cred" (the copy of
 * Zephyr's wifi_credentials_shell.c, common/compat/subsys/net/lib/
 * wifi_credentials, registered there with SHELL_SUBCMD_ADD()).  Zephyr's
 * other subcommands (ap, twt, ps, reg_domain, statistics, ...) are not
 * here: the driver has none of their operations, so in the Zephyr build
 * they only fail.
 *
 * Changes from Zephyr's file:
 * - the subcommand set of "wifi" is listed here (Zephyr's
 *   SHELL_SUBCMD_SET_CREATE(wifi_commands, (wifi)) collects the
 *   SHELL_SUBCMD_ADD() entries of all sources, sorted by name; see
 *   <zephyr/shell/shell.h> of the layer);
 * - the structures of the layer's <zephyr/net/wifi_mgmt.h> have only the
 *   members the driver uses: "wifi connect" checks the options of the other
 *   members (enterprise, AP mode) as Zephyr does and then drops them; the
 *   WPA3 enterprise type and WEP key type of the scan results and the
 *   status are Zephyr's "none" (the driver reports neither);
 * - get_iface(): the one interface is index 1 (net_if_get_by_index());
 * - printk() of "Connection request failed with error" is LOG_ERR() (the
 *   Zephyr build's printk() goes to its log, CONFIG_LOG_PRINTK, so the
 *   message is in "dmesg" in both, here with the log's prefix);
 * - the scan's event callback is removed before it is added again: Zephyr's
 *   net_mgmt_add_event_callback() does that itself, the layer's does not
 *   (common/compat/zephyr_net_mgmt.c);
 * - the helpers it needs from Zephyr's wifi_mgmt.c (wifi_state_txt(),
 *   wifi_mode_txt(), wifi_link_mode_txt(), wifi_mfp_txt(),
 *   wifi_conn_status_txt(), wifi_disconn_reason_txt(), ...),
 *   wifi_utils.c (wifi_utils_parse_scan_bands(), _ssids(), _chan(), without
 *   their NET_ERR(), compiled out in the Zephyr build), net/ip/utils.c
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

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_getopt.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(net_wifi_shell, LOG_LEVEL_INF);

/* <zephyr/toolchain/gcc.h> */
#ifndef __fallthrough
#define __fallthrough __attribute__((__fallthrough__))
#endif

/* subsys/net/lib/shell/net_shell_private.h (sh is never NULL here: the
 * commands run on a session, and the events print to the session of the
 * last command).
 */
#define PR(fmt, ...)         shell_fprintf_normal(sh, fmt, ##__VA_ARGS__)
#define PR_ERROR(fmt, ...)   shell_fprintf_error(sh, fmt, ##__VA_ARGS__)
#define PR_INFO(fmt, ...)    shell_fprintf_info(sh, fmt, ##__VA_ARGS__)
#define PR_WARNING(fmt, ...) shell_fprintf_warn(sh, fmt, ##__VA_ARGS__)

/* <zephyr/net/wifi.h>, <zephyr/net/wifi_mgmt.h>, <zephyr/net/wifi_utils.h> */
#define WIFI_MGMT_SCAN_MAX_BSS_CNT  65535
#define WIFI_MGMT_BAND_STR_SIZE_MAX 8
#define WIFI_UTILS_MAX_BAND_STR_LEN 3
#define WIFI_UTILS_MAX_CHAN_STR_LEN 4
#define WIFI_ENT_IDENTITY_MAX_USERS 8

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

enum wifi_wpa3_enterprise_type {
	WIFI_WPA3_ENTERPRISE_NA = 0,
	WIFI_WPA3_ENTERPRISE_SUITEB = 1,
	WIFI_WPA3_ENTERPRISE_SUITEB_192,
	WIFI_WPA3_ENTERPRISE_ONLY,
};

enum wifi_wep_key_type {
	WIFI_WEP_KEY_TYPE_UNKNOWN = 0,
	WIFI_WEP_KEY_TYPE_64,
	WIFI_WEP_KEY_TYPE_128,
};

/* The members of Zephyr's struct wifi_connect_req_params that the layer's
 * structure does not have: "wifi connect" checks their options and drops
 * them (the NINA driver has no enterprise or AP mode).
 */
struct connect_extra {
	const uint8_t *anon_id;
	uint8_t aid_length;
	const uint8_t *key_passwd;
	uint8_t key_passwd_length;
	const uint8_t *key2_passwd;
	uint8_t key2_passwd_length;
	int wpa3_ent_mode;
	int TLS_cipher;
	bool verify_peer_cert;
	int eap_ver;
	const uint8_t *eap_identity;
	uint8_t eap_id_length;
	uint8_t nusers;
	const uint8_t *eap_password;
	uint8_t eap_passwd_length;
	uint8_t passwds;
	bool ft_used;
	int ssid_protection;
	uint8_t ignore_broadcast_ssid;
	const char *server_cert_domain_exact;
	const char *server_cert_domain_suffix;
	uint8_t transition_disable;
};

/* subsys/net/l2/wifi/wifi_mgmt.c ------------------------------------------*/

static const char *wifi_wpa3_enterprise_txt(enum wifi_wpa3_enterprise_type wpa3_ent)
{
	switch (wpa3_ent) {
	case WIFI_WPA3_ENTERPRISE_SUITEB:
		return "WPA3-SuiteB";
	case WIFI_WPA3_ENTERPRISE_SUITEB_192:
		return "WPA3-SuiteB-192";
	case WIFI_WPA3_ENTERPRISE_ONLY:
		return "WPA3-Enterprise-Only";
	default:
		return "";
	}
}

static const char *wifi_wep_key_type_txt(enum wifi_wep_key_type wep_key_type)
{
	switch (wep_key_type) {
	case WIFI_WEP_KEY_TYPE_64:
		return " (64-bit key)";
	case WIFI_WEP_KEY_TYPE_128:
		return " (128-bit key)";
	case WIFI_WEP_KEY_TYPE_UNKNOWN:
	default:
		return "";
	}
}

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

/* subsys/net/l2/wifi/wifi_utils.c (without its NET_ERR() messages) ---------*/

static const uint8_t valid_5g_chans_20mhz[] = {
	32,  36,  40,  44,  48,  52,  56,  60,  64,  68,  96,  100, 104, 108, 112, 116, 120, 124,
	128, 132, 136, 140, 144, 149, 153, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177};

static enum wifi_frequency_bands wifi_utils_map_band_str_to_idx(char *band_str)
{
	enum wifi_frequency_bands band = WIFI_FREQ_BAND_UNKNOWN;

	if (!strcmp(band_str, "2")) {
		band = WIFI_FREQ_BAND_2_4_GHZ;
	} else if (!strcmp(band_str, "5")) {
		band = WIFI_FREQ_BAND_5_GHZ;
	} else if (!strcmp(band_str, "6")) {
		band = WIFI_FREQ_BAND_6_GHZ;
	}

	return band;
}

static uint16_t wifi_utils_get_next_chan_6g(uint16_t chan)
{
	if (chan == 1) {
		return 2;
	} else if (chan == 2) {
		return 5;
	} else {
		return chan + 4;
	}
}

static int wifi_utils_get_all_chans_in_range(uint8_t chan_start, uint8_t chan_end,
					     struct wifi_band_channel *band_chan, uint8_t band_idx,
					     uint8_t *chan_idx)
{
	uint8_t i;
	bool start = false;
	bool end = false;
	uint8_t idx;

	if (!wifi_utils_validate_chan(band_idx, chan_start)) {
		return -EINVAL;
	}

	if (!wifi_utils_validate_chan(band_idx, chan_end)) {
		return -EINVAL;
	}

	if (chan_end < chan_start) {
		return -EINVAL;
	}

	switch (band_idx) {
	case WIFI_FREQ_BAND_2_4_GHZ:
		idx = *chan_idx;

		for (i = chan_start + 1; i <= chan_end; i++) {
			band_chan[idx].band = band_idx;
			band_chan[idx].channel = i;
			idx++;
		}

		*chan_idx = idx;

		break;
	case WIFI_FREQ_BAND_5_GHZ:
		idx = *chan_idx;

		for (i = 0; i < ARRAY_SIZE(valid_5g_chans_20mhz); i++) {
			if (valid_5g_chans_20mhz[i] == chan_start) {
				start = true;
				continue;
			}

			if (valid_5g_chans_20mhz[i] == chan_end) {
				end = true;
			}

			if (start) {
				if (idx > WIFI_MGMT_SCAN_CHAN_MAX_MANUAL) {
					*chan_idx = idx;
					break;
				}
				band_chan[idx].band = band_idx;
				band_chan[idx].channel = valid_5g_chans_20mhz[i];
				idx++;
			}

			if (end) {
				*chan_idx = idx;
				break;
			}
		}

		break;
	case WIFI_FREQ_BAND_6_GHZ:
		idx = *chan_idx;

		i = wifi_utils_get_next_chan_6g(chan_start);

		while (i <= chan_end) {
			band_chan[idx].band = band_idx;
			band_chan[idx].channel = i;
			idx++;

			i = wifi_utils_get_next_chan_6g(i);
		}

		*chan_idx = idx;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wifi_utils_validate_chan_str(const char *chan_str)
{
	size_t i;
	size_t len;

	len = strlen(chan_str);
	if (len == 0) {
		return -EINVAL;
	}

	for (i = 0; i < len; i++) {
		if (!isdigit((int)chan_str[i])) {
			return -EINVAL;
		}
	}

	return 0;
}

static int wifi_utils_parse_scan_bands(char *scan_bands_str, uint8_t *band_map)
{
	char parse_str[WIFI_MGMT_BAND_STR_SIZE_MAX + 1];
	char *band_str = NULL;
	char *ctx = NULL;
	enum wifi_frequency_bands band = WIFI_FREQ_BAND_UNKNOWN;
	int len;

	if (!scan_bands_str) {
		return -EINVAL;
	}

	len = strlen(scan_bands_str);

	if (len > WIFI_MGMT_BAND_STR_SIZE_MAX) {
		return -EINVAL;
	}

	strncpy(parse_str, scan_bands_str, sizeof(parse_str) - 1);
	parse_str[sizeof(parse_str) - 1] = '\0';

	band_str = strtok_r(parse_str, ",", &ctx);

	while (band_str) {
		band = wifi_utils_map_band_str_to_idx(band_str);

		if (band == WIFI_FREQ_BAND_UNKNOWN) {
			return -EINVAL;
		}

		*band_map |= (1 << band);

		band_str = strtok_r(NULL, ",", &ctx);
	}

	return 0;
}

static int wifi_utils_parse_scan_ssids(char *scan_ssids_str, const char *ssids[], uint8_t num_ssids)
{
	int len;

	if (!scan_ssids_str) {
		return -EINVAL;
	}

	len = strlen(scan_ssids_str);
	if (len > WIFI_SSID_MAX_LEN) {
		return -EINVAL;
	}

	for (int i = 0; i < num_ssids; i++) {
		if (ssids[i] != NULL) {
			continue;
		}
		ssids[i] = scan_ssids_str;
		return 0;
	}

	return 0;
}

static int wifi_utils_parse_scan_chan(char *scan_chan_str, struct wifi_band_channel *band_chan,
				      uint8_t max_channels)
{
	char band_str[WIFI_UTILS_MAX_BAND_STR_LEN] = {0};
	char chan_str[WIFI_UTILS_MAX_CHAN_STR_LEN] = {0};
	enum wifi_frequency_bands band = WIFI_FREQ_BAND_UNKNOWN;
	uint16_t band_str_start_idx = 0;
	uint16_t chan_str_start_idx = 0;
	uint8_t chan_idx = 0;
	uint8_t chan_start = 0;
	uint8_t chan_val = 0;
	uint16_t i = 0;
	bool valid_band = false;
	bool valid_chan = false;

	while (scan_chan_str[i] != '\0') {
		if (scan_chan_str[i] != ':') {
			i++;
			continue;
		}

		if (((i - band_str_start_idx) <= 0) ||
		    ((i - band_str_start_idx) > WIFI_UTILS_MAX_BAND_STR_LEN)) {
			return -EINVAL;
		}

		strncpy(band_str, &scan_chan_str[band_str_start_idx], (i - band_str_start_idx));

		band = wifi_utils_map_band_str_to_idx(band_str);

		if (band == WIFI_FREQ_BAND_UNKNOWN) {
			return -EINVAL;
		}

		i++;
		chan_str_start_idx = i;
		valid_band = true;

		while (1) {
			if ((scan_chan_str[i] != ',') && (scan_chan_str[i] != '_') &&
			    (scan_chan_str[i] != '-') && (scan_chan_str[i] != '\0')) {
				i++;
				continue;
			}

			if ((i - chan_str_start_idx) > WIFI_UTILS_MAX_CHAN_STR_LEN) {
				return -EINVAL;
			}

			strncpy(chan_str, &scan_chan_str[chan_str_start_idx],
				(i - chan_str_start_idx));

			if (wifi_utils_validate_chan_str(chan_str)) {
				return -EINVAL;
			}

			chan_val = atoi(chan_str);

			memset(chan_str, 0, sizeof(chan_str));

			if (chan_start) {
				if (wifi_utils_get_all_chans_in_range(chan_start, chan_val,
								      band_chan, band,
								      &chan_idx)) {
					return -EINVAL;
				}

				if (chan_idx > max_channels) {
					return -EINVAL;
				}
				chan_start = 0;
			} else {
				if (!wifi_utils_validate_chan(band, chan_val)) {
					return -EINVAL;
				}
				if (chan_idx == max_channels) {
					return -EINVAL;
				}

				band_chan[chan_idx].band = band;
				band_chan[chan_idx].channel = chan_val;
				chan_idx++;
			}

			valid_chan = true;

			if (scan_chan_str[i] == '_') {
				band_str_start_idx = ++i;
				break;
			} else if (scan_chan_str[i] == ',') {
				chan_str_start_idx = ++i;
			} else if (scan_chan_str[i] == '-') {
				chan_start = chan_val;
				chan_str_start_idx = ++i;
			} else if (scan_chan_str[i] == '\0') {
				break;
			}
		}
	}

	if (!valid_band) {
		return -EINVAL;
	}

	if (!valid_chan) {
		return -EINVAL;
	}

	return 0;
}

/* subsys/net/l2/wifi/wifi_shell.c ------------------------------------------*/

#define WIFI_SHELL_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

#define WIFI_SHELL_SCAN_EVENTS (NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE)

#define MAX_BANDS_STR_LEN 64

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

enum iface_type {
	IFACE_TYPE_STA,
	IFACE_TYPE_SAP,
};

static struct net_if *get_iface(enum iface_type type, int argc, char *argv[])
{
	struct net_if *iface = NULL;
	int iface_index = -1;

	/* Parse arguments manually to find -i or --iface,
	 * it's intentional not to use sys_getopt() here to avoid
	 * permuting the arguments.
	 */
	for (int i = 1; i < argc; i++) {
		if ((strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--iface") == 0) &&
		    (i + 1 < argc)) {
			int err = 0;

			iface_index = shell_strtol(argv[i + 1], 10, &err);
			if (err || iface_index < 0) {
				LOG_ERR("Invalid interface index: %s", argv[i + 1]);
				return NULL;
			}
			break;
		}
	}

	if (iface_index != -1) {
		/* net_if_get_by_index(): the one interface is 1. */
		iface = iface_index == 1 ? net_if_get_default() : NULL;
		if (iface == NULL) {
			LOG_WRN("Invalid interface index: %d, using default", iface_index);
		}
	}

	if (iface == NULL) {
		/* Fallback to STA or SAP if user didn't specify a valid interface */
		if (type == IFACE_TYPE_STA) {
			iface = net_if_get_wifi_sta();
		}

		if (iface == NULL) {
			LOG_ERR("No default interface found for type: %s",
				type == IFACE_TYPE_STA ? "STA" : "SAP");
			return NULL;
		}
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
				 struct wifi_connect_req_params *params,
				 enum wifi_iface_mode iface_mode)
{
	int opt;
	int opt_index = 0;
	struct sys_getopt_state *state;
	static const struct sys_getopt_option long_options[] = {
		{"ssid", sys_getopt_required_argument, 0, 's'},
		{"passphrase", sys_getopt_required_argument, 0, 'p'},
		{"key-mgmt", sys_getopt_required_argument, 0, 'k'},
		{"pbkdf2", sys_getopt_no_argument, 0, 'D'},
		{"ieee-80211w", sys_getopt_required_argument, 0, 'w'},
		{"bssid", sys_getopt_required_argument, 0, 'm'},
		{"band", sys_getopt_required_argument, 0, 'b'},
		{"channel", sys_getopt_required_argument, 0, 'c'},
		{"timeout", sys_getopt_required_argument, 0, 't'},
		{"anon-id", sys_getopt_required_argument, 0, 'a'},
		{"bandwidth", sys_getopt_required_argument, 0, 'B'},
		{"key1-pwd", sys_getopt_required_argument, 0, 'K'},
		{"key2-pwd", sys_getopt_required_argument, 0, 'K'},
		{"wpa3-enterprise", sys_getopt_required_argument, 0, 'S'},
		{"TLS-cipher", sys_getopt_required_argument, 0, 'T'},
		{"verify-peer-cert", sys_getopt_required_argument, 0, 'A'},
		{"eap-version", sys_getopt_required_argument, 0, 'V'},
		{"eap-id1", sys_getopt_required_argument, 0, 'I'},
		{"eap-id2", sys_getopt_required_argument, 0, 'I'},
		{"eap-id3", sys_getopt_required_argument, 0, 'I'},
		{"eap-id4", sys_getopt_required_argument, 0, 'I'},
		{"eap-id5", sys_getopt_required_argument, 0, 'I'},
		{"eap-id6", sys_getopt_required_argument, 0, 'I'},
		{"eap-id7", sys_getopt_required_argument, 0, 'I'},
		{"eap-id8", sys_getopt_required_argument, 0, 'I'},
		{"eap-pwd1", sys_getopt_required_argument, 0, 'P'},
		{"eap-pwd2", sys_getopt_required_argument, 0, 'P'},
		{"eap-pwd3", sys_getopt_required_argument, 0, 'P'},
		{"eap-pwd4", sys_getopt_required_argument, 0, 'P'},
		{"eap-pwd5", sys_getopt_required_argument, 0, 'P'},
		{"eap-pwd6", sys_getopt_required_argument, 0, 'P'},
		{"eap-pwd7", sys_getopt_required_argument, 0, 'P'},
		{"eap-pwd8", sys_getopt_required_argument, 0, 'P'},
		{"ignore-broadcast-ssid", sys_getopt_required_argument, 0, 'g'},
		{"ieee-80211r", sys_getopt_no_argument, 0, 'R'},
		{"iface", sys_getopt_required_argument, 0, 'i'},
		{"server-cert-domain-exact", sys_getopt_required_argument, 0, 'e'},
		{"server-cert-domain-suffix", sys_getopt_required_argument, 0, 'x'},
		{"ssid-protection", sys_getopt_required_argument, 0, 'C'},
		{"transition-disable", sys_getopt_required_argument, 0, 'd'},
		{"help", sys_getopt_no_argument, 0, 'h'},
		{0, 0, 0, 0}};
	char *endptr;
	int idx = 1;
	bool secure_connection = false;
	uint8_t band;
	const uint8_t all_bands[] = {WIFI_FREQ_BAND_2_4_GHZ, WIFI_FREQ_BAND_5_GHZ,
				     WIFI_FREQ_BAND_6_GHZ};
	bool found = false;
	char bands_str[MAX_BANDS_STR_LEN] = {0};
	size_t offset = 0;
	long channel;
	int key_passwd_cnt = 0;
	int ret = 0;
	long val = 0;
	struct connect_extra extra = {0};

	/* Defaults */
	params->band = WIFI_FREQ_BAND_UNKNOWN;
	params->channel = WIFI_CHANNEL_ANY;
	params->security = WIFI_SECURITY_TYPE_NONE;
	params->mfp = WIFI_MFP_OPTIONAL;
	extra.eap_ver = 1;
	extra.ignore_broadcast_ssid = 0;
	extra.verify_peer_cert = false;

	while ((opt = sys_getopt_long(argc, argv, "s:p:k:e:x:w:b:c:m:t:a:B:K:S:C:T:A:V:I:P:g:Rh:i:d:",
				      long_options, &opt_index)) != -1) {
		state = sys_getopt_state_get();
		switch (opt) {
		case 's':
			params->ssid = (const uint8_t *)state->optarg;
			params->ssid_length = strlen(state->optarg);
			if (strlen(state->optarg) > WIFI_SSID_MAX_LEN) {
				PR_WARNING("SSID too long (max %d characters)\n", WIFI_SSID_MAX_LEN);
				return -EINVAL;
			}
			break;
		case 'k':
			params->security = atoi(state->optarg);
			if (params->security) {
				secure_connection = true;
			}
			/* WPA3 security types (SAE and OWE) require MFP (802.11w) as required,
			 * if not otherwise set.
			 */
			if (params->security == WIFI_SECURITY_TYPE_SAE_HNP ||
			    params->security == WIFI_SECURITY_TYPE_SAE_H2E ||
			    params->security == WIFI_SECURITY_TYPE_SAE_AUTO ||
			    params->security == WIFI_SECURITY_TYPE_OWE) {
				params->mfp = WIFI_MFP_REQUIRED;
			}
			break;
		case 'p':
			params->psk = (const uint8_t *)state->optarg;
			params->psk_length = strlen(state->optarg);
			break;
		case 'D':
			params->psk_is_pbkdf2 = true;
			break;
		case 'c':
			channel = strtol(state->optarg, &endptr, 10);
			if (iface_mode == WIFI_MODE_AP && channel == 0) {
				params->channel = channel;
				break;
			}
			for (band = 0; band < ARRAY_SIZE(all_bands); band++) {
				offset += snprintf(bands_str + offset, sizeof(bands_str) - offset,
						   "%s%s", band ? "," : "",
						   wifi_band_txt(all_bands[band]));
				if (offset >= sizeof(bands_str)) {
					PR_ERROR("Failed to parse channel: %s: "
						 "band string too long\n",
						 argv[idx]);
					return -EINVAL;
				}

				if (wifi_utils_validate_chan(all_bands[band], channel)) {
					found = true;
					break;
				}
			}

			if (!found) {
				PR_ERROR("Invalid channel: %ld, checked bands: %s\n", channel,
					 bands_str);
				return -EINVAL;
			}

			params->channel = channel;
			break;
		case 'b':
			if (iface_mode == WIFI_MODE_INFRA || iface_mode == WIFI_MODE_AP) {
				switch (atoi(state->optarg)) {
				case 2:
					params->band = WIFI_FREQ_BAND_2_4_GHZ;
					break;
				case 5:
					params->band = WIFI_FREQ_BAND_5_GHZ;
					break;
				case 6:
					params->band = WIFI_FREQ_BAND_6_GHZ;
					break;
				case 0:
					/* Allow default value when connecting */
					if (iface_mode == WIFI_MODE_INFRA) {
						params->band = WIFI_FREQ_BAND_UNKNOWN;
						break;
					}
					__fallthrough;
				default:
					PR_ERROR("Invalid band: %d\n", atoi(state->optarg));
					return -EINVAL;
				}
			}
			break;
		case 'w':
			if (params->security == WIFI_SECURITY_TYPE_NONE ||
			    params->security == WIFI_SECURITY_TYPE_WPA_PSK) {
				PR_ERROR("MFP not supported for security type %s\n",
					 wifi_security_txt(params->security));
				return -EINVAL;
			}
			params->mfp = atoi(state->optarg);
			break;
		case 'm':
			if (net_bytes_from_str(params->bssid, sizeof(params->bssid),
					       state->optarg) < 0) {
				PR_WARNING("Invalid MAC address\n");
				return -EINVAL;
			}
			break;
		case 't':
			if (iface_mode == WIFI_MODE_INFRA) {
				params->timeout = strtol(state->optarg, &endptr, 10);
				if (*endptr != '\0') {
					PR_ERROR("Invalid timeout: %s\n", state->optarg);
					return -EINVAL;
				}
			}
			break;
		case 'a':
			extra.anon_id = (const uint8_t *)state->optarg;
			extra.aid_length = strlen(state->optarg);
			if (strlen(state->optarg) > WIFI_ENT_IDENTITY_MAX_LEN) {
				PR_WARNING("anon_id too long (max %d characters)\n",
					   WIFI_ENT_IDENTITY_MAX_LEN);
				return -EINVAL;
			}
			break;
		case 'B':
			/* Only for the AP mode (bandwidth). */
			break;
		case 'K':
			if (key_passwd_cnt >= 2) {
				PR_WARNING("too many key_passwd (max 2 key_passwd)\n");
				return -EINVAL;
			}

			if (key_passwd_cnt == 0) {
				extra.key_passwd = (const uint8_t *)state->optarg;
				extra.key_passwd_length = strlen(state->optarg);
				if (strlen(state->optarg) > WIFI_ENT_PSWD_MAX_LEN) {
					PR_WARNING("key_passwd too long (max %d characters)\n",
						   WIFI_ENT_PSWD_MAX_LEN);
					return -EINVAL;
				}
			} else if (key_passwd_cnt == 1) {
				extra.key2_passwd = (const uint8_t *)state->optarg;
				extra.key2_passwd_length = strlen(state->optarg);
				if (strlen(state->optarg) > WIFI_ENT_PSWD_MAX_LEN) {
					PR_WARNING("key2_passwd too long (max %d characters)\n",
						   WIFI_ENT_PSWD_MAX_LEN);
					return -EINVAL;
				}
			}
			key_passwd_cnt++;
			break;
		case 'S':
			extra.wpa3_ent_mode = atoi(state->optarg);
			if (extra.wpa3_ent_mode != WIFI_WPA3_ENTERPRISE_NA) {
				params->mfp = WIFI_MFP_REQUIRED;
			}
			break;
		case 'T':
			extra.TLS_cipher = atoi(state->optarg);
			break;
		case 'A':
			if (iface_mode == WIFI_MODE_INFRA) {
				extra.verify_peer_cert = !!atoi(state->optarg);
			}
			break;
		case 'V':
			extra.eap_ver = atoi(state->optarg);
			if (extra.eap_ver != 0 && extra.eap_ver != 1 && extra.eap_ver != -1) {
				PR_WARNING("eap_ver error %d\n", extra.eap_ver);
				return -EINVAL;
			}
			break;
		case 'I':
			if (extra.nusers >= WIFI_ENT_IDENTITY_MAX_USERS) {
				PR_WARNING("too many eap identities (max %d identities)\n",
					   WIFI_ENT_IDENTITY_MAX_USERS);
				return -EINVAL;
			}

			extra.eap_identity = (const uint8_t *)state->optarg;
			extra.eap_id_length = strlen(state->optarg);

			extra.nusers++;
			if (strlen(state->optarg) > WIFI_ENT_IDENTITY_MAX_LEN) {
				PR_WARNING("eap identity too long (max %d characters)\n",
					   WIFI_ENT_IDENTITY_MAX_LEN);
				return -EINVAL;
			}
			break;
		case 'P':
			if (extra.passwds >= WIFI_ENT_IDENTITY_MAX_USERS) {
				PR_WARNING("too many eap passwds (max %d passwds)\n",
					   WIFI_ENT_IDENTITY_MAX_USERS);
				return -EINVAL;
			}

			extra.eap_password = (const uint8_t *)state->optarg;
			extra.eap_passwd_length = strlen(state->optarg);

			extra.passwds++;
			if (strlen(state->optarg) > WIFI_ENT_PSWD_MAX_LEN) {
				PR_WARNING("eap password length too long (max %d characters)\n",
					   WIFI_ENT_PSWD_MAX_LEN);
				return -EINVAL;
			}
			break;
		case 'R':
			extra.ft_used = true;
			break;
		case 'C':
			extra.ssid_protection = atoi(state->optarg);
			if (extra.ssid_protection != 0 && extra.ssid_protection != 1) {
				PR_WARNING("ssid_protection error %d\n", extra.ssid_protection);
				return -EINVAL;
			}
			break;
		case 'g':
			extra.ignore_broadcast_ssid = shell_strtol(state->optarg, 10, &ret);
			break;
		case 'i':
			/* Unused, but parsing to avoid unknown option error */
			break;
		case 'e':
			extra.server_cert_domain_exact = state->optarg;
			break;
		case 'x':
			extra.server_cert_domain_suffix = state->optarg;
			break;
		case 'd':
			if (iface_mode == WIFI_MODE_AP) {
				val = shell_strtol(state->optarg, 16, &ret);
				if (ret || val < 0x01 || val > 0x0F) {
					PR_WARNING("Invalid transition_disable "
						   "value (0x01-0x0F)\n");
					return -EINVAL;
				}
				extra.transition_disable = (uint8_t)val;
			}
			break;
		case 'h':
			shell_help(sh);
			return -ENOEXEC;
		default:
			PR_ERROR("Invalid option %c\n", state->optopt);
			return -EINVAL;
		}

		if (ret) {
			PR_ERROR("Invalid argument %d ret %d\n", opt_index, ret);
			return -EINVAL;
		}
	}

	/* For SAE types, use -p passphrase as SAE password and clear PSK. */
	if (params->psk != NULL && (params->security == WIFI_SECURITY_TYPE_SAE_HNP ||
				    params->security == WIFI_SECURITY_TYPE_SAE_H2E ||
				    params->security == WIFI_SECURITY_TYPE_SAE_AUTO ||
				    params->security == WIFI_SECURITY_TYPE_FT_SAE ||
				    params->security == WIFI_SECURITY_TYPE_SAE_EXT_KEY)) {
		params->sae_password = params->psk;
		params->sae_password_length = params->psk_length;
		params->psk = NULL;
		params->psk_length = 0;
	}

	if (params->psk && !secure_connection) {
		PR_WARNING("Passphrase provided without security configuration\n");
	}
	if (params->psk && params->security == WIFI_SECURITY_TYPE_OWE) {
		PR_WARNING("OWE (Enhanced open) does not use passphrase; ignoring -p\n");
		params->psk = NULL;
		params->psk_length = 0;
	}

	if (!params->ssid) {
		PR_ERROR("SSID not provided\n");
		return -EINVAL;
	}

	if (iface_mode == WIFI_MODE_AP && params->channel == WIFI_CHANNEL_ANY) {
		PR_ERROR("Channel not provided\n");
		return -EINVAL;
	}

	/* AP mode: channel 0 means ACS, band must be specified */
	if (iface_mode == WIFI_MODE_AP && params->channel == 0 &&
	    params->band == WIFI_FREQ_BAND_UNKNOWN) {
		PR_ERROR("Band not provided when channel is 0 (ACS)\n");
		return -EINVAL;
	}

	/* Validate band and channel combination when both are explicitly provided */
	if (params->channel != WIFI_CHANNEL_ANY && params->channel != 0 &&
	    params->band != WIFI_FREQ_BAND_UNKNOWN) {
		if (!wifi_utils_validate_chan(params->band, params->channel)) {
			PR_ERROR("Channel %d is not valid for %s band\n", params->channel,
				 wifi_band_txt(params->band));
			return -EINVAL;
		}
	}

	if (extra.ignore_broadcast_ssid > 2) {
		PR_ERROR("Invalid ignore_broadcast_ssid value\n");
		return -EINVAL;
	}

	if (params->psk_is_pbkdf2) {
		/* params->security also required, but hard to enumerate exhaustively */
		if (params->psk == NULL) {
			PR_ERROR("--pbkdf2 requires a PSK\n");
			return -EINVAL;
		}
		PR_ERROR("PBKDF2 computation only supported with WPA Supplicant\n");
		return -ENOSYS;
	}

	return 0;
}

static int cmd_wifi_connect(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = get_iface(IFACE_TYPE_STA, argc, argv);
	struct wifi_connect_req_params cnx_params = {0};
	int ret;

	context.sh = sh;
	if (__wifi_args_to_params(sh, argc, argv, &cnx_params, WIFI_MODE_INFRA)) {
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
	struct net_if *iface = get_iface(IFACE_TYPE_STA, argc, argv);
	int status;

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

static int wifi_scan_args_to_params(const struct shell *sh, size_t argc, char *argv[],
				    struct wifi_scan_params *params, bool *do_scan)
{
	int opt;
	int opt_index = 0;
	struct sys_getopt_state *state;
	static const struct sys_getopt_option long_options[] = {
		{"type", sys_getopt_required_argument, 0, 't'},
		{"bands", sys_getopt_required_argument, 0, 'b'},
		{"dwell_time_active", sys_getopt_required_argument, 0, 'a'},
		{"dwell_time_passive", sys_getopt_required_argument, 0, 'p'},
		{"ssid", sys_getopt_required_argument, 0, 's'},
		{"max_bss", sys_getopt_required_argument, 0, 'm'},
		{"chans", sys_getopt_required_argument, 0, 'c'},
		{"iface", sys_getopt_required_argument, 0, 'i'},
		{"help", sys_getopt_no_argument, 0, 'h'},
		{0, 0, 0, 0}};
	int val;
	int opt_num = 0;

	*do_scan = true;

	while ((opt = sys_getopt_long(argc, argv, "t:b:a:p:s:m:c:i:h", long_options,
				      &opt_index)) != -1) {
		state = sys_getopt_state_get();
		switch (opt) {
		case 't':
			if (!strncasecmp(state->optarg, "passive", 7)) {
				params->scan_type = WIFI_SCAN_TYPE_PASSIVE;
			} else if (!strncasecmp(state->optarg, "active", 6)) {
				params->scan_type = WIFI_SCAN_TYPE_ACTIVE;
			} else {
				PR_ERROR("Invalid scan type %s\n", state->optarg);
				return -ENOEXEC;
			}

			opt_num++;
			break;
		case 'b':
			if (wifi_utils_parse_scan_bands(state->optarg, &params->bands)) {
				PR_ERROR("Invalid band value(s)\n");
				return -ENOEXEC;
			}

			opt_num++;
			break;
		case 'a':
			val = atoi(state->optarg);

			if (val < 0) {
				PR_ERROR("Invalid dwell_time_active val\n");
				return -ENOEXEC;
			}

			params->dwell_time_active = val;
			opt_num++;
			break;
		case 'p':
			val = atoi(state->optarg);

			if (val < 0) {
				PR_ERROR("Invalid dwell_time_passive val\n");
				return -ENOEXEC;
			}

			params->dwell_time_passive = val;
			opt_num++;
			break;
		case 's':
			if (wifi_utils_parse_scan_ssids(state->optarg, params->ssids,
							ARRAY_SIZE(params->ssids))) {
				PR_ERROR("Invalid SSID(s)\n");
				return -ENOEXEC;
			}

			opt_num++;
			break;
		case 'm':
			val = atoi(state->optarg);

			if ((val < 0) || (val > WIFI_MGMT_SCAN_MAX_BSS_CNT)) {
				PR_ERROR("Invalid max_bss val\n");
				return -ENOEXEC;
			}

			params->max_bss_cnt = val;
			opt_num++;
			break;
		case 'c':
			if (wifi_utils_parse_scan_chan(state->optarg, params->band_chan,
						       ARRAY_SIZE(params->band_chan))) {
				PR_ERROR("Invalid band or channel value(s)\n");
				return -ENOEXEC;
			}

			opt_num++;
			break;
		case 'h':
			shell_help(sh);
			*do_scan = false;
			opt_num++;
			break;
		case 'i':
			/* Unused, but parsing to avoid unknown option error */
			opt_num++;
			break;
		case '?':
		default:
			PR_ERROR("Invalid option or option usage: %s\n", argv[opt_index + 1]);
			return -ENOEXEC;
		}
	}

	return opt_num;
}

static int cmd_wifi_scan(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = get_iface(IFACE_TYPE_STA, argc, argv);
	struct wifi_scan_params params = {0};
	bool do_scan = true;
	int opt_num;

	context.sh = sh;

	if (argc > 1) {
		opt_num = wifi_scan_args_to_params(sh, argc, argv, &params, &do_scan);

		if (opt_num < 0) {
			shell_help(sh);
			return -ENOEXEC;
		} else if (!opt_num) {
			PR_WARNING("No valid option(s) found\n");
			do_scan = false;
		}
	}

	if (do_scan) {
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

	PR_WARNING("Scan not initiated\n");
	return -ENOEXEC;
}

static int cmd_wifi_status(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = get_iface(IFACE_TYPE_STA, argc, argv);
	struct wifi_iface_status status = {0};

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
		PR("Security: %s %s%s\n", wifi_wpa3_enterprise_txt(WIFI_WPA3_ENTERPRISE_NA),
		   wifi_security_txt(status.security),
		   wifi_wep_key_type_txt(WIFI_WEP_KEY_TYPE_UNKNOWN));
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
			    "[-i, --iface=<interface index>]\n"
			    "<-s --ssid \"<SSID>\">: SSID to connect\n"
			    "[-c --channel]: Channel that needs to be scanned for connection. "
			    "Value 0 indicates any channel\n"
			    "[-b, --band] 0: any band (2:2.4GHz, 5:5GHz, 6:6GHz]\n"
			    "[-p, --passphrase]: Passphrase (valid only for secure SSIDs)\n"
			    "[-k, --key-mgmt]: Key Management type (valid only for secure SSIDs)\n"
			    "0:None, 1:WPA2-PSK, 2:WPA2-PSK-256, 3:SAE-HNP, 4:SAE-H2E, "
			    "5:SAE-AUTO, 6:WAPI, "
			    "7:EAP-TLS, 8:WEP, 9:WPA-PSK, 10:WPA-Auto-Personal, 11:DPP, "
			    "12:EAP-PEAP-MSCHAPv2, 13:EAP-PEAP-GTC, 14:EAP-TTLS-MSCHAPv2, "
			    "15:EAP-PEAP-TLS, 20:SAE-EXT-KEY, 21:WEP-OPEN, 22:WEP-SHARED\n"
			    "[--pbkdf2] Precompute PBKDF2 credentials (valid only for PSK types)\n"
			    "[-w, --ieee-80211w]: MFP (optional: needs security type to be "
			    "specified): 0:Disable, 1:Optional, 2:Required\n"
			    "[-m, --bssid]: MAC address of the AP (BSSID)\n"
			    "[-t, --timeout]: Timeout for the connection attempt (in seconds)\n"
			    "[-a, --anon-id]: Anonymous identity for enterprise mode\n"
			    "[-K, --key1-pwd for eap phase1 or --key2-pwd for eap phase2]:\n"
			    "Private key passwd for enterprise mode. Default is no password "
			    "for private key\n"
			    "[-S, --wpa3-enterprise]: WPA3 enterprise mode:\n"
			    "Default is 0. 0:No WPA3 enterprise mode, "
			    "1:Suite-b mode, 2:Suite-b-192-bit mode, 3:WPA3-enterprise-only mode\n"
			    "[-T, --TLS-cipher]: 0:TLS-NONE, 1:TLS-ECC-P384, 2:TLS-RSA-3K\n"
			    "[-A, --verify-peer-cert]: apply for EAP-PEAP-MSCHAPv2, "
			    "EAP-PEAP-GTC and EAP-TTLS-MSCHAPv2\n"
			    "Default is 0. 0:do not use CA to verify peer, "
			    "1:use CA to verify peer\n"
			    "[-V, --eap-version]: Forced eap-version. 0, 1 or -1.\n"
			    "Default 1: eap version 1. -1: no forced version\n"
			    "[-I, --eap-id1]: Client Identity. Default is no eap identity\n"
			    "[-P, --eap-pwd1]: Client Password. "
			    "Default is no password for eap user\n"
			    "[-R, --ieee-80211r]: Use IEEE80211R fast BSS transition connect\n"
			    "[-e, --server-cert-domain-exact]: Full domain names for "
			    "server certificate match\n"
			    "[-x, --server-cert-domain-suffix]: Domain name suffixes for "
			    "server certificate match\n"
			    "[-C, --ssid-protection]: Whether to use SSID protection in\n"
			    "4-way handshake: 0:Disable, 1:Enable"),
		 cmd_wifi_connect, 2, 49);

SHELL_SUBCMD_ADD((wifi), disconnect, NULL,
		 SHELL_HELP("Disconnect from the Wi-Fi AP", "[-i, --iface=<interface index>]"),
		 cmd_wifi_disconnect, 1, 2);

SHELL_SUBCMD_ADD((wifi), scan, NULL,
		 SHELL_HELP("Scan for Wi-Fi APs",
			    "[-i, --iface=<interface index>]\n"
			    "[-t, --type <active/passive>] : Preferred mode of scan. "
			    "The actual mode of scan can depend on factors such as the Wi-Fi chip "
			    "implementation, regulatory domain restrictions. "
			    "Default type is active\n"
			    "[-b, --bands <Comma separated list of band values (2/5/6)>] : "
			    "Bands to be scanned where 2: 2.4 GHz, 5: 5 GHz, 6: 6 GHz\n"
			    "[-a, --dwell_time_active <val_in_ms>] : "
			    "Active scan dwell time (in ms) on a channel. Range 5 ms to 1000 ms\n"
			    "[-p, --dwell_time_passive <val_in_ms>] : "
			    "Passive scan dwell time (in ms) on a channel. "
			    "Range 10 ms to 1000 ms\n"
			    "[-s, --ssid] : SSID to scan for. Can be provided multiple times\n"
			    "[-m, --max_bss <val>] : Maximum BSSes to scan for. Range 1 - 65535\n"
			    "[-c, --chans <Comma separated list of channel ranges>] : "
			    "Channels to be scanned. The channels must be specified in the form "
			    "band1:chan1,chan2_band2:chan3,..etc. band1, band2 must be valid band "
			    "values and chan1, chan2, chan3 must be specified as a list of comma "
			    "separated values where each value is either a single channel or a "
			    "channel range specified as chan_start-chan_end. Each band channel "
			    "set has to be separated by a _. For example, a valid channel "
			    "specification can be 2:1,6_5:36 or 2:1,6-11,14_5:36,163-177,52. "
			    "Care should be taken to ensure that configured channels don't exceed "
			    "CONFIG_WIFI_MGMT_SCAN_CHAN_MAX_MANUAL"),
		 cmd_wifi_scan, 1, 10);

SHELL_SUBCMD_ADD((wifi), status, NULL,
		 SHELL_HELP("Status of the Wi-Fi interface", "[-i, --iface=<interface index>]"),
		 cmd_wifi_status, 1, 2);

/* SHELL_SUBCMD_SET_CREATE(wifi_commands, (wifi)): the subcommands, sorted
 * by name as Zephyr's linker section sorts them.  "cred" is the copy of
 * wifi_credentials_shell.c's (with the stored credentials,
 * CONFIG_NANO_WIFI_CREDENTIALS).
 */
#ifdef CONFIG_NANO_WIFI_CREDENTIALS
extern const struct shell_static_entry z_shell_subcmd_wifi_cred;
#endif

static const struct shell_static_entry *const wifi_subcmds[] = {
	&z_shell_subcmd_wifi_connect,
#ifdef CONFIG_NANO_WIFI_CREDENTIALS
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
