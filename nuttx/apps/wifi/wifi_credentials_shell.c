/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* NuttX port: the "wifi" command with its "cred" subcommands: a copy of
 * Zephyr's subsys/net/lib/wifi_credentials/wifi_credentials_shell.c
 * (Zephyr v4.5.0) with the same syntax, messages and return values, on the
 * Zephyr compatibility layer (nuttx/drivers/compat) and ../subcmd, with
 * these changes:
 * - sys_getopt_long() is the C library's getopt_long() (NuttX's: it does not
 *   move the words that are not options to the end, and it logs nothing for
 *   an unknown option; Zephyr's does both, the log at <wrn>), through the
 *   names below; each run of the command is a new task with a fresh getopt()
 *   state, as Zephyr's shell resets it before each command,
 * - shell_help() is subcmd_help() of the command's entry, and the
 *   registration at the end makes the builtin "wifi" command on ../subcmd
 *   ("wifi" has only "cred": the Zephyr build's other wifi subcommands are
 *   NuttX's wapi and ifconfig, see nuttx/README.md),
 * - %zu / PRIu32 for size_t and uint32_t values that Zephyr prints with %d
 *   (-Wformat of NuttX's build; the same digits),
 * - shell_fprintf() with the default color is printf().
 * shell_print() goes to stdout, shell_warn() and shell_error() to stderr
 * (<zephyr/shell/shell.h> of the layer).
 */

#include <nuttx/config.h>

#include <getopt.h>
#include <inttypes.h>
#include <unistd.h>

/* The C library's getopt names are macros in NuttX's <unistd.h>; read them
 * here, then give the names back to the copied code's state structure.
 */
static inline char *z_getopt_optarg(void)
{
	return optarg;
}

static inline int z_getopt_optopt(void)
{
	return optopt;
}

#undef optarg
#undef optopt

#include <string.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/shell/shell.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h>

#include <zephyr/net/wifi_credentials.h>

#include "subcmd.h"

/* Zephyr's <zephyr/sys/sys_getopt.h> on getopt_long().  Zephyr's optopt is
 * 0 when the command starts (sys_getopt_init()) and changes only when an
 * option is refused, to that option; NuttX's starts as '?'.
 */
struct sys_getopt_state {
	char *optarg;
	int optopt;
};

static int z_getopt_last_optopt;

static struct sys_getopt_state *sys_getopt_state_get(void)
{
	static struct sys_getopt_state state;

	state.optarg = z_getopt_optarg();
	state.optopt = z_getopt_last_optopt;
	return &state;
}

static int z_getopt_long(int argc, char *const argv[], const char *optstring,
			 const struct option *longopts, int *longindex)
{
	int ret = getopt_long(argc, argv, optstring, longopts, longindex);

	if (ret == '?' || ret == ':') {
		z_getopt_last_optopt = z_getopt_optopt();
	}
	return ret;
}

#define sys_getopt_long              z_getopt_long
#define sys_getopt_option            option
#define sys_getopt_required_argument required_argument
#define sys_getopt_no_argument       no_argument

/* Zephyr's shell_fprintf() with SHELL_VT100_COLOR_DEFAULT (no color). */
#define shell_fprintf(sh, color, fmt, ...) ((void)(sh), printf(fmt, ##__VA_ARGS__))

/* shell_help(sh) in cmd_add_network(): the help of "wifi cred add". */
static void cred_add_help(void);
#define shell_help(sh) ((void)(sh), cred_add_help())

#define MAX_BANDS_STR_LEN 64
#define MACSTR "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx"

static void print_network_info(void *cb_arg, const char *ssid, size_t ssid_len)
{
	int ret = 0;
	struct wifi_credentials_personal creds = {0};
	const struct shell *sh = (const struct shell *)cb_arg;

	ret = wifi_credentials_get_by_ssid_personal_struct(ssid, ssid_len, &creds);
	if (ret) {
		shell_error(sh,
			    "An error occurred when trying to load credentials for network \"%.*s\""
			    ". err: %d",
			    (int)ssid_len, ssid, ret);
		return;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
		      "  network ssid: \"%.*s\", ssid_len: %zu, type: %s", (int)ssid_len, ssid,
		      ssid_len, wifi_security_txt(creds.header.type));

	if (creds.header.type == WIFI_SECURITY_TYPE_PSK ||
	    creds.header.type == WIFI_SECURITY_TYPE_PSK_SHA256 ||
	    creds.header.type == WIFI_SECURITY_TYPE_SAE ||
	    creds.header.type == WIFI_SECURITY_TYPE_WPA_PSK) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
			      ", password: \"%.*s\", password_len: %zu", (int)creds.password_len,
			      creds.password, creds.password_len);
	}

#ifdef CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE
	if (creds.header.type == WIFI_SECURITY_TYPE_EAP_TLS) {
		if (creds.header.key_passwd_length > 0) {
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
				      ", key_passwd: \"%.*s\", key_passwd_len: %d",
				      creds.header.key_passwd_length, creds.header.key_passwd,
				      creds.header.key_passwd_length);
		}
		if (creds.header.aid_length > 0) {
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
				      ", anon_id: \"%.*s\", anon_id_len: %d",
				      creds.header.aid_length, creds.header.anon_id,
				      creds.header.aid_length);
		}
	}
#endif /* CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE */

	if (creds.header.flags & WIFI_CREDENTIALS_FLAG_BSSID) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", bssid: " MACSTR,
			      creds.header.bssid[0], creds.header.bssid[1], creds.header.bssid[2],
			      creds.header.bssid[3], creds.header.bssid[4], creds.header.bssid[5]);
	}

	if (creds.header.flags & WIFI_CREDENTIALS_FLAG_2_4GHz) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", band: 2.4GHz");
	}

	if (creds.header.flags & WIFI_CREDENTIALS_FLAG_5GHz) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", band: 5GHz");
	}

	if (creds.header.channel) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", channel: %d", creds.header.channel);
	}

	if (creds.header.flags & WIFI_CREDENTIALS_FLAG_FAVORITE) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", favorite");
	}

	if (creds.header.flags & WIFI_CREDENTIALS_FLAG_MFP_REQUIRED) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", MFP_REQUIRED");
	} else if (creds.header.flags & WIFI_CREDENTIALS_FLAG_MFP_DISABLED) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", MFP_DISABLED");
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", MFP_OPTIONAL");
	}

	if (creds.header.timeout) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, ", timeout: %" PRIu32,
			      creds.header.timeout);
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
}

static int cmd_add_network(const struct shell *sh, size_t argc, char *argv[])
{
	int opt;
	int opt_index = 0;
	struct sys_getopt_state *state;
	static const struct sys_getopt_option long_options[] = {
		{"ssid", sys_getopt_required_argument, 0, 's'},
		{"passphrase", sys_getopt_required_argument, 0, 'p'},
		{"key-mgmt", sys_getopt_required_argument, 0, 'k'},
		{"ieee-80211w", sys_getopt_required_argument, 0, 'w'},
		{"bssid", sys_getopt_required_argument, 0, 'm'},
		{"band", sys_getopt_required_argument, 0, 'b'},
		{"channel", sys_getopt_required_argument, 0, 'c'},
		{"timeout", sys_getopt_required_argument, 0, 't'},
		{"identity", sys_getopt_required_argument, 0, 'a'},
		{"key-passwd", sys_getopt_required_argument, 0, 'K'},
		{"help", sys_getopt_no_argument, 0, 'h'},		 {0, 0, 0, 0}};
	char *endptr;
	bool secure_connection = false;
	uint8_t band;
	struct wifi_credentials_personal creds = {0};

	const uint8_t all_bands[] = {WIFI_FREQ_BAND_2_4_GHZ, WIFI_FREQ_BAND_5_GHZ,
				     WIFI_FREQ_BAND_6_GHZ};
	bool found = false;
	char bands_str[MAX_BANDS_STR_LEN] = {0};
	size_t offset = 0;
	long channel;
	long mfp = WIFI_MFP_OPTIONAL;
	int ret;

	while ((opt = sys_getopt_long(argc, argv, "s:p:k:w:b:c:m:t:a:K:h",
				      long_options, &opt_index)) != -1) {
		state = sys_getopt_state_get();
		switch (opt) {
		case 's':
			creds.header.ssid_len = strlen(state->optarg);
			if (creds.header.ssid_len > WIFI_SSID_MAX_LEN) {
				shell_warn(sh, "SSID too long (max %d characters)\n",
					   WIFI_SSID_MAX_LEN);
				return -EINVAL;
			}
			memcpy(creds.header.ssid, state->optarg, creds.header.ssid_len);
			break;
		case 'k':
			creds.header.type = atoi(state->optarg);
			if (creds.header.type) {
				secure_connection = true;
			}
			break;
		case 'p':
			creds.password_len = strlen(state->optarg);
			if (creds.password_len < WIFI_PSK_MIN_LEN) {
				shell_warn(sh, "Passphrase should be minimum %d characters\n",
					   WIFI_PSK_MIN_LEN);
				return -EINVAL;
			}
			if (creds.password_len > WIFI_PSK_MAX_LEN) {
				shell_warn(sh, "Passphrase too long (max %d characters)\n",
					   WIFI_PSK_MAX_LEN);
				return -EINVAL;
			}
			memcpy(creds.password, state->optarg, creds.password_len);
			break;
		case 'c':
			channel = strtol(state->optarg, &endptr, 10);
			if (*endptr != '\0') {
				shell_error(sh, "Invalid channel: %s\n", state->optarg);
				return -EINVAL;
			}

			for (band = 0; band < ARRAY_SIZE(all_bands); band++) {
				offset += snprintf(bands_str + offset, sizeof(bands_str) - offset,
						   "%s%s", band ? "," : "",
						   wifi_band_txt(all_bands[band]));
				if (offset >= sizeof(bands_str)) {
					shell_error(sh,
						    "Failed to parse channel: %ld: "
						    "band string too long\n",
						    channel);
					return -EINVAL;
				}

				if (wifi_utils_validate_chan(all_bands[band], channel)) {
					found = true;
					break;
				}
			}

			if (!found) {
				shell_error(sh, "Invalid channel: %ld, checked bands: %s\n",
					    channel, bands_str);
				return -EINVAL;
			}

			creds.header.channel = channel;
			break;
		case 'b':
			switch (atoi(state->optarg)) {
			case 2:
				creds.header.flags |= WIFI_CREDENTIALS_FLAG_2_4GHz;
				break;
			case 5:
				creds.header.flags |= WIFI_CREDENTIALS_FLAG_5GHz;
				break;
			case 6:
				creds.header.flags |= WIFI_CREDENTIALS_FLAG_6GHz;
				break;
			default:
				shell_error(sh, "Invalid band: %d\n", atoi(state->optarg));
				return -EINVAL;
			}
			break;
		case 'w':
			if (creds.header.type == WIFI_SECURITY_TYPE_NONE ||
			    creds.header.type == WIFI_SECURITY_TYPE_WPA_PSK) {
				shell_error(sh, "MFP not supported for security type %s",
					    wifi_security_txt(creds.header.type));
				return -ENOTSUP;
			}
			mfp = strtol(state->optarg, &endptr, 10);
			if (*endptr != '\0') {
				shell_error(sh, "Invalid IEEE 802.11w value: %s", state->optarg);
				return -EINVAL;
			}
			if (mfp == WIFI_MFP_DISABLE) {
				creds.header.flags |= WIFI_CREDENTIALS_FLAG_MFP_DISABLED;
			} else if (mfp == WIFI_MFP_REQUIRED) {
				creds.header.flags |= WIFI_CREDENTIALS_FLAG_MFP_REQUIRED;
			} else if (mfp > 2) {
				shell_error(sh, "Invalid IEEE 802.11w value: %s",
					    state->optarg);
				return -EINVAL;
			}
			break;
		case 'm':
			if (net_bytes_from_str(creds.header.bssid, sizeof(creds.header.bssid),
					       state->optarg) < 0) {
				shell_warn(sh, "Invalid MAC address\n");
				return -EINVAL;
			}
			creds.header.flags |= WIFI_CREDENTIALS_FLAG_BSSID;
			break;
		case 'a':
			creds.header.aid_length = strlen(state->optarg);
			if (creds.header.aid_length > WIFI_ENT_IDENTITY_MAX_LEN) {
				shell_warn(sh, "anon_id too long (max %d characters)\n",
					   WIFI_ENT_IDENTITY_MAX_LEN);
				return -EINVAL;
			}
			memcpy(creds.header.anon_id, state->optarg, creds.header.aid_length);
			creds.header.flags |= WIFI_CREDENTIALS_FLAG_ANONYMOUS_IDENTITY;
			break;
		case 'K':
			creds.header.key_passwd_length = strlen(state->optarg);
			if (creds.header.key_passwd_length > WIFI_ENT_PSWD_MAX_LEN) {
				shell_warn(sh, "key_passwd too long (max %d characters)\n",
					   WIFI_ENT_PSWD_MAX_LEN);
				return -EINVAL;
			}
			memcpy(creds.header.key_passwd, state->optarg,
			       creds.header.key_passwd_length);
			creds.header.flags |= WIFI_CREDENTIALS_FLAG_KEY_PASSWORD;
			break;
		case 'h':
			shell_help(sh);
			return -ENOEXEC;
		default:
			shell_error(sh, "Invalid option %c\n", state->optopt);
			return -EINVAL;
		}
	}
	if (creds.password_len > 0 && !secure_connection) {
		shell_warn(sh, "Passphrase provided without security configuration\n");
	}

	if (creds.password_len > 0 && creds.header.type == WIFI_SECURITY_TYPE_OWE) {
		shell_warn(sh, "Passphrase is not required for OWE connection\n");
		memset(creds.password, 0, sizeof(creds.password));
		creds.password_len = 0;
	}

	if (creds.header.ssid_len == 0) {
		shell_error(sh, "SSID not provided\n");
		shell_help(sh);
		return -EINVAL;
	}

#ifdef CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE
	struct net_if *iface = net_if_get_wifi_sta();

	/* Load the enterprise credentials if needed */
	if (creds.header.type == WIFI_SECURITY_TYPE_EAP_TLS ||
	    creds.header.type == WIFI_SECURITY_TYPE_EAP_PEAP_MSCHAPV2 ||
	    creds.header.type == WIFI_SECURITY_TYPE_EAP_PEAP_GTC ||
	    creds.header.type == WIFI_SECURITY_TYPE_EAP_TTLS_MSCHAPV2 ||
	    creds.header.type == WIFI_SECURITY_TYPE_EAP_PEAP_TLS) {
		ret = wifi_set_enterprise_credentials(iface, 0);
		if (ret != 0) {
			shell_error(sh, "Failed to set enterprise credentials (%d)\n", ret);
			return -ENOEXEC;
		}
	}
#endif /* CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE */

	ret = wifi_credentials_set_personal_struct(&creds);

	if (ret != 0) {
		shell_error(sh, "Failed to add network credentials, err: %d", ret);
	}

	return ret;
}

static int cmd_delete_network(const struct shell *sh, size_t argc, char *argv[])
{
	if (argc != 2) {
		shell_print(sh, "Usage: wifi cred delete \"network name\"");
		return -EINVAL;
	}

	if (strnlen(argv[1], WIFI_SSID_MAX_LEN + 1) > WIFI_SSID_MAX_LEN) {
		shell_error(sh, "SSID too long");
		return -EINVAL;
	}

	shell_print(sh, "\tDeleting network ssid: \"%s\", ssid_len: %zu", argv[1], strlen(argv[1]));

#ifdef CONFIG_WIFI_SHELL_RUNTIME_CERTIFICATES
	/* Clear the certificates */
	wifi_clear_enterprise_credentials();
#endif /* CONFIG_WIFI_SHELL_RUNTIME_CERTIFICATES */

	int ret = wifi_credentials_delete_by_ssid(argv[1], strlen(argv[1]));

	if (ret == -ENOENT) {
		shell_error(sh, "No matching network with SSID: \"%s\" found", argv[1]);
	} else if (ret != 0) {
		shell_error(sh, "Failed to delete network credentials, err: %d", ret);
	}

	return ret;
}

static int cmd_list_networks(const struct shell *sh, size_t argc, char *argv[])
{
	wifi_credentials_for_each_ssid(print_network_info, (void *)sh);
	return 0;
}

#if CONFIG_WIFI_CREDENTIALS_CONNECT_STORED

static int cmd_auto_connect(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = net_if_get_wifi_sta();
	int rc;

#ifdef CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE
	rc = wifi_set_enterprise_credentials(iface, 0);
	if (rc != 0) {
		shell_error(sh, "Failed to set enterprise credentials (%d)\n", rc);
		return -ENOEXEC;
	}
#endif /* CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE */
	rc = net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, NULL, 0);

	if (rc) {
		shell_error(sh,
			    "An error occurred when trying to auto-connect to a network. err: %d",
			    rc);
	}

	return 0;
}

#endif /* CONFIG_WIFI_CREDENTIALS_CONNECT_STORED */

/* NuttX: SHELL_STATIC_SUBCMD_SET_CREATE(), SHELL_SUBCMD_ADD((wifi), cred, ...)
 * and the Zephyr build's SHELL_CMD_REGISTER(wifi, ..., "Wi-Fi commands", NULL)
 * (subsys/net/l2/wifi/wifi_shell.c) as the builtin "wifi" command on
 * ../subcmd, which prints the Zephyr shell's help and argument count errors.
 * Its handlers take (argc, argv).
 */
static int wifi_cred_add(int argc, char **argv)
{
	z_getopt_last_optopt = 0;
	return cmd_add_network(NULL, argc, argv);
}

static int wifi_cred_delete(int argc, char **argv)
{
	return cmd_delete_network(NULL, argc, argv);
}

static int wifi_cred_list(int argc, char **argv)
{
	return cmd_list_networks(NULL, argc, argv);
}

#if CONFIG_WIFI_CREDENTIALS_CONNECT_STORED
static int wifi_cred_auto_connect(int argc, char **argv)
{
	return cmd_auto_connect(NULL, argc, argv);
}
#endif /* CONFIG_WIFI_CREDENTIALS_CONNECT_STORED */

static const struct subcmd sub_wifi_cred[] = {
	SUBCMD_ARG(add, NULL,
		   "Add network to storage.\n"
		   "<-s --ssid \"<SSID>\">: SSID.\n"
		   "[-c --channel]: Channel that needs to be scanned for connection. 0:any channel.\n"
		   "[-b, --band] 0: any band (2:2.4GHz, 5:5GHz, 6:6GHz]\n"
		   "[-p, --passphrase]: Passphrase (valid only for secure SSIDs)\n"
		   "[-k, --key-mgmt]: Key Management type (valid only for secure SSIDs)\n"
		   "0:None, 1:WPA2-PSK, 2:WPA2-PSK-256, 3:SAE-HNP, 4:SAE-H2E, 5:SAE-AUTO, 6:WAPI,"
		   " 7:EAP-TLS, 8:WEP, 9: WPA-PSK, 10: WPA-Auto-Personal, 11: DPP\n"
		   "12: EAP-PEAP-MSCHAPv2, 13: EAP-PEAP-GTC, 14: EAP-TTLS-MSCHAPv2,\n"
		   "15: EAP-PEAP-TLS, 20: SAE-EXT-KEY\n"
		   "[-w, --ieee-80211w]: MFP (optional: needs security type to be specified)\n"
		   ": 0:Disable, 1:Optional, 2:Required.\n"
		   "[-m, --bssid]: MAC address of the AP (BSSID).\n"
		   "[-t, --timeout]: Timeout for the connection attempt (in seconds).\n"
		   "[-a, --anon-id]: Anonymous identity for enterprise mode.\n"
		   "[-K, --key1-pwd for eap phase1 or --key2-pwd for eap phase2]:\n"
		   "Private key passwd for enterprise mode. Default no password for private key.\n"
		   "[-S, --wpa3-enterprise]: WPA3 enterprise mode:\n"
		   "Default 0: Not WPA3 enterprise mode.\n"
		   "1:Suite-b mode, 2:Suite-b-192-bit mode, 3:WPA3-enterprise-only mode.\n"
		   "[-T, --TLS-cipher]: 0:TLS-NONE, 1:TLS-ECC-P384, 2:TLS-RSA-3K.\n"
		   "[-V, --eap-version]: 0 or 1. Default 1: eap version 1.\n"
		   "[-I, --eap-id1]: Client Identity. Default no eap identity.\n"
		   "[-P, --eap-pwd1]: Client Password.\n"
		   "Default no password for eap user.\n"
		   "[-R, --ieee-80211r]: Use IEEE80211R fast BSS transition connect."
		   "[-h, --help]: Print out the help for the add network command.\n",
		   wifi_cred_add,
		   2, 12),
	SUBCMD_ARG(delete, NULL,
		   "Delete network from storage.\n",
		   wifi_cred_delete,
		   0, 0),
	SUBCMD_ARG(list, NULL,
		   "List stored networks.\n",
		   wifi_cred_list,
		   0, 0),

#if CONFIG_WIFI_CREDENTIALS_CONNECT_STORED
	SUBCMD_ARG(auto_connect, NULL,
		   "Connect to any stored network.\n",
		   wifi_cred_auto_connect,
		   0, 0),
#endif /* CONFIG_WIFI_CREDENTIALS_CONNECT_STORED */

	SUBCMD_SET_END
};

static const struct subcmd wifi_commands[] = {
	SUBCMD_ARG(cred, sub_wifi_cred,
		   "Wifi credentials management.\n",
		   NULL,
		   0, 0),
	SUBCMD_SET_END
};

static const struct subcmd wifi_root = SUBCMD(wifi, wifi_commands, "Wi-Fi commands", NULL);

static void cred_add_help(void)
{
	subcmd_help(&sub_wifi_cred[0]);
}

int main(int argc, char *argv[])
{
	return subcmd_main(&wifi_root, argc, argv);
}
