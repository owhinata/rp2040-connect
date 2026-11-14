/* SPDX-License-Identifier: Apache-2.0 */
/* Text and parsing helpers of Zephyr's Wi-Fi management (Zephyr v4.5.0)
 * that the "wifi cred" command (nuttx/apps/wifi) uses, with Zephyr's code:
 *
 *   wifi_security_txt(), wifi_band_txt()   subsys/net/l2/wifi/wifi_mgmt.c
 *   wifi_utils_validate_chan*()            subsys/net/l2/wifi/wifi_utils.c
 *   net_bytes_from_str()                   subsys/net/ip/utils.c
 *
 * The NET_ERR() of wifi_utils_validate_chan() is left out: it is compiled
 * out in the Zephyr build (CONFIG_NET_L2_WIFI_MGMT_LOG_LEVEL=0).
 * OS-independent (host-tested with the command, nuttx/tests/wifi_cred.c).
 */

/*
 * Copyright (c) 2017 Intel Corporation.
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h>
#include <zephyr/sys/util.h>

/* subsys/net/l2/wifi/wifi_mgmt.c */

const char *wifi_security_txt(enum wifi_security_type security)
{
	switch (security) {
	case WIFI_SECURITY_TYPE_NONE:
		return "OPEN";
	case WIFI_SECURITY_TYPE_OWE:
		return "OWE";
	case WIFI_SECURITY_TYPE_PSK:
		return "WPA2-PSK";
	case WIFI_SECURITY_TYPE_PSK_SHA256:
		return "WPA2-PSK-SHA256";
	case WIFI_SECURITY_TYPE_SAE_HNP:
		return "WPA3-SAE-HNP";
	case WIFI_SECURITY_TYPE_SAE_H2E:
		return "WPA3-SAE-H2E";
	case WIFI_SECURITY_TYPE_SAE_AUTO:
		return "WPA3-SAE-AUTO";
	case WIFI_SECURITY_TYPE_WAPI:
		return "WAPI";
	case WIFI_SECURITY_TYPE_EAP_TLS:
		return "EAP-TLS";
	case WIFI_SECURITY_TYPE_WEP:
		return "WEP";
	case WIFI_SECURITY_TYPE_WPA_PSK:
		return "WPA-PSK";
	case WIFI_SECURITY_TYPE_WPA_AUTO_PERSONAL:
		return "WPA/WPA2/WPA3 PSK";
	case WIFI_SECURITY_TYPE_DPP:
		return "DPP";
	case WIFI_SECURITY_TYPE_EAP_PEAP_MSCHAPV2:
		return "EAP-PEAP-MSCHAPV2";
	case WIFI_SECURITY_TYPE_EAP_PEAP_GTC:
		return "EAP-PEAP-GTC";
	case WIFI_SECURITY_TYPE_EAP_TTLS_MSCHAPV2:
		return "EAP-TTLS-MSCHAPV2";
	case WIFI_SECURITY_TYPE_EAP_PEAP_TLS:
		return "EAP-PEAP-TLS";
	case WIFI_SECURITY_TYPE_FT_PSK:
		return "FT-PSK";
	case WIFI_SECURITY_TYPE_FT_SAE:
		return "FT-SAE";
	case WIFI_SECURITY_TYPE_FT_EAP:
		return "FT-EAP";
	case WIFI_SECURITY_TYPE_FT_EAP_SHA384:
		return "FT-EAP-SHA384";
	case WIFI_SECURITY_TYPE_SAE_EXT_KEY:
		return "WPA3-SAE-EXT-KEY";
	case WIFI_SECURITY_TYPE_WEP_OPEN:
		return "WEP-OPEN";
	case WIFI_SECURITY_TYPE_WEP_SHARED:
		return "WEP-SHARED";
	case WIFI_SECURITY_TYPE_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

const char *wifi_band_txt(enum wifi_frequency_bands band)
{
	switch (band) {
	case WIFI_FREQ_BAND_2_4_GHZ:
		return "2.4GHz";
	case WIFI_FREQ_BAND_5_GHZ:
		return "5GHz";
	case WIFI_FREQ_BAND_6_GHZ:
		return "6GHz";
	case WIFI_FREQ_BAND_SUB_1_GHZ:
		return "Sub-1GHz";
	case WIFI_FREQ_BAND_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

/* subsys/net/l2/wifi/wifi_utils.c */

static const uint8_t valid_5g_chans_20mhz[] = {32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 96, 100,
	104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 159, 161,
	163, 165, 167, 169, 171, 173, 175, 177};

bool wifi_utils_validate_chan_2g(uint16_t chan)
{
	if ((chan >= 1) && (chan <= 14)) {
		return true;
	}

	return false;
}


bool wifi_utils_validate_chan_5g(uint16_t chan)
{
	uint16_t i;

	for (i = 0; i < ARRAY_SIZE(valid_5g_chans_20mhz); i++) {
		if (chan == valid_5g_chans_20mhz[i]) {
			return true;
		}
	}

	return false;
}


bool wifi_utils_validate_chan_6g(uint16_t chan)
{
	if (((chan >= 1) && (chan <= 233) && (!((chan - 1) % 4))) ||
	    (chan == 2)) {
		return true;
	}

	return false;
}


bool wifi_utils_validate_chan(uint8_t band,
			      uint16_t chan)
{
	bool result = false;

	switch (band) {
	case WIFI_FREQ_BAND_2_4_GHZ:
		result = wifi_utils_validate_chan_2g(chan);
		break;
	case WIFI_FREQ_BAND_5_GHZ:
		result = wifi_utils_validate_chan_5g(chan);
		break;
	case WIFI_FREQ_BAND_6_GHZ:
		result = wifi_utils_validate_chan_6g(chan);
		break;
	default:
		break;
	}

	return result;
}

/* subsys/net/ip/utils.c */

int net_bytes_from_str(uint8_t *buf, int buf_len, const char *src)
{
	size_t i;
	size_t src_len = strlen(src);
	char *endptr;

	for (i = 0U; i < src_len; i++) {
		if (!isxdigit((unsigned char)src[i]) &&
		    src[i] != ':') {
			return -EINVAL;
		}
	}

	(void)memset(buf, 0, buf_len);

	for (i = 0U; i < (size_t)buf_len; i++) {
		buf[i] = (uint8_t)strtol(src, &endptr, 16);
		src = ++endptr;
	}

	return 0;
}
