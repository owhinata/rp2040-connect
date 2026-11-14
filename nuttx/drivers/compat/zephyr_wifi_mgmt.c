/* SPDX-License-Identifier: Apache-2.0 */
/* The Wi-Fi requests and events of <zephyr/net/wifi_mgmt.h>: the handlers
 * of Zephyr's subsys/net/l2/wifi/wifi_mgmt.c (Zephyr v4.5.0) for
 * NET_REQUEST_WIFI_SCAN, _CONNECT, _DISCONNECT and _IFACE_STATUS, with the
 * same checks before the driver's operation (the Zephyr build has no
 * wpa_supplicant: CONFIG_WIFI_NM_WPA_SUPPLICANT is off), the scan callback
 * that turns each result into NET_EVENT_WIFI_SCAN_RESULT and the end into
 * NET_EVENT_WIFI_SCAN_DONE, and the raise functions of the connect and
 * disconnect results.  The operations are those of the interface's
 * offloaded API (NET_DEVICE_DT_OFFLOAD_DEFINE() of the driver).
 *
 * With the stored credentials (CONFIG_NANO_WIFI_CREDENTIALS, the Zephyr
 * build's CONFIG_WIFI_CREDENTIALS_CONNECT_STORED), also the handler of
 * NET_REQUEST_WIFI_CONNECT_STORED, connect_stored_command() of the same
 * file with its helpers: each stored network in turn becomes a
 * NET_REQUEST_WIFI_CONNECT until one is accepted.  The Zephyr build has no
 * static network (CONFIG_WIFI_CREDENTIALS_STATIC off) and no enterprise
 * security, and compiles this file's log messages out
 * (CONFIG_NET_L2_WIFI_MGMT_LOG_LEVEL=0), so those parts are left out.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <string.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#ifdef CONFIG_NANO_WIFI_CREDENTIALS
#  include <zephyr/net/wifi_credentials.h>
#endif

static const struct wifi_mgmt_ops *get_wifi_api(struct net_if *iface)
{
  const struct net_wifi_mgmt_offload *api;

  if (iface == NULL || iface->api == NULL)
    {
      return NULL;
    }

  api = iface->api;
  return api->wifi_mgmt_api;
}

static void scan_result_cb(struct net_if *iface, int status,
                           struct wifi_scan_result *entry)
{
  if (!iface)
    {
      return;
    }

  if (!entry)
    {
      struct wifi_status scan_status =
        {
          .status = status,
        };

      net_mgmt_event_notify_with_info(NET_EVENT_WIFI_SCAN_DONE, iface,
                                      &scan_status,
                                      sizeof(struct wifi_status));
      return;
    }

  net_mgmt_event_notify_with_info(NET_EVENT_WIFI_SCAN_RESULT, iface, entry,
                                  sizeof(struct wifi_scan_result));
}

int net_mgmt_NET_REQUEST_WIFI_SCAN(uint64_t mgmt_request,
                                   struct net_if *iface, void *data,
                                   size_t len)
{
  struct wifi_scan_params *params = data;
  const struct wifi_mgmt_ops *const wifi_mgmt_api = get_wifi_api(iface);

  (void)mgmt_request;
  (void)len;
  if (wifi_mgmt_api == NULL || wifi_mgmt_api->scan == NULL)
    {
      return -ENOTSUP;
    }

  if (!net_if_is_admin_up(iface))
    {
      return -ENETDOWN;
    }

  return wifi_mgmt_api->scan(NULL, iface, params, scan_result_cb);
}

int net_mgmt_NET_REQUEST_WIFI_CONNECT(uint64_t mgmt_request,
                                      struct net_if *iface, void *data,
                                      size_t len)
{
  struct wifi_connect_req_params *params = data;
  const struct wifi_mgmt_ops *const wifi_mgmt_api = get_wifi_api(iface);

  (void)mgmt_request;
  (void)len;
  if (wifi_mgmt_api == NULL || wifi_mgmt_api->connect == NULL)
    {
      return -ENOTSUP;
    }

  if (!net_if_is_admin_up(iface))
    {
      return -ENETDOWN;
    }

  if (params->channel > WIFI_CHANNEL_MAX &&
      params->channel != WIFI_CHANNEL_ANY)
    {
      return -EINVAL;
    }

  if (!params->ssid)
    {
      return -EINVAL;
    }

  if (params->ssid_length <= 0 || params->ssid_length > WIFI_SSID_MAX_LEN)
    {
      return -EINVAL;
    }

  if (params->security == WIFI_SECURITY_TYPE_WEP ||
      params->security == WIFI_SECURITY_TYPE_WEP_OPEN ||
      params->security == WIFI_SECURITY_TYPE_WEP_SHARED)
    {
      if (params->psk_length &&
          params->psk_length != 5 && params->psk_length != 13 &&
          params->psk_length != 10 && params->psk_length != 26)
        {
          return -EINVAL;
        }
    }
  else if (params->psk_length &&
           (params->psk_length < 8 || params->psk_length > 64))
    {
      return -EINVAL;
    }

  if (params->sae_password_length &&
      (params->sae_password_length < 8 ||
       params->sae_password_length > WIFI_SAE_PSWD_MAX_LEN))
    {
      return -EINVAL;
    }

  if (params->security > WIFI_SECURITY_TYPE_MAX)
    {
      return -EINVAL;
    }

  switch (params->security)
    {
      case WIFI_SECURITY_TYPE_PSK:
      case WIFI_SECURITY_TYPE_WPA_PSK:
      case WIFI_SECURITY_TYPE_PSK_SHA256:
      case WIFI_SECURITY_TYPE_WPA_AUTO_PERSONAL:
        if (!params->psk_length || !params->psk)
          {
            return -EINVAL;
          }
        break;

      case WIFI_SECURITY_TYPE_SAE_HNP:
      case WIFI_SECURITY_TYPE_SAE_H2E:
      case WIFI_SECURITY_TYPE_SAE_AUTO:
        if ((!params->psk_length || !params->psk) &&
            (!params->sae_password_length || !params->sae_password))
          {
            return -EINVAL;
          }
        break;

      case WIFI_SECURITY_TYPE_OWE:
        if (params->psk_length || params->sae_password_length)
          {
            return -EINVAL;
          }
        break;

      case WIFI_SECURITY_TYPE_WEP:
      case WIFI_SECURITY_TYPE_WEP_OPEN:
      case WIFI_SECURITY_TYPE_WEP_SHARED:
        if (!params->psk_length || !params->psk)
          {
            return -EINVAL;
          }
        break;

      default:
        break;
    }

  return wifi_mgmt_api->connect(NULL, iface, params);
}

int net_mgmt_NET_REQUEST_WIFI_DISCONNECT(uint64_t mgmt_request,
                                         struct net_if *iface, void *data,
                                         size_t len)
{
  const struct wifi_mgmt_ops *const wifi_mgmt_api = get_wifi_api(iface);

  (void)mgmt_request;
  (void)data;
  (void)len;
  if (wifi_mgmt_api == NULL || wifi_mgmt_api->disconnect == NULL)
    {
      return -ENOTSUP;
    }

  if (!net_if_is_admin_up(iface))
    {
      return -ENETDOWN;
    }

  return wifi_mgmt_api->disconnect(NULL, iface);
}

int net_mgmt_NET_REQUEST_WIFI_IFACE_STATUS(uint64_t mgmt_request,
                                           struct net_if *iface, void *data,
                                           size_t len)
{
  const struct wifi_mgmt_ops *const wifi_mgmt_api = get_wifi_api(iface);
  struct wifi_iface_status *status = data;

  (void)mgmt_request;
  if (wifi_mgmt_api == NULL || wifi_mgmt_api->iface_status == NULL)
    {
      return -ENOTSUP;
    }

  if (!data || len != sizeof(*status))
    {
      return -EINVAL;
    }

  return wifi_mgmt_api->iface_status(NULL, iface, status);
}

void wifi_mgmt_raise_connect_result_event(struct net_if *iface, int status)
{
  struct wifi_status cnx_status =
    {
      .status = status,
    };

  net_mgmt_event_notify_with_info(NET_EVENT_WIFI_CONNECT_RESULT, iface,
                                  &cnx_status, sizeof(struct wifi_status));
}

void wifi_mgmt_raise_disconnect_result_event(struct net_if *iface,
                                             int status)
{
  struct wifi_status cnx_status =
    {
      .status = status,
    };

  net_mgmt_event_notify_with_info(NET_EVENT_WIFI_DISCONNECT_RESULT, iface,
                                  &cnx_status, sizeof(struct wifi_status));
}

#ifdef CONFIG_NANO_WIFI_CREDENTIALS

/* Zephyr's subsys/net/l2/wifi/wifi_mgmt.c, CONFIG_WIFI_CREDENTIALS_CONNECT_STORED. */

static int __stored_creds_to_params(struct wifi_credentials_personal *creds,
				    struct wifi_connect_req_params *params)
{
	/* SSID */
	if (creds->header.ssid_len > WIFI_SSID_MAX_LEN) {
		return -EINVAL;
	}

	memcpy((uint8_t *)params->ssid, creds->header.ssid, creds->header.ssid_len);
	params->ssid_length = creds->header.ssid_len;

	/* PSK (optional) */
	if (creds->password_len > 0 && creds->password_len <= WIFI_PSK_MAX_LEN) {
		memcpy((uint8_t *)params->psk, creds->password, creds->password_len);
		params->psk_length = creds->password_len;
	}

	/* Defaults */
	params->security = creds->header.type;

	/* If channel is set to 0 we default to ANY. 0 is not a valid Wi-Fi channel. */
	params->channel = (creds->header.channel != 0) ? creds->header.channel : WIFI_CHANNEL_ANY;
	params->timeout = (creds->header.timeout != 0)
				  ? creds->header.timeout
				  : CONFIG_WIFI_CREDENTIALS_CONNECT_STORED_CONNECTION_TIMEOUT;

	/* Security type (optional) */
	if (creds->header.type > WIFI_SECURITY_TYPE_MAX) {
		params->security = WIFI_SECURITY_TYPE_NONE;
	}

	if (creds->header.flags & WIFI_CREDENTIALS_FLAG_2_4GHz) {
		params->band = WIFI_FREQ_BAND_2_4_GHZ;
	} else if (creds->header.flags & WIFI_CREDENTIALS_FLAG_5GHz) {
		params->band = WIFI_FREQ_BAND_5_GHZ;
	} else {
		params->band = WIFI_FREQ_BAND_UNKNOWN;
	}

	/* MFP setting (default: optional) */
	if (creds->header.flags & WIFI_CREDENTIALS_FLAG_MFP_DISABLED) {
		params->mfp = WIFI_MFP_DISABLE;
	} else if (creds->header.flags & WIFI_CREDENTIALS_FLAG_MFP_REQUIRED) {
		params->mfp = WIFI_MFP_REQUIRED;
	} else {
		params->mfp = WIFI_MFP_OPTIONAL;
	}

	return 0;
}

static int add_network_from_credentials_struct_personal(struct wifi_credentials_personal *creds,
							struct net_if *iface)
{
	int ret = 0;
	uint8_t ssid[WIFI_SSID_MAX_LEN + 1] = {0};
	uint8_t psk[WIFI_PSK_MAX_LEN + 1] = {0};

	struct wifi_connect_req_params cnx_params = {
		.ssid = ssid,
		.psk = psk,
	};

	if (__stored_creds_to_params(creds, &cnx_params) != 0) {
		ret = -ENOEXEC;
		goto out;
	}

	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params,
		       sizeof(struct wifi_connect_req_params));
	if (ret < 0) {
		ret = -ENOEXEC;
		goto out;
	}

out:
	return ret;
}

struct add_stored_network_arg {
	struct net_if *iface;
	bool connected;
};

static void add_stored_network(void *cb_arg, const char *ssid, size_t ssid_len)
{
	struct add_stored_network_arg *arg = cb_arg;
	int ret = 0;
	struct wifi_credentials_personal creds;

	if (arg->connected) {
		/* Already connected */
		return;
	}

	/* load stored data */
	ret = wifi_credentials_get_by_ssid_personal_struct(ssid, ssid_len, &creds);

	if (ret) {
		return;
	}

	ret = add_network_from_credentials_struct_personal(&creds, arg->iface);
	if (ret == 0) {
		/* Indicate that we are connected */
		arg->connected = true;
	}
}

static int add_static_network_config(struct net_if *iface)
{
	(void)iface;
	return 0;
}

int net_mgmt_NET_REQUEST_WIFI_CONNECT_STORED(uint64_t mgmt_request,
					     struct net_if *iface, void *data,
					     size_t len)
{
	struct add_stored_network_arg cb_arg = {
		.iface = iface,
	};
	int ret = 0;

	(void)mgmt_request;
	(void)data;
	(void)len;
	ret = add_static_network_config(iface);
	if (ret) {
		return ret;
	}

	wifi_credentials_for_each_ssid(add_stored_network, &cb_arg);

	return ret;
}

#endif /* CONFIG_NANO_WIFI_CREDENTIALS */
