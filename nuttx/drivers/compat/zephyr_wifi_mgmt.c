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
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#define WIFI_SAE_PSWD_MAX_LEN 128  /* CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH */

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
