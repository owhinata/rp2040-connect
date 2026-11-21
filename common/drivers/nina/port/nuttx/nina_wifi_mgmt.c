/* SPDX-License-Identifier: Apache-2.0 */
/* The Zephyr Wi-Fi management that the applications use on NuttX, on the
 * core's Wi-Fi operations (common/drivers/nina/nina_wifi.h).  The
 * applications are the sources shared with the Zephyr build, on the
 * Zephyr compatibility layer:
 *
 *   wifi_profile.c (common/imu)  NET_REQUEST_WIFI_CONNECT_STORED,
 *                                NET_EVENT_WIFI_CONNECT_RESULT,
 *                                net_if_get_wifi_sta(),
 *                                net_if_ipv4_get_global_addr(), and the
 *                                core's reconnection hooks
 *   "wifi cred auto_connect"     NET_REQUEST_WIFI_CONNECT_STORED
 *   (nuttx/apps/wifi)
 *   time_sync.c (common/imu)     NET_EVENT_WIFI_CONNECT_RESULT and
 *                                _DISCONNECT_RESULT, net_if_get_default(),
 *                                net_if_ipv4_get_gw()
 *   imu_raw_mqtt.c (common/imu)  net_if_get_default(), net_if_ipv4_get_gw()
 *
 * The layer's NET_REQUEST_WIFI_CONNECT_STORED becomes one
 * NET_REQUEST_WIFI_CONNECT for each stored network
 * (common/compat/zephyr_wifi_mgmt.c), which reaches nina_wifi_connect()
 * through the one operation of this interface; the core's results come
 * back as the layer's events from wlan0's binding (nina_netdev.c).  The
 * interface is wlan0's device (net_if.h of nuttx/drivers/compat reads its
 * addresses).  wapi does not come here: nina_netdev.c calls the core.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "nina_wifi.h"
#include "nina_wifi_mgmt.h"
#ifdef CONFIG_NANO_WIFI_PROFILE
#  include "wifi_profile.h"
#endif

static struct net_if g_iface;

/* NET_REQUEST_WIFI_CONNECT, after the layer's checks: Zephyr's parameters
 * as the Zephyr build's binding passes them
 * (common/drivers/nina/port/zephyr/nina_net.c).
 */

static int wifi_connect(const struct device *dev, struct net_if *iface,
                        struct wifi_connect_req_params *params)
{
  struct nina_wifi_connect_params p;

  (void)dev;
  (void)iface;
  if (params == NULL)
    {
      return -EINVAL;
    }

  p.ssid = params->ssid;
  p.ssid_len = params->ssid_length;
  p.security = params->security == WIFI_SECURITY_TYPE_NONE ?
               NINA_WIFI_SECURITY_NONE :
               params->security == WIFI_SECURITY_TYPE_PSK ?
               NINA_WIFI_SECURITY_PSK : NINA_WIFI_SECURITY_OTHER;
  p.psk = params->psk;
  p.psk_len = params->psk_length;
  p.psk_is_pbkdf2 = params->psk_is_pbkdf2;
  return nina_wifi_connect(&p);
}

static const struct wifi_mgmt_ops g_mgmt =
{
  .connect = wifi_connect,
};

static const struct net_wifi_mgmt_offload g_api =
{
  .wifi_mgmt_api = &g_mgmt,
};

int nina_wifi_mgmt_initialize(struct net_driver_s *dev)
{
  int ret;

  /* Zephyr's net_init(): the event thread, then the interfaces. */

  ret = net_mgmt_event_init();
  if (ret < 0)
    {
      return ret;
    }

  g_iface.dev = dev;
  g_iface.api = &g_api;
  z_net_if_register(&g_iface);
  return 0;
}

void nina_wifi_mgmt_connect_result(int status)
{
  wifi_mgmt_raise_connect_result_event(&g_iface, status);
}

void nina_wifi_mgmt_disconnect_result(void)
{
  wifi_mgmt_raise_disconnect_result_event(&g_iface, 0);
}

bool nina_wifi_mgmt_connection_allowed(void)
{
#ifdef CONFIG_NANO_WIFI_PROFILE
  return wifi_profile_connection_allowed();
#else
  return true;
#endif
}

void nina_wifi_mgmt_pause_reconnect(void)
{
#ifdef CONFIG_NANO_WIFI_PROFILE
  wifi_profile_pause_reconnect();
#endif
}

void nina_wifi_mgmt_request_reconnect(void)
{
#ifdef CONFIG_NANO_WIFI_PROFILE
  wifi_profile_request_reconnect();
#endif
}
