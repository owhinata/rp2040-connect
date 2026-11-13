/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA-W102 as the NuttX network device wlan0, for ifconfig and wapi.
 *
 * The Zephyr build defines its interface with NET_DEVICE_DT_OFFLOAD_DEFINE()
 * (drivers/nina/nina_net.c): Zephyr calls driver_init() and the
 * interface's init (iface_init(): dormant, MAC address from the module) at
 * boot, and the "wifi" shell commands reach the driver's operations
 * through net_mgmt() requests (subsys/net/l2/wifi/wifi_mgmt.c) and print
 * the results the driver raises as net_mgmt events.  Here
 * nina_net_initialize() starts net_mgmt's event thread, registers wlan0
 * (NET_LL_IEEE80211, administratively up), and calls the same two
 * functions in the same order; the wireless ioctls of NuttX's wapi
 * (SIOCxIW*, via netdev's d_ioctl) are turned into the same net_mgmt
 * requests, and the scan results come back from the same events:
 *
 *   SIOCSIWSCAN      NET_REQUEST_WIFI_SCAN (ESSID, passive, channels are
 *                    passed on and refused by the driver as in Zephyr)
 *   SIOCGIWSCAN      -EAGAIN while scanning, the scan's error, or the
 *                    results (SIOCGIWAP, SIOCGIWFREQ with the channel,
 *                    IWEVQUAL with the RSSI in dBm, SIOCGIWESSID,
 *                    SIOCGIWENCODE open/encrypted, left out when the
 *                    module reports an unknown type) in the module's order;
 *                    -E2BIG if the buffer is too small
 *   SIOCSIWAUTH      kept: WPA version and pairwise cipher
 *   SIOCSIWENCODEEXT kept: the passphrase
 *                    (all three are used and forgotten by the next
 *                    SIOCSIWESSID)
 *   SIOCSIWESSID     NET_REQUEST_WIFI_CONNECT with the kept settings (WPA2
 *                    -> WIFI_SECURITY_TYPE_PSK, none -> _NONE, WPA ->
 *                    _WPA_PSK, WPA3 -> _SAE, WEP -> _WEP: the driver takes
 *                    only NONE and PSK); flags 0 (wapi's bogus SSID of
 *                    "disconnect") does nothing
 *   SIOCSIWAP        00:00:00:00:00:00 or ff:..:ff: NET_REQUEST_WIFI_DISCONNECT
 *   SIOCGIWESSID, SIOCGIWAP, SIOCGIWSENS (RSSI)
 *                    NET_REQUEST_WIFI_IFACE_STATUS (asks the module)
 *   SIOCGIWMODE      managed (IW_MODE_INFRA), SIOCSIWMODE managed only
 *
 * Other wireless ioctls return -ENOTTY.  The interface addresses (IP,
 * netmask, gateway) and the MAC address are in the device structure
 * (<zephyr/net/net_if.h> of the compatibility layer), where netdev's own
 * ioctls and /proc/net/wlan0 (ifconfig) read them.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/wireless.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "nina_net.h"
#include "nina_proto.h"

/* The module reports at most 10 networks (nina_net.c scan_thread()). */

#define NINA_SCAN_MAX        10

/* NET_DEVICE_DT_OFFLOAD_DEFINE(..., 1500) */

#define NINA_NET_MTU         1500

/* One scan result as wireless events: SIOCGIWAP, SIOCGIWFREQ, IWEVQUAL,
 * SIOCGIWESSID (with the SSID after it, as wapi expects), SIOCGIWENCODE.
 */

#define IWE_HDR              offsetof(struct iw_event, u)
#define IWE_AP_LEN           (IWE_HDR + sizeof(struct sockaddr))
#define IWE_FREQ_LEN         (IWE_HDR + sizeof(struct iw_freq))
#define IWE_QUAL_LEN         (IWE_HDR + sizeof(struct iw_quality))
#define IWE_ESSID_LEN        (IWE_HDR + sizeof(struct iw_point) + \
                              IW_ESSID_MAX_SIZE)
#define IWE_ENCODE_LEN       (IWE_HDR + sizeof(struct iw_point))
#define IWE_RESULT_LEN       (IWE_AP_LEN + IWE_FREQ_LEN + IWE_QUAL_LEN + \
                              IWE_ESSID_LEN + IWE_ENCODE_LEN)

static struct net_driver_s g_dev;
static struct net_if g_iface =
{
  .dev = &g_dev,
};

/* The last scan, filled by the net_mgmt events. */

static K_MUTEX_DEFINE(g_scan_lock);
static struct net_mgmt_event_callback g_scan_events;
static struct wifi_scan_result g_scan_results[NINA_SCAN_MAX];
static unsigned int g_scan_count;
static bool g_scan_busy;
static int g_scan_status;

/* What wapi set before SIOCSIWESSID (forgotten by it). */

static K_MUTEX_DEFINE(g_wext_lock);
static uint32_t g_wpa_version;          /* IW_AUTH_WPA_VERSION_*, 0 unset */
static uint32_t g_cipher;               /* IW_AUTH_CIPHER_*, 0 unset */
static uint8_t g_key[WIFI_PSK_MAX_LEN];
static size_t g_key_len;

/* Scan results --------------------------------------------------------- */

static void scan_event(struct net_mgmt_event_callback *cb,
                       uint64_t mgmt_event, struct net_if *iface)
{
  (void)iface;
  k_mutex_lock(&g_scan_lock, K_FOREVER);
  if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT)
    {
      if (g_scan_busy && cb->info != NULL &&
          cb->info_length == sizeof(struct wifi_scan_result) &&
          g_scan_count < NINA_SCAN_MAX)
        {
          memcpy(&g_scan_results[g_scan_count++], cb->info,
                 sizeof(struct wifi_scan_result));
        }
    }
  else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE)
    {
      struct wifi_status status;

      status.status = -EIO;
      if (cb->info != NULL && cb->info_length == sizeof(status))
        {
          memcpy(&status, cb->info, sizeof(status));
        }

      g_scan_status = status.status;
      g_scan_busy = false;
    }

  k_mutex_unlock(&g_scan_lock);
}

static int ioctl_scan_start(struct iwreq *iwr)
{
  struct wifi_scan_params params;
  struct iw_scan_req req;
  char ssid[IW_ESSID_MAX_SIZE + 1];
  int ret;

  memset(&params, 0, sizeof(params));
  params.scan_type = WIFI_SCAN_TYPE_ACTIVE;
  if (iwr->u.data.pointer != NULL && iwr->u.data.length >= sizeof(req))
    {
      memcpy(&req, iwr->u.data.pointer, sizeof(req));
      if (req.scan_type == IW_SCAN_TYPE_PASSIVE)
        {
          params.scan_type = WIFI_SCAN_TYPE_PASSIVE;
        }

      if ((iwr->u.data.flags & IW_SCAN_THIS_ESSID) != 0 &&
          req.essid_len > 0)
        {
          size_t len = MIN(req.essid_len, IW_ESSID_MAX_SIZE);

          memcpy(ssid, req.essid, len);
          ssid[len] = '\0';
          params.ssids[0] = ssid;
        }

      for (size_t i = 0;
           i < MIN(req.num_channels, WIFI_MGMT_SCAN_CHAN_MAX_MANUAL); i++)
        {
          params.band_chan[i].band = WIFI_FREQ_BAND_2_4_GHZ;
          params.band_chan[i].channel = (uint8_t)req.channel_list[i].m;
        }
    }

  k_mutex_lock(&g_scan_lock, K_FOREVER);
  if (g_scan_busy)
    {
      k_mutex_unlock(&g_scan_lock);
      return -EBUSY;
    }

  g_scan_busy = true;
  g_scan_count = 0;
  g_scan_status = 0;
  k_mutex_unlock(&g_scan_lock);

  ret = net_mgmt(NET_REQUEST_WIFI_SCAN, &g_iface, &params, sizeof(params));
  if (ret != 0)
    {
      k_mutex_lock(&g_scan_lock, K_FOREVER);
      g_scan_busy = false;
      g_scan_status = ret;
      k_mutex_unlock(&g_scan_lock);
    }

  return ret;
}

/* The bytes of one scan result as events (SIOCGIWENCODE only when the
 * security is known: wapi then shows ffff).
 */

static size_t result_len(const struct wifi_scan_result *r)
{
  return IWE_RESULT_LEN -
         (r->security == WIFI_SECURITY_TYPE_UNKNOWN ? IWE_ENCODE_LEN : 0);
}

/* Writes one scan result at DST (result_len() bytes, any alignment). */

static void put_result(uint8_t *dst, const struct wifi_scan_result *r)
{
  struct iw_event iwe;
  uint8_t essid[IW_ESSID_MAX_SIZE];

  memset(&iwe, 0, sizeof(iwe));
  iwe.len = IWE_AP_LEN;
  iwe.cmd = SIOCGIWAP;
  iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
  memcpy(iwe.u.ap_addr.sa_data, r->mac, WIFI_MAC_ADDR_LEN);
  memcpy(dst, &iwe, IWE_AP_LEN);
  dst += IWE_AP_LEN;

  memset(&iwe, 0, sizeof(iwe));
  iwe.len = IWE_FREQ_LEN;
  iwe.cmd = SIOCGIWFREQ;
  iwe.u.freq.m = r->channel;       /* e 0: a channel number */
  iwe.u.freq.e = 0;
  memcpy(dst, &iwe, IWE_FREQ_LEN);
  dst += IWE_FREQ_LEN;

  memset(&iwe, 0, sizeof(iwe));
  iwe.len = IWE_QUAL_LEN;
  iwe.cmd = IWEVQUAL;
  iwe.u.qual.level = (uint8_t)r->rssi;
  iwe.u.qual.updated = IW_QUAL_DBM;
  memcpy(dst, &iwe, IWE_QUAL_LEN);
  dst += IWE_QUAL_LEN;

  /* The SSID follows the iw_point; its pointer is the offset of the SSID
   * from the iw_point (wapi's wireless.c).
   */

  memset(&iwe, 0, sizeof(iwe));
  iwe.len = IWE_ESSID_LEN;
  iwe.cmd = SIOCGIWESSID;
  iwe.u.essid.pointer = (FAR void *)(uintptr_t)sizeof(struct iw_point);
  iwe.u.essid.length = r->ssid_length;
  iwe.u.essid.flags = 1;
  memcpy(dst, &iwe, IWE_HDR + sizeof(struct iw_point));
  memset(essid, 0, sizeof(essid));
  memcpy(essid, r->ssid, MIN(r->ssid_length, IW_ESSID_MAX_SIZE));
  memcpy(dst + IWE_HDR + sizeof(struct iw_point), essid, sizeof(essid));
  dst += IWE_ESSID_LEN;

  if (r->security != WIFI_SECURITY_TYPE_UNKNOWN)
    {
      memset(&iwe, 0, sizeof(iwe));
      iwe.len = IWE_ENCODE_LEN;
      iwe.cmd = SIOCGIWENCODE;
      iwe.u.data.flags = r->security == WIFI_SECURITY_TYPE_NONE ?
                         IW_ENCODE_DISABLED :
                         IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
      memcpy(dst, &iwe, IWE_ENCODE_LEN);
    }
}

static int ioctl_scan_results(struct iwreq *iwr)
{
  size_t need;
  uint8_t *dst;
  int ret = 0;

  k_mutex_lock(&g_scan_lock, K_FOREVER);
  if (g_scan_busy)
    {
      ret = -EAGAIN;
    }
  else if (g_scan_status < 0)
    {
      ret = g_scan_status;
    }
  else
    {
      need = 0;
      for (unsigned int i = 0; i < g_scan_count; i++)
        {
          need += result_len(&g_scan_results[i]);
        }

      if (need == 0)
        {
          iwr->u.data.length = 0;
        }
      else if (iwr->u.data.pointer == NULL || iwr->u.data.length < need)
        {
          ret = -E2BIG;
        }
      else
        {
          /* wapi puts each network at the head of its list: hand them
           * over last first so that it lists them in the module's order.
           */

          dst = iwr->u.data.pointer;
          for (unsigned int i = g_scan_count; i > 0; i--)
            {
              put_result(dst, &g_scan_results[i - 1]);
              dst += result_len(&g_scan_results[i - 1]);
            }

          iwr->u.data.length = need;
        }
    }

  k_mutex_unlock(&g_scan_lock);
  return ret;
}

/* Connect and disconnect ------------------------------------------------ */

static enum wifi_security_type wext_security(void)
{
  if (g_wpa_version == 0)
    {
      return g_key_len ? WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE;
    }

  if ((g_wpa_version & IW_AUTH_WPA_VERSION_DISABLED) != 0)
    {
      return (g_cipher & (IW_AUTH_CIPHER_WEP40 | IW_AUTH_CIPHER_WEP104)) ?
             WIFI_SECURITY_TYPE_WEP : WIFI_SECURITY_TYPE_NONE;
    }

  if ((g_wpa_version & IW_AUTH_WPA_VERSION_WPA3) != 0)
    {
      return WIFI_SECURITY_TYPE_SAE;
    }

  if ((g_wpa_version & IW_AUTH_WPA_VERSION_WPA2) != 0)
    {
      return WIFI_SECURITY_TYPE_PSK;
    }

  return WIFI_SECURITY_TYPE_WPA_PSK;
}

static int ioctl_connect(struct iwreq *iwr)
{
  struct wifi_connect_req_params params;
  uint8_t ssid[WIFI_SSID_MAX_LEN];
  size_t len = iwr->u.essid.length;
  int ret;

  /* wapi's "disconnect" sets a random SSID with the flag off after
   * clearing the BSSID (SIOCSIWAP); there is nothing to do for it.
   */

  if (iwr->u.essid.flags == 0)
    {
      return 0;
    }

  if (iwr->u.essid.pointer == NULL || len == 0 || len > WIFI_SSID_MAX_LEN)
    {
      return -EINVAL;
    }

  memcpy(ssid, iwr->u.essid.pointer, len);

  /* The wifi shell's defaults (cmd_wifi_connect()). */

  memset(&params, 0, sizeof(params));
  params.ssid = ssid;
  params.ssid_length = len;
  params.band = WIFI_FREQ_BAND_UNKNOWN;
  params.channel = WIFI_CHANNEL_ANY;
  params.mfp = WIFI_MFP_OPTIONAL;
  params.timeout = -1;

  k_mutex_lock(&g_wext_lock, K_FOREVER);
  params.security = wext_security();
  if (g_key_len > 0)
    {
      params.psk = g_key;
      params.psk_length = g_key_len;
    }

  ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, &g_iface, &params,
                 sizeof(params));

  /* The settings are used once: the passphrase is wiped, and the next
   * connection without "wapi psk" is an open one.
   */

  memset(g_key, 0, sizeof(g_key));
  g_key_len = 0;
  g_wpa_version = 0;
  g_cipher = 0;
  k_mutex_unlock(&g_wext_lock);
  return ret;
}

static int ioctl_set_auth(struct iwreq *iwr)
{
  int ret = 0;

  k_mutex_lock(&g_wext_lock, K_FOREVER);
  switch (iwr->u.param.flags & IW_AUTH_INDEX)
    {
      case IW_AUTH_WPA_VERSION:
        g_wpa_version = (uint32_t)iwr->u.param.value;
        break;

      case IW_AUTH_CIPHER_PAIRWISE:
        g_cipher = (uint32_t)iwr->u.param.value;
        break;

      default:
        ret = -ENOTSUP;
        break;
    }

  k_mutex_unlock(&g_wext_lock);
  return ret;
}

static int ioctl_set_key(struct iwreq *iwr)
{
  struct iw_encode_ext ext;
  const uint8_t *src = iwr->u.encoding.pointer;

  if (src == NULL || iwr->u.encoding.length < sizeof(ext))
    {
      return -EINVAL;
    }

  memcpy(&ext, src, sizeof(ext));
  if (ext.key_len > WIFI_PSK_MAX_LEN ||
      iwr->u.encoding.length < sizeof(ext) + ext.key_len)
    {
      return -EINVAL;
    }

  k_mutex_lock(&g_wext_lock, K_FOREVER);
  memset(g_key, 0, sizeof(g_key));
  g_key_len = 0;
  if (ext.alg != IW_ENCODE_ALG_NONE)
    {
      memcpy(g_key, src + sizeof(ext), ext.key_len);
      g_key_len = ext.key_len;
    }

  k_mutex_unlock(&g_wext_lock);
  return 0;
}

static int ioctl_set_ap(struct iwreq *iwr)
{
  static const uint8_t zero[WIFI_MAC_ADDR_LEN];
  static const uint8_t all[WIFI_MAC_ADDR_LEN] =
    {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

  const uint8_t *bssid = (const uint8_t *)iwr->u.ap_addr.sa_data;

  if (memcmp(bssid, zero, sizeof(zero)) != 0 &&
      memcmp(bssid, all, sizeof(all)) != 0)
    {
      return -ENOTSUP;
    }

  return net_mgmt(NET_REQUEST_WIFI_DISCONNECT, &g_iface, NULL, 0);
}

/* Status --------------------------------------------------------------- */

static int get_status(struct wifi_iface_status *status)
{
  memset(status, 0, sizeof(*status));
  return net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, &g_iface, status,
                  sizeof(*status));
}

static int ioctl_get_essid(struct iwreq *iwr)
{
  struct wifi_iface_status status;
  int ret;

  if (iwr->u.essid.pointer == NULL)
    {
      return -EINVAL;
    }

  ret = get_status(&status);
  if (ret != 0)
    {
      return ret;
    }

  if (status.state == WIFI_STATE_COMPLETED)
    {
      memcpy(iwr->u.essid.pointer, status.ssid,
             MIN(status.ssid_len, iwr->u.essid.length));
      iwr->u.essid.length = status.ssid_len;
      iwr->u.essid.flags = 1;
    }
  else
    {
      iwr->u.essid.length = 0;
      iwr->u.essid.flags = 0;
    }

  return 0;
}

static int ioctl_get_ap(struct iwreq *iwr)
{
  struct wifi_iface_status status;
  int ret;

  ret = get_status(&status);
  if (ret != 0)
    {
      return ret;
    }

  memset(&iwr->u.ap_addr, 0, sizeof(iwr->u.ap_addr));
  iwr->u.ap_addr.sa_family = ARPHRD_ETHER;
  if (status.state == WIFI_STATE_COMPLETED)
    {
      memcpy(iwr->u.ap_addr.sa_data, status.bssid, WIFI_MAC_ADDR_LEN);
    }

  return 0;
}

static int ioctl_get_rssi(struct iwreq *iwr)
{
  struct wifi_iface_status status;
  int ret;

  ret = get_status(&status);
  if (ret != 0)
    {
      return ret;
    }

  if (status.state != WIFI_STATE_COMPLETED)
    {
      return -ENOTCONN;
    }

  /* wapi shows -value as "Sense" (dBm), as for bcm43xxx. */

  memset(&iwr->u.sens, 0, sizeof(iwr->u.sens));
  iwr->u.sens.value = -status.rssi;
  return 0;
}

/* netdev --------------------------------------------------------------- */

static int nina_ioctl(FAR struct net_driver_s *dev, int cmd,
                      unsigned long arg)
{
  struct iwreq *iwr = (struct iwreq *)(uintptr_t)arg;

  (void)dev;
  if (iwr == NULL)
    {
      return -EINVAL;
    }

  switch (cmd)
    {
      case SIOCSIWSCAN:
        return ioctl_scan_start(iwr);

      case SIOCGIWSCAN:
        return ioctl_scan_results(iwr);

      case SIOCSIWAUTH:
        return ioctl_set_auth(iwr);

      case SIOCSIWENCODEEXT:
        return ioctl_set_key(iwr);

      case SIOCSIWESSID:
        return ioctl_connect(iwr);

      case SIOCSIWAP:
        return ioctl_set_ap(iwr);

      case SIOCGIWESSID:
        return ioctl_get_essid(iwr);

      case SIOCGIWAP:
        return ioctl_get_ap(iwr);

      case SIOCGIWSENS:
        return ioctl_get_rssi(iwr);

      case SIOCGIWMODE:
        iwr->u.mode = IW_MODE_INFRA;
        return 0;

      case SIOCSIWMODE:
        return iwr->u.mode == IW_MODE_INFRA ? 0 : -ENOTSUP;

      default:
        return -ENOTTY;
    }
}

int nina_net_initialize(void)
{
  const struct net_wifi_mgmt_offload *api = z_net_offload_api;
  int ret;

  /* Zephyr's net_init(): the event thread, then the interfaces. */

  ret = net_mgmt_event_init();
  if (ret < 0)
    {
      return ret;
    }

  net_mgmt_init_event_callback(&g_scan_events, scan_event,
                               NET_EVENT_WIFI_SCAN_RESULT |
                               NET_EVENT_WIFI_SCAN_DONE);
  net_mgmt_add_event_callback(&g_scan_events);

  memset(&g_dev, 0, sizeof(g_dev));
  g_dev.d_ioctl = nina_ioctl;
  ret = netdev_register(&g_dev, NET_LL_IEEE80211);
  if (ret < 0)
    {
      return ret;
    }

  /* No link-layer frames: the module does TCP/IP (usrsock). */

  g_dev.d_llhdrlen = 0;
  g_dev.d_pktsize = NINA_NET_MTU;

  /* Zephyr brings an interface up administratively when it is
   * initialized; whether it is on a network is its dormant state.
   */

  netdev_lock(&g_dev);
  g_dev.d_flags |= IFF_UP;
  netdev_unlock(&g_dev);

  g_iface.api = api;
  z_net_if_register(&g_iface);

  ret = z_net_offload_init(NULL);
  if (ret < 0)
    {
      return ret;
    }

  api->wifi_iface.iface_api.init(&g_iface);
  return 0;
}


#ifndef CONFIG_NANO_NINA_DNS
/* iface_init() of nina_net.c calls it; without nina_dns.c there is no
 * resolver to register (nina_dns.c has an empty one too).
 */

void nina_dns_init(void)
{
}
#endif
