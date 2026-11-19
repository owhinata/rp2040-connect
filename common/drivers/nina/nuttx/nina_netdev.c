/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA-W102 as the NuttX network device wlan0, for ifconfig and wapi:
 * the NuttX binding of the core's Wi-Fi operations
 * (common/drivers/nina/nina_wifi.h).
 *
 * nina_net_initialize() (the board's bring-up) starts the applications'
 * Zephyr Wi-Fi management (nina_wifi_mgmt.c: net_mgmt's event thread and
 * the compatibility layer's interface), registers wlan0 (NET_LL_IEEE80211,
 * administratively up, no carrier) and initializes the core
 * (nina_wifi_init(): the MAC address, the module's capabilities), as Zephyr
 * does at boot; nina_net_start() (the init task) starts the core's three
 * threads.  The wireless ioctls of NuttX's wapi (SIOCxIW*, via netdev's
 * d_ioctl) call the core:
 *
 *   SIOCSIWSCAN      nina_wifi_scan() (a passive scan, an ESSID or
 *                    channels: -ENOTSUP, as the Zephyr build's driver
 *                    refuses them; -ENETDOWN while wlan0 is down, as
 *                    Zephyr's Wi-Fi management answers, for SIOCSIWESSID
 *                    and SIOCSIWAP too)
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
 *   SIOCSIWESSID     nina_wifi_connect() with the kept settings (WPA2 ->
 *                    PSK, none -> open; WPA, WPA3 and WEP are refused),
 *                    after the checks of Zephyr's Wi-Fi management that
 *                    those settings reach (wext_check()); flags 0 (wapi's
 *                    bogus SSID of "disconnect") does nothing
 *   SIOCSIWAP        00:00:00:00:00:00 or ff:..:ff: nina_wifi_disconnect()
 *   SIOCGIWESSID, SIOCGIWAP, SIOCGIWSENS (RSSI)
 *                    nina_wifi_status() (asks the module)
 *   SIOCGIWMODE      managed (IW_MODE_INFRA), SIOCSIWMODE managed only
 *
 * Other wireless ioctls return -ENOTTY.  The core's struct nina_wifi_ops
 * puts the connection on wlan0: its IPv4 address, netmask and gateway
 * (d_ipaddr, d_netmask, d_draddr, which netdev's own ioctls and
 * /proc/net/wlan0, ifconfig, read) and its carrier (IFF_RUNNING); the
 * results go to the applications as the compatibility layer's events
 * (nina_wifi_mgmt.c).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <nuttx/compiler.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/semaphore.h>
#include <nuttx/wireless/wireless.h>

#include "nina_cmd.h"
#include "nina_net.h"
#include "nina_wifi.h"
#include "nina_wifi_mgmt.h"
#include "priorities.h"

#ifndef MIN
#  define MIN(a, b)          ((a) < (b) ? (a) : (b))
#endif

/* The MTU of the Zephyr build's interface (NET_DEVICE_DT_OFFLOAD_DEFINE(...,
 * 1500), common/drivers/nina/zephyr/nina_net.c).
 */

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

/* The longest passphrase that SIOCSIWENCODEEXT keeps (Zephyr's
 * WIFI_PSK_MAX_LEN), and the channels of a scan request that are looked
 * at (Zephyr's CONFIG_WIFI_MGMT_SCAN_CHAN_MAX_MANUAL in the Zephyr build).
 */

#define WEXT_KEY_MAX         64
#define WEXT_SCAN_CHAN_MAX   3

/* The settings of SIOCSIWESSID: the securities of Zephyr's Wi-Fi
 * management that wapi's settings name (wext_security()).
 */

enum wext_security
{
  WEXT_SECURITY_NONE,
  WEXT_SECURITY_PSK,            /* WPA2 */
  WEXT_SECURITY_WPA_PSK,        /* WPA */
  WEXT_SECURITY_SAE,            /* WPA3 */
  WEXT_SECURITY_WEP,
};

static struct net_driver_s g_dev;

/* The interface's link address: what the module reports. */

static uint8_t g_mac[NINA_WIFI_MAC_LEN];

/* The last scan, filled by the core's scan callback. */

static mutex_t g_scan_lock = NXMUTEX_INITIALIZER;
static struct nina_wifi_scan_result g_scan_results[NINA_WIFI_SCAN_MAX];
static unsigned int g_scan_count;
static bool g_scan_busy;
static int g_scan_status;

/* What wapi set before SIOCSIWESSID (forgotten by it). */

static mutex_t g_wext_lock = NXMUTEX_INITIALIZER;
static uint32_t g_wpa_version;          /* IW_AUTH_WPA_VERSION_*, 0 unset */
static uint32_t g_cipher;               /* IW_AUTH_CIPHER_*, 0 unset */
static uint8_t g_key[WEXT_KEY_MAX];
static size_t g_key_len;

/* The core's threads (nina_net_start()). */

static uint8_t g_scan_stack[NANO_STACK_NINA_SCAN] aligned_data(8);
static uint8_t g_connect_stack[NANO_STACK_NINA_CONNECT] aligned_data(8);
static uint8_t g_monitor_stack[NANO_STACK_NINA_MONITOR] aligned_data(8);
static sem_t g_thread_started = SEM_INITIALIZER(0);

static void lock(FAR mutex_t *mutex)
{
  while (nxmutex_lock(mutex) == -EINTR)
    {
    }
}

/* The core's operations ------------------------------------------------ */

static int ip_set(FAR const uint8_t *addr, FAR const uint8_t *netmask,
                  FAR const uint8_t *gateway)
{
  in_addr_t value;

  netdev_lock(&g_dev);
  memcpy(&value, addr, sizeof(value));
  g_dev.d_ipaddr = value;
  memcpy(&value, netmask, sizeof(value));
  g_dev.d_netmask = value;
  memcpy(&value, gateway, sizeof(value));
  g_dev.d_draddr = value;
  netdev_unlock(&g_dev);
  return 0;
}

static void link_up(void)
{
  netdev_lock(&g_dev);
  netdev_carrier_on(&g_dev);
  netdev_unlock(&g_dev);
}

static void link_down(FAR const uint8_t *addr)
{
  in_addr_t value;

  memcpy(&value, addr, sizeof(value));
  netdev_lock(&g_dev);
  if (value != INADDR_ANY && g_dev.d_ipaddr == value)
    {
      g_dev.d_ipaddr = INADDR_ANY;
    }

  g_dev.d_draddr = INADDR_ANY;
  netdev_carrier_off(&g_dev);
  netdev_unlock(&g_dev);
}

static void disconnected(enum nina_wifi_disconnect_reason reason)
{
  (void)reason;
  nina_wifi_mgmt_disconnect_result();
}

static const struct nina_wifi_ops g_ops =
{
  .ip_set             = ip_set,
  .link_up            = link_up,
  .link_down          = link_down,
  .connect_result     = nina_wifi_mgmt_connect_result,
  .disconnected       = disconnected,
  .connection_allowed = nina_wifi_mgmt_connection_allowed,
  .reconnect_pause    = nina_wifi_mgmt_pause_reconnect,
  .reconnect_request  = nina_wifi_mgmt_request_reconnect,
};

/* Scan ------------------------------------------------------------------ */

/* In the core's scan thread. */

static void scan_result(FAR void *arg, int status,
                        FAR const struct nina_wifi_scan_result *result)
{
  (void)arg;
  lock(&g_scan_lock);
  if (result != NULL)
    {
      if (g_scan_busy && g_scan_count < NINA_WIFI_SCAN_MAX)
        {
          g_scan_results[g_scan_count++] = *result;
        }
    }
  else
    {
      g_scan_status = status;
      g_scan_busy = false;
    }

  nxmutex_unlock(&g_scan_lock);
}

static int ioctl_scan_start(FAR struct iwreq *iwr)
{
  struct iw_scan_req req;
  int ret = 0;

  if (iwr->u.data.pointer != NULL && iwr->u.data.length >= sizeof(req))
    {
      memcpy(&req, iwr->u.data.pointer, sizeof(req));
      if (req.scan_type == IW_SCAN_TYPE_PASSIVE ||
          ((iwr->u.data.flags & IW_SCAN_THIS_ESSID) != 0 &&
           req.essid_len > 0))
        {
          ret = -ENOTSUP;
        }

      for (size_t i = 0; i < MIN(req.num_channels, WEXT_SCAN_CHAN_MAX);
           i++)
        {
          if ((uint8_t)req.channel_list[i].m != 0)
            {
              ret = -ENOTSUP;
            }
        }
    }

  lock(&g_scan_lock);
  if (g_scan_busy)
    {
      nxmutex_unlock(&g_scan_lock);
      return -EBUSY;
    }

  g_scan_busy = true;
  g_scan_count = 0;
  g_scan_status = 0;
  nxmutex_unlock(&g_scan_lock);

  /* Zephyr's Wi-Fi management refuses a request on an interface that is
   * down (ifconfig wlan0 down) before the driver sees it.
   */

  if (!IFF_IS_UP(g_dev.d_flags))
    {
      ret = -ENETDOWN;
    }
  else if (ret == 0)
    {
      ret = nina_wifi_scan(scan_result, NULL);
    }

  if (ret != 0)
    {
      lock(&g_scan_lock);
      g_scan_busy = false;
      g_scan_status = ret;
      nxmutex_unlock(&g_scan_lock);
    }

  return ret;
}

/* Whether the module's encryption type is one of the four it names (the
 * others are an unknown security, which wapi shows as ffff).
 */

static bool enc_known(uint8_t type)
{
  return type == NINA_ENC_TYPE_NONE || type == NINA_ENC_TYPE_WEP ||
         type == NINA_ENC_TYPE_TKIP || type == NINA_ENC_TYPE_CCMP;
}

/* The bytes of one scan result as events (SIOCGIWENCODE only when the
 * security is known).
 */

static size_t result_len(FAR const struct nina_wifi_scan_result *r)
{
  return IWE_RESULT_LEN - (enc_known(r->enc_type) ? 0 : IWE_ENCODE_LEN);
}

/* Writes one scan result at DST (result_len() bytes, any alignment). */

static void put_result(FAR uint8_t *dst,
                       FAR const struct nina_wifi_scan_result *r)
{
  struct iw_event iwe;
  uint8_t essid[IW_ESSID_MAX_SIZE];

  memset(&iwe, 0, sizeof(iwe));
  iwe.len = IWE_AP_LEN;
  iwe.cmd = SIOCGIWAP;
  iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
  memcpy(iwe.u.ap_addr.sa_data, r->bssid, NINA_WIFI_MAC_LEN);
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
  iwe.u.essid.length = r->ssid_len;
  iwe.u.essid.flags = 1;
  memcpy(dst, &iwe, IWE_HDR + sizeof(struct iw_point));
  memset(essid, 0, sizeof(essid));
  memcpy(essid, r->ssid, MIN(r->ssid_len, IW_ESSID_MAX_SIZE));
  memcpy(dst + IWE_HDR + sizeof(struct iw_point), essid, sizeof(essid));
  dst += IWE_ESSID_LEN;

  if (enc_known(r->enc_type))
    {
      memset(&iwe, 0, sizeof(iwe));
      iwe.len = IWE_ENCODE_LEN;
      iwe.cmd = SIOCGIWENCODE;
      iwe.u.data.flags = r->enc_type == NINA_ENC_TYPE_NONE ?
                         IW_ENCODE_DISABLED :
                         IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
      memcpy(dst, &iwe, IWE_ENCODE_LEN);
    }
}

static int ioctl_scan_results(FAR struct iwreq *iwr)
{
  size_t need;
  FAR uint8_t *dst;
  int ret = 0;

  lock(&g_scan_lock);
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

  nxmutex_unlock(&g_scan_lock);
  return ret;
}

/* Connect and disconnect ------------------------------------------------ */

static enum wext_security wext_security(void)
{
  if (g_wpa_version == 0)
    {
      return g_key_len ? WEXT_SECURITY_PSK : WEXT_SECURITY_NONE;
    }

  if ((g_wpa_version & IW_AUTH_WPA_VERSION_DISABLED) != 0)
    {
      return (g_cipher & (IW_AUTH_CIPHER_WEP40 | IW_AUTH_CIPHER_WEP104)) ?
             WEXT_SECURITY_WEP : WEXT_SECURITY_NONE;
    }

  if ((g_wpa_version & IW_AUTH_WPA_VERSION_WPA3) != 0)
    {
      return WEXT_SECURITY_SAE;
    }

  if ((g_wpa_version & IW_AUTH_WPA_VERSION_WPA2) != 0)
    {
      return WEXT_SECURITY_PSK;
    }

  return WEXT_SECURITY_WPA_PSK;
}

/* The checks of NET_REQUEST_WIFI_CONNECT in Zephyr's Wi-Fi management
 * (net_mgmt_NET_REQUEST_WIFI_CONNECT() of common/compat/zephyr_wifi_mgmt.c)
 * that the settings of wapi reach (any channel, an SSID of 1 to 32 bytes,
 * no SAE password): the key's length for WEP or for the others, and a key
 * for the securities that need one.  The core then refuses all but open
 * networks and WPA2-PSK.
 */

static int wext_check(enum wext_security security, size_t key_len)
{
  if (security == WEXT_SECURITY_WEP)
    {
      if (key_len && key_len != 5 && key_len != 13 && key_len != 10 &&
          key_len != 26)
        {
          return -EINVAL;
        }
    }
  else if (key_len && (key_len < 8 || key_len > 64))
    {
      return -EINVAL;
    }

  if (security != WEXT_SECURITY_NONE && key_len == 0)
    {
      return -EINVAL;
    }

  return 0;
}

static int ioctl_connect(FAR struct iwreq *iwr)
{
  struct nina_wifi_connect_params params;
  enum wext_security security;
  uint8_t ssid[NINA_WIFI_SSID_MAX_LEN];
  size_t len = iwr->u.essid.length;
  int ret;

  /* wapi's "disconnect" sets a random SSID with the flag off after
   * clearing the BSSID (SIOCSIWAP); there is nothing to do for it.
   */

  if (iwr->u.essid.flags == 0)
    {
      return 0;
    }

  if (iwr->u.essid.pointer == NULL || len == 0 ||
      len > NINA_WIFI_SSID_MAX_LEN)
    {
      return -EINVAL;
    }

  memcpy(ssid, iwr->u.essid.pointer, len);

  lock(&g_wext_lock);
  security = wext_security();
  ret = IFF_IS_UP(g_dev.d_flags) ? wext_check(security, g_key_len) :
        -ENETDOWN;
  if (ret == 0)
    {
      memset(&params, 0, sizeof(params));
      params.ssid = ssid;
      params.ssid_len = len;
      params.security = security == WEXT_SECURITY_NONE ?
                        NINA_WIFI_SECURITY_NONE :
                        security == WEXT_SECURITY_PSK ?
                        NINA_WIFI_SECURITY_PSK : NINA_WIFI_SECURITY_OTHER;
      if (g_key_len > 0)
        {
          params.psk = g_key;
          params.psk_len = g_key_len;
        }

      ret = nina_wifi_connect(&params);
    }

  /* The settings are used once: the passphrase is wiped, and the next
   * connection without "wapi psk" is an open one.
   */

  memset(g_key, 0, sizeof(g_key));
  g_key_len = 0;
  g_wpa_version = 0;
  g_cipher = 0;
  nxmutex_unlock(&g_wext_lock);
  return ret;
}

static int ioctl_set_auth(FAR struct iwreq *iwr)
{
  int ret = 0;

  lock(&g_wext_lock);
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

  nxmutex_unlock(&g_wext_lock);
  return ret;
}

static int ioctl_set_key(FAR struct iwreq *iwr)
{
  struct iw_encode_ext ext;
  FAR const uint8_t *src = iwr->u.encoding.pointer;

  if (src == NULL || iwr->u.encoding.length < sizeof(ext))
    {
      return -EINVAL;
    }

  memcpy(&ext, src, sizeof(ext));
  if (ext.key_len > WEXT_KEY_MAX ||
      iwr->u.encoding.length < sizeof(ext) + ext.key_len)
    {
      return -EINVAL;
    }

  lock(&g_wext_lock);
  memset(g_key, 0, sizeof(g_key));
  g_key_len = 0;
  if (ext.alg != IW_ENCODE_ALG_NONE)
    {
      memcpy(g_key, src + sizeof(ext), ext.key_len);
      g_key_len = ext.key_len;
    }

  nxmutex_unlock(&g_wext_lock);
  return 0;
}

static int ioctl_set_ap(FAR struct iwreq *iwr)
{
  static const uint8_t zero[NINA_WIFI_MAC_LEN];
  static const uint8_t all[NINA_WIFI_MAC_LEN] =
    {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

  FAR const uint8_t *bssid = (FAR const uint8_t *)iwr->u.ap_addr.sa_data;

  if (memcmp(bssid, zero, sizeof(zero)) != 0 &&
      memcmp(bssid, all, sizeof(all)) != 0)
    {
      return -ENOTSUP;
    }

  if (!IFF_IS_UP(g_dev.d_flags))
    {
      return -ENETDOWN;
    }

  return nina_wifi_disconnect();
}

/* Status --------------------------------------------------------------- */

static int ioctl_get_essid(FAR struct iwreq *iwr)
{
  struct nina_wifi_status status;
  int ret;

  if (iwr->u.essid.pointer == NULL)
    {
      return -EINVAL;
    }

  ret = nina_wifi_status(&status);
  if (ret != 0)
    {
      return ret;
    }

  if (status.connected)
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

static int ioctl_get_ap(FAR struct iwreq *iwr)
{
  struct nina_wifi_status status;
  int ret;

  ret = nina_wifi_status(&status);
  if (ret != 0)
    {
      return ret;
    }

  memset(&iwr->u.ap_addr, 0, sizeof(iwr->u.ap_addr));
  iwr->u.ap_addr.sa_family = ARPHRD_ETHER;
  if (status.connected)
    {
      memcpy(iwr->u.ap_addr.sa_data, status.bssid, NINA_WIFI_MAC_LEN);
    }

  return 0;
}

static int ioctl_get_rssi(FAR struct iwreq *iwr)
{
  struct nina_wifi_status status;
  int ret;

  ret = nina_wifi_status(&status);
  if (ret != 0)
    {
      return ret;
    }

  if (!status.connected)
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
  FAR struct iwreq *iwr = (FAR struct iwreq *)(uintptr_t)arg;

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
  int ret;

  ret = nina_wifi_mgmt_initialize(&g_dev);
  if (ret < 0)
    {
      return ret;
    }

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
   * initialized; whether it is on a network is its carrier.
   */

  netdev_lock(&g_dev);
  g_dev.d_flags |= IFF_UP;
  netdev_carrier_off(&g_dev);
  netdev_unlock(&g_dev);

  if (nina_wifi_init(&g_ops, g_mac) == 0)
    {
      netdev_lock(&g_dev);
      memcpy(g_dev.d_mac.ether.ether_addr_octet, g_mac, sizeof(g_mac));
      netdev_unlock(&g_dev);
    }

  return 0;
}

/* The threads ----------------------------------------------------------- */

static int scan_main(int argc, FAR char *argv[])
{
  (void)argc;
  (void)argv;
  nxsem_post(&g_thread_started);
  nina_wifi_scan_thread();
  return 0;
}

static int connect_main(int argc, FAR char *argv[])
{
  (void)argc;
  (void)argv;
  nxsem_post(&g_thread_started);
  nina_wifi_connect_thread();
  return 0;
}

static int monitor_main(int argc, FAR char *argv[])
{
  (void)argc;
  (void)argv;
  nxsem_post(&g_thread_started);
  nina_wifi_monitor_thread();
  return 0;
}

/* A kernel thread on its static stack; returns once it runs, as Zephyr's
 * static threads are all running when main() starts.
 */

static int start_thread(FAR const char *name, int priority,
                        FAR uint8_t *stack, int stack_size, main_t entry)
{
  int pid;

  pid = kthread_create_with_stack(name, priority, stack, stack_size,
                                  entry, NULL);
  if (pid < 0)
    {
      return pid;
    }

  nxsem_wait_uninterruptible(&g_thread_started);
  return 0;
}

int nina_net_start(void)
{
  int ret;

  ret = start_thread("nina_scan_tid", NANO_PRIO_NINA_SCAN, g_scan_stack,
                     sizeof(g_scan_stack), scan_main);
  if (ret == 0)
    {
      ret = start_thread("nina_connect_tid", NANO_PRIO_NINA_CONNECT,
                         g_connect_stack, sizeof(g_connect_stack),
                         connect_main);
    }

  if (ret == 0)
    {
      ret = start_thread("nina_monitor_tid", NANO_PRIO_NINA_MONITOR,
                         g_monitor_stack, sizeof(g_monitor_stack),
                         monitor_main);
    }

  return ret;
}
