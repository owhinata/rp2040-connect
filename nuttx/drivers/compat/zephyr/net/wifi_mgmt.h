/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's <zephyr/net/wifi.h>, <zephyr/net/wifi_mgmt.h> and
 * <zephyr/net/offloaded_netdev.h> (Zephyr v4.5.0) that the NINA network
 * device (drivers/nina/nina_net.c) and the application use, with Zephyr's
 * names, types and values:
 *
 *   the Wi-Fi types (security, band, MFP, state, mode), struct
 *   wifi_scan_params, wifi_scan_result, wifi_connect_req_params,
 *   wifi_iface_status, wifi_status; struct wifi_mgmt_ops (scan, connect,
 *   disconnect, iface_status) and the offloaded interface API;
 *   NET_DEVICE_DT_OFFLOAD_DEFINE();
 *   the requests NET_REQUEST_WIFI_SCAN, _CONNECT, _DISCONNECT,
 *   _IFACE_STATUS and _CONNECT_STORED (net_mgmt(), handlers in
 *   zephyr_wifi_mgmt.c with the checks of Zephyr's
 *   subsys/net/l2/wifi/wifi_mgmt.c);
 *   wifi_security_txt(), wifi_band_txt() (zephyr_wifi_utils.c);
 *   the events NET_EVENT_WIFI_SCAN_RESULT, _SCAN_DONE, _CONNECT_RESULT and
 *   _DISCONNECT_RESULT and wifi_mgmt_raise_connect_result_event(),
 *   wifi_mgmt_raise_disconnect_result_event().
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_WIFI_MGMT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_WIFI_MGMT_H

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

/* <zephyr/net/wifi.h> ----------------------------------------------------*/

#define WIFI_SSID_MAX_LEN  32
#define WIFI_PSK_MIN_LEN   8
#define WIFI_PSK_MAX_LEN   64
#define WIFI_SAE_PSWD_MAX_LEN 128
#define WIFI_MAC_ADDR_LEN  6
#define WIFI_ENT_IDENTITY_MAX_LEN 64
#define WIFI_ENT_PSWD_MAX_LEN 128
#define WIFI_CHANNEL_MIN   1
#define WIFI_CHANNEL_MAX   233
#define WIFI_CHANNEL_ANY   255

/* The Zephyr build's CONFIG_WIFI_MGMT_SCAN_SSID_FILT_MAX and
 * CONFIG_WIFI_MGMT_SCAN_CHAN_MAX_MANUAL.
 */

#define WIFI_MGMT_SCAN_SSID_FILT_MAX     1
#define WIFI_MGMT_SCAN_CHAN_MAX_MANUAL   3

enum wifi_security_type
{
  WIFI_SECURITY_TYPE_NONE = 0,
  WIFI_SECURITY_TYPE_PSK,
  WIFI_SECURITY_TYPE_PSK_SHA256,
  WIFI_SECURITY_TYPE_SAE,
  WIFI_SECURITY_TYPE_SAE_HNP = WIFI_SECURITY_TYPE_SAE,
  WIFI_SECURITY_TYPE_SAE_H2E,
  WIFI_SECURITY_TYPE_SAE_AUTO,
  WIFI_SECURITY_TYPE_WAPI,
  WIFI_SECURITY_TYPE_EAP,
  WIFI_SECURITY_TYPE_EAP_TLS = WIFI_SECURITY_TYPE_EAP,
  WIFI_SECURITY_TYPE_WEP,
  WIFI_SECURITY_TYPE_WPA_PSK,
  WIFI_SECURITY_TYPE_WPA_AUTO_PERSONAL,
  WIFI_SECURITY_TYPE_DPP,
  WIFI_SECURITY_TYPE_EAP_PEAP_MSCHAPV2,
  WIFI_SECURITY_TYPE_EAP_PEAP_GTC,
  WIFI_SECURITY_TYPE_EAP_TTLS_MSCHAPV2,
  WIFI_SECURITY_TYPE_EAP_PEAP_TLS,
  WIFI_SECURITY_TYPE_FT_PSK,
  WIFI_SECURITY_TYPE_FT_SAE,
  WIFI_SECURITY_TYPE_FT_EAP,
  WIFI_SECURITY_TYPE_FT_EAP_SHA384,
  WIFI_SECURITY_TYPE_SAE_EXT_KEY,
  WIFI_SECURITY_TYPE_WEP_OPEN,
  WIFI_SECURITY_TYPE_WEP_SHARED,
  WIFI_SECURITY_TYPE_OWE,
  WIFI_SECURITY_TYPE_PSK_SHA384,
  WIFI_SECURITY_TYPE_FT_PSK_SHA384,
  __WIFI_SECURITY_TYPE_AFTER_LAST,
  WIFI_SECURITY_TYPE_MAX = WIFI_SECURITY_TYPE_FT_PSK_SHA384,
  WIFI_SECURITY_TYPE_UNKNOWN = __WIFI_SECURITY_TYPE_AFTER_LAST
};

enum wifi_mfp_options
{
  WIFI_MFP_DISABLE = 0,
  WIFI_MFP_OPTIONAL,
  WIFI_MFP_REQUIRED,
  __WIFI_MFP_AFTER_LAST,
  WIFI_MFP_MAX = __WIFI_MFP_AFTER_LAST - 1,
  WIFI_MFP_UNKNOWN
};

enum wifi_frequency_bands
{
  WIFI_FREQ_BAND_2_4_GHZ = 0,
  WIFI_FREQ_BAND_5_GHZ,
  WIFI_FREQ_BAND_6_GHZ,
  WIFI_FREQ_BAND_SUB_1_GHZ,
  __WIFI_FREQ_BAND_AFTER_LAST,
  WIFI_FREQ_BAND_MAX = __WIFI_FREQ_BAND_AFTER_LAST - 1,
  WIFI_FREQ_BAND_UNKNOWN
};

enum wifi_iface_state
{
  WIFI_STATE_DISCONNECTED = 0,
  WIFI_STATE_INTERFACE_DISABLED,
  WIFI_STATE_INACTIVE,
  WIFI_STATE_SCANNING,
  WIFI_STATE_AUTHENTICATING,
  WIFI_STATE_ASSOCIATING,
  WIFI_STATE_ASSOCIATED,
  WIFI_STATE_4WAY_HANDSHAKE,
  WIFI_STATE_GROUP_HANDSHAKE,
  WIFI_STATE_COMPLETED,
  __WIFI_STATE_AFTER_LAST,
  WIFI_STATE_MAX = __WIFI_STATE_AFTER_LAST - 1,
  WIFI_STATE_UNKNOWN
};

enum wifi_iface_mode
{
  WIFI_MODE_INFRA = 0,
  WIFI_MODE_IBSS = 1,
  WIFI_MODE_AP = 2,
  WIFI_MODE_P2P_GO = 3,
  WIFI_MODE_P2P_GROUP_FORMATION = 4,
  WIFI_MODE_MESH = 5,
  __WIFI_MODE_AFTER_LAST,
  WIFI_MODE_MAX = __WIFI_MODE_AFTER_LAST - 1,
  WIFI_MODE_UNKNOWN
};

enum wifi_link_mode
{
  WIFI_0 = 0,
  WIFI_1,
  WIFI_2,
  WIFI_3,
  WIFI_4,
  WIFI_5,
  WIFI_6,
  WIFI_6E,
  WIFI_7,
  __WIFI_LINK_MODE_AFTER_LAST,
  WIFI_LINK_MODE_MAX = __WIFI_LINK_MODE_AFTER_LAST - 1,
  WIFI_LINK_MODE_UNKNOWN
};

enum wifi_scan_type
{
  WIFI_SCAN_TYPE_ACTIVE = 0,
  WIFI_SCAN_TYPE_PASSIVE,
};

/* User-friendly names of a security type and a band (Zephyr's
 * subsys/net/l2/wifi/wifi_mgmt.c; here in zephyr_wifi_utils.c).
 */

const char *wifi_security_txt(enum wifi_security_type security);
const char *wifi_band_txt(enum wifi_frequency_bands band);

/* <zephyr/net/wifi_mgmt.h> -----------------------------------------------*/

struct wifi_band_channel
{
  uint8_t band;
  uint8_t channel;
};

/* Only the members the NINA driver checks or the port fills. */

struct wifi_scan_params
{
  enum wifi_scan_type scan_type;
  uint8_t bands;
  uint16_t dwell_time_active;
  uint16_t dwell_time_passive;
  const char *ssids[WIFI_MGMT_SCAN_SSID_FILT_MAX];
  uint16_t max_bss_cnt;
  struct wifi_band_channel band_chan[WIFI_MGMT_SCAN_CHAN_MAX_MANUAL];
};

struct wifi_scan_result
{
  uint8_t ssid[WIFI_SSID_MAX_LEN + 1];
  uint8_t ssid_length;
  uint8_t band;
  uint8_t channel;
  enum wifi_security_type security;
  enum wifi_mfp_options mfp;
  int8_t rssi;
  uint8_t mac[WIFI_MAC_ADDR_LEN];
  uint8_t mac_length;
};

struct wifi_connect_req_params
{
  const uint8_t *ssid;
  uint8_t ssid_length;          /* Max 32 */
  const uint8_t *psk;
  uint8_t psk_length;           /* Min 8 - Max 64 */
  bool psk_is_pbkdf2;
  const uint8_t *sae_password;
  uint8_t sae_password_length;
  uint8_t band;
  uint8_t channel;
  enum wifi_security_type security;
  enum wifi_mfp_options mfp;
  uint8_t bssid[WIFI_MAC_ADDR_LEN];
  int timeout;
};

struct wifi_status
{
  int status;
  uint16_t status_code;
  uint16_t reason_code;
};

struct wifi_iface_status
{
  int state;
  unsigned int ssid_len;
  char ssid[WIFI_SSID_MAX_LEN + 1];
  char bssid[WIFI_MAC_ADDR_LEN];
  enum wifi_frequency_bands band;
  unsigned int channel;
  enum wifi_iface_mode iface_mode;
  enum wifi_link_mode link_mode;
  enum wifi_security_type security;
  enum wifi_mfp_options mfp;
  int rssi;
  unsigned char dtim_period;
  unsigned short beacon_interval;
  bool twt_capable;
  float current_phy_tx_rate;
};

typedef void (*scan_result_cb_t)(struct net_if *iface, int status,
                                 struct wifi_scan_result *entry);

struct wifi_mgmt_ops
{
  int (*scan)(const struct device *dev, struct net_if *iface,
              struct wifi_scan_params *params, scan_result_cb_t cb);
  int (*connect)(const struct device *dev, struct net_if *iface,
                 struct wifi_connect_req_params *params);
  int (*disconnect)(const struct device *dev, struct net_if *iface);
  int (*iface_status)(const struct device *dev, struct net_if *iface,
                      struct wifi_iface_status *status);
};

/* <zephyr/net/offloaded_netdev.h> */

enum offloaded_net_if_types
{
  L2_OFFLOADED_NET_IF_TYPE_UNKNOWN,
  L2_OFFLOADED_NET_IF_TYPE_ETHERNET,
  L2_OFFLOADED_NET_IF_TYPE_MODEM,
  L2_OFFLOADED_NET_IF_TYPE_WIFI,
};

struct net_if_api
{
  void (*init)(struct net_if *iface);
};

struct offloaded_if_api
{
  struct net_if_api iface_api;
  enum offloaded_net_if_types (*get_type)(void);
};

struct net_wifi_mgmt_offload
{
  struct offloaded_if_api wifi_iface;
  const struct wifi_mgmt_ops *const wifi_mgmt_api;
};

/* NET_DEVICE_DT_OFFLOAD_DEFINE(node_id, init_fn, pm, data, config, prio,
 * api, mtu): Zephyr defines the device and its interface and calls
 * init_fn and then api's iface_api.init at boot.  Here it names the two
 * for the NuttX network device (drivers/nina/nina_netdev.c), which calls
 * them in the same order when it registers wlan0.  The other arguments
 * are not used.
 */

#define NET_DEVICE_DT_OFFLOAD_DEFINE(node_id, init_fn, pm, data, config, \
                                     prio, api, mtu) \
  int (*const z_net_offload_init)(const struct device *dev) = (init_fn); \
  const struct net_wifi_mgmt_offload *const z_net_offload_api = (api)

extern int (*const z_net_offload_init)(const struct device *dev);
extern const struct net_wifi_mgmt_offload *const z_net_offload_api;

/* Requests (net_mgmt(), <zephyr/net/net_mgmt.h>).  The numbers only tell
 * the requests apart.
 */

#define NET_REQUEST_WIFI_SCAN          1
#define NET_REQUEST_WIFI_CONNECT       2
#define NET_REQUEST_WIFI_DISCONNECT    3
#define NET_REQUEST_WIFI_IFACE_STATUS  4

NET_MGMT_DEFINE_REQUEST_HANDLER(NET_REQUEST_WIFI_SCAN);
NET_MGMT_DEFINE_REQUEST_HANDLER(NET_REQUEST_WIFI_CONNECT);
NET_MGMT_DEFINE_REQUEST_HANDLER(NET_REQUEST_WIFI_DISCONNECT);
NET_MGMT_DEFINE_REQUEST_HANDLER(NET_REQUEST_WIFI_IFACE_STATUS);

/* NET_REQUEST_WIFI_CONNECT_STORED (CONFIG_WIFI_CREDENTIALS_CONNECT_STORED):
 * a connection request (NET_REQUEST_WIFI_CONNECT) with each stored network
 * (<zephyr/net/wifi_credentials.h>) in turn until one is accepted; the
 * handler is Zephyr's connect_stored_command(), in zephyr_wifi_mgmt.c, with
 * the stored credentials of nuttx/apps/wifi (CONFIG_NANO_WIFI_CREDENTIALS).
 */

#ifdef CONFIG_NANO_WIFI_CREDENTIALS
#  define NET_REQUEST_WIFI_CONNECT_STORED 5
NET_MGMT_DEFINE_REQUEST_HANDLER(NET_REQUEST_WIFI_CONNECT_STORED);
#endif

/* Events (bits of this port, see <zephyr/net/net_mgmt.h>).  The
 * information is a struct wifi_scan_result (SCAN_RESULT) or a struct
 * wifi_status.
 */

#define NET_EVENT_WIFI_SCAN_RESULT         (1ULL << 0)
#define NET_EVENT_WIFI_SCAN_DONE           (1ULL << 1)
#define NET_EVENT_WIFI_CONNECT_RESULT      (1ULL << 2)
#define NET_EVENT_WIFI_DISCONNECT_RESULT   (1ULL << 3)

void wifi_mgmt_raise_connect_result_event(struct net_if *iface, int status);
void wifi_mgmt_raise_disconnect_result_event(struct net_if *iface,
                                             int status);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_WIFI_MGMT_H */
