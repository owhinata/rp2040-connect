/* SPDX-License-Identifier: Apache-2.0 */
/* The Zephyr build's src/wifi_profile.h: the stored Wi-Fi profile and the
 * automatic reconnection, which the NINA network device
 * (nuttx/drivers/nina/nina_net.c) calls.  The functions are in
 * nuttx/apps/wifi/wifi_profile.c (CONFIG_NANO_WIFI_PROFILE); without it, the
 * kernel side links nina/wifi_profile_null.c (reconnection never requested,
 * every connection allowed).
 */
#ifndef WIFI_PROFILE_H_
#define WIFI_PROFILE_H_
#include <zephyr/net/wifi_mgmt.h>
int wifi_profile_init(void);
void wifi_profile_pause_reconnect(void);
void wifi_profile_request_reconnect(void);
bool wifi_profile_connection_allowed(void);
#endif
