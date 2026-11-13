/* SPDX-License-Identifier: Apache-2.0 */
/* The calls of the NINA network device into the Wi-Fi profile
 * (<wifi_profile.h>, the Zephyr build's src/wifi_profile.c) while the port
 * has no stored profile and no automatic reconnection
 * (CONFIG_NANO_WIFI_PROFILE off): the Zephyr build with no stored
 * credentials behaves the same way, as its reconnection is never enabled
 * (auto_enabled stays false) and only its own reconnection thread is ever
 * refused a connection.
 */

#include <nuttx/config.h>

#include <stdbool.h>

#include "wifi_profile.h"

void wifi_profile_pause_reconnect(void)
{
}

void wifi_profile_request_reconnect(void)
{
}

bool wifi_profile_connection_allowed(void)
{
  return true;
}
