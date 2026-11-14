/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's <zephyr/net/wifi_utils.h> (Zephyr v4.5.0) that the
 * "wifi cred add" command (nuttx/apps/wifi) uses: the channel checks of
 * subsys/net/l2/wifi/wifi_utils.c, and net_bytes_from_str() of
 * subsys/net/ip/utils.c (declared in Zephyr's <zephyr/net/net_ip.h>), with
 * Zephyr's code, in zephyr_wifi_utils.c.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_WIFI_UTILS_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_WIFI_UTILS_H

#include <stdbool.h>
#include <stdint.h>

/* True if CHAN is a channel of BAND (enum wifi_frequency_bands: 2.4, 5 or
 * 6 GHz).
 */

bool wifi_utils_validate_chan(uint8_t band, uint16_t chan);
bool wifi_utils_validate_chan_2g(uint16_t chan);
bool wifi_utils_validate_chan_5g(uint16_t chan);
bool wifi_utils_validate_chan_6g(uint16_t chan);

/* Hexadecimal bytes separated by ':' ("aa:bb:...") to BUF; -EINVAL if SRC
 * has another character.
 */

int net_bytes_from_str(uint8_t *buf, int buf_len, const char *src);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_WIFI_UTILS_H */
