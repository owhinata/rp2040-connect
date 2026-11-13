/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's <zephyr/net/net_if.h>.  Nothing of it is provided yet: the
 * interface functions (net_if_get_default(), net_if_ipv4_get_gw(), ...)
 * come with the NINA network device (T16 of the port plan).  The header
 * exists so that a ported source that includes it but uses none of it
 * (imu_stream.c) keeps its include list.  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H

#include <netinet/in.h>

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H */
