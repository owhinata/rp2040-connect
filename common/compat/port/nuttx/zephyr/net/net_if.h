/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's <zephyr/net/net_if.h> (Zephyr v4.5.0) that the
 * applications (wifi_profile.c, time_sync.c, imu_raw_mqtt.c of common/imu)
 * and the layer's Wi-Fi requests (common/compat/zephyr_wifi_mgmt.c) use,
 * with Zephyr's names and meanings, on one NuttX network device:
 *
 *   struct net_if: the interface of a NuttX network device (struct
 *   net_driver_s: wlan0, whose binding,
 *   common/drivers/nina/port/nuttx/nina_netdev.c, sets its addresses and
 *   flags; nina_wifi_mgmt.c registers the interface).  Zephyr's one IPv4
 *   address and gateway of the interface are the device's d_ipaddr and
 *   d_draddr, which ifconfig shows; administratively up is IFF_UP.
 *
 * The functions take the device's lock (netdev_lock()), as ifconfig's
 * ioctls do when they read the addresses.  Not here: IPv6, more than one
 * address or interface, address lifetimes, the interface events of
 * net_mgmt (NET_EVENT_IF_UP/DOWN, NET_EVENT_IPV4_ADDR_ADD/DEL): nothing in
 * the port listens to them.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (common/compat/port/nuttx, see nuttx/drivers/README.md).
 */

#ifndef __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_NET_NET_IF_H
#define __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_NET_NET_IF_H

#include <netinet/in.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Zephyr's struct net_in_addr is NuttX's struct in_addr (s_addr in network
 * byte order).
 */

#define net_in_addr in_addr

struct net_driver_s;

struct net_if
{
  struct net_driver_s *dev;     /* the NuttX network device */
  const void *api;              /* struct net_wifi_mgmt_offload */
};

/* include/zephyr/net/net_if.h: the state of an address.  The port's
 * addresses are always preferred (no duplicate address detection on IPv4
 * in the Zephyr build either).
 */

enum net_addr_state
{
  NET_ADDR_ANY_STATE = -1,
  NET_ADDR_TENTATIVE = 0,
  NET_ADDR_PREFERRED,
  NET_ADDR_DEPRECATED,
};

/* Makes IFACE the interface net_if_get_default() and
 * net_if_get_wifi_sta() return (common/drivers/nina/port/nuttx/nina_wifi_mgmt.c).
 */

void z_net_if_register(struct net_if *iface);

/* The registered interface, or NULL before it is registered. */

struct net_if *net_if_get_default(void);
struct net_if *net_if_get_wifi_sta(void);

/* The interface's address (d_ipaddr) if it has one and STATE is
 * NET_ADDR_PREFERRED or NET_ADDR_ANY_STATE, else NULL (Zephyr: the first
 * global unicast address in that state; the driver's addresses are always
 * preferred).  The pointer stays valid; read the address under no lock,
 * as Zephyr's callers do.
 */

struct net_in_addr *net_if_ipv4_get_global_addr(struct net_if *iface,
                                                enum net_addr_state state);

/* d_draddr: the default gateway (0.0.0.0 when there is none). */

struct net_in_addr net_if_ipv4_get_gw(struct net_if *iface);

/* IFF_UP. */

bool net_if_is_admin_up(struct net_if *iface);

#endif /* __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_NET_NET_IF_H */
