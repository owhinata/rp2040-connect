/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's <zephyr/net/net_if.h> (Zephyr v4.5.0) that the NINA
 * network device (drivers/nina/nina_net.c) and the application
 * (wifi_profile.c, time_sync.c, imu_raw_mqtt.c of the Zephyr build) use,
 * with Zephyr's names and meanings, on one NuttX network device:
 *
 *   struct net_if: the interface of a NuttX network device (struct
 *   net_driver_s, registered as wlan0 by drivers/nina/nina_netdev.c).
 *   Zephyr's one IPv4 address, netmask and gateway of the interface are
 *   the device's d_ipaddr, d_netmask and d_draddr, which ifconfig shows.
 *   Dormant (Zephyr's "connected to the network or not") is IFF_RUNNING
 *   clear; administratively up is IFF_UP.
 *
 * The functions take the device's lock (netdev_lock()), as ifconfig's
 * ioctls do when they read the addresses.  Not here: IPv6, more than one
 * address or interface, address lifetimes, the interface events of
 * net_mgmt (NET_EVENT_IF_UP/DOWN, NET_EVENT_IPV4_ADDR_ADD/DEL): nothing in
 * the port listens to them.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H

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

/* include/zephyr/net/net_if.h: how an address was obtained, and its
 * state.  The port's addresses are always preferred (no duplicate address
 * detection on IPv4 in the Zephyr build either).
 */

enum net_addr_type
{
  NET_ADDR_ANY = 0,
  NET_ADDR_AUTOCONF,
  NET_ADDR_DHCP,
  NET_ADDR_MANUAL,
  NET_ADDR_OVERRIDABLE,
};

enum net_addr_state
{
  NET_ADDR_ANY_STATE = -1,
  NET_ADDR_TENTATIVE = 0,
  NET_ADDR_PREFERRED,
  NET_ADDR_DEPRECATED,
};

enum net_link_type
{
  NET_LINK_UNKNOWN = 0,
  NET_LINK_ETHERNET,
};

struct net_if_addr
{
  struct net_in_addr address;
  enum net_addr_type addr_type;
  enum net_addr_state addr_state;
};

/* Makes IFACE the interface net_if_get_default() and
 * net_if_get_wifi_sta() return (drivers/nina/nina_netdev.c).
 */

void z_net_if_register(struct net_if *iface);

/* The registered interface, or NULL before it is registered. */

struct net_if *net_if_get_default(void);
struct net_if *net_if_get_wifi_sta(void);

/* The address in d_ipaddr.  Adding replaces the one address the interface
 * has (CONFIG_NET_IF_UNICAST_IPV4_ADDR_COUNT is 1 in the Zephyr build's
 * use of it); removing clears it if it is ADDR.  NULL / false if ADDR is
 * 0.0.0.0 or not the interface's address.
 */

struct net_if_addr *net_if_ipv4_addr_add(struct net_if *iface,
                                         const struct net_in_addr *addr,
                                         enum net_addr_type addr_type,
                                         uint32_t vlifetime);
bool net_if_ipv4_addr_rm(struct net_if *iface,
                         const struct net_in_addr *addr);

/* The interface's address if it has one and STATE is NET_ADDR_PREFERRED
 * or NET_ADDR_ANY_STATE, else NULL (Zephyr: the first global unicast
 * address in that state).  The pointer stays valid; read the address under
 * no lock, as Zephyr's callers do.
 */

struct net_in_addr *net_if_ipv4_get_global_addr(struct net_if *iface,
                                                enum net_addr_state state);

/* d_netmask, when ADDR is the interface's address (true), else false. */

bool net_if_ipv4_set_netmask_by_addr(struct net_if *iface,
                                     const struct net_in_addr *addr,
                                     const struct net_in_addr *netmask);

/* d_draddr: the default gateway (0.0.0.0 when there is none). */

void net_if_ipv4_set_gw(struct net_if *iface, const struct net_in_addr *gw);
struct net_in_addr net_if_ipv4_get_gw(struct net_if *iface);

/* Dormant on: IFF_RUNNING cleared (netdev_carrier_off()); off: set
 * (netdev_carrier_on()).
 */

void net_if_dormant_on(struct net_if *iface);
void net_if_dormant_off(struct net_if *iface);
bool net_if_is_dormant(struct net_if *iface);

/* IFF_UP. */

bool net_if_is_admin_up(struct net_if *iface);

/* The device's MAC address (d_mac.ether).  Only NET_LINK_ETHERNET and 6
 * bytes: 0, else -EINVAL.
 */

int net_if_set_link_addr(struct net_if *iface, uint8_t *addr, uint8_t len,
                         enum net_link_type type);

/* Zephyr routes the sockets of an offloaded interface to the driver's
 * socket(); NuttX does it through usrsock (drivers/nina/nina_usrsock.c),
 * so there is nothing to set.
 */

#define net_if_socket_offload_set(iface, socket_create) ((void)(iface))

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H */
