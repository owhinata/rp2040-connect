/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/net/net_if.h> (Zephyr v4.5.0) that the NINA
 * network interface (common/drivers/nina/port/zephyr/nina_net.c), the Wi-Fi profile
 * (common/imu/wifi_profile.c), the "wifi" command and the IMU service use,
 * with Zephyr's names and meanings, on the one interface of the FreeRTOS
 * build (zephyr_net_if.c), the NINA-W102's, which does TCP/IP and DHCP
 * itself:
 *
 *   struct net_if: the interface; its state (one IPv4 address with its
 *   netmask, the default gateway, the link address, the MTU, dormant and
 *   administratively up) is kept by zephyr_net_if.c, as the NuttX build
 *   keeps it in its network device (common/compat/port/nuttx/zephyr/net/
 *   net_if.h has the same functions).  Dormant is Zephyr's "connected to
 *   the network or not".
 *
 * Not here: IPv6, more than one address or interface, address lifetimes,
 * the interface events of net_mgmt (NET_EVENT_IF_UP/DOWN,
 * NET_EVENT_IPV4_ADDR_ADD/DEL): nothing in the port listens to them.  Any
 * task may call the functions (not interrupt handlers).
 * Part of the network part of the Zephyr compatibility layer for an OS
 * without a network stack of its own (common/compat/net_offload), which the
 * FreeRTOS build uses.
 */

#ifndef COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_NET_IF_H
#define COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_NET_IF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* struct net_in_addr: <netinet/in.h>'s struct in_addr (s_addr in network
 * byte order).
 */
#include <zephyr/net/net_ip.h>

struct net_if {
	const void *api; /* struct net_wifi_mgmt_offload */
};

/* include/zephyr/net/net_if.h: how an address was obtained, and its
 * state.  The port's addresses are always preferred (no duplicate address
 * detection on IPv4 in the Zephyr build either).
 */
enum net_addr_type {
	NET_ADDR_ANY = 0,
	NET_ADDR_AUTOCONF,
	NET_ADDR_DHCP,
	NET_ADDR_MANUAL,
	NET_ADDR_OVERRIDABLE,
};

enum net_addr_state {
	NET_ADDR_ANY_STATE = -1,
	NET_ADDR_TENTATIVE = 0,
	NET_ADDR_PREFERRED,
	NET_ADDR_DEPRECATED,
};

enum net_link_type {
	NET_LINK_UNKNOWN = 0,
	NET_LINK_ETHERNET,
};

struct net_if_addr {
	struct net_in_addr address;
	enum net_addr_type addr_type;
	enum net_addr_state addr_state;
};

/* include/zephyr/net/net_linkaddr.h: a link address (the MAC address of an
 * Ethernet or Wi-Fi interface, 6 bytes).
 */
#define NET_LINK_ADDR_MAX_LENGTH 6

struct net_linkaddr {
	uint8_t type; /* enum net_link_type */
	uint8_t len;
	uint8_t addr[NET_LINK_ADDR_MAX_LENGTH];
};

/* Makes IFACE the interface that net_if_get_default() and
 * net_if_get_wifi_sta() return, administratively up (Zephyr brings an
 * interface up when it is initialized), dormant, with no address
 * (common/compat/net_offload/zephyr_net_init.c).
 */
void z_net_if_register(struct net_if *iface);

/* The registered interface, or NULL before it is registered. */
struct net_if *net_if_get_default(void);
struct net_if *net_if_get_wifi_sta(void);

/* The one IPv4 address.  Adding replaces the address the interface has
 * (CONFIG_NET_IF_UNICAST_IPV4_ADDR_COUNT is 1 in the Zephyr build's use of
 * it); removing clears it if it is ADDR.  NULL / false if ADDR is 0.0.0.0
 * or not the interface's address.
 */
struct net_if_addr *net_if_ipv4_addr_add(struct net_if *iface, const struct net_in_addr *addr,
					 enum net_addr_type addr_type, uint32_t vlifetime);
bool net_if_ipv4_addr_rm(struct net_if *iface, const struct net_in_addr *addr);

/* The interface's address if it has one and STATE is NET_ADDR_PREFERRED or
 * NET_ADDR_ANY_STATE, else NULL (Zephyr: the first global unicast address
 * in that state).  The pointer stays valid; the address is read under no
 * lock, as Zephyr's callers do.
 */
struct net_in_addr *net_if_ipv4_get_global_addr(struct net_if *iface, enum net_addr_state state);

/* The netmask, when ADDR is the interface's address (true), else false. */
bool net_if_ipv4_set_netmask_by_addr(struct net_if *iface, const struct net_in_addr *addr,
				     const struct net_in_addr *netmask);

/* The netmask of ADDR, when it is the interface's address, else 0.0.0.0
 * (the "ifconfig" command, common/shell/ifconfig_shell.c).
 */
struct net_in_addr net_if_ipv4_get_netmask_by_addr(struct net_if *iface,
						   const struct net_in_addr *addr);

/* The default gateway (0.0.0.0 when there is none). */
void net_if_ipv4_set_gw(struct net_if *iface, const struct net_in_addr *gw);
struct net_in_addr net_if_ipv4_get_gw(struct net_if *iface);

/* Dormant: not on a network. */
void net_if_dormant_on(struct net_if *iface);
void net_if_dormant_off(struct net_if *iface);
bool net_if_is_dormant(struct net_if *iface);

/* Administratively up (from z_net_if_register()). */
bool net_if_is_admin_up(struct net_if *iface);

/* The link (MAC) address.  Only NET_LINK_ETHERNET and 6 bytes: 0, else
 * -EINVAL.
 */
int net_if_set_link_addr(struct net_if *iface, uint8_t *addr, uint8_t len, enum net_link_type type);

/* The link address (len 0 until one is set).  As in Zephyr, a pointer to the
 * interface's own, read under no lock.
 */
struct net_linkaddr *net_if_get_link_addr(struct net_if *iface);

/* The MTU, which Zephyr takes from the device definition
 * (NET_DEVICE_DT_OFFLOAD_DEFINE(..., mtu): 1500 for the NINA) and the
 * FreeRTOS build's interface sets at its start (common/compat/net_offload/
 * zephyr_net_init.c).  0 until set.
 */
void net_if_set_mtu(struct net_if *iface, uint16_t mtu);
uint16_t net_if_get_mtu(struct net_if *iface);

/* Zephyr's dispatcher of offloaded interfaces (SO_BINDTODEVICE) routes a
 * socket bound to the interface to the driver's socket(); the layer has no
 * dispatcher (zsock_socket() takes the registered implementations,
 * <zephyr/net/socket.h>), so there is nothing to set.
 */
#define net_if_socket_offload_set(iface, socket_create) ((void)(iface))

#endif /* COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_NET_IF_H */
