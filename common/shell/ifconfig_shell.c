/* SPDX-License-Identifier: MIT */
/* The "ifconfig" shell command of the builds that run the Zephyr shell's
 * commands on the compatibility layer's shell and its network part for an
 * OS without a network stack (the FreeRTOS build,
 * common/compat/port/pico_sdk/wifi.cmake; NuttX has nsh's): the NuttX build's
 * "ifconfig" (nsh) without arguments or with the interface's name, with the
 * same lines, on the state of the layer's one interface
 * (<zephyr/net/net_if.h>, common/compat/net_offload/zephyr_net_if.c), which
 * the NINA driver's Zephyr binding (common/drivers/nina/port/zephyr/nina_net.c)
 * keeps as in the Zephyr build (the NuttX build's binding keeps it in
 * wlan0).  It sets nothing (NuttX's "ifconfig <name> <address>
 * ..." is not here).
 *
 * NuttX's nsh prints /proc/net/wlan0 (net/procfs/netdev_statistics.c,
 * NuttX 13.1.0, with CONFIG_NET_IPv4 and without CONFIG_NET_IPv6 and
 * CONFIG_NETDEV_STATISTICS, as the NuttX build has them), and nothing more
 * (no CONFIG_NET_STATISTICS, so no /proc/net/stat):
 *
 *   "%s\tLink encap:Ethernet HWaddr %s at %s mtu %d\n"
 *   "\tinet addr:%s DRaddr:%s Mask:%s\n\n"
 *
 * The name is wlan0 (netdev_register() with NET_LL_IEEE80211; the NuttX
 * build's common/drivers/nina/port/nuttx/nina_netdev.c), the MAC address in lower case
 * with colons (ether_ntoa_r()), the state RUNNING with a carrier (the NuttX
 * build turns it on when the core's connection is up: link_up() of its
 * binding), UP without one (the interface is administratively
 * up from its start), DOWN otherwise (not up); the MTU 1500.  The addresses
 * are dotted (inet_ntoa_r()), 0.0.0.0 when there is none, and the last line
 * is followed by an empty one ("Double space at end of device description").
 *
 * One difference: the NuttX build leaves the netmask in its device when the
 * address is removed at a disconnection, and shows it until the next
 * connection; the layer's interface here, as Zephyr's, drops the netmask
 * with the address (Mask:0.0.0.0 while disconnected).
 *
 * Without arguments the command shows every interface: before the NINA's
 * interface is registered (common/compat/net_offload/zephyr_net_init.c, at the start)
 * there is none and nothing is printed, as NuttX prints nothing without a
 * network device.  Another name gets an error (-ENODEV).
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <zephyr/net/net_if.h>
#include <zephyr/shell/shell.h>

#define IFCONFIG_NAME "wlan0"

static const char *link_state(struct net_if *iface)
{
	if (!net_if_is_dormant(iface)) {
		return "RUNNING";
	}
	return net_if_is_admin_up(iface) ? "UP" : "DOWN";
}

static void show(const struct shell *sh, struct net_if *iface)
{
	const struct net_linkaddr *ll = net_if_get_link_addr(iface);
	struct net_in_addr *unicast = net_if_ipv4_get_global_addr(iface, NET_ADDR_ANY_STATE);
	struct net_in_addr addr = {.s_addr = INADDR_ANY};
	struct net_in_addr mask = {.s_addr = INADDR_ANY};
	struct net_in_addr gw = net_if_ipv4_get_gw(iface);
	char addr_text[INET_ADDRSTRLEN], gw_text[INET_ADDRSTRLEN], mask_text[INET_ADDRSTRLEN];
	uint8_t mac[6] = {0};

	if (ll->len == sizeof(mac)) {
		memcpy(mac, ll->addr, sizeof(mac));
	}
	if (unicast != NULL) {
		addr = *unicast;
		mask = net_if_ipv4_get_netmask_by_addr(iface, &addr);
	}
	(void)inet_ntop(AF_INET, &addr, addr_text, sizeof(addr_text));
	(void)inet_ntop(AF_INET, &gw, gw_text, sizeof(gw_text));
	(void)inet_ntop(AF_INET, &mask, mask_text, sizeof(mask_text));

	shell_print(sh, "%s\tLink encap:Ethernet HWaddr %02x:%02x:%02x:%02x:%02x:%02x at %s mtu %d",
		    IFCONFIG_NAME, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
		    link_state(iface), (int)net_if_get_mtu(iface));
	shell_print(sh, "\tinet addr:%s DRaddr:%s Mask:%s", addr_text, gw_text, mask_text);
	shell_print(sh, "");
}

static int cmd_ifconfig(const struct shell *sh, size_t argc, char **argv)
{
	struct net_if *iface = net_if_get_default();

	if (argc > 1 && (iface == NULL || strcmp(argv[1], IFCONFIG_NAME) != 0)) {
		shell_error(sh, "%s: no such interface", argv[1]);
		return -ENODEV;
	}
	if (iface != NULL) {
		show(sh, iface);
	}
	return 0;
}

SHELL_CMD_ARG_REGISTER(ifconfig, NULL,
		       "Show the network interface: ifconfig [wlan0]\n"
		       "(MAC address, state, IPv4 address, default router, netmask)",
		       cmd_ifconfig, 1, 1);
