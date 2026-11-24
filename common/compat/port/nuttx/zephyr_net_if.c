/* SPDX-License-Identifier: Apache-2.0 */
/* <zephyr/net/net_if.h> of the Zephyr compatibility layer on a NuttX
 * network device: the interface's IPv4 address and gateway are the device's
 * d_ipaddr and d_draddr (what ifconfig shows), administratively up is
 * IFF_UP.  See the header.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <net/if.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include <zephyr/net/net_if.h>

static struct net_if *g_default_iface;

/* What net_if_ipv4_get_global_addr() points to (Zephyr: the interface's
 * address slot).
 */

static struct net_in_addr g_global_addr;

void z_net_if_register(struct net_if *iface)
{
  g_default_iface = iface;
}

struct net_if *net_if_get_default(void)
{
  return g_default_iface;
}

struct net_if *net_if_get_wifi_sta(void)
{
  return g_default_iface;
}

struct net_in_addr *net_if_ipv4_get_global_addr(struct net_if *iface,
                                                enum net_addr_state state)
{
  struct net_driver_s *dev;
  bool found;

  if (iface == NULL)
    {
      return NULL;
    }

  dev = iface->dev;
  netdev_lock(dev);
  found = dev->d_ipaddr != INADDR_ANY &&
          (state == NET_ADDR_ANY_STATE || state == NET_ADDR_PREFERRED);
  if (found)
    {
      g_global_addr.s_addr = dev->d_ipaddr;
    }

  netdev_unlock(dev);
  return found ? &g_global_addr : NULL;
}

struct net_in_addr net_if_ipv4_get_gw(struct net_if *iface)
{
  struct net_driver_s *dev = iface->dev;
  struct net_in_addr gw;

  netdev_lock(dev);
  gw.s_addr = dev->d_draddr;
  netdev_unlock(dev);
  return gw;
}

bool net_if_is_admin_up(struct net_if *iface)
{
  return IFF_IS_UP(iface->dev->d_flags);
}
