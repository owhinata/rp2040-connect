/* SPDX-License-Identifier: Apache-2.0 */
/* <zephyr/net/net_if.h> of the Zephyr compatibility layer on a NuttX
 * network device: the interface's IPv4 address, netmask and gateway are the
 * device's d_ipaddr, d_netmask and d_draddr (what ifconfig shows), dormant
 * is IFF_RUNNING clear.  See the header.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <net/if.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include <zephyr/net/net_if.h>

static struct net_if *g_default_iface;

/* What net_if_ipv4_addr_add() returns and
 * net_if_ipv4_get_global_addr() points to (Zephyr: the interface's
 * address slot).
 */

static struct net_if_addr g_unicast;

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

struct net_if_addr *net_if_ipv4_addr_add(struct net_if *iface,
                                         const struct net_in_addr *addr,
                                         enum net_addr_type addr_type,
                                         uint32_t vlifetime)
{
  struct net_driver_s *dev = iface->dev;

  (void)vlifetime;
  if (addr->s_addr == INADDR_ANY)
    {
      return NULL;
    }

  netdev_lock(dev);
  dev->d_ipaddr = addr->s_addr;
  g_unicast.address = *addr;
  g_unicast.addr_type = addr_type;
  g_unicast.addr_state = NET_ADDR_PREFERRED;
  netdev_unlock(dev);
  return &g_unicast;
}

bool net_if_ipv4_addr_rm(struct net_if *iface,
                         const struct net_in_addr *addr)
{
  struct net_driver_s *dev = iface->dev;
  bool removed = false;

  netdev_lock(dev);
  if (addr->s_addr != INADDR_ANY && dev->d_ipaddr == addr->s_addr)
    {
      dev->d_ipaddr = INADDR_ANY;
      g_unicast.address.s_addr = INADDR_ANY;
      removed = true;
    }

  netdev_unlock(dev);
  return removed;
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
          (state == NET_ADDR_ANY_STATE ||
           state == g_unicast.addr_state);
  netdev_unlock(dev);
  return found ? &g_unicast.address : NULL;
}

bool net_if_ipv4_set_netmask_by_addr(struct net_if *iface,
                                     const struct net_in_addr *addr,
                                     const struct net_in_addr *netmask)
{
  struct net_driver_s *dev = iface->dev;
  bool set = false;

  netdev_lock(dev);
  if (dev->d_ipaddr != INADDR_ANY && dev->d_ipaddr == addr->s_addr)
    {
      dev->d_netmask = netmask->s_addr;
      set = true;
    }

  netdev_unlock(dev);
  return set;
}

void net_if_ipv4_set_gw(struct net_if *iface, const struct net_in_addr *gw)
{
  struct net_driver_s *dev = iface->dev;

  netdev_lock(dev);
  dev->d_draddr = gw->s_addr;
  netdev_unlock(dev);
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

void net_if_dormant_on(struct net_if *iface)
{
  netdev_lock(iface->dev);
  netdev_carrier_off(iface->dev);
  netdev_unlock(iface->dev);
}

void net_if_dormant_off(struct net_if *iface)
{
  netdev_lock(iface->dev);
  netdev_carrier_on(iface->dev);
  netdev_unlock(iface->dev);
}

bool net_if_is_dormant(struct net_if *iface)
{
  return !IFF_IS_RUNNING(iface->dev->d_flags);
}

bool net_if_is_admin_up(struct net_if *iface)
{
  return IFF_IS_UP(iface->dev->d_flags);
}

int net_if_set_link_addr(struct net_if *iface, uint8_t *addr, uint8_t len,
                         enum net_link_type type)
{
  struct net_driver_s *dev = iface->dev;

  if (type != NET_LINK_ETHERNET || len != sizeof(dev->d_mac.ether))
    {
      return -EINVAL;
    }

  netdev_lock(dev);
  memcpy(dev->d_mac.ether.ether_addr_octet, addr, len);
  netdev_unlock(dev);
  return 0;
}
