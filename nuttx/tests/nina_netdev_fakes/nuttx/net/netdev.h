/* SPDX-License-Identifier: Apache-2.0 */
/* The part of NuttX's network device (<nuttx/net/netdev.h>) that
 * nina_netdev.c uses, with NuttX's member names; tests/nina_netdev.c
 * defines the functions.
 */

#ifndef TEST_NINA_NETDEV_NUTTX_NET_NETDEV_H
#define TEST_NINA_NETDEV_NUTTX_NET_NETDEV_H

#include <netinet/in.h>
#include <stdint.h>

#include <nuttx/net/net.h>

struct net_driver_s
{
  uint32_t d_flags;
  union
  {
    struct
    {
      uint8_t ether_addr_octet[6];
    } ether;
  } d_mac;
  uint16_t d_llhdrlen;
  uint16_t d_pktsize;
  in_addr_t d_ipaddr;
  in_addr_t d_draddr;
  in_addr_t d_netmask;
  int (*d_ioctl)(struct net_driver_s *dev, int cmd, unsigned long arg);
};

int netdev_register(struct net_driver_s *dev, enum net_lltype_e lltype);
void netdev_lock(struct net_driver_s *dev);
void netdev_unlock(struct net_driver_s *dev);
void netdev_carrier_on(struct net_driver_s *dev);
void netdev_carrier_off(struct net_driver_s *dev);

#endif /* TEST_NINA_NETDEV_NUTTX_NET_NETDEV_H */
