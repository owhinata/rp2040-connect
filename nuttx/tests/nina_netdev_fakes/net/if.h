/* SPDX-License-Identifier: Apache-2.0 */
/* The part of NuttX's <net/if.h> (NuttX 13.1.0) that nina_netdev.c and
 * NuttX's <nuttx/wireless/wireless.h> use, with NuttX's values, for the
 * host test tests/nina_netdev.c.
 */

#ifndef TEST_NINA_NETDEV_NET_IF_H
#define TEST_NINA_NETDEV_NET_IF_H

#define IFNAMSIZ                16
#define IFF_UP                  (1 << 1)
#define IFF_RUNNING             (1 << 2)
#define IFF_IS_UP(f)            (((f) & IFF_UP) != 0)
#define IFF_IS_RUNNING(f)       (((f) & IFF_RUNNING) != 0)

#endif /* TEST_NINA_NETDEV_NET_IF_H */
