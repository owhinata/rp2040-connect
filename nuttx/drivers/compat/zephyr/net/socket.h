/* SPDX-License-Identifier: Apache-2.0 */
/* The address conversions of Zephyr's <zephyr/net/socket.h>,
 * zsock_inet_pton() and zsock_inet_ntop(), as the C library's inet_pton()
 * and inet_ntop() (Zephyr's are the same functions under its own names:
 * 1 / the buffer on success).  NuttX builds AF_INET in them with
 * CONFIG_NET_IPv4 or, without networking, CONFIG_LIBC_IPv4_ADDRCONV (the
 * imu application selects it).  The sockets themselves (zsock_socket(),
 * zsock_poll(), ...) come with the NINA socket layer (T14-T16 of the port
 * plan).  Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).  Include NuttX's
 * headers before the layer's (see <zephyr/sys/atomic.h>).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H

#include <arpa/inet.h>
#include <netinet/in.h>

#define zsock_inet_pton(family, src, dst) inet_pton(family, src, dst)
#define zsock_inet_ntop(family, src, dst, size) \
  inet_ntop(family, src, dst, size)

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H */
