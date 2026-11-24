/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/net/net_ip.h> (Zephyr v4.5.0) that the
 * FreeRTOS build uses: the IPv4 address type struct in_addr (imu_raw_mqtt.h)
 * and Zephyr 4.5's net_* names of the socket types and constants, which the
 * Zephyr binding of the NINA's sockets (common/drivers/nina/port/zephyr/
 * nina_socket.c) and the layer's socket part (<zephyr/net/socket.h>) are
 * written with.  Zephyr defines the POSIX names on its net_* ones (its
 * namespace compatibility mode, <zephyr/net/net_compat.h>); here it is the
 * other way round: the net_* names are the POSIX types and constants of
 * common/compat/libc_net/include (newlib has none), which every other source
 * of the build uses, so one type has both names.  Their values are
 * Zephyr's except NET_AF_INET (2 here and on NuttX, 1 in Zephyr; nothing
 * leaves the build with it).
 *
 * This header takes the place of common/compat/include/zephyr/net/net_ip.h (which
 * the NuttX build uses) in the FreeRTOS build: common/compat/net_offload/
 * include comes before common/compat/include on its include path.
 * Part of the network part of the Zephyr compatibility layer for an OS
 * without a network stack of its own (common/compat/net_offload), which the
 * FreeRTOS build uses.
 */

#ifndef COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_NET_IP_H
#define COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_NET_IP_H

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>

/* The types: struct tags and typedefs. */
#define net_sockaddr         sockaddr
#define net_sockaddr_in      sockaddr_in
#define net_sockaddr_storage sockaddr_storage
#define net_in_addr          in_addr
#define net_msghdr           msghdr
#define net_iovec            iovec

typedef sa_family_t net_sa_family_t;
typedef socklen_t net_socklen_t;

/* The families, types and protocols. */
#define NET_AF_UNSPEC       AF_UNSPEC
#define NET_AF_INET         AF_INET
#define NET_AF_INET6        AF_INET6
#define NET_SOCK_STREAM     SOCK_STREAM
#define NET_SOCK_DGRAM      SOCK_DGRAM
#define NET_IPPROTO_TCP     IPPROTO_TCP
#define NET_IPPROTO_UDP     IPPROTO_UDP
#define NET_INADDR_ANY      INADDR_ANY
#define NET_INET_ADDRSTRLEN INET_ADDRSTRLEN

#endif /* COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_NET_IP_H */
