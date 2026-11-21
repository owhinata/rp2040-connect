/* SPDX-License-Identifier: MIT */
/* The POSIX socket names of the shared NINA core (common/drivers/nina:
 * nina_sock.c, nina_dns.c and their headers), on Zephyr's own socket API.
 * Only the core and its Zephyr glue (../nina_socket.c) have this directory
 * on their include path (CMakeLists.txt); the rest of the application uses
 * Zephyr's names.
 *
 * The names come from Zephyr's namespace compatibility mode
 * (CONFIG_NET_NAMESPACE_COMPAT_MODE, on by default; <zephyr/net/net_ip.h>
 * includes <zephyr/net/net_compat.h> with it): struct sockaddr,
 * sockaddr_in, iovec and msghdr are Zephyr's struct net_*, AF_INET,
 * SOCK_STREAM, IPPROTO_TCP, ... are NET_*, MSG_DONTWAIT, TCP_NODELAY,
 * SOL_SOCKET, AI_* are ZSOCK_*, htons() is net_htons().  poll.h, netdb.h
 * and arpa/inet.h here add the few names the mode leaves out.  ssize_t is
 * the C library's (<sys/types.h>).  Zephyr's own POSIX headers
 * (include/zephyr/posix, CONFIG_POSIX_API) are not used: they come with
 * the POSIX API's functions, which this build does not have.
 */

#ifndef NINA_POSIX_SYS_SOCKET_H_
#define NINA_POSIX_SYS_SOCKET_H_

#include <sys/types.h>

#include <zephyr/net/socket.h>

#ifndef CONFIG_NET_NAMESPACE_COMPAT_MODE
#error "the shared NINA core needs CONFIG_NET_NAMESPACE_COMPAT_MODE"
#endif

#endif /* NINA_POSIX_SYS_SOCKET_H_ */
