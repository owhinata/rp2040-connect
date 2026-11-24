/* SPDX-License-Identifier: Apache-2.0 */
/* The address conversions of Zephyr's <zephyr/net/socket.h>,
 * zsock_inet_pton() and zsock_inet_ntop(), as the C library's inet_pton()
 * and inet_ntop() (Zephyr's are the same functions under its own names:
 * 1 / the buffer on success).  NuttX builds AF_INET in them with
 * CONFIG_NET_IPv4 or, without networking, CONFIG_LIBC_IPv4_ADDRCONV (the
 * imu application selects it).  zsock_getaddrinfo() and
 * zsock_freeaddrinfo() are the NINA resolver's (<nina_dns.h>, the Zephyr
 * build's offloaded getaddrinfo() of drivers/nina/nina_dns.c); their
 * errors are NuttX's EAI_* values.
 *
 * Sockets: the zsock_*() calls are NuttX's socket(), connect(), send(),
 * sendto(), sendmsg(), recv(), poll() and close() (the NINA's sockets
 * through usrsock, common/drivers/nina/port/nuttx/nina_usrsock.c), -1 and
 * errno as in Zephyr, with Zephyr's names of the flags and poll events
 * (ZSOCK_MSG_DONTWAIT, ZSOCK_POLL*, the same values here).  The waits are
 * the driver's: a blocking connect or send waits in it as in the Zephyr
 * build, a receive or poll() waits for the events that the NINA's event
 * thread (nina_poller.c) makes from the module's state (nina_usrsock.c
 * describes when they come).  What the Zephyr build's NINA poll() says
 * differently: a TCP socket that is not connected is POLLIN, POLLOUT and
 * POLLHUP (NuttX's usrsock), and the module's error flag or a failed poll
 * of a connected TCP socket is POLLHUP (usrsock's REMOTE_CLOSED) where the
 * Zephyr build gives POLLERR.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (common/compat/port/nuttx, see nuttx/drivers/README.md).  Include
 * NuttX's headers before the layer's (see <zephyr/sys/atomic.h>).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <nina_dns.h>

/* <unistd.h>'s (not included: its getopt() names are macros, see
 * <zephyr/kernel.h>).
 */

int close(int fd);

#define ZSOCK_MSG_DONTWAIT MSG_DONTWAIT

#define zsock_socket(family, type, proto) socket(family, type, proto)
#define zsock_sendto(sock, buf, len, flags, dest_addr, addrlen) \
  sendto(sock, buf, len, flags, dest_addr, addrlen)
#define zsock_close(sock) close(sock)
#define zsock_connect(sock, addr, addrlen) connect(sock, addr, addrlen)
#define zsock_send(sock, buf, len, flags) send(sock, buf, len, flags)
#define zsock_sendmsg(sock, msg, flags) sendmsg(sock, msg, flags)
#define zsock_recv(sock, buf, max_len, flags) recv(sock, buf, max_len, flags)

#define zsock_pollfd pollfd
#define ZSOCK_POLLIN   POLLIN
#define ZSOCK_POLLOUT  POLLOUT
#define ZSOCK_POLLERR  POLLERR
#define ZSOCK_POLLHUP  POLLHUP
#define ZSOCK_POLLNVAL POLLNVAL
#define zsock_poll(fds, nfds, timeout) poll(fds, nfds, timeout)

#define zsock_inet_pton(family, src, dst) inet_pton(family, src, dst)
#define zsock_inet_ntop(family, src, dst, size) \
  inet_ntop(family, src, dst, size)

#define zsock_addrinfo addrinfo
#define zsock_getaddrinfo(host, service, hints, res) \
  nina_dns_getaddrinfo(host, service, hints, res)
#define zsock_freeaddrinfo(res) nina_dns_freeaddrinfo(res)

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H */
