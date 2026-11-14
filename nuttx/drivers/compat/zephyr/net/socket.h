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
 * Sockets: zsock_sendto() and zsock_close() are NuttX's sendto() and
 * close() (the NINA's sockets through usrsock,
 * nuttx/drivers/nina/nina_usrsock.c), -1 and errno as in Zephyr;
 * zsock_connect(), zsock_send() and zsock_sendmsg() are connect(), send()
 * and sendmsg() (the calls of Zephyr's MQTT TCP transport,
 * mqtt_transport_socket_tcp.c; a blocking socket waits in the driver as in
 * the Zephyr build's: connect up to NINA_CONNECT_WAIT_MS, checked every
 * NINA_POLL_INTERVAL_MS by the NINA's event thread, a send that finds no
 * room up to NINA_SEND_WAIT_MS, its passes in the calling thread).
 *
 * The Zephyr build's NINA sockets (drivers/nina/nina_socket.c) are asked
 * about only by the threads that use them: recv() polls the module
 * (CMD_POLL) on every call, poll() polls each socket at once and then every
 * NINA_POLL_INTERVAL_MS (20 ms) until one is ready or the timeout ends
 * (do_poll()).  NuttX's usrsock instead waits for events, which the NINA's
 * event thread (nina_poller.c) makes by polling the sockets every 20 ms.
 * The layer makes the Zephyr build's polls itself, from the calling
 * thread, with the driver's options (<nina_sockopt.h>):
 *
 * - zsock_socket() is socket() with NINA_SO_CALLER_POLLS set: the event
 *   thread leaves the socket alone except while it connects.  Not a NINA
 *   socket, or the option refused: -1, the socket closed.
 * - zsock_recv() with ZSOCK_MSG_DONTWAIT: ioctl(FIONREAD), one CMD_POLL
 *   now (nina_sock_fionread(): 1 when data or the end of a stream is
 *   there, 0 when not), which raises RECVFROM_AVAIL when there is
 *   something; then recv(MSG_DONTWAIT) reads it, without a second poll
 *   (the driver keeps what the poll found, read_hint).  Nothing there:
 *   -1 with errno EAGAIN, as Zephyr's.  The module sees the Zephyr build's
 *   commands: one CMD_POLL, then the read when it found data.  Without the
 *   flag (do_recv(), udp_recv()): the same pass every NINA_POLL_INTERVAL_MS
 *   until something is read (k_sleep(), as there; SO_RCVTIMEO and
 *   O_NONBLOCK are not looked at: no caller sets them).
 * - zsock_poll(): do_poll().  Each socket gets getsockopt(NINA_SO_POLL),
 *   one CMD_POLL in the calling thread, answered as there: ZSOCK_POLLIN and
 *   ZSOCK_POLLOUT when asked for and the module says so, ZSOCK_POLLERR for
 *   its POLL_ERROR, ZSOCK_POLLHUP for a TCP socket the module does not have
 *   yet.  A failed poll of the module ends the call with -1 and its errno.
 *   A descriptor that is not a NINA socket gets NuttX's poll() without
 *   waiting (ZSOCK_POLLNVAL for a bad one), as there.  Nothing ready: again
 *   after k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS)) until the timeout (in
 *   k_uptime_get() milliseconds, -1 for none) has gone by.
 *
 * A receive that waits (recv() without MSG_DONTWAIT) or NuttX's poll() on
 * a socket from zsock_socket() would wait for an event that does not come:
 * use the zsock_*() calls on these sockets.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).  Include NuttX's
 * headers before the layer's (see <zephyr/sys/atomic.h>).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <zephyr/kernel.h>

#include <nina_dns.h>
#include <nina_sockopt.h>

/* <unistd.h>'s (not included: its getopt() names are macros, see
 * <zephyr/kernel.h>).
 */

int close(int fd);

#define ZSOCK_MSG_DONTWAIT MSG_DONTWAIT

#define zsock_sendto(sock, buf, len, flags, dest_addr, addrlen) \
  sendto(sock, buf, len, flags, dest_addr, addrlen)
#define zsock_close(sock) close(sock)
#define zsock_connect(sock, addr, addrlen) connect(sock, addr, addrlen)
#define zsock_send(sock, buf, len, flags) send(sock, buf, len, flags)
#define zsock_sendmsg(sock, msg, flags) sendmsg(sock, msg, flags)

#define zsock_pollfd pollfd
#define ZSOCK_POLLIN   POLLIN
#define ZSOCK_POLLOUT  POLLOUT
#define ZSOCK_POLLERR  POLLERR
#define ZSOCK_POLLHUP  POLLHUP
#define ZSOCK_POLLNVAL POLLNVAL

static inline int zsock_socket(int family, int type, int proto)
{
  int sock = socket(family, type, proto);
  int on = 1;
  int err;

  if (sock >= 0 &&
      setsockopt(sock, NINA_SOL, NINA_SO_CALLER_POLLS, &on, sizeof(on)) < 0)
    {
      err = errno;
      close(sock);
      errno = err;
      return -1;
    }

  return sock;
}

static inline int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
  int64_t deadline = timeout < 0 ? INT64_MAX : k_uptime_get() + timeout;

  for (; ; )
    {
      int ready = 0;
      int i;

      for (i = 0; i < nfds; i++)
        {
          socklen_t len = sizeof(int);
          int revents;

          fds[i].revents = 0;
          if (fds[i].fd < 0)
            {
              continue;
            }

          if (getsockopt(fds[i].fd, NINA_SOL, NINA_SO_POLL, &revents,
                         &len) == 0)
            {
              fds[i].revents = revents & (fds[i].events | ZSOCK_POLLERR |
                                          ZSOCK_POLLHUP);
            }
          else if (errno == EBADF || errno == ENOTSOCK ||
                   errno == ENOPROTOOPT || errno == ENOTTY)
            {
              struct zsock_pollfd native = fds[i];

              if (poll(&native, 1, 0) < 0)
                {
                  return -1;
                }

              fds[i].revents = native.revents;
            }
          else
            {
              return -1;
            }

          if (fds[i].revents)
            {
              ready++;
            }
        }

      if (ready)
        {
          return ready;
        }

      if (k_uptime_get() >= deadline)
        {
          return 0;
        }

      k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS));
    }
}

static inline ssize_t zsock_recv(int sock, void *buf, size_t max_len,
                                 int flags)
{
  for (; ; )
    {
      int avail = 0;
      ssize_t n;

      if (ioctl(sock, FIONREAD, &avail) < 0)
        {
          return -1;
        }

      if (avail > 0)
        {
          n = recv(sock, buf, max_len, flags | ZSOCK_MSG_DONTWAIT);
          if (n >= 0 || errno != EAGAIN || (flags & ZSOCK_MSG_DONTWAIT))
            {
              return n;
            }

          /* A datagram from another sender (connected UDP): wait on. */
        }
      else if (flags & ZSOCK_MSG_DONTWAIT)
        {
          errno = EAGAIN;
          return -1;
        }

      k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS));
    }
}

#define zsock_inet_pton(family, src, dst) inet_pton(family, src, dst)
#define zsock_inet_ntop(family, src, dst, size) \
  inet_ntop(family, src, dst, size)

#define zsock_addrinfo addrinfo
#define zsock_getaddrinfo(host, service, hints, res) \
  nina_getaddrinfo(host, service, hints, res)
#define zsock_freeaddrinfo(res) nina_freeaddrinfo(res)

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H */
