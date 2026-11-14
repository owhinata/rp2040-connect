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
 * Sockets: zsock_socket(), zsock_sendto() and zsock_close() are NuttX's
 * socket(), sendto() and close() (the NINA's sockets through usrsock,
 * nuttx/drivers/nina/nina_usrsock.c), -1 and errno as in Zephyr.
 * zsock_recv() is recv(), except with ZSOCK_MSG_DONTWAIT: the Zephyr
 * build's recv() on a NINA socket (drivers/nina/nina_socket.c udp_recv(),
 * do_recv()) asks the module (CMD_POLL) on every call and reads what is
 * there.  NuttX's usrsock answers a call that does not wait from the
 * RECVFROM_AVAIL it holds, which the NINA's event thread refreshes only
 * every 20 ms (nina_poller.c).  So zsock_recv() with ZSOCK_MSG_DONTWAIT
 * first asks the module with ioctl(FIONREAD), which polls it once in the
 * calling thread (nina_sock_fionread(): 1 when data or the end of a stream
 * is there, 0 when not) and, when there is something, raises
 * RECVFROM_AVAIL; then recv(MSG_DONTWAIT) reads it, without a second poll
 * (the driver keeps what the poll found, read_hint).  Nothing there:
 * -1 with errno EAGAIN, as Zephyr's.  The module sees the same commands as
 * from the Zephyr build: one CMD_POLL, then the read when it found data.
 *
 * zsock_connect(), zsock_send() and zsock_sendmsg() are connect(), send()
 * and sendmsg() (the calls of Zephyr's MQTT TCP transport,
 * mqtt_transport_socket_tcp.c; a blocking socket waits in the driver as in
 * the Zephyr build's: connect up to NINA_CONNECT_WAIT_MS, a send that finds
 * no room up to NINA_SEND_WAIT_MS).
 *
 * zsock_poll() and struct zsock_pollfd, ZSOCK_POLL*: NuttX's poll() and
 * struct pollfd, POLL*, except how a NINA socket is asked.  The Zephyr
 * build's poll() on NINA sockets (drivers/nina/nina_socket.c do_poll())
 * polls the module (CMD_POLL) for each socket at once, in the calling
 * thread, then again every NINA_POLL_INTERVAL_MS (20 ms) until one is
 * ready or the timeout ends.  NuttX's poll() answers from the events
 * usrsock holds, which the NINA's event thread refreshes every 20 ms
 * (nina_poller.c).  So zsock_poll() first asks the module about each
 * socket polled for ZSOCK_POLLIN with ioctl(FIONREAD): one CMD_POLL now,
 * in the calling thread, which raises RECVFROM_AVAIL when data or the end
 * of the stream is there (as for zsock_recv()).  Then NuttX's poll() with
 * no wait reports what is held; if nothing is ready and TIMEOUT is not 0,
 * NuttX's poll() waits for the rest of it, on the event thread's polls
 * every 20 ms (the Zephyr build polls from the caller at the same
 * interval).  A failed poll of the module ends the call with -1 and its
 * errno, as there; a TCP socket that is not connected is left to NuttX's
 * poll() (the Zephyr build reports POLLHUP for one the module does not
 * have).  The module's POLL_ERROR, which the Zephyr build reports as
 * ZSOCK_POLLERR, arrives here as usrsock's REMOTE_CLOSED from the event
 * thread's poll and poll() reports ZSOCK_POLLHUP.  Sockets only: other
 * descriptors would get a FIONREAD of their own.
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
#include <sys/ioctl.h>
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

#define zsock_pollfd pollfd
#define ZSOCK_POLLIN   POLLIN
#define ZSOCK_POLLOUT  POLLOUT
#define ZSOCK_POLLERR  POLLERR
#define ZSOCK_POLLHUP  POLLHUP
#define ZSOCK_POLLNVAL POLLNVAL

static inline int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
  int ready;
  int i;

  for (i = 0; i < nfds; i++)
    {
      int avail = 0;

      if (fds[i].fd < 0 || !(fds[i].events & ZSOCK_POLLIN))
        {
          continue;
        }

      if (ioctl(fds[i].fd, FIONREAD, &avail) < 0 && errno != ENOTCONN)
        {
          return -1;
        }
    }

  ready = poll(fds, nfds, 0);
  if (ready != 0 || timeout == 0)
    {
      return ready;
    }

  return poll(fds, nfds, timeout);
}

static inline ssize_t zsock_recv(int sock, void *buf, size_t max_len,
                                 int flags)
{
  if (flags & ZSOCK_MSG_DONTWAIT)
    {
      int avail = 0;

      if (ioctl(sock, FIONREAD, &avail) < 0)
        {
          return -1;
        }

      if (avail <= 0)
        {
          errno = EAGAIN;
          return -1;
        }
    }

  return recv(sock, buf, max_len, flags);
}

#define zsock_inet_pton(family, src, dst) inet_pton(family, src, dst)
#define zsock_inet_ntop(family, src, dst, size) \
  inet_ntop(family, src, dst, size)

#define zsock_addrinfo addrinfo
#define zsock_getaddrinfo(host, service, hints, res) \
  nina_getaddrinfo(host, service, hints, res)
#define zsock_freeaddrinfo(res) nina_freeaddrinfo(res)

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H */
