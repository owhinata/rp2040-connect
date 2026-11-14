/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX usrsock on the NINA socket table (CONFIG_NET_USRSOCK_CUSTOM).
 *
 * The Zephyr build registers the NINA as a socket offload
 * (drivers/nina/nina_socket.c: NET_SOCKET_OFFLOAD_REGISTER and the
 * socket_op_vtable); its socket calls run in the calling thread, one at a
 * time per socket, the NINA commands serialized by nina_lock.  Here NuttX's
 * usrsock calls usrsock_request() in the calling thread with the usrsock
 * lock held, one request at a time.  The request is handled there, at the
 * caller's priority, and answered through usrsock_response() before
 * returning, as arch/arm/src/nrf91/nrf91_modem_sock.c does.
 *
 *   usrsock request          Zephyr build             here
 *   SOCKET                   nina_socket_create()     nina_sock_create()
 *   CLOSE                    do_close()               nina_sock_close()
 *   CONNECT                  do_connect()             nina_sock_connect(), then
 *                                                     nina_poller.c
 *   BIND, LISTEN, ACCEPT     do_bind/listen/accept()  nina_sock_bind/listen/accept()
 *   SETSOCKOPT               do_setsockopt()          nina_sock_setsockopt()
 *   GETSOCKOPT               no vtable entry:         NINA_SO_POLL:
 *                            EOPNOTSUPP               nina_sock_poll_revents()
 *                            (do_poll() for one       (not in the Zephyr build);
 *                            socket)                  others -EOPNOTSUPP
 *   GETSOCKNAME,             no vtable entry:         -EOPNOTSUPP
 *   GETPEERNAME, SHUTDOWN    EOPNOTSUPP
 *   IOCTL                    do_ioctl()               FIONREAD: nina_sock_fionread()
 *                                                     (not in the Zephyr build);
 *                                                     others -ENOTTY (NuttX's own
 *                                                     handling)
 *   SENDTO                   do_send(), udp_send()    nina_sock_send() (one data
 *                            (send, sendto)           vector)
 *                            do_sendmsg()             nina_sock_sendmsg() (more)
 *   RECVFROM                 do_recv(), udp_recv()    nina_sock_recv()
 *
 * A TCP connect is answered "in progress" when the module has not finished
 * the handshake at the first check; nina_poller.c checks it every
 * NINA_POLL_INTERVAL_MS until NINA_CONNECT_WAIT_MS, as the Zephyr build's
 * connect() does in its caller, and answers the request then.
 *
 * Send and receive (see nina_sock.c): each request makes one pass of the
 * Zephyr build's wait loop in the calling thread; usrsock does the waiting
 * between passes.  The answers:
 *
 *   data sent                 the count, SENDTO_READY kept
 *   TCP send, no room, the    -EAGAIN without SENDTO_READY; nina_poller.c
 *   call waits                reports SENDTO_READY after NINA_SEND_RETRY_MS
 *                             and usrsock sends the request again
 *   any other EAGAIN: a call  NINA_USRSOCK_EAGAIN_FINAL with SENDTO_READY:
 *   that does not wait, the   the call returns -EAGAIN at once (usrsock
 *   10 s deadline, a datagram would repeat a -EAGAIN of a waiting call)
 *   the module did not take
 *   data received             the count, RECVFROM_AVAIL dropped, and
 *                             nina_poller.c polls now for more
 *   nothing to receive        -EAGAIN without RECVFROM_AVAIL; nina_poller.c
 *                             polls every NINA_POLL_INTERVAL_MS
 *
 * A socket marked NINA_SO_CALLER_POLLS (<nina_sockopt.h>; the Zephyr
 * compatibility layer's sockets) is not polled by nina_poller.c, nor after
 * a receive: its callers poll it, through FIONREAD and NINA_SO_POLL, in
 * their own thread, as the Zephyr build's recv() and poll() do.  Only a
 * connection in progress and the timer of a waiting send's next pass are
 * left to nina_poller.c.
 *
 * SENDTO_READY is dropped only between the passes of a waiting send.  A
 * send that waits for it without asking the driver would miss its passes
 * and its deadline (usrsock asks only when it holds SENDTO_READY), so a
 * TCP socket is always writable for poll(), where the Zephyr build's
 * poll() asks the module.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <nuttx/net/usrsock.h>

#include "nina_sock.h"
#include "nina_usrsock.h"

#ifndef CONFIG_NET_USRSOCK_CUSTOM
#  error CONFIG_NET_USRSOCK_CUSTOM must be set
#endif

/* The answer that ends a send call with -EAGAIN without another try:
 * nuttx/patches/nuttx/0005 makes usrsock_sendmsg() return -EAGAIN for it.
 * usrsock repeats a request answered -EAGAIN while the call waits, and
 * the Zephyr build's send returns EAGAIN after NINA_SEND_WAIT_MS (and at
 * once for a datagram).  nina_request() never returns -ETIME.
 */

#define NINA_USRSOCK_EAGAIN_FINAL (-ETIME)

#if CONFIG_NET_USRSOCK_PREALLOC_CONNS + CONFIG_NET_USRSOCK_ALLOC_CONNS == 0
#  error usrsock needs connections
#endif

/* The request with the start of what follows it (an address, an option
 * value).  Requests are handled one at a time (usrsock's request lock).
 * Data (T15) is read from the request's iovec, not copied here.  The
 * request structures are packed; what follows them is copied out with
 * memcpy() (get_addr()) before it is used: the Cortex-M0+ faults on
 * unaligned word access.  Data to send is read from the caller's vectors
 * in place (nina_sock.c joins several into one buffer).  Received data is
 * read into g_receive and copied to the caller by usrsock_response().
 */

#define NINA_USRSOCK_REQBUF 64

static uint8_t g_request[NINA_USRSOCK_REQBUF] aligned_data(4);
static uint8_t g_receive[NINA_MAX_IO] aligned_data(4);

/* Answers ********************************************************************/

static uint16_t usrsock_events(uint8_t ev)
{
  return ((ev & NINA_SOCK_EV_READ) ? USRSOCK_EVENT_RECVFROM_AVAIL : 0) |
         ((ev & NINA_SOCK_EV_WRITE) ? USRSOCK_EVENT_SENDTO_READY : 0) |
         ((ev & NINA_SOCK_EV_CLOSED) ? USRSOCK_EVENT_REMOTE_CLOSED : 0);
}

/* The events an answer carries, recorded as held by usrsock. */

static uint16_t report(int usockid, uint8_t ev)
{
  struct nina_sock *s = nina_sock_get(usockid);

  if (s != NULL)
    {
      s->reported |= ev;
    }

  return usrsock_events(ev);
}

/* Passes one message to usrsock.  Returns 0 when it took a request's
 * answer, which wakes the requester (usrsock_do_request() waits for that
 * only when usrsock_request() returns 0 or more), or a negative errno.
 */

static int respond(const void *msg, size_t len, bool answer)
{
  bool done = false;
  ssize_t ret;

  ret = usrsock_response(msg, len, answer ? &done : NULL);
  if (ret < 0)
    {
      return (int)ret;
    }

  if ((size_t)ret != len || (answer && !done))
    {
      return -EIO;
    }

  return 0;
}

/* ANSWER: the request being handled (usrsock_request()), which usrsock
 * waits for; false for the final answer of a connection that was answered
 * "in progress" before (nina_usrsock_connected()).
 */

static int ack(uint32_t xid, int32_t result, uint16_t events,
               bool inprogress, bool answer)
{
  struct usrsock_message_req_ack_s ack;

  memset(&ack, 0, sizeof(ack));
  ack.head.msgid  = USRSOCK_MESSAGE_RESPONSE_ACK;
  ack.head.flags  = inprogress ? USRSOCK_MESSAGE_FLAG_REQ_IN_PROGRESS : 0;
  ack.head.events = events;
  ack.result      = result;
  ack.xid         = xid;
  return respond(&ack, sizeof(ack), answer);
}

static int send_ack(uint32_t xid, int32_t result, uint16_t events,
                    bool inprogress)
{
  return ack(xid, result, events, inprogress, true);
}

/* A data answer: VALUE (an address or an ioctl value, VALUELEN of its
 * VALUELEN_FULL bytes), then RESULT bytes of DATA.  A negative RESULT
 * carries neither.  The header and value go to usrsock in one message,
 * the data in a second (usrsock copies both into the caller's buffers).
 */

static int send_dack(uint32_t xid, int32_t result, uint16_t events,
                     const void *value, uint16_t valuelen,
                     uint16_t valuelen_full, const void *data)
{
  struct
  {
    struct usrsock_message_datareq_ack_s ack;
    uint8_t value[sizeof(struct sockaddr_in)];
  } msg;

  size_t len = sizeof(msg.ack);
  int ret;

  memset(&msg.ack, 0, sizeof(msg.ack));
  msg.ack.reqack.head.msgid  = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  msg.ack.reqack.head.events = events;
  msg.ack.reqack.result      = result;
  msg.ack.reqack.xid         = xid;

  if (result >= 0)
    {
      if (valuelen > valuelen_full)
        {
          valuelen = valuelen_full;
        }

      if (valuelen > sizeof(msg.value))
        {
          return -EINVAL;
        }

      msg.ack.valuelen          = valuelen;
      msg.ack.valuelen_nontrunc = valuelen_full;
      memcpy(msg.value, value, valuelen);
      len += valuelen;
    }

  ret = respond(&msg, len, true);
  if (ret == 0 && result > 0)
    {
      ret = respond(data, result, false);
    }

  return ret;
}

/* Requests *******************************************************************/

/* The address that follows a request header of HDRLEN bytes, ADDRLEN bytes
 * long, of which the first LEN bytes of the request were copied.  Copies
 * up to a struct sockaddr_in into *ADDR (what nina_sock.c reads of it).
 * Returns ADDR, NULL for no address, or -EINVAL in *ERR for a short copy.
 */

static const struct sockaddr *get_addr(size_t hdrlen, uint16_t addrlen,
                                       size_t len, struct sockaddr_in *addr,
                                       int *err)
{
  size_t n = addrlen < sizeof(*addr) ? addrlen : sizeof(*addr);

  *err = 0;
  if (addrlen == 0)
    {
      return NULL;
    }

  if (len < hdrlen + n)
    {
      *err = -EINVAL;
      return NULL;
    }

  memset(addr, 0, sizeof(*addr));
  memcpy(addr, g_request + hdrlen, n);
  return (const struct sockaddr *)addr;
}

static int handle_socket(const struct usrsock_request_socket_s *req)
{
  int ret;

  /* SOCK_CTRL sockets carry the network device ioctls of ifconfig, wapi
   * and netlib: NuttX's own socket layer handles them.  Other families are
   * NuttX's too (the Zephyr build offloads AF_INET only).
   */

  if (req->type == SOCK_CTRL ||
      (req->domain != AF_INET && req->domain != AF_INET6))
    {
      return send_ack(req->head.xid, -ENOTSUP, 0, false);
    }

  ret = nina_sock_create(req->domain, req->type, req->protocol);
  if (ret < 0)
    {
      return send_ack(req->head.xid, ret, 0, false);
    }

  /* The Zephyr build's poll() reports a datagram socket writable before
   * the module has it, and its sends never wait for room.
   */

  nina_poller_start();
  return send_ack(req->head.xid, ret,
                  report(ret, g_nina_sockets[ret].udp ?
                              NINA_SOCK_EV_WRITE : 0), false);
}

static int handle_close(const struct usrsock_request_close_s *req)
{
  nina_poller_cancel(req->usockid);
  return send_ack(req->head.xid, nina_sock_close(req->usockid), 0, false);
}

static int handle_connect(const struct usrsock_request_connect_s *req,
                          size_t len)
{
  struct sockaddr_in in;
  const struct sockaddr *addr;
  struct nina_sock *s;
  int ret;

  addr = get_addr(sizeof(*req), req->addrlen, len, &in, &ret);
  if (ret < 0)
    {
      return send_ack(req->head.xid, ret, 0, false);
    }

  ret = nina_sock_connect(req->usockid, addr, req->addrlen);
  if (ret < 0)
    {
      return send_ack(req->head.xid, ret, 0, false);
    }

  if (ret == 0)
    {
      /* Connected (a UDP socket at once): it has a firmware socket now,
       * which the thread polls for data.
       */

      ret = send_ack(req->head.xid, 0,
                     report(req->usockid, NINA_SOCK_EV_WRITE), false);
      if (!nina_sock_get(req->usockid)->caller_polls)
        {
          nina_poller_wake();
        }

      return ret;
    }

  /* In progress: nina_poller.c answers this request. */

  s = nina_sock_get(req->usockid);
  s->connect_xid = req->head.xid;
  ret = send_ack(req->head.xid, 0, 0, true);
  nina_poller_wake();
  return ret;
}

static int handle_bind(const struct usrsock_request_bind_s *req, size_t len)
{
  struct sockaddr_in in;
  const struct sockaddr *addr;
  int ret;

  addr = get_addr(sizeof(*req), req->addrlen, len, &in, &ret);
  if (ret < 0)
    {
      return send_ack(req->head.xid, ret, 0, false);
    }

  ret = nina_sock_bind(req->usockid, addr, req->addrlen);
  ret = send_ack(req->head.xid, ret, 0, false);
  nina_poller_wake();
  return ret;
}

static int handle_listen(const struct usrsock_request_listen_s *req)
{
  int ret;

  ret = send_ack(req->head.xid, nina_sock_listen(req->usockid,
                                                 req->backlog), 0, false);
  nina_poller_wake();
  return ret;
}

static int handle_accept(const struct usrsock_request_accept_s *req)
{
  struct nina_sock *listener = nina_sock_get(req->usockid);
  struct sockaddr_in peer;
  int16_t usockid;
  int ret;

  ret = nina_sock_accept(req->usockid, &peer);

  /* usrsock drops RECVFROM_AVAIL of the listening socket after a
   * connection or -EAGAIN (usrsock_accept.c, accept_event()); the thread
   * looks for the next connection.
   */

  if (listener != NULL && (ret >= 0 || ret == -EAGAIN))
    {
      listener->reported &= (uint8_t)~NINA_SOCK_EV_READ;
      nina_poller_wake();
    }

  if (ret < 0)
    {
      return send_dack(req->head.xid, ret, 0, NULL, 0, 0, NULL);
    }

  usockid = (int16_t)ret;
  ret = send_dack(req->head.xid, sizeof(usockid), 0, &peer,
                  req->max_addrlen < sizeof(peer) ?
                  req->max_addrlen : sizeof(peer),
                  sizeof(peer), &usockid);
  if (ret == 0)
    {
      /* The new socket is connected: writable, as after connect(). */

      nina_usrsock_report(usockid, NINA_SOCK_EV_WRITE);
    }

  return ret;
}

static int handle_setsockopt(const struct usrsock_request_setsockopt_s *req,
                             size_t len)
{
  const void *value = req + 1;

  if (len < sizeof(*req) + req->valuelen)
    {
      value = NULL;
    }

  return send_ack(req->head.xid,
                  nina_sock_setsockopt(req->usockid, req->level,
                                       req->option, value, req->valuelen),
                  0, false);
}

/* SENDTO: the request, the address (addrlen bytes, maybe none), then the
 * caller's data vectors (usrsock_sendmsg.c, do_sendto_request()).  One
 * vector is the Zephyr build's send()/sendto(), more its sendmsg().
 */

static int handle_sendto(const struct usrsock_request_sendto_s *req,
                         size_t len, FAR struct iovec *iov,
                         unsigned int iovcnt)
{
  struct sockaddr_in in;
  const struct sockaddr *addr;
  struct nina_sock *s;
  int16_t usockid = req->usockid;
  int32_t flags = req->flags;
  uint32_t xid = req->head.xid;
  bool retry = false;
  ssize_t ret;
  int err;

  addr = get_addr(sizeof(*req), req->addrlen, len, &in, &err);
  if (err < 0)
    {
      return send_ack(xid, err, 0, false);
    }

  if (iovcnt < 2 || iov[0].iov_len != sizeof(*req) ||
      iov[1].iov_len != req->addrlen)
    {
      return send_ack(xid, -EINVAL, 0, false);
    }

  if (iovcnt == 3)
    {
      ret = nina_sock_send(usockid, iov[2].iov_base, iov[2].iov_len, flags,
                           addr, req->addrlen, &retry);
    }
  else
    {
      ret = nina_sock_sendmsg(usockid, iov + 2, iovcnt - 2, addr != NULL,
                              flags, &retry);
    }

  s = nina_sock_get(usockid);
  if (ret >= 0 || s == NULL)
    {
      return send_ack(xid, ret, ret >= 0 ? report(usockid,
                                                  NINA_SOCK_EV_WRITE) : 0,
                      false);
    }

  if (ret != -EAGAIN)
    {
      return send_ack(xid, ret, 0, false);
    }

  if (retry)
    {
      /* The call waits: the next pass after NINA_SEND_RETRY_MS. */

      s->reported &= (uint8_t)~NINA_SOCK_EV_WRITE;
      ret = send_ack(xid, -EAGAIN, 0, false);
      nina_poller_retry(usockid);
      return ret;
    }

  /* A call that does not wait, the deadline of a waiting call, or a
   * datagram: EAGAIN now.  The next call tries at once, as there.
   */

  return send_ack(xid, NINA_USRSOCK_EAGAIN_FINAL,
                  report(usockid, NINA_SOCK_EV_WRITE), false);
}

static int handle_recvfrom(const struct usrsock_request_recvfrom_s *req)
{
  struct sockaddr_in from;
  struct nina_sock *s;
  int16_t usockid = req->usockid;
  uint16_t max_addrlen = req->max_addrlen;
  uint32_t xid = req->head.xid;
  uint16_t valuelen;
  ssize_t ret;

  /* The address is wanted when there is room for it (recvfrom()). */

  memset(&from, 0, sizeof(from));
  ret = nina_sock_recv(usockid, g_receive, req->max_buflen, req->flags,
                       max_addrlen > 0 ? &from : NULL);
  s = nina_sock_get(usockid);
  if (ret == -EAGAIN && s != NULL)
    {
      /* Nothing there: the call waits for RECVFROM_AVAIL (or returns
       * -EAGAIN), which nina_poller.c reports from its polls.
       */

      s->reported &= (uint8_t)~NINA_SOCK_EV_READ;
      ret = send_dack(xid, -EAGAIN, 0, NULL, 0, 0, NULL);
      if (!s->caller_polls)
        {
          nina_poller_kick();
        }

      return ret;
    }

  if (ret < 0 || s == NULL)
    {
      return send_dack(xid, ret < 0 ? ret : -EBADF, 0, NULL, 0, 0, NULL);
    }

  /* Whether more is there is for the next poll, which comes now (for a
   * socket whose callers poll: with their next call): the Zephyr build's
   * next receive polls first, too.
   */

  s->reported &= (uint8_t)~NINA_SOCK_EV_READ;
  valuelen = max_addrlen < sizeof(from) ? max_addrlen : sizeof(from);
  ret = send_dack(xid, ret, 0, &from, s->udp ? valuelen : 0,
                  s->udp ? sizeof(from) : 0, g_receive);
  if (!s->caller_polls)
    {
      nina_poller_wake();
    }

  return ret;
}

static int handle_getsockopt(const struct usrsock_request_getsockopt_s *req)
{
  uint32_t xid = req->head.xid;
  int value;

  /* NINA_SO_POLL: one pass of the Zephyr build's poll() for the socket,
   * now, in this thread (nina_sock.h).  No events: the caller reads with
   * FIONREAD first, which raises RECVFROM_AVAIL.
   */

  if (req->level != NINA_SOL || req->option != NINA_SO_POLL)
    {
      return send_dack(xid, -EOPNOTSUPP, 0, NULL, 0, 0, NULL);
    }

  if (req->max_valuelen < sizeof(value))
    {
      return send_dack(xid, -EINVAL, 0, NULL, 0, 0, NULL);
    }

  value = nina_sock_poll_revents(req->usockid);
  if (value < 0)
    {
      return send_dack(xid, value, 0, NULL, 0, 0, NULL);
    }

  return send_dack(xid, 0, 0, &value, sizeof(value), sizeof(value), NULL);
}

static int handle_ioctl(const struct usrsock_request_ioctl_s *req)
{
  int16_t usockid = req->usockid;
  uint32_t xid = req->head.xid;
  int value;

  /* FIONREAD: a poll of the module now, in this thread (nina_sock.h).
   * When data is there, RECVFROM_AVAIL comes with the answer, so that a
   * receive with MSG_DONTWAIT asks the module for it.
   */

  if (req->cmd == FIONREAD)
    {
      value = nina_sock_fionread(usockid);
      if (value < 0)
        {
          return send_dack(xid, value, 0, NULL, 0, 0, NULL);
        }

      return send_dack(xid, 0,
                       value > 0 ? report(usockid, NINA_SOCK_EV_READ) : 0,
                       &value, sizeof(value), sizeof(value), NULL);
    }

  /* Everything else goes on to NuttX's own handling of the command (the
   * network device's, for SIOC*): usrsock does that only for -ENOTTY
   * (netdev_ioctl.c).
   */

  return send_dack(xid, -ENOTTY, 0, NULL, 0, 0, NULL);
}

/* Header sizes by request, for the length check. */

static const uint8_t g_reqlen[USRSOCK_REQUEST__MAX] =
{
  [USRSOCK_REQUEST_SOCKET]      = sizeof(struct usrsock_request_socket_s),
  [USRSOCK_REQUEST_CLOSE]       = sizeof(struct usrsock_request_close_s),
  [USRSOCK_REQUEST_CONNECT]     = sizeof(struct usrsock_request_connect_s),
  [USRSOCK_REQUEST_SENDTO]      = sizeof(struct usrsock_request_sendto_s),
  [USRSOCK_REQUEST_RECVFROM]    = sizeof(struct usrsock_request_recvfrom_s),
  [USRSOCK_REQUEST_SETSOCKOPT]  = sizeof(struct usrsock_request_setsockopt_s),
  [USRSOCK_REQUEST_GETSOCKOPT]  = sizeof(struct usrsock_request_getsockopt_s),
  [USRSOCK_REQUEST_GETSOCKNAME] =
    sizeof(struct usrsock_request_getsockname_s),
  [USRSOCK_REQUEST_GETPEERNAME] =
    sizeof(struct usrsock_request_getpeername_s),
  [USRSOCK_REQUEST_BIND]        = sizeof(struct usrsock_request_bind_s),
  [USRSOCK_REQUEST_LISTEN]      = sizeof(struct usrsock_request_listen_s),
  [USRSOCK_REQUEST_ACCEPT]      = sizeof(struct usrsock_request_accept_s),
  [USRSOCK_REQUEST_IOCTL]       = sizeof(struct usrsock_request_ioctl_s),
  [USRSOCK_REQUEST_SHUTDOWN]    = sizeof(struct usrsock_request_shutdown_s),
};

/* Public Functions ***********************************************************/

void nina_usrsock_report(int usockid, uint8_t ev)
{
  struct usrsock_message_socket_event_s event;

  memset(&event, 0, sizeof(event));
  event.head.msgid  = USRSOCK_MESSAGE_SOCKET_EVENT;
  event.head.flags  = USRSOCK_MESSAGE_FLAG_EVENT;
  event.head.events = report(usockid, ev);
  event.usockid     = usockid;
  respond(&event, sizeof(event), false);
}

void nina_usrsock_connected(int usockid, int result)
{
  struct nina_sock *s = nina_sock_get(usockid);
  uint32_t xid = s != NULL ? s->connect_xid : 0;

  if (s != NULL)
    {
      s->connect_xid = 0;
    }

  if (result == 0)
    {
      /* usrsock completes the connection on SENDTO_READY
       * (usrsock_event.c), with the result of the "in progress" answer.
       */

      nina_usrsock_report(usockid, NINA_SOCK_EV_WRITE);
    }
  else if (xid != 0)
    {
      /* The final answer to the connect request: connect() returns the
       * Zephyr build's errno (an event would always give ECONNREFUSED,
       * usrsock_connect.c).
       */

      ack(xid, result, 0, false, false);
    }
  else
    {
      /* Another request on the socket took the place of the connect
       * request in usrsock, which would drop the answer: report the
       * failure as the connection closed.
       */

      nina_usrsock_report(usockid, NINA_SOCK_EV_CLOSED);
    }
}

/* Called by usrsock_initialize() very early (net_initialize()).  The
 * module is reset by its first command and the thread is started by the
 * first socket, so there is nothing to do here.
 */

void usrsock_register(void)
{
}

int usrsock_request(FAR struct iovec *iov, unsigned int iovcnt)
{
  const struct usrsock_request_common_s *common =
    (const struct usrsock_request_common_s *)g_request;
  struct nina_sock *s;
  ssize_t len;
  bool done;

  len = usrsock_iovec_get(g_request, sizeof(g_request), iov, iovcnt, 0,
                          &done);
  if (len < (ssize_t)sizeof(*common) ||
      common->reqid >= USRSOCK_REQUEST__MAX ||
      len < g_reqlen[common->reqid])
    {
      return -EINVAL;
    }

  /* A connection in progress whose socket gets another request: usrsock
   * now waits for that request's answer instead (one per connection).
   */

  if (common->reqid != USRSOCK_REQUEST_SOCKET &&
      common->reqid != USRSOCK_REQUEST_CLOSE)
    {
      s = nina_sock_get(((const struct usrsock_request_close_s *)
                         g_request)->usockid);
      if (s != NULL && s->connecting)
        {
          s->connect_xid = 0;
        }
    }

  switch (common->reqid)
    {
      case USRSOCK_REQUEST_SOCKET:
        return handle_socket((const void *)g_request);

      case USRSOCK_REQUEST_CLOSE:
        return handle_close((const void *)g_request);

      case USRSOCK_REQUEST_CONNECT:
        return handle_connect((const void *)g_request, len);

      case USRSOCK_REQUEST_BIND:
        return handle_bind((const void *)g_request, len);

      case USRSOCK_REQUEST_LISTEN:
        return handle_listen((const void *)g_request);

      case USRSOCK_REQUEST_ACCEPT:
        return handle_accept((const void *)g_request);

      case USRSOCK_REQUEST_SETSOCKOPT:
        return handle_setsockopt((const void *)g_request, len);

      case USRSOCK_REQUEST_IOCTL:
        return handle_ioctl((const void *)g_request);

      case USRSOCK_REQUEST_GETSOCKOPT:
        return handle_getsockopt((const void *)g_request);

      case USRSOCK_REQUEST_GETSOCKNAME:
      case USRSOCK_REQUEST_GETPEERNAME:
        return send_dack(common->xid, -EOPNOTSUPP, 0, NULL, 0, 0, NULL);

      case USRSOCK_REQUEST_SHUTDOWN:
        return send_ack(common->xid, -EOPNOTSUPP, 0, false);

      case USRSOCK_REQUEST_SENDTO:
        return handle_sendto((const void *)g_request, len, iov, iovcnt);

      case USRSOCK_REQUEST_RECVFROM:
        return handle_recvfrom((const void *)g_request);

      default:
        return -EINVAL;
    }
}
