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
 *   GETSOCKOPT, GETSOCKNAME, no vtable entry:         -EOPNOTSUPP
 *   GETPEERNAME, SHUTDOWN    EOPNOTSUPP
 *   IOCTL                    do_ioctl()               -ENOTTY (NuttX's own
 *                                                     handling); FIONREAD T15
 *   SENDTO, RECVFROM         do_send(), do_recv()     T15: -ENOSYS for now
 *
 * A TCP connect is answered "in progress" when the module has not finished
 * the handshake at the first check; nina_poller.c checks it every
 * NINA_POLL_INTERVAL_MS until NINA_CONNECT_WAIT_MS, as the Zephyr build's
 * connect() does in its caller, and answers the request then.
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

#if CONFIG_NET_USRSOCK_PREALLOC_CONNS + CONFIG_NET_USRSOCK_ALLOC_CONNS == 0
#  error usrsock needs connections
#endif

/* The request with the start of what follows it (an address, an option
 * value).  Requests are handled one at a time (usrsock's request lock).
 * Data (T15) is read from the request's iovec, not copied here.  The
 * request structures are packed; what follows them is copied out with
 * memcpy() (get_addr()) before it is used: the Cortex-M0+ faults on
 * unaligned word access.
 */

#define NINA_USRSOCK_REQBUF 64

static uint8_t g_request[NINA_USRSOCK_REQBUF] aligned_data(4);

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

/* A data answer: VALUE (an address, VALUELEN of its VALUELEN_FULL bytes),
 * then RESULT bytes of DATA.  A negative RESULT carries neither.
 */

static int send_dack(uint32_t xid, int32_t result, uint16_t events,
                     const void *value, uint16_t valuelen,
                     uint16_t valuelen_full, const void *data)
{
  struct
  {
    struct usrsock_message_datareq_ack_s ack;
    uint8_t payload[sizeof(struct sockaddr_in) + sizeof(int16_t)];
  } msg;

  size_t len = sizeof(msg.ack);

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

      if (valuelen + (size_t)result > sizeof(msg.payload))
        {
          return -EINVAL;
        }

      msg.ack.valuelen          = valuelen;
      msg.ack.valuelen_nontrunc = valuelen_full;
      memcpy(msg.payload, value, valuelen);
      memcpy(msg.payload + valuelen, data, result);
      len += valuelen + result;
    }

  return respond(&msg, len, true);
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
      nina_poller_wake();
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

static int handle_ioctl(const struct usrsock_request_ioctl_s *req)
{
  /* FIONREAD: T15.  Everything else goes on to NuttX's own handling of
   * the command (the network device's, for SIOC*): usrsock does that only
   * for -ENOTTY (netdev_ioctl.c).
   */

  return send_dack(req->head.xid, req->cmd == FIONREAD ? -ENOSYS : -ENOTTY,
                   0, NULL, 0, 0, NULL);
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
      case USRSOCK_REQUEST_GETSOCKNAME:
      case USRSOCK_REQUEST_GETPEERNAME:
        return send_dack(common->xid, -EOPNOTSUPP, 0, NULL, 0, 0, NULL);

      case USRSOCK_REQUEST_SHUTDOWN:
        return send_ack(common->xid, -EOPNOTSUPP, 0, false);

      case USRSOCK_REQUEST_SENDTO:

        /* T15 (nina_socket.c do_send(), udp_send(), do_sendmsg()). */

        return send_ack(common->xid, -ENOSYS, 0, false);

      case USRSOCK_REQUEST_RECVFROM:

        /* T15 (nina_socket.c do_recv(), udp_recv()). */

        return send_dack(common->xid, -ENOSYS, 0, NULL, 0, 0, NULL);

      default:
        return -EINVAL;
    }
}
