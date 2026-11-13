/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA socket table and the socket commands of the Zephyr build's
 * drivers/nina/nina_socket.c, without the Zephyr socket layer around them
 * (nina_sock.c).  nina_usrsock.c turns NuttX's usrsock requests into these
 * calls and nina_poller.c turns the module's socket state into usrsock
 * events; the host test (nuttx/tests/nina_sock.c) runs them on a fake
 * nina_request().
 *
 * Sockets are numbered 0..NINA_SOCK_MAX-1 (the usrsock "usockid").  Every
 * function takes the number, returns 0 (or the value it describes) on
 * success and a negative errno on failure, where the Zephyr build sets
 * errno and returns -1: the same errno names.
 *
 * The caller holds the table lock.  On NuttX that is the usrsock lock
 * (usrsock_lock()), which every usrsock request holds and nina_poller.c
 * takes; it stands for the Zephyr build's socket_lock and for the per-socket
 * lock its socket layer takes around each call.  nina_request() takes the
 * NINA lock inside it, so the order is always: table lock, then NINA lock.
 */

#ifndef __NUTTX_DRIVERS_NINA_NINA_SOCK_H
#define __NUTTX_DRIVERS_NINA_NINA_SOCK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>

/* nina_socket.c:17-26.  The firmware's lwIP is built with
 * CONFIG_LWIP_MAX_SOCKETS=10.
 */

#define NINA_SOCK_MAX                10    /* MAX_NINA_SOCKETS */
#define NINA_MAX_IO                  512
#define NINA_NO_SOCKET               UINT8_MAX
#define NINA_SEND_WAIT_MS            10000
#define NINA_CONNECT_WAIT_MS         10000
#define NINA_SEND_STATE_INTERVAL_MS  100
#define NINA_POLL_INTERVAL_MS        20

/* CONFIG_NINA_TCP_MAX_IO of the Zephyr build. */

#ifdef CONFIG_NANO_NINA_TCP_MAX_IO
#  define NINA_TCP_MAX_IO            CONFIG_NANO_NINA_TCP_MAX_IO
#else
#  define NINA_TCP_MAX_IO            3584
#endif

/* CONFIG_NINA_SEND_RETRY_MS of the Zephyr build: the wait before a TCP send
 * that found no room is tried again.
 */

#ifdef CONFIG_NANO_NINA_SEND_RETRY_MS
#  define NINA_SEND_RETRY_MS         CONFIG_NANO_NINA_SEND_RETRY_MS
#else
#  define NINA_SEND_RETRY_MS         1
#endif

/* The firmware's socket commands (nina_socket.c:28-40). */

#define NINA_CMD_SOCKET              0x70
#define NINA_CMD_CLOSE               0x71
#define NINA_CMD_BIND                0x73
#define NINA_CMD_LISTEN              0x74
#define NINA_CMD_ACCEPT              0x75
#define NINA_CMD_CONNECT             0x76
#define NINA_CMD_SEND                0x77
#define NINA_CMD_RECV                0x78
#define NINA_CMD_SENDTO              0x79
#define NINA_CMD_RECVFROM            0x7a
#define NINA_CMD_IOCTL               0x7b
#define NINA_CMD_POLL                0x7c
#define NINA_CMD_GETSOCKOPT          0x7e

/* The state byte of NINA_CMD_POLL (nina_socket.c:42-45). */

#define NINA_POLL_READ               0x01
#define NINA_POLL_WRITE              0x02
#define NINA_POLL_ERROR              0x04
#define NINA_POLL_FAIL               0x80

/* What usrsock has been told about a socket and still holds, as the flags
 * of its connection (USRSOCK_EVENT_RECVFROM_AVAIL, _SENDTO_READY,
 * _REMOTE_CLOSED).  nina_poller.c only asks the module about a socket while
 * something it could report is not held yet.
 */

#define NINA_SOCK_EV_READ            0x01
#define NINA_SOCK_EV_WRITE           0x02
#define NINA_SOCK_EV_CLOSED          0x04

/* nina_socket.c:47-57, without "nonblocking" and "receive_timeout_ms":
 * NuttX's socket layer keeps O_NONBLOCK and SO_RCVTIMEO itself (usrsock
 * never passes them on; whether a send or receive call waits reaches the
 * driver as MSG_DONTWAIT, see nuttx/patches/nuttx/0005).  The rest is
 * NuttX's: a connection in progress (the Zephyr build waits for it inside
 * connect(); here nina_poller.c checks it every NINA_POLL_INTERVAL_MS), the
 * events reported, the state of a send that waits for room (the local
 * variables of the Zephyr build's do_send() loop, kept between the usrsock
 * requests that make its passes), and what the last poll said about data.
 */

struct nina_sock
{
  bool used;
  bool connected;
  bool udp;
  bool bound;
  bool listening;
  uint8_t id;
  struct sockaddr_in peer;

  bool connecting;
  int64_t connect_deadline;   /* nina_os_uptime_ms() */
  uint32_t connect_xid;       /* usrsock request to answer, 0 if none */
  uint8_t reported;           /* NINA_SOCK_EV_* */

  /* A waiting TCP send between its passes: do_send()'s "deadline" and
   * "next_state_check", and which call it is (the data's address and
   * length: usrsock repeats the same request for each pass).
   */

  bool send_waiting;
  int64_t send_deadline;
  int64_t send_next_state_check;
  const void *send_key;
  size_t send_key_len;

  /* The module's last poll of this socket found data (or, for TCP, the end
   * of the stream).  The next receive takes it for its first poll: nothing
   * else takes the data away in between.
   */

  bool read_hint;
};

extern struct nina_sock g_nina_sockets[NINA_SOCK_MAX];

/* Returns the socket of a usrsock id, or NULL if the number is out of range
 * or the socket is not in use.
 */

struct nina_sock *nina_sock_get(int usockid);

/* nina_socket.c:835-842: AF_INET with SOCK_STREAM (protocol 0 or TCP) or
 * SOCK_DGRAM (0 or UDP).
 */

bool nina_sock_supported(int family, int type, int proto);

/* nina_socket_create() (:844-880) without the file descriptor: takes a free
 * entry.  The firmware socket is created on first use.  Returns the number,
 * -EAFNOSUPPORT or -EMFILE.
 */

int nina_sock_create(int family, int type, int proto);

/* do_bind() (:175-210). */

int nina_sock_bind(int usockid, const struct sockaddr *addr,
                   size_t addrlen);

/* do_listen() (:212-232). */

int nina_sock_listen(int usockid, int backlog);

/* do_accept() (:234-292) without the file descriptor: returns the new
 * socket's number and the peer in *peer.
 */

int nina_sock_accept(int usockid, struct sockaddr_in *peer);

/* do_connect() (:294-372) in two parts.  nina_sock_connect() validates,
 * creates the firmware socket, sends the connect command and makes the
 * first check (the first pass of the Zephyr build's wait loop, which polls
 * before it sleeps).  Returns 0 when connected (a UDP socket: at once),
 * 1 while the TCP handshake is in progress, or a negative errno (the
 * firmware socket is then closed, as there).  nina_sock_connect_check() is
 * one later pass of the loop, made every NINA_POLL_INTERVAL_MS: same
 * results.  NOW is nina_os_uptime_ms().
 */

int nina_sock_connect(int usockid, const struct sockaddr *addr,
                      size_t addrlen);
int nina_sock_connect_check(int usockid, int64_t now);

/* do_close() (:661-682).  Frees the entry even when the module fails. */

int nina_sock_close(int usockid);

/* do_setsockopt() (:764-791) for what reaches it on NuttX: TCP_NODELAY is
 * accepted without effect, everything else is -ENOPROTOOPT.  NuttX keeps
 * SO_RCVTIMEO itself (see struct nina_sock).
 */

int nina_sock_setsockopt(int usockid, int level, int option,
                         const void *value, size_t valuelen);

/* do_send() (:421-490) and udp_send() (:374-419), one pass of the wait
 * loop.  BUF and LEN are the data, TO and TOLEN the address of a datagram
 * (NULL, 0 for none), FLAGS the call's flags (MSG_DONTWAIT when the call
 * does not wait: a non-blocking socket or the flag).  Returns the bytes
 * sent, or a negative errno.  -EAGAIN with *RETRY set: nothing was sent and
 * the call waits; it is to make the next pass after NINA_SEND_RETRY_MS
 * (the k_sleep() of the loop) with the same data.  -EAGAIN with *RETRY
 * clear is the final answer (a non-blocking call, the NINA_SEND_WAIT_MS
 * deadline, a datagram the module did not take).
 */

ssize_t nina_sock_send(int usockid, const void *buf, size_t len, int flags,
                       const struct sockaddr *to, size_t tolen,
                       bool *retry);

/* do_sendmsg() (:492-529): the vectors joined into one buffer of up to
 * NINA_TCP_MAX_IO bytes, sent as one NINA command by nina_sock_send().
 * HAS_NAME: the message has an address (msg_name).  Same results.
 */

ssize_t nina_sock_sendmsg(int usockid, const struct iovec *iov, size_t iovcnt,
                          bool has_name, int flags, bool *retry);

/* do_recv() (:608-659) and udp_recv() (:536-606), one pass of the wait
 * loop: -EAGAIN when the module has nothing (the caller waits for the
 * socket's RECVFROM_AVAIL).  BUF holds at least MIN(LEN, NINA_MAX_IO)
 * bytes.  FROM (NULL: not wanted) gets the sender of a datagram; a TCP
 * socket refuses it, as there.  Returns the bytes received; 0 is the end
 * of a TCP stream or an empty datagram.
 */

ssize_t nina_sock_recv(int usockid, void *buf, size_t len, int flags,
                       struct sockaddr_in *from);

/* ioctl(FIONREAD), which the Zephyr build does not have: one poll of the
 * module now, in the caller's thread (the time synchronization's 1 ms
 * receive polling, Zephyr build src/time_sync.c exchange(), which polls
 * through recv(MSG_DONTWAIT)).  Returns 1 when data (or the end of a TCP
 * stream) is there, 0 when not, or a negative errno.  The module does not
 * tell how many bytes.
 */

int nina_sock_fionread(int usockid);

/* poll_state() (:80-89): the module's state byte of the socket's firmware
 * socket.  Records what it says about data (read_hint).
 */

int nina_sock_poll_state(struct nina_sock *s, uint8_t *state);

/* What nina_poller.c should look at for this socket now: a connection in
 * progress, or the NINA_SOCK_EV_* bits it could still report.  0: nothing,
 * the module is not asked.  A TCP send that waits for room is not polled
 * for WRITE: its passes are timed (NINA_SEND_RETRY_MS), as there.
 */

#define NINA_SOCK_WATCH_CONNECT      0x80

uint8_t nina_sock_watch(const struct nina_sock *s);

/* The NINA_SOCK_EV_* bits to report after nina_sock_poll_state() returned
 * POLL_RESULT and STATE, among those usrsock does not hold yet: READ (a
 * listening socket: a connection to accept), WRITE, and for a connected TCP
 * socket CLOSED on the error flag or a failed poll (the Zephyr build's
 * poll() fails then, and its callers give the connection up).  A UDP
 * socket's error is not reported, as the Zephyr build's datagram calls
 * carry on after it; neither is a listening socket's failed poll (it is
 * polled again).
 */

uint8_t nina_sock_events(const struct nina_sock *s, int poll_result,
                         uint8_t state);

#endif /* __NUTTX_DRIVERS_NINA_NINA_SOCK_H */
