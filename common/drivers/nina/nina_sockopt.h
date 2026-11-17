/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA socket driver's own socket options (nuttx/drivers/nina), for the
 * Zephyr compatibility layer's zsock_*() calls (<zephyr/net/socket.h>).
 *
 * The Zephyr build's NINA sockets (drivers/nina/nina_socket.c) ask the
 * module about a socket only from the thread that calls recv() or poll():
 * recv() polls the module (CMD_POLL) on every call, poll() polls each
 * socket at once and then every NINA_POLL_INTERVAL_MS until one is ready
 * or the timeout ends (do_poll()).  NuttX's usrsock instead waits for
 * events, which the driver's event thread (nina_event, nina_poller.c)
 * makes by polling the sockets every NINA_POLL_INTERVAL_MS.  These options
 * let the layer make the Zephyr build's polls itself:
 *
 * - NINA_SO_CALLER_POLLS (setsockopt, an int, 1 or 0): 1 marks the socket
 *   as asked about by its callers only.  The event thread then leaves it
 *   alone, except to finish a connection in progress (connect() waits for
 *   its events) and to time the next pass of a send that waits for room
 *   (NINA_SEND_RETRY_MS; the pass itself runs in the caller's thread).  A
 *   receive that waits on such a socket gets no event from the driver: the
 *   layer's zsock_recv() polls instead.  zsock_socket() sets it.
 * - NINA_SO_POLL (getsockopt, an int): one CMD_POLL of the socket now, in
 *   the calling thread, answered as the Zephyr build's do_poll() answers
 *   for a socket polled for both ZSOCK_POLLIN and ZSOCK_POLLOUT: POLLIN
 *   (data, or the end of a TCP stream), POLLOUT, POLLERR (the module's
 *   POLL_ERROR), or POLLHUP alone for a TCP socket the module does not have
 *   yet (POLLOUT alone for such a UDP socket).  A failed poll of the module
 *   fails the call with its errno.  Any NINA socket, marked or not.
 *
 * The level is the driver's own; NuttX passes options it does not know to
 * the socket's usrsock daemon.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_NINA_SOCKOPT_H
#define __NUTTX_DRIVERS_INCLUDE_NINA_SOCKOPT_H

#define NINA_SOL                     0x4e49 /* "NI" */
#define NINA_SO_CALLER_POLLS         1
#define NINA_SO_POLL                 2

/* The Zephyr build's NINA_POLL_INTERVAL_MS (nina_socket.c:26): how often a
 * waiting poll() or receive asks the module again.
 */

#define NINA_POLL_INTERVAL_MS        20

#endif /* __NUTTX_DRIVERS_INCLUDE_NINA_SOCKOPT_H */
