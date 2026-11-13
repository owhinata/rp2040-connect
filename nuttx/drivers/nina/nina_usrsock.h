/* SPDX-License-Identifier: Apache-2.0 */
/* Between the usrsock requests (nina_usrsock.c) and the thread that turns
 * the module's socket state into usrsock events (nina_poller.c).  Every
 * function is called with the usrsock lock held (nina_sock.h).
 */

#ifndef __NUTTX_DRIVERS_NINA_NINA_USRSOCK_H
#define __NUTTX_DRIVERS_NINA_NINA_USRSOCK_H

#include <stdint.h>

/* Tells usrsock about NINA_SOCK_EV_* bits of a socket and records them as
 * held (struct nina_sock "reported").
 */

void nina_usrsock_report(int usockid, uint8_t ev);

/* Ends a connection in progress with RESULT (0 or a negative errno, from
 * nina_sock_connect_check()): the answer to the connect request.
 */

void nina_usrsock_connected(int usockid, int result);

/* Starts the thread (once; the first socket request does it) and makes it
 * look at the sockets now rather than after its interval.
 */

int nina_poller_start(void);
void nina_poller_wake(void);

/* Makes the thread look at the sockets within NINA_POLL_INTERVAL_MS: a
 * receive found nothing (the Zephyr build's receive loop and poll() check
 * again after that interval).
 */

void nina_poller_kick(void);

/* The next pass of a TCP send that waits for room: SENDTO_READY for the
 * socket after NINA_SEND_RETRY_MS, if the send still waits then.
 * nina_poller_cancel() drops the timer (the socket is closed).
 */

void nina_poller_retry(int usockid);
void nina_poller_cancel(int usockid);

#endif /* __NUTTX_DRIVERS_NINA_NINA_USRSOCK_H */
