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

#endif /* __NUTTX_DRIVERS_NINA_NINA_USRSOCK_H */
