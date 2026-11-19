/* SPDX-License-Identifier: MIT */
/* <poll.h> for the shared NINA core on Zephyr (see sys/socket.h here): the
 * poll bits are Zephyr's ZSOCK_POLL* (the names of Zephyr's
 * include/zephyr/posix/poll.h).
 */

#ifndef NINA_POSIX_POLL_H_
#define NINA_POSIX_POLL_H_

#include <sys/socket.h>

#define POLLIN   ZSOCK_POLLIN
#define POLLOUT  ZSOCK_POLLOUT
#define POLLERR  ZSOCK_POLLERR
#define POLLHUP  ZSOCK_POLLHUP
#define POLLNVAL ZSOCK_POLLNVAL

#endif /* NINA_POSIX_POLL_H_ */
