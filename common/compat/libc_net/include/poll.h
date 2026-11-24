/* SPDX-License-Identifier: MIT */
/* The types and event bits of POSIX's <poll.h>, which newlib does not
 * have.  There is no poll() here: the build's sockets are polled with
 * zsock_poll() of the Zephyr compatibility layer (<zephyr/net/socket.h>),
 * whose ZSOCK_POLL* are these (its struct zsock_pollfd is Zephyr's struct
 * zvfs_pollfd, of the same layout).  The values are Linux's, Zephyr's and NuttX's.
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#ifndef COMMON_COMPAT_LIBC_NET_INCLUDE_POLL_H
#define COMMON_COMPAT_LIBC_NET_INCLUDE_POLL_H

typedef unsigned int nfds_t;

struct pollfd {
	int fd;
	short events;
	short revents;
};

#define POLLIN   0x01
#define POLLPRI  0x02
#define POLLOUT  0x04
#define POLLERR  0x08
#define POLLHUP  0x10
#define POLLNVAL 0x20

#endif /* COMMON_COMPAT_LIBC_NET_INCLUDE_POLL_H */
