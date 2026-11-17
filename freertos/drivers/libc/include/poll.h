/* SPDX-License-Identifier: MIT */
/* The types and event bits of POSIX's <poll.h>, which newlib does not
 * have.  There is no poll() here: the build's sockets are polled with
 * zsock_poll() of the Zephyr compatibility layer (<zephyr/net/socket.h>,
 * freertos/drivers/nina/nina_socket.c), whose struct zsock_pollfd and
 * ZSOCK_POLL* are these.  The values are Linux's, Zephyr's and NuttX's.
 * Part of newlib's glue of the FreeRTOS build (freertos/drivers/libc).
 */

#ifndef FREERTOS_DRIVERS_LIBC_POLL_H
#define FREERTOS_DRIVERS_LIBC_POLL_H

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

#endif /* FREERTOS_DRIVERS_LIBC_POLL_H */
