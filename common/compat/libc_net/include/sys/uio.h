/* SPDX-License-Identifier: MIT */
/* struct iovec of POSIX's <sys/uio.h>, which newlib does not have: the
 * vectors of struct msghdr (<sys/socket.h>) for sendmsg(), which the MQTT
 * client uses (common/imu/mqtt_min.c) and the NINA core joins into one
 * command (common/drivers/nina/nina_sock.c).  No readv()/writev().
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#ifndef COMMON_COMPAT_LIBC_NET_INCLUDE_SYS_UIO_H
#define COMMON_COMPAT_LIBC_NET_INCLUDE_SYS_UIO_H

#include <stddef.h>

struct iovec {
	void *iov_base;
	size_t iov_len;
};

#endif /* COMMON_COMPAT_LIBC_NET_INCLUDE_SYS_UIO_H */
