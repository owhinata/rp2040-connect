/* SPDX-License-Identifier: Apache-2.0 */
/* Host build of Zephyr's own MQTT library: Zephyr's socket names
 * (<zephyr/net/socket.h>, net_ip.h) on the host's types, and the zsock_*()
 * calls of its TCP transport on the port's socket layer (mqtt_min.h), which
 * the test fakes.  So Zephyr's library and the port's copy run on the same
 * fake socket.
 */
#ifndef TEST_MQTT_REF_SOCKET_H
#define TEST_MQTT_REF_SOCKET_H
#include <errno.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include "mqtt_min.h"

#define net_sockaddr sockaddr
#define net_sockaddr_in sockaddr_in
#define net_sockaddr_in6 sockaddr_in6
#define net_msghdr msghdr
#define net_iovec iovec
#define net_ifreq ifreq
#define NET_AF_INET AF_INET
#define NET_SOCK_STREAM SOCK_STREAM
#define NET_IPPROTO_TCP IPPROTO_TCP
#define ZSOCK_MSG_DONTWAIT MSG_DONTWAIT
#define ZSOCK_SOL_SOCKET SOL_SOCKET
#define ZSOCK_SO_BINDTODEVICE SO_BINDTODEVICE

#define zsock_socket mqtt_min_sock_socket
#define zsock_connect mqtt_min_sock_connect
#define zsock_send mqtt_min_sock_send
#define zsock_sendmsg mqtt_min_sock_sendmsg
#define zsock_recv mqtt_min_sock_recv
#define zsock_close mqtt_min_sock_close

/* Binding to an interface is not tested (the port does not support it). */
static inline int zsock_setsockopt(int sock, int level, int name, const void *value,
				   socklen_t length)
{
	(void)sock;
	(void)level;
	(void)name;
	(void)value;
	(void)length;
	errno = ENOTSUP;
	return -1;
}
#endif
