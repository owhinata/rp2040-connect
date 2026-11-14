/* SPDX-License-Identifier: Apache-2.0 */
/* The socket layer of the minimal MQTT client (mqtt_min.h): the zsock_*()
 * calls that Zephyr's TCP transport (subsys/net/lib/mqtt/
 * mqtt_transport_socket_tcp.c) makes, with the same arguments, from the
 * Zephyr compatibility layer (<zephyr/net/socket.h>): NuttX's sockets,
 * the NINA's through usrsock, on the board.  zsock_recv() with
 * ZSOCK_MSG_DONTWAIT, which the client's reads use, asks the module first
 * (ioctl(FIONREAD)), as the Zephyr build's recv() does with its CMD_POLL:
 * NuttX's usrsock would answer such a call from the RECVFROM_AVAIL it
 * holds, which the NINA's event thread refreshes only every 20 ms, and a
 * packet that arrives in pieces (CONNACK: 2 bytes, then 2) would stop
 * after the first.  The mqtt_min host test links a fake of these functions
 * instead; the imu_raw_mqtt host test builds this file on fake sockets.
 */

#include <nuttx/config.h> /* NuttX's headers before the compatibility layer's */
#include <sys/types.h>
#include <sys/socket.h>

#include <zephyr/net/socket.h>

#include "mqtt_min.h"

int mqtt_min_sock_socket(int family, int type, int protocol)
{
  return zsock_socket(family, type, protocol);
}

int mqtt_min_sock_connect(int sock, const struct sockaddr *addr,
                          socklen_t addrlen)
{
  return zsock_connect(sock, addr, addrlen);
}

ssize_t mqtt_min_sock_send(int sock, const void *buf, size_t len, int flags)
{
  return zsock_send(sock, buf, len, flags);
}

ssize_t mqtt_min_sock_sendmsg(int sock, const struct msghdr *msg, int flags)
{
  return zsock_sendmsg(sock, msg, flags);
}

ssize_t mqtt_min_sock_recv(int sock, void *buf, size_t len, int flags)
{
  /* mqtt_min.c's MSG_DONTWAIT is ZSOCK_MSG_DONTWAIT. */

  return zsock_recv(sock, buf, len, flags);
}

int mqtt_min_sock_close(int sock)
{
  return zsock_close(sock);
}
