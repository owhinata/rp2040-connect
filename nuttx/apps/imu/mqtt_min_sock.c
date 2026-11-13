/* SPDX-License-Identifier: Apache-2.0 */
/* The socket layer of the minimal MQTT client (mqtt_min.h) on POSIX
 * sockets: NuttX's (through the NINA usrsock device) on the board.  The
 * Zephyr build's MQTT library calls the same functions under Zephyr's
 * names (zsock_*()), with the same arguments and results.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mqtt_min.h"

int mqtt_min_sock_socket(int family, int type, int protocol)
{
  return socket(family, type, protocol);
}

int mqtt_min_sock_connect(int sock, const struct sockaddr *addr,
                          socklen_t addrlen)
{
  return connect(sock, addr, addrlen);
}

ssize_t mqtt_min_sock_send(int sock, const void *buf, size_t len, int flags)
{
  return send(sock, buf, len, flags);
}

ssize_t mqtt_min_sock_sendmsg(int sock, const struct msghdr *msg, int flags)
{
  return sendmsg(sock, msg, flags);
}

ssize_t mqtt_min_sock_recv(int sock, void *buf, size_t len, int flags)
{
  return recv(sock, buf, len, flags);
}

int mqtt_min_sock_close(int sock)
{
  return close(sock);
}
