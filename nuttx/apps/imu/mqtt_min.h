/* SPDX-License-Identifier: Apache-2.0 */
/* The port's minimal MQTT client (mqtt_min.c): the build-time settings of
 * the Zephyr build's MQTT library and the thin socket layer under it.  The
 * client's API is Zephyr's, <zephyr/net/mqtt.h> of the compatibility layer
 * (nuttx/drivers/compat/zephyr/net/mqtt.h).
 *
 * The socket layer has the meaning of the Zephyr socket calls that
 * Zephyr's TCP transport (subsys/net/lib/mqtt/mqtt_transport_socket_tcp.c)
 * makes, zsock_socket(), zsock_connect(), zsock_send(), zsock_sendmsg(),
 * zsock_recv() and zsock_close(): POSIX's, returning -1 with errno set on
 * failure.  mqtt_min_sock.c implements it with NuttX's (POSIX) sockets; a
 * host test links a fake instead.
 */

#ifndef __NUTTX_APPS_IMU_MQTT_MIN_H
#define __NUTTX_APPS_IMU_MQTT_MIN_H

#include <sys/types.h>
#include <sys/socket.h>

/* CONFIG_MQTT_KEEPALIVE of the Zephyr build (seconds). */

#define MQTT_MIN_KEEPALIVE      60

/* CONFIG_MQTT_CLEAN_SESSION of the Zephyr build (off). */

#define MQTT_MIN_CLEAN_SESSION  0U

/* The socket layer ---------------------------------------------------------*/

int mqtt_min_sock_socket(int family, int type, int protocol);
int mqtt_min_sock_connect(int sock, const struct sockaddr *addr,
                          socklen_t addrlen);
ssize_t mqtt_min_sock_send(int sock, const void *buf, size_t len,
                           int flags);
ssize_t mqtt_min_sock_sendmsg(int sock, const struct msghdr *msg,
                              int flags);
ssize_t mqtt_min_sock_recv(int sock, void *buf, size_t len, int flags);
int mqtt_min_sock_close(int sock);

#endif /* __NUTTX_APPS_IMU_MQTT_MIN_H */
