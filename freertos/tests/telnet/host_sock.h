/* SPDX-License-Identifier: MIT */
/* The test's controls of its zsock_*() on the host's sockets (host_sock.c). */

#ifndef FREERTOS_TESTS_TELNET_HOST_SOCK_H
#define FREERTOS_TESTS_TELNET_HOST_SOCK_H

/* The descriptors in use. */
int fake_sock_open_count(void);

/* Every connection fails, as when Wi-Fi drops: poll reports POLLERR, and
 * the peer sees the connection go.  The listening sockets stay.
 */
void fake_sock_fail_connections(void);

#endif /* FREERTOS_TESTS_TELNET_HOST_SOCK_H */
