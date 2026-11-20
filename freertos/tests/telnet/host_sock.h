/* SPDX-License-Identifier: MIT */
/* The test's controls of its zsock_*() on the host's sockets (host_sock.c). */

#ifndef FREERTOS_TESTS_TELNET_HOST_SOCK_H
#define FREERTOS_TESTS_TELNET_HOST_SOCK_H

/* The send buffer (bytes) of the sockets that connect; 0 leaves the host's.
 * A small one makes a receiver that stops reading block the sends soon, as
 * the NINA's small buffers do.
 */
extern int host_sock_connect_sndbuf;

/* The descriptors in use. */
int fake_sock_open_count(void);

/* Every connection fails, as when Wi-Fi drops: poll reports POLLERR, and
 * the peer sees the connection go.  The listening sockets stay.
 */
void fake_sock_fail_connections(void);

/* The listening sockets fail (poll reports POLLERR), and the next
 * BIND_FAILURES binds fail with EIO, as when the NINA does not answer.
 */
void fake_sock_fail_listeners(int bind_failures);

#endif /* FREERTOS_TESTS_TELNET_HOST_SOCK_H */
