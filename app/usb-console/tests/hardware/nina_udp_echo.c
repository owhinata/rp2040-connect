/* SPDX-License-Identifier: Apache-2.0 */
/* Hardware test: echo UDP datagrams so that UDP receive and send can be
 * exercised while other sockets are busy. Runs in the shell thread. */
#include <errno.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>

static int cmd_udp_echo(const struct shell *sh, size_t argc, char **argv)
{
	static uint8_t buffer[512];
	struct sockaddr_in local = {.sin_family = AF_INET};
	unsigned long port = strtoul(argv[1], NULL, 10);
	unsigned long seconds = strtoul(argv[2], NULL, 10);
	uint32_t received = 0, echoed = 0, send_errors = 0, receive_errors = 0;
	int64_t deadline;
	int fd, rc = 0;

	ARG_UNUSED(argc);
	if (!port || port > 65535 || !seconds || seconds > 3600) {
		shell_error(sh, "Use nina_udp_echo <port> <seconds>");
		return -EINVAL;
	}
	fd = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		shell_error(sh, "socket failed: %d", errno);
		return -errno;
	}
	local.sin_port = htons((uint16_t)port);
	if (zsock_bind(fd, (struct sockaddr *)&local, sizeof(local)) < 0) {
		rc = -errno;
		shell_error(sh, "bind failed: %d", errno);
		(void)zsock_close(fd);
		return rc;
	}
	shell_print(sh, "Echoing UDP port %lu for %lu s", port, seconds);
	deadline = k_uptime_get() + (int64_t)seconds * 1000;
	while (k_uptime_get() < deadline) {
		struct sockaddr_in peer;
		socklen_t peer_len = sizeof(peer);
		ssize_t n = zsock_recvfrom(fd, buffer, sizeof(buffer), ZSOCK_MSG_DONTWAIT,
					   (struct sockaddr *)&peer, &peer_len);

		if (n < 0) {
			if (errno != EAGAIN) {
				receive_errors++;
			}
			k_sleep(K_MSEC(2));
			continue;
		}
		received++;
		if (zsock_sendto(fd, buffer, n, 0, (struct sockaddr *)&peer, peer_len) == n) {
			echoed++;
		} else {
			send_errors++;
		}
	}
	rc = zsock_close(fd) < 0 ? -errno : 0;
	shell_print(sh, "udp_echo received=%u echoed=%u send_errors=%u receive_errors=%u close=%d",
		    received, echoed, send_errors, receive_errors, rc);
	return rc;
}
SHELL_CMD_ARG_REGISTER(nina_udp_echo, NULL, "Test: echo UDP datagrams: <port> <seconds>",
		       cmd_udp_echo, 3, 0);
