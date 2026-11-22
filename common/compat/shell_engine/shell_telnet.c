/* SPDX-License-Identifier: MIT */
/* The Telnet shell of the shell engine (shell_telnet.h): the shell of the
 * serial session on TCP port CONFIG_SHELL_TELNET_PORT for one client at a
 * time, with the behaviour of the Zephyr build's Telnet shell backend
 * (Zephyr's subsys/shell/backends/shell_telnet.c with the Zephyr build's
 * configuration: port 23, CONFIG_SHELL_TELNET_LINE_BUF_SIZE 80,
 * CONFIG_SHELL_TELNET_SEND_TIMEOUT 100 ms, no
 * CONFIG_SHELL_TELNET_SUPPORT_COMMAND, IPv4 only) and of Zephyr's shell on
 * it, written for the engine's consoles (shell_console.h) on the
 * compatibility layer's kernel API and Zephyr's sockets (zsock_*(); the
 * FreeRTOS build's are the layer's network part, common/compat/net_offload,
 * on the NINA).
 *
 * Two threads, as Zephyr's two: "telnet_service" (Zephyr's
 * net_socket_service thread, APP_PRIO_TELNET_SERVICE, 2048 bytes) polls the
 * listening socket and the client (zsock_poll(), which asks the NINA every
 * 20 ms, as Zephyr's offloaded poll: the NINA's accept never waits),
 * accepts and refuses clients, reads the client and ends the connection;
 * "shell_telnet" (Zephyr's shell thread of the backend, APP_PRIO_SHELL,
 * APP_STACK_SHELL) is the session: it is bound to the Telnet console (its
 * printf(), shell_print() and the getopt state are the session's,
 * shell_console.h) and runs the commands of the lines it is given.
 *
 * As in Zephyr:
 * - the listening socket is opened once at boot (any address, backlog 1, no
 *   SO_REUSEADDR) and stays open across Wi-Fi disconnections (the NINA
 *   keeps it); a failure is logged with Zephyr's messages and leaves no
 *   Telnet;
 * - a second client is accepted and closed at once without a word, with
 *   "Telnet client already connected." in the log; the first one goes on;
 * - nothing is sent when a client connects (the prompt comes after the
 *   first line; output while no client is connected is dropped);
 * - the Telnet commands (IAC ...) are taken out of the input and not
 *   answered (no option negotiation, so the client echoes and edits the
 *   line itself); a subnegotiation (IAC SB) ends the connection;
 * - the shell runs with echo off, which is how Zephyr's backend sets it at
 *   each connection: printable characters are collected without being
 *   echoed; Backspace, Delete, Tab, Ctrl-C and the other control
 *   characters, the escape sequences (the arrows: no history) and NUL are
 *   ignored; bytes above 127 are dropped (CONFIG_SHELL_ASCII_FILTER); CR,
 *   LF, CR LF, LF CR and CR NUL each end one line (Zephyr's process_nl():
 *   after a CR the LFs, and after an LF the CRs, are skipped until another
 *   character comes; CR CR and LF LF are two lines).  At the end of a line "\r\n" is sent, the line
 * runs (an empty one does nothing) and the prompt follows.  At most 511 characters; more are
 * dropped;
 * - output: "\n" goes out as "\r\n" (SHELL_FLAG_OLF_CRLF); it is collected
 *   in a line buffer of 80 bytes that is sent when it holds a line end or is
 *   full, and otherwise 100 ms after its first byte (the service does it;
 *   Zephyr's delayed work);
 * - the connection ends when the client closes it, when a read fails, when
 *   poll reports an error on it ("Telnet socket <fd> error (<n>)"; the
 *   NINA's sockets have no SO_ERROR, so n is 0; ENETDOWN would log "Network
 *   is down"; a failed poll of the module is an error on that socket, as in
 *   the Zephyr build, common/drivers/nina/port/zephyr/nina_socket.c), and when a
 *   send of the session fails ("Failed to send <errno>, shutting down", then
 *   "Error <-errno>, shutting down" or "Network disconnected, shutting
 *   down"); then the next client is accepted.  An error on the listening
 *   socket restarts the server (closes both, sets it up again; "Telnet fatal
 *   error, failed to restart server" if that fails);
 * - output from another thread to the session's backend (the "wifi" events of
 *   a command run here, the "zperf tcp download" reports) goes to the
 *   client; LOG_*() goes to the RAM log only, and printf() of a thread that is
 *   not a session's to the USB console (the default console), as in the
 *   Zephyr build (no shell log backend, printk() to the log);
 * - a command that still runs when its client goes keeps running, and its
 *   output goes to the next client, if one connects meanwhile.
 *
 * Different from Zephyr, and why:
 * - a send waits for room without holding the client: the session's own
 *   output waits up to 10 s per line (the NINA's blocking send,
 *   NINA_SEND_WAIT_MS, which Zephyr's blocking send ends up in), retrying
 *   every 10 ms (the NINA's blocking send retries every 1 ms), then the
 *   connection is ended; another thread's output is tried once and dropped
 *   when the client does not take it, and that thread waits at most 100 ms for
 *   the console (Zephyr's would wait for the shell's mutex and its blocking
 *   send): a client that stops reading holds up the session only, never
 *   "net_mgmt" or the other threads;
 * - only the service closes the client (a failed send marks it): the
 *   descriptor of a closed socket is soon another socket's, and the session
 *   must never send to that;
 * - the session sends what is left in the line buffer as soon as it has
 *   handled what the client sent (the prompt; Zephyr waits for the 100 ms of
 *   its delayed work);
 * - a new client starts with an empty line (Zephyr keeps what the previous
 *   client had typed without an end of line);
 * - before refusing a client, the service looks once more whether the
 *   connected one has closed (it polls every 20 ms; a client that closes
 *   and connects again at once, as a script that connects for each command
 *   does, would otherwise be refused);
 * - after an accept error the service waits 100 ms (Zephyr polls again at
 *   once; a lasting error, such as no free NINA socket, would turn into a
 *   busy loop with a log line each time); a failed poll call is logged once
 *   and retried every 100 ms (Zephyr's socket service thread ends, which
 *   ends Telnet and the zperf server);
 * - when the restart of the server fails (the NINA does not answer at that
 *   moment), the service sets it up again every RESTART_RETRY_MS without
 *   logging the failures, and logs "Telnet shell backend initialized" when
 *   it succeeds (Zephyr's service ends: no more Telnet until a reboot).
 */

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>

#include "app_log.h"
#include "shell_console.h"
#include "shell_engine.h"
#include "shell_telnet.h"
#include "priorities.h"

LOG_MODULE_REGISTER(shell_telnet, LOG_LEVEL_INF);

#ifndef CONFIG_SHELL_TELNET_PORT
#define CONFIG_SHELL_TELNET_PORT 23
#endif

/* CONFIG_SHELL_TELNET_LINE_BUF_SIZE, CONFIG_SHELL_TELNET_SEND_TIMEOUT (ms). */
#define TELNET_LINE_SIZE 80
#define TELNET_TIMEOUT   100
/* The input not yet taken by the session (Zephyr: CONFIG_SHELL_CMD_BUFF_SIZE)
 * and the longest line with its NUL (CONFIG_SHELL_CMD_BUFF_SIZE).
 */
#define RX_BUF_SIZE      512
#define CMD_BUFF_SIZE    512
/* The session's own send: how long it waits for room, and how often it
 * tries again.
 */
#define SEND_WAIT_MS     10000
#define SEND_RETRY_MS    10
/* How often the service tries to set the server up again after a failed
 * restart.
 */
#define RESTART_RETRY_MS 1000
/* How long another thread waits for the console's mutex. */
#define OTHER_LOCK_MS    TELNET_TIMEOUT
/* Bytes read from the socket, and taken by the session, at a time. */
#define READ_CHUNK       128
#define SESSION_CHUNK    64

/* RFC 854 */
#define NVT_CMD_SB   250
#define NVT_CMD_WILL 251
#define NVT_CMD_WONT 252
#define NVT_CMD_DO   253
#define NVT_CMD_DONT 254
#define NVT_CMD_IAC  255

#define CHAR_ESC 0x1b

enum receive_state {
	RECEIVE_DEFAULT,
	RECEIVE_ESC,
	RECEIVE_ESC_SEQ,
};

static struct {
	int listen_fd; /* the service's */
	/* The client and whether a send of the session failed on it: changed
	 * under conn_lock, the descriptor by the service only; a send holds
	 * conn_lock for one try.  generation: one more for each client.
	 */
	int client_fd;
	bool broken;
	uint32_t generation;
	struct k_mutex conn_lock;

	/* What the client sent, without the Telnet commands, until the
	 * session takes it (under rx_lock), and the client it came from.
	 */
	uint8_t rx_buf[RX_BUF_SIZE];
	size_t rx_len;
	uint32_t rx_generation;
	struct k_mutex rx_lock;
	/* Given by the service when it has added to rx_buf. */
	struct k_sem rx_sem;
	/* The service's place in a Telnet command (cmd_len bytes of it read,
	 * the operation the second one).
	 */
	uint8_t cmd_len;
	uint8_t cmd_op;

	/* The output line buffer: under the console's mutex. */
	char line[TELNET_LINE_SIZE];
	size_t line_len;
	int64_t line_since;
	uint32_t line_generation;

	/* The session's line (its thread only). */
	char cmd_buff[CMD_BUFF_SIZE];
	size_t cmd_buff_len;
	uint8_t last_nl;
	enum receive_state receive_state;
	uint32_t session_generation;
	const char *prompt;

	k_tid_t session_tid;
	k_tid_t service_tid;
} tn = {
	.listen_fd = -1,
	.client_fd = -1,
};

static struct shell_console telnet_console;

static K_THREAD_STACK_DEFINE(session_stack, APP_STACK_SHELL);
static K_THREAD_STACK_DEFINE(service_stack, APP_STACK_TELNET_SERVICE);

/* Output ------------------------------------------------------------------*/

/* Whether there is a client to send to; empties the line buffer when it
 * holds the previous client's text.  Under the console's mutex.
 */
static bool out_connected(void)
{
	uint32_t generation;
	bool connected;

	(void)k_mutex_lock(&tn.conn_lock, K_FOREVER);
	connected = tn.client_fd >= 0 && !tn.broken;
	generation = tn.generation;
	(void)k_mutex_unlock(&tn.conn_lock);
	if (tn.line_generation != generation) {
		tn.line_len = 0;
		tn.line_generation = generation;
	}
	return connected;
}

/* Sends the line buffer, under the console's mutex.  BLOCK (the session's
 * own output): waits up to SEND_WAIT_MS for room; otherwise one try, which
 * keeps the rest.  0 when all went out, 1 when some is kept (not BLOCK),
 * -ENOTCONN when the client is gone (silently, as Zephyr's telnet_send()
 * finds the descriptor closed), another negative errno when the send failed
 * (-EAGAIN: no room for SEND_WAIT_MS; the connection is marked to be
 * ended); the buffer is empty then.
 */
static int line_send(bool block)
{
	int64_t deadline = k_uptime_get() + SEND_WAIT_MS;
	size_t off = 0;

	while (off < tn.line_len) {
		ssize_t ret = -1;
		int err = -ENOTCONN;
		bool failed = false;

		(void)k_mutex_lock(&tn.conn_lock, K_FOREVER);
		if (tn.client_fd >= 0 && !tn.broken && tn.line_generation == tn.generation) {
			ret = zsock_send(tn.client_fd, tn.line + off, tn.line_len - off,
					 ZSOCK_MSG_DONTWAIT);
			/* Nothing taken is no room, as EAGAIN. */
			err = ret < 0 ? -errno : (ret == 0 ? -EAGAIN : 0);
			failed = ret <= 0 &&
				 (err != -EAGAIN || (block && k_uptime_get() >= deadline));
			if (failed) {
				tn.broken = true;
			}
		}
		(void)k_mutex_unlock(&tn.conn_lock);

		if (ret > 0) {
			off += (size_t)ret;
			continue;
		}
		if (failed) {
			LOG_ERR("Failed to send %d, shutting down", -err);
		}
		if (failed || err != -EAGAIN) {
			tn.line_len = 0;
			return err;
		}
		if (!block) {
			memmove(tn.line, tn.line + off, tn.line_len - off);
			tn.line_len -= off;
			return 1;
		}
		k_msleep(SEND_RETRY_MS);
	}
	tn.line_len = 0;
	return 0;
}

/* The console's terminal (shell_console_out_t), under its mutex: the line
 * buffer of Zephyr's telnet_write().
 */
static void telnet_out(struct shell_console *con, const char *buf, size_t len)
{
	bool block = k_current_get() == tn.session_tid;

	ARG_UNUSED(con);

	while (len > 0) {
		size_t n;
		int err;

		if (!out_connected()) {
			tn.line_len = 0;
			return;
		}
		if (tn.line_len == 0) {
			tn.line_since = k_uptime_get();
		}
		n = MIN(len, TELNET_LINE_SIZE - tn.line_len);
		memcpy(tn.line + tn.line_len, buf, n);
		tn.line_len += n;
		buf += n;
		len -= n;
		if (tn.line[tn.line_len - 1] != '\n' && tn.line_len < TELNET_LINE_SIZE) {
			continue;
		}
		err = line_send(block);
		if (err > 0) {
			if (tn.line_len == TELNET_LINE_SIZE) {
				/* No room for the rest of another thread's text. */
				tn.line_len = 0;
			}
		} else if (err < 0) {
			if (block) {
				if (err == -ENOTCONN || err == -ENETDOWN) {
					LOG_ERR("Network disconnected, shutting down");
				} else {
					LOG_ERR("Error %d, shutting down", err);
				}
			}
			return;
		}
	}
}

/* Sends what the line buffer holds: BLOCK as line_send(); from the service,
 * only text that has waited TELNET_TIMEOUT and without waiting for the
 * console.
 */
static void line_flush(bool block)
{
	if (!shell_console_lock(&telnet_console, block ? 1000 : 0)) {
		return;
	}
	if (tn.line_len > 0 && out_connected() &&
	    (block || k_uptime_get() - tn.line_since >= TELNET_TIMEOUT)) {
		(void)line_send(block);
	}
	shell_console_unlock(&telnet_console);
}

/* The service ----------------------------------------------------------------*/

/* QUIET: without the error messages (the retries after a failed restart). */
static int telnet_setup_server(bool quiet)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(CONFIG_SHELL_TELNET_PORT),
		.sin_addr = {.s_addr = htonl(INADDR_ANY)},
	};
	int ret;
	int fd;

	fd = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) {
		ret = -errno;
		if (!quiet) {
			LOG_ERR("Failed to create telnet NET_AF_INET socket");
		}
		goto error;
	}
	if (zsock_bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ret = -errno;
		if (!quiet) {
			LOG_ERR("Cannot bind on family NET_AF_INET (%d)", ret);
		}
		goto error;
	}
	if (zsock_listen(fd, 1) < 0) {
		ret = -errno;
		if (!quiet) {
			LOG_ERR("Cannot listen on family NET_AF_INET (%d)", ret);
		}
		goto error;
	}
	tn.listen_fd = fd;
	return 0;

error:
	if (!quiet) {
		LOG_ERR("Unable to start telnet on NET_AF_INET (%d)", ret);
	}
	if (fd >= 0) {
		(void)zsock_close(fd);
	}
	return ret;
}

static void telnet_end_client_connection(void)
{
	(void)k_mutex_lock(&tn.conn_lock, K_FOREVER);
	if (tn.client_fd >= 0) {
		(void)zsock_close(tn.client_fd);
	}
	tn.client_fd = -1;
	tn.broken = false;
	(void)k_mutex_unlock(&tn.conn_lock);
}

/* An error on the listening socket: close everything and start again.  When
 * that fails, the listening descriptor stays -1 and the service tries again
 * (service_main()).
 */
static void telnet_restart_server(void)
{
	int ret;

	if (tn.listen_fd >= 0) {
		(void)zsock_close(tn.listen_fd);
		tn.listen_fd = -1;
	}
	telnet_end_client_connection();
	ret = telnet_setup_server(false);
	if (ret < 0) {
		LOG_ERR("Telnet fatal error, failed to restart server (%d)", ret);
		return;
	}
	LOG_INF("Telnet shell backend initialized");
}

static void telnet_event(const struct zsock_pollfd *pfd);

static void telnet_accept(void)
{
	struct sockaddr_in addr;
	socklen_t addrlen = sizeof(addr);
	uint32_t generation;
	int sock;

	sock = zsock_accept(tn.listen_fd, (struct sockaddr *)&addr, &addrlen);
	if (sock < 0) {
		LOG_ERR("Telnet accept error (%d)", -errno);
		k_msleep(TELNET_TIMEOUT);
		return;
	}
	if (tn.client_fd >= 0) {
		/* The client may have gone since the last poll (one that
		 * connects again at once): look before refusing the new one.
		 */
		struct zsock_pollfd pfd = {.fd = tn.client_fd, .events = ZSOCK_POLLIN};

		if (zsock_poll(&pfd, 1, 0) > 0) {
			telnet_event(&pfd);
		}
	}
	if (tn.client_fd >= 0) {
		/* Too many connections. */
		LOG_ERR("Telnet client already connected.");
		(void)zsock_close(sock);
		return;
	}

	generation = tn.generation + 1;
	(void)k_mutex_lock(&tn.rx_lock, K_FOREVER);
	tn.rx_len = 0;
	tn.rx_generation = generation;
	(void)k_mutex_unlock(&tn.rx_lock);
	tn.cmd_len = 0;
	(void)k_mutex_lock(&tn.conn_lock, K_FOREVER);
	tn.client_fd = sock;
	tn.broken = false;
	tn.generation = generation;
	(void)k_mutex_unlock(&tn.conn_lock);
}

static size_t telnet_command_length(uint8_t op)
{
	return (op == NVT_CMD_SB || op == NVT_CMD_WILL || op == NVT_CMD_WONT || op == NVT_CMD_DO ||
		op == NVT_CMD_DONT)
		       ? 3
		       : 2;
}

/* Takes the Telnet commands (IAC and one or two bytes) out of BUF, also
 * across reads: the data bytes left (at the front of BUF), or -EOPNOTSUPP
 * for a subnegotiation.
 */
static int telnet_filter(uint8_t *buf, size_t len)
{
	size_t out = 0;

	for (size_t i = 0; i < len; i++) {
		uint8_t c = buf[i];

		if (tn.cmd_len == 0) {
			if (c == NVT_CMD_IAC) {
				tn.cmd_len = 1;
			} else {
				buf[out++] = c;
			}
			continue;
		}
		if (tn.cmd_len == 1) {
			tn.cmd_op = c;
			if (telnet_command_length(c) > 2) {
				tn.cmd_len = 2;
				continue;
			}
		}
		/* The command is complete; it is not answered. */
		tn.cmd_len = 0;
		if (tn.cmd_op == NVT_CMD_SB) {
			return -EOPNOTSUPP;
		}
	}
	return (int)out;
}

static void telnet_recv(void)
{
	uint8_t buf[READ_CHUNK];
	size_t room;
	ssize_t ret;
	int n;

	(void)k_mutex_lock(&tn.rx_lock, K_FOREVER);
	room = sizeof(tn.rx_buf) - tn.rx_len;
	(void)k_mutex_unlock(&tn.rx_lock);
	if (room == 0) {
		return;
	}
	ret = zsock_recv(tn.client_fd, buf, MIN(room, sizeof(buf)), ZSOCK_MSG_DONTWAIT);
	if (ret < 0 && errno == EAGAIN) {
		return;
	}
	if (ret <= 0) {
		/* An error, or the client closed the connection. */
		telnet_end_client_connection();
		return;
	}
	n = telnet_filter(buf, (size_t)ret);
	if (n < 0) {
		telnet_end_client_connection();
		return;
	}
	if (n == 0) {
		return;
	}
	(void)k_mutex_lock(&tn.rx_lock, K_FOREVER);
	memcpy(tn.rx_buf + tn.rx_len, buf, (size_t)n);
	tn.rx_len += (size_t)n;
	(void)k_mutex_unlock(&tn.rx_lock);
	k_sem_give(&tn.rx_sem);
}

/* Zephyr's telnet_server_cb() for one descriptor. */
static void telnet_event(const struct zsock_pollfd *pfd)
{
	if (pfd->revents & (ZSOCK_POLLERR | ZSOCK_POLLNVAL)) {
		int sock_error = 0;
		socklen_t optlen = sizeof(sock_error);
		int ret;

		(void)zsock_getsockopt(pfd->fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_ERROR, &sock_error,
				       &optlen);
		ret = -sock_error;
		if (ret == -ENETDOWN) {
			LOG_INF("Network is down");
		} else {
			LOG_ERR("Telnet socket %d error (%d)", pfd->fd, ret);
		}
		if (pfd->fd == tn.client_fd) {
			telnet_end_client_connection();
			return;
		}
		telnet_restart_server();
		return;
	}
	if (!(pfd->revents & ZSOCK_POLLIN)) {
		return;
	}
	if (pfd->fd == tn.listen_fd) {
		telnet_accept();
	} else if (pfd->fd == tn.client_fd) {
		telnet_recv();
	}
}

static void service_main(void *p1, void *p2, void *p3)
{
	bool poll_failed = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		struct zsock_pollfd fds[2];
		bool room;
		int ret;

		if (tn.listen_fd < 0) {
			/* A restart failed: set the server up again. */
			k_msleep(RESTART_RETRY_MS);
			if (telnet_setup_server(true) == 0) {
				LOG_INF("Telnet shell backend initialized");
			}
			continue;
		}
		if (tn.broken) {
			telnet_end_client_connection();
		}
		line_flush(false);

		(void)k_mutex_lock(&tn.rx_lock, K_FOREVER);
		room = tn.rx_len < sizeof(tn.rx_buf);
		(void)k_mutex_unlock(&tn.rx_lock);
		fds[0] = (struct zsock_pollfd){.fd = tn.listen_fd, .events = ZSOCK_POLLIN};
		/* A full input buffer waits for the session; errors still come. */
		fds[1] = (struct zsock_pollfd){.fd = tn.client_fd,
					       .events = room ? ZSOCK_POLLIN : 0};

		ret = zsock_poll(fds, ARRAY_SIZE(fds), TELNET_TIMEOUT);
		if (ret < 0) {
			if (!poll_failed) {
				LOG_ERR("Telnet poll failed (%d)", -errno);
			}
			poll_failed = true;
			k_msleep(TELNET_TIMEOUT);
			continue;
		}
		poll_failed = false;
		if (ret == 0) {
			continue;
		}
		if (fds[0].revents != 0) {
			telnet_event(&fds[0]);
		}
		if (fds[0].fd != tn.listen_fd) {
			continue; /* restarted, or to be set up again: the client is gone */
		}
		if (fds[1].fd >= 0 && fds[1].fd == tn.client_fd && fds[1].revents != 0) {
			telnet_event(&fds[1]);
		}
	}
}

/* The session ------------------------------------------------------------------*/

/* Zephyr's process_nl(): whether C ends a line. */
static bool process_nl(char c)
{
	if (c != '\r' && c != '\n') {
		tn.last_nl = 0;
		return false;
	}
	if (tn.last_nl == 0 || (uint8_t)c == tn.last_nl) {
		tn.last_nl = (uint8_t)c;
		return true;
	}
	return false;
}

static void session_write(const char *text)
{
	shell_console_write(&telnet_console, text, strlen(text));
}

/* Zephyr's state_collect() with echo off, for one received character. */
static void session_input(char c)
{
	if ((uint8_t)c > 127) {
		return; /* CONFIG_SHELL_ASCII_FILTER */
	}
	switch (tn.receive_state) {
	case RECEIVE_DEFAULT:
		if (process_nl(c)) {
			session_write("\n");
			if (tn.cmd_buff_len > 0) {
				tn.cmd_buff[tn.cmd_buff_len] = '\0';
				(void)shell_engine_execute(tn.cmd_buff);
			}
			tn.cmd_buff_len = 0;
			session_write(tn.prompt);
			break;
		}
		if (c == CHAR_ESC) {
			tn.receive_state = RECEIVE_ESC;
		} else if (isprint((unsigned char)c) && tn.cmd_buff_len + 1 < CMD_BUFF_SIZE) {
			tn.cmd_buff[tn.cmd_buff_len++] = c;
		}
		/* NUL, Tab, Backspace, Delete and the other control
		 * characters: nothing, with echo off.
		 */
		break;
	case RECEIVE_ESC:
		tn.receive_state = c == '[' ? RECEIVE_ESC_SEQ : RECEIVE_DEFAULT;
		break;
	default:
		/* The byte after "ESC [" is dropped with echo off. */
		tn.receive_state = RECEIVE_DEFAULT;
		break;
	}
}

static void session_main(void *p1, void *p2, void *p3)
{
	char buf[SESSION_CHUNK];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	shell_console_bind(&telnet_console);

	for (;;) {
		(void)k_sem_take(&tn.rx_sem, K_FOREVER);
		for (;;) {
			uint32_t generation;
			size_t n;

			(void)k_mutex_lock(&tn.rx_lock, K_FOREVER);
			n = MIN(tn.rx_len, sizeof(buf));
			memcpy(buf, tn.rx_buf, n);
			memmove(tn.rx_buf, tn.rx_buf + n, tn.rx_len - n);
			tn.rx_len -= n;
			generation = tn.rx_generation;
			(void)k_mutex_unlock(&tn.rx_lock);
			if (n == 0) {
				break;
			}
			if (generation != tn.session_generation) {
				/* A new client: a new line. */
				tn.session_generation = generation;
				tn.cmd_buff_len = 0;
				tn.last_nl = 0;
				tn.receive_state = RECEIVE_DEFAULT;
			}
			for (size_t i = 0; i < n; i++) {
				session_input(buf[i]);
			}
		}
		line_flush(true);
	}
}

int shell_telnet_init(const char *prompt)
{
	/* The threads' start data, which they may read while they run. */
	static const struct z_thread_static session = {
		.name = "shell_telnet",
		.stack = session_stack,
		.stack_size = K_THREAD_STACK_SIZEOF(session_stack),
		.entry = session_main,
		.prio = APP_PRIO_SHELL,
		.tid = &tn.session_tid,
	};
	static const struct z_thread_static service = {
		.name = "telnet_service",
		.stack = service_stack,
		.stack_size = K_THREAD_STACK_SIZEOF(service_stack),
		.entry = service_main,
		.prio = APP_PRIO_TELNET_SERVICE,
		.tid = &tn.service_tid,
	};
	int ret;

	tn.prompt = prompt;
	(void)k_mutex_init(&tn.conn_lock);
	(void)k_mutex_init(&tn.rx_lock);
	(void)k_sem_init(&tn.rx_sem, 0, 1);
	shell_console_init(&telnet_console, "telnet", telnet_out, NULL);
	shell_console_set_other_lock_timeout(&telnet_console, OTHER_LOCK_MS);

	ret = telnet_setup_server(false);
	if (ret < 0) {
		return ret;
	}
	LOG_INF("Telnet shell backend initialized");

	ret = z_thread_static_start(&session);
	if (ret == 0) {
		ret = z_thread_static_start(&service);
	}
	if (ret < 0) {
		LOG_ERR("Could not start the Telnet threads (%d)", ret);
	}
	return ret;
}
