/* SPDX-License-Identifier: MIT */
/* The Telnet shell of the shell engine: the shell of the serial session
 * (the same commands, prompt and output) on TCP port
 * CONFIG_SHELL_TELNET_PORT (23) for one client at a time, as the Zephyr
 * build's Telnet shell backend (Zephyr's
 * subsys/shell/backends/shell_telnet.c, CONFIG_SHELL_BACKEND_TELNET), on
 * the compatibility layer's kernel API and Zephyr's sockets.  See
 * shell_telnet.c for what it does and how it differs.
 */

#ifndef COMMON_COMPAT_INCLUDE_SHELL_TELNET_H
#define COMMON_COMPAT_INCLUDE_SHELL_TELNET_H

/* Opens the listening socket (any address, the port, a backlog of 1) and
 * starts the threads "shell_telnet" (the session: runs its commands;
 * APP_PRIO_SHELL, APP_STACK_SHELL) and "telnet_service" (accepts the
 * client, reads it; APP_PRIO_TELNET_SERVICE, APP_STACK_TELNET_SERVICE,
 * common/compat/include/priorities.h), from a thread, before the threads that may
 * use the network start (the Zephyr build's SYS_INIT() at APPLICATION 95).
 * PROMPT is the session's prompt (the serial session's).  Logs "Telnet
 * shell backend initialized", or the Zephyr build's messages and returns a
 * negative errno when the socket cannot be set up (no Telnet then); a
 * negative errno also when a thread could not be started.  Once only.
 */
int shell_telnet_init(const char *prompt);

#endif /* COMMON_COMPAT_INCLUDE_SHELL_TELNET_H */
