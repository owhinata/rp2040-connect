/* SPDX-License-Identifier: MIT */
/* The Telnet shell of the FreeRTOS build: the shell of the USB console
 * (freertos/drivers/shell: the same commands, prompt and output) on TCP
 * port CONFIG_SHELL_TELNET_PORT (23) for one client at a time, as the
 * Zephyr build's Telnet shell backend (Zephyr's
 * subsys/shell/backends/shell_telnet.c, CONFIG_SHELL_BACKEND_TELNET).  See
 * nano_telnet.c for what it does and how it differs.
 */

#ifndef FREERTOS_DRIVERS_TELNET_NANO_TELNET_H
#define FREERTOS_DRIVERS_TELNET_NANO_TELNET_H

/* Opens the listening socket (any address, the port, a backlog of 1) and
 * starts the tasks "telnet_service" (accepts the client, reads it) and
 * "shell_telnet" (the session: runs its commands), from a task, before the
 * threads that may use the network start (the Zephyr build's SYS_INIT() at
 * APPLICATION 95).  PROMPT is the session's prompt (the USB console's).
 * Logs "Telnet shell backend initialized", or the Zephyr build's messages
 * and returns a negative errno when the socket cannot be set up (no Telnet
 * then).  Once only.
 */
int nano_telnet_init(const char *prompt);

#endif /* FREERTOS_DRIVERS_TELNET_NANO_TELNET_H */
