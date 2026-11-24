/* SPDX-License-Identifier: MIT */
/* The console transport: what the shell engine's serial session
 * (shell_serial.c) needs from the OS and its terminal driver, and the stdio
 * redirection every console needs.  shell_serial.c includes this header; it
 * includes the OS's part, "shell_serial_os_impl.h", then declares the
 * functions that part must define (static inline), so a different one does
 * not compile.  The FreeRTOS build's part is
 * common/compat/port/pico_sdk/shell_serial_os_impl.h (the Pico SDK's USB CDC ACM
 * driver, stdio_usb, and its stdio).
 *
 * The terminal is one serial line (a USB CDC ACM port, a UART) that a
 * terminal program may open and close.
 */

#ifndef COMMON_COMPAT_PORT_SHELL_SERIAL_OS_H
#define COMMON_COMPAT_PORT_SHELL_SERIAL_OS_H

#include <stdbool.h>
#include <stddef.h>

/* Provided by the engine (shell_serial.c): the OS calls it when characters
 * have arrived, from any context (interrupt handlers included).
 */
void shell_serial_rx_ready(void);

#include "shell_serial_os_impl.h"

/* Makes every thread's stdout and stderr (printf(), puts(), putchar(),
 * fputs(), fwrite(), fprintf(), ... of the C library) go to
 * shell_console_write(shell_console_current(), ...) (shell_console.h) as
 * they are written, with nothing buffered (so that they come in order with
 * shell_print()), and fflush(stdout) do nothing more.  Called once, from
 * shell_serial_init(), before the kernel runs threads.
 */
static inline void shell_serial_os_stdio_init(void);

/* Arranges that shell_serial_rx_ready() is called when characters arrive.
 * Called once, by the session's thread, before it first reads.
 */
static inline void shell_serial_os_rx_init(void);

/* Whether a terminal has the line open now (USB CDC ACM: DTR).  The session
 * looks at least every SHELL_SERIAL_POLL_MS.  From the session's thread.
 */
static inline bool shell_serial_os_connected(void);

/* Takes up to len received characters into buf without waiting: their
 * count, or 0 (or less) when there are none.  From the session's thread.
 */
static inline int shell_serial_os_read(char *buf, size_t len);

/* Writes len bytes to the terminal as they are (the console has made the
 * line ends "\r\n"), from a thread, with the console's mutex held: drops
 * what the terminal cannot take (no terminal now, or it stopped reading);
 * must not block for long.
 */
static inline void shell_serial_os_write(const char *buf, size_t len);

#endif /* COMMON_COMPAT_PORT_SHELL_SERIAL_OS_H */
