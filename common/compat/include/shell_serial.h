/* SPDX-License-Identifier: MIT */
/* The serial session of the shell engine: the shell on the OS's serial
 * terminal (shell_serial_os.h; the FreeRTOS build's USB CDC ACM port), as
 * Zephyr's shell on its UART backend.  Its console is also the default
 * console, where the threads bound to no console write (shell_console.h).
 *
 * The thread "console" (APP_PRIO_SHELL, APP_STACK_SHELL,
 * common/compat/include/priorities.h; Zephyr's shell thread) is bound to the
 * console and runs the commands of the lines typed (line editing and
 * history, cli.h).  When a terminal opens the line it writes "\r\n", calls
 * the application's on_connect() (a banner) and shows the prompt; while
 * none has it open, the output is dropped.
 */

#ifndef COMMON_COMPAT_INCLUDE_SHELL_SERIAL_H
#define COMMON_COMPAT_INCLUDE_SHELL_SERIAL_H

#include "shell_console.h"

/* The longest the session waits for input before it looks at the line
 * again (whether a terminal has opened it).
 */
#define SHELL_SERIAL_POLL_MS 50

/* Sets up the console NAME on the terminal, makes it the default console
 * and redirects the stdio to the consoles (shell_serial_os_stdio_init()).
 * Before the kernel runs threads, once, after the OS has set up the
 * terminal's driver.
 */
void shell_serial_init(const char *name);

/* Starts the thread "console" with PROMPT; ON_CONNECT (or NULL) is called
 * in it, bound to the console, each time a terminal opens the line.  Before
 * the kernel runs threads or from a thread, once.  0 or a negative errno
 * (z_thread_static_start()).
 */
int shell_serial_start(const char *prompt, void (*on_connect)(void));

#endif /* COMMON_COMPAT_INCLUDE_SHELL_SERIAL_H */
