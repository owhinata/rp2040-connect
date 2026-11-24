/* SPDX-License-Identifier: MIT */
/* The consoles of the shell engine and the output of each thread, on the
 * compatibility layer's kernel API.
 *
 * A console is the terminal of one shell session: the serial session
 * (shell_serial.h; the FreeRTOS build's USB CDC ACM port) and the Telnet
 * client (shell_telnet.c).  Each thread may be bound to a console (its
 * custom data, k_thread_custom_data_set()); everything the thread writes to
 * stdout or stderr goes to that console, the rest of the threads write to
 * the default console.  So a command prints with printf() and
 * fprintf(stderr), as the shared sources do (common/shell/zshell.c,
 * subcmd.c: help and errors), and its text reaches the session that runs
 * it.  The OS routes its stdout and stderr to shell_console_write() of
 * shell_console_current() (the console transport, shell_serial_os.h; the
 * FreeRTOS build's common/compat/port/pico_sdk/shell_stdio.c and
 * common/compat/port/pico_sdk/stdio_wrap.c).
 *
 * A console turns "\n" into "\r\n" (unless "\r" came just before, also at
 * the end of the previous write), as the Pico SDK's stdio does for a
 * driver, but per console.  Its writes are serialized by a k_mutex of its
 * own, so a stalled terminal holds up only the threads that write to it.
 * A write from an interrupt handler is dropped (nothing may wait there, and
 * the terminals' code takes locks).
 *
 * A console is also the session's shell backend (struct shell of
 * <zephyr/shell/shell.h>, zephyr_os_shell.h) and holds its getopt state
 * (<zephyr/sys/sys_getopt.h>, getopt_state.c), so that two sessions can
 * run commands at the same time.
 */

#ifndef COMMON_COMPAT_INCLUDE_SHELL_CONSOLE_H
#define COMMON_COMPAT_INCLUDE_SHELL_CONSOLE_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_getopt.h>

struct shell_console;

/* Writes LEN bytes to the terminal as they are (the line ends are already
 * "\r\n"), from a thread, with the console's mutex held.  Drops what the
 * terminal cannot take (no terminal now, or it stopped reading); must not
 * block for long.
 */
typedef void (*shell_console_out_t)(struct shell_console *con, const char *buf, size_t len);

struct shell_console {
	const char *name;
	shell_console_out_t out;
	void *ctx; /* for out */
	struct k_mutex lock;
	/* How long a thread that is not bound to this console waits for its
	 * mutex (shell_console_write()); the bound threads wait 1 s.
	 */
	uint32_t other_lock_timeout_ms;
	bool last_cr;                   /* the last byte written was '\r' */
	struct shell sh;                /* the session's backend */
	struct sys_getopt_state getopt; /* the session's getopt state */
};

/* Sets up CON (before the kernel runs threads, or in a thread). */
void shell_console_init(struct shell_console *con, const char *name, shell_console_out_t out,
			void *ctx);

/* The console of the threads that are not bound to one (by default none:
 * their output is dropped).
 */
void shell_console_set_default(struct shell_console *con);

/* Binds the calling thread to CON (NULL: unbinds it). */
void shell_console_bind(struct shell_console *con);

/* The console the calling thread is bound to, or NULL (also before the
 * kernel runs threads).
 */
struct shell_console *shell_console_bound(void);

/* Where the calling context's stdout and stderr go: its bound console, else
 * the default one; NULL in an interrupt handler.
 */
struct shell_console *shell_console_current(void);

/* Writes LEN bytes to CON with the line ends turned into "\r\n".  Nothing
 * for CON == NULL or in an interrupt handler.  Before the kernel runs
 * threads the mutex is not taken.  A writer that has waited for the mutex
 * 1 s (a thread bound to CON) or CON's other_lock_timeout_ms (any other
 * thread) drops its text.
 */
void shell_console_write(struct shell_console *con, const char *buf, size_t len);

/* How long the threads that are not bound to CON wait for its mutex (by
 * default 1 s, as the bound ones): a console whose terminal may keep its
 * session's thread in a write for long (the Telnet client that stops
 * reading) gives the other threads a short wait, so that they drop their
 * text instead of stalling.
 */
void shell_console_set_other_lock_timeout(struct shell_console *con, uint32_t ms);

/* Takes CON's mutex (waiting up to TIMEOUT_MS) for work on its terminal
 * outside shell_console_write(), such as sending what the terminal has
 * buffered; false when it was not free in time.
 */
bool shell_console_lock(struct shell_console *con, uint32_t timeout_ms);
void shell_console_unlock(struct shell_console *con);

/* Formats into a buffer on the stack (a longer text: on the heap, or cut
 * when that fails) and writes it with shell_console_write().
 */
void shell_console_vprintf(struct shell_console *con, const char *fmt, va_list args)
	__attribute__((format(printf, 2, 0)));

#endif /* COMMON_COMPAT_INCLUDE_SHELL_CONSOLE_H */
