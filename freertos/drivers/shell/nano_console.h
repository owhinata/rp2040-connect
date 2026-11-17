/* SPDX-License-Identifier: MIT */
/* The consoles of the FreeRTOS build and the output of each task.
 *
 * A console is the terminal of one shell session: the USB CDC ACM port
 * (freertos/app/usb-console/src/main.c) and the Telnet client
 * (freertos/drivers/telnet/nano_telnet.c).  Each
 * task may be bound to a console (a FreeRTOS thread local storage pointer,
 * index NANO_CONSOLE_TLS_INDEX); everything the task writes to stdout or
 * stderr goes to that console, the rest of the tasks write to the default
 * console.  So a command prints with printf() and fprintf(stderr), as the
 * shared sources do (common/shell/zshell.c, subcmd.c: help and errors), and
 * its text reaches the session that runs it.  The stdio paths of the Pico
 * SDK and newlib all end in nano_console_write() (nano_stdio.c,
 * freertos/drivers/libc/stdio_wrap.c).
 *
 * A console turns "\n" into "\r\n" (unless "\r" came just before, also at
 * the end of the previous write), as the Pico SDK's stdio does for a
 * driver, but per console.  Its writes are serialized by a mutex of its
 * own, so a stalled terminal holds up only the tasks that write to it.
 * A write from an interrupt handler is dropped (FreeRTOS cannot block
 * there and the USB code takes a lock).
 *
 * A console is also the session's shell backend (struct shell of
 * <zephyr/shell/shell.h>, zephyr_os_shell.h) and holds its getopt state
 * (<zephyr/sys/sys_getopt.h>, getopt_state.c), so that two sessions can
 * run commands at the same time.
 */

#ifndef FREERTOS_DRIVERS_SHELL_NANO_CONSOLE_H
#define FREERTOS_DRIVERS_SHELL_NANO_CONSOLE_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_getopt.h>

/* The thread local storage pointer of a task that holds its console. */
#define NANO_CONSOLE_TLS_INDEX 0

#if configNUM_THREAD_LOCAL_STORAGE_POINTERS <= NANO_CONSOLE_TLS_INDEX
#error "nano_console.h needs configNUM_THREAD_LOCAL_STORAGE_POINTERS >= 1"
#endif

struct nano_console;

/* Writes LEN bytes to the terminal as they are (the line ends are already
 * "\r\n"), from a task, with the console's mutex held.  Drops what the
 * terminal cannot take (no terminal now, or it stopped reading); must not
 * block for long.
 */
typedef void (*nano_console_out_t)(struct nano_console *con, const char *buf, size_t len);

struct nano_console {
	const char *name;
	nano_console_out_t out;
	void *ctx; /* for out */
	SemaphoreHandle_t lock;
	StaticSemaphore_t lock_mem;
	/* How long a task that is not bound to this console waits for its
	 * mutex (nano_console_write()); the bound tasks wait 1 s.
	 */
	uint32_t other_lock_timeout_ms;
	bool last_cr;                   /* the last byte written was '\r' */
	struct shell sh;                /* the session's backend */
	struct sys_getopt_state getopt; /* the session's getopt state */
};

/* Sets up CON (before the scheduler starts, or in a task). */
void nano_console_init(struct nano_console *con, const char *name, nano_console_out_t out,
		       void *ctx);

/* The console of the tasks that are not bound to one (by default none:
 * their output is dropped).
 */
void nano_console_set_default(struct nano_console *con);

/* Binds TASK (NULL: the calling task) to CON (NULL: unbinds it). */
void nano_console_bind(TaskHandle_t task, struct nano_console *con);

/* The console the calling task is bound to, or NULL. */
struct nano_console *nano_console_bound(void);

/* Where the calling context's stdout and stderr go: its bound console, else
 * the default one; NULL in an interrupt handler.
 */
struct nano_console *nano_console_current(void);

/* Writes LEN bytes to CON with the line ends turned into "\r\n".  Nothing
 * for CON == NULL or in an interrupt handler.  Before the scheduler starts
 * the mutex is not taken.  A writer that has waited for the mutex 1 s (a
 * task bound to CON) or CON's other_lock_timeout_ms (any other task) drops
 * its text.
 */
void nano_console_write(struct nano_console *con, const char *buf, size_t len);

/* How long the tasks that are not bound to CON wait for its mutex (by
 * default 1 s, as the bound ones): a console whose terminal may keep its
 * session's task in a write for long (the Telnet client that stops
 * reading) gives the other tasks a short wait, so that they drop their
 * text instead of stalling.
 */
void nano_console_set_other_lock_timeout(struct nano_console *con, uint32_t ms);

/* Takes CON's mutex (waiting up to TIMEOUT_MS) for work on its terminal
 * outside nano_console_write(), such as sending what the terminal has
 * buffered; false when it was not free in time.
 */
bool nano_console_lock(struct nano_console *con, uint32_t timeout_ms);
void nano_console_unlock(struct nano_console *con);

/* Formats into a buffer on the stack (a longer text: on the heap, or cut
 * when that fails) and writes it with nano_console_write().
 */
void nano_console_vprintf(struct nano_console *con, const char *fmt, va_list args)
	__attribute__((format(printf, 2, 0)));

#endif /* FREERTOS_DRIVERS_SHELL_NANO_CONSOLE_H */
