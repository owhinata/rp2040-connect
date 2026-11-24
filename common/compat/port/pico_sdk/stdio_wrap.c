/* SPDX-License-Identifier: MIT */
/* newlib's functions of stdout and stderr that the shared sources call
 * (common/shell/zshell.c and subcmd.c: fputs(), fwrite(), fprintf(),
 * vfprintf(), fflush(); others may use fputc() and putc()), linked with
 * -Wl,--wrap=<name> (common/compat/port/pico_sdk/compat.cmake).  On stdout and
 * stderr they go the way of the Pico SDK's printf() (pico_stdio's
 * vprintf() and stdio_put_string(), which write to the calling task's
 * console, common/compat/port/pico_sdk/shell_stdio.c) instead of newlib's FILE:
 *
 * - order: newlib would buffer stdout while printf() and putchar() of the
 *   SDK go out at once; the help layout of subcmd.c mixes fwrite() and
 *   putchar() in one line;
 * - stack: newlib's fprintf() on an unbuffered stream formats into a
 *   BUFSIZ (1024-byte) buffer on the stack (__sbprintf()), the SDK's
 *   printf() uses 128 bytes (PICO_STDIO_STACK_BUFFER_SIZE).
 *
 * There are no other streams (no file system): on them the calls fail
 * (EOF or 0, errno EBADF), which also keeps newlib's own vfprintf()
 * (about 9 KB) out of the image.  The two streams have nothing to flush
 * (the console writes at once).
 */

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdio.h"

/* pico_stdio's printf path, by its own name: the compiler may turn a call
 * of vprintf() into one of vfprintf(stdout, ...), which is wrapped to this
 * file again (GCC does so on the host).
 */
int __wrap_vprintf(const char *fmt, va_list args);

int __wrap_fputs(const char *s, FILE *fp);
size_t __wrap_fwrite(const void *buf, size_t size, size_t n, FILE *fp);
int __wrap_fputc(int c, FILE *fp);
int __wrap_putc(int c, FILE *fp);
int __wrap_vfprintf(FILE *fp, const char *fmt, va_list args);
int __wrap_fprintf(FILE *fp, const char *fmt, ...);
int __wrap_fflush(FILE *fp);

static bool is_console(FILE *fp)
{
	return fp == stdout || fp == stderr;
}

static int no_stream(void)
{
	errno = EBADF;
	return EOF;
}

int __wrap_fputs(const char *s, FILE *fp)
{
	if (!is_console(fp)) {
		return no_stream();
	}
	stdio_put_string(s, (int)strlen(s), false, true);
	return 0;
}

size_t __wrap_fwrite(const void *buf, size_t size, size_t n, FILE *fp)
{
	if (!is_console(fp)) {
		(void)no_stream();
		return 0;
	}
	if (size != 0 && n != 0) {
		stdio_put_string(buf, (int)(size * n), false, true);
	}
	return n;
}

static int console_putc(int c)
{
	char ch = (char)c;

	stdio_put_string(&ch, 1, false, true);
	return (unsigned char)ch;
}

int __wrap_fputc(int c, FILE *fp)
{
	return is_console(fp) ? console_putc(c) : no_stream();
}

int __wrap_putc(int c, FILE *fp)
{
	return is_console(fp) ? console_putc(c) : no_stream();
}

int __wrap_vfprintf(FILE *fp, const char *fmt, va_list args)
{
	return is_console(fp) ? __wrap_vprintf(fmt, args) : no_stream();
}

int __wrap_fprintf(FILE *fp, const char *fmt, ...)
{
	va_list args;
	int n;

	va_start(args, fmt);
	n = __wrap_vfprintf(fp, fmt, args);
	va_end(args);
	return n;
}

int __wrap_fflush(FILE *fp)
{
	/* NULL: all streams. */
	return fp == NULL || is_console(fp) ? 0 : no_stream();
}
