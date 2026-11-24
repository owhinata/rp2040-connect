/* SPDX-License-Identifier: MIT */
/* The Pico SDK's stdio on the consoles of shell_console.h.
 *
 * Every output of the SDK's stdio (printf(), puts(), putchar(), which
 * pico_stdio wraps, and newlib's write() of stdout and stderr) ends in
 * stdio_put_string(), which hands it to the enabled stdio drivers.
 * shell_stdio_init() takes the console driver of the SDK (the USB CDC ACM
 * one) off that list and puts one driver in its place that writes to the
 * calling task's console (shell_console_current()).  The SDK driver stays
 * initialized: the console that owns the terminal calls it directly.  The
 * SDK's CR LF translation is off for the new driver (the console does it),
 * and so is its global stdout mutex (PICO_STDOUT_MUTEX 0 in the build; each
 * console has its own).  newlib's stdout is made unbuffered, so that what
 * reaches it comes in order with printf().
 */

#ifndef COMMON_PICO_SDK_DRIVERS_SHELL_SHELL_STDIO_H
#define COMMON_PICO_SDK_DRIVERS_SHELL_SHELL_STDIO_H

#include "pico/stdio/driver.h"

/* After stdio_init_all(): REPLACED (&stdio_usb) leaves the SDK's list. */
void shell_stdio_init(stdio_driver_t *replaced);

#endif /* COMMON_PICO_SDK_DRIVERS_SHELL_SHELL_STDIO_H */
