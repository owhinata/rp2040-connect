/* SPDX-License-Identifier: MIT */
/* The console transport of the builds on the Pico SDK
 * (common/compat/shell_engine/
 * shell_serial_os.h, which reads this file and says what each item must
 * do): the Pico SDK's USB CDC ACM driver (stdio_usb, TinyUSB), which
 * stdio_init_all() has set up in main(), and the SDK's stdio on the
 * consoles (shell_stdio.h; newlib's FILE functions of stdout and stderr:
 * common/pico_sdk/drivers/libc/stdio_wrap.c).
 */

#ifndef COMMON_PICO_SDK_DRIVERS_SHELL_SHELL_SERIAL_OS_IMPL_H
#define COMMON_PICO_SDK_DRIVERS_SHELL_SHELL_SERIAL_OS_IMPL_H

#include <stdbool.h>
#include <stddef.h>

#include "pico/stdio_usb.h"

#include "shell_stdio.h"

static inline void shell_serial_os_stdio_init(void)
{
	shell_stdio_init(&stdio_usb);
}

/* Called by the SDK's USB code when characters have arrived: from its
 * interrupt, or from a task that is writing to the console.
 */
static void shell_serial_os_chars_available(void *param)
{
	(void)param;
	shell_serial_rx_ready();
}

static inline void shell_serial_os_rx_init(void)
{
	stdio_usb.set_chars_available_callback(shell_serial_os_chars_available, NULL);
}

static inline bool shell_serial_os_connected(void)
{
	return stdio_usb_connected();
}

static inline int shell_serial_os_read(char *buf, size_t len)
{
	return stdio_usb.in_chars(buf, (int)len);
}

/* The driver drops the text while no terminal has the port open (DTR) and,
 * when the terminal stops reading, waits up to 500 ms for room in its
 * 64-byte buffer and then drops the rest until room comes again
 * (PICO_STDIO_USB_STDOUT_TIMEOUT_US).
 */
static inline void shell_serial_os_write(const char *buf, size_t len)
{
	stdio_usb.out_chars(buf, (int)len);
}

#endif /* COMMON_PICO_SDK_DRIVERS_SHELL_SHELL_SERIAL_OS_IMPL_H */
