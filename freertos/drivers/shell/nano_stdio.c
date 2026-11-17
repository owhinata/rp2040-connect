/* SPDX-License-Identifier: MIT */
/* The Pico SDK's stdio on the consoles (nano_stdio.h). */

#include <stdbool.h>
#include <stdio.h>

#include "pico/stdio.h"
#include "pico/stdio/driver.h"

#include "nano_console.h"
#include "nano_stdio.h"

#if PICO_STDOUT_MUTEX
#error "the FreeRTOS build sets PICO_STDOUT_MUTEX 0: each console has a mutex"
#endif

static void out_chars(const char *buf, int len)
{
	if (len > 0) {
		nano_console_write(nano_console_current(), buf, (size_t)len);
	}
}

static stdio_driver_t console_driver = {
	.out_chars = out_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
	.crlf_enabled = false,
#endif
};

void nano_stdio_init(stdio_driver_t *replaced)
{
	stdio_set_driver_enabled(replaced, false);
	stdio_set_driver_enabled(&console_driver, true);
	setvbuf(stdout, NULL, _IONBF, 0);
}
