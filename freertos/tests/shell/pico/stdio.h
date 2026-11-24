/* SPDX-License-Identifier: MIT */
/* The part of the Pico SDK's "pico/stdio.h" that
 * common/pico_sdk/drivers/libc/stdio_wrap.c uses, for the host test of the shell
 * (freertos/tests/shell/main.c implements it as pico_stdio does: on the
 * consoles of shell_console.h).
 */

#ifndef FREERTOS_TESTS_SHELL_PICO_STDIO_H
#define FREERTOS_TESTS_SHELL_PICO_STDIO_H

#include <stdbool.h>

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation);

#endif /* FREERTOS_TESTS_SHELL_PICO_STDIO_H */
