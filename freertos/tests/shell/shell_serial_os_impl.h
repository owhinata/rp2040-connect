/* SPDX-License-Identifier: MIT */
/* The host test's console transport (common/compat/shell_engine/
 * shell_serial_os.h): a line the test opens and closes, input the test
 * puts in, and output it captures (freertos/tests/shell/main.c).
 */

#ifndef TESTS_SHELL_SHELL_SERIAL_OS_IMPL_H
#define TESTS_SHELL_SHELL_SERIAL_OS_IMPL_H

#include <stdbool.h>
#include <stddef.h>

extern volatile bool test_serial_open;
extern unsigned int test_serial_rx_inits;

/* Input waiting to be read, and the output (main.c). */
int test_serial_take(char *buf, size_t len);
void test_serial_put_out(const char *buf, size_t len);

static inline void shell_serial_os_stdio_init(void)
{
	/* The test's stdio already writes to shell_console_current(). */
}

static inline void shell_serial_os_rx_init(void)
{
	test_serial_rx_inits++;
}

static inline bool shell_serial_os_connected(void)
{
	return test_serial_open;
}

static inline int shell_serial_os_read(char *buf, size_t len)
{
	return test_serial_take(buf, len);
}

static inline void shell_serial_os_write(const char *buf, size_t len)
{
	test_serial_put_out(buf, len);
}

#endif /* TESTS_SHELL_SHELL_SERIAL_OS_IMPL_H */
