/* SPDX-License-Identifier: MIT */
/* The test's part of the Zephyr shell's command handling
 * (common/shell/zshell.c, which includes this file first through
 * common/shell/port/zshell_os.h) for the "i2c" command of main.c: one backend,
 * whose output goes to main.c's capture (test_shell_capture()), in place of
 * the shell engine's consoles (common/shell/port/generic/zshell_os_impl.h).
 */

#ifndef FREERTOS_TESTS_I2C_DW_ZSHELL_OS_IMPL_H
#define FREERTOS_TESTS_I2C_DW_ZSHELL_OS_IMPL_H

#include <stdarg.h>
#include <stddef.h>

#include <zephyr/shell/shell.h>

/* main.c */
void test_shell_capture(const char *fmt, va_list args);

static struct shell test_shell;

static struct shell *zshell_os_backend_get(void)
{
	return &test_shell;
}

void z_shell_backend_vfprintf(const struct shell *sh, enum shell_vt100_color color, const char *fmt,
			      va_list args)
{
	(void)sh;
	(void)color;
	test_shell_capture(fmt, args);
}

static inline void zshell_os_cmd_enter(struct shell *sh)
{
	(void)sh;
}

static inline void zshell_os_cmd_exit(struct shell *sh)
{
	(void)sh;
}

#define ZSHELL_OS_ACTIVE_CMD 1

static inline void zshell_os_active_cmd_set(struct shell *sh, const struct shell_static_entry *cmd)
{
	sh->active_cmd = cmd;
}

static inline const struct shell_static_entry *zshell_os_active_cmd_get(const struct shell *sh)
{
	return sh->active_cmd;
}

#endif /* FREERTOS_TESTS_I2C_DW_ZSHELL_OS_IMPL_H */
