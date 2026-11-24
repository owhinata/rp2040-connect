/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of <zephyr/shell/shell.h> of the Zephyr compatibility
 * layer (common/compat/include/zephyr/shell/shell.h, which includes this file):
 * struct shell, a shell backend of common/shell/port/nuttx/zshell_os_impl.h.  A
 * backend is the terminal of an nsh session, known by the path of the
 * command's stdout (/dev/ttyACM0 for USB, /dev/telnet0 for Telnet), and
 * the task of the command that runs on it.  Part of the NuttX
 * implementation of the layer (common/compat/port/nuttx, see
 * nuttx/drivers/README.md).
 */

#ifndef __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_OS_SHELL_H
#define __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_OS_SHELL_H

#include <sys/types.h>

#define Z_SHELL_PATH_MAX 32

struct shell
{
  char path[Z_SHELL_PATH_MAX];  /* the session's terminal, "" if unused */
  pid_t cmd_pid;                /* the task of its running command, or 0 */
};

#endif /* __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_OS_SHELL_H */
