/* SPDX-License-Identifier: Apache-2.0 */
/* The "ntp" builtin command: the Zephyr build's SHELL_CMD_REGISTER(ntp, ...)
 * of common/imu/time_sync.c.  The command set ("status") and its handler
 * (cmd_ntp_status()) are in that file, next to the state they show, as in
 * the Zephyr build; it is not this application's main source, so it is
 * compiled with Z_SHELL_CMD_NAMED_MAIN and the compatibility layer's
 * SHELL_CMD_REGISTER() makes z_shell_main_ntp() (<zephyr/shell/shell.h>),
 * which runs the command on common/shell/zshell.c, with the Zephyr
 * shell's help and argument errors.
 */

#include <nuttx/config.h>

/* common/imu/time_sync.c */

int z_shell_main_ntp(int argc, char *argv[]);

int main(int argc, char *argv[])
{
  return z_shell_main_ntp(argc, argv);
}
