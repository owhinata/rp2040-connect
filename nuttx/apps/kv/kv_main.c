/* SPDX-License-Identifier: Apache-2.0 */
/* The "kv" builtin command: the Zephyr build's SHELL_CMD_REGISTER(kv, ...)
 * of common/imu/kv_store.c, with the same syntax and output.  The command
 * set ("set", "sethex", "get", "delete", and "wipe" of
 * common/compat/include/kv_store_os.h) and its handlers are in that file,
 * next to the store, as in the Zephyr build; it is not this application's main
 * source, so it is compiled with Z_SHELL_CMD_NAMED_MAIN and the
 * compatibility layer's SHELL_CMD_REGISTER() makes z_shell_main_kv()
 * (<zephyr/shell/shell.h>), which runs the command on common/shell/zshell.c,
 * with the Zephyr shell's help and argument errors.
 */

#include <nuttx/config.h>

/* common/imu/kv_store.c */

int z_shell_main_kv(int argc, char *argv[]);

int main(int argc, char *argv[])
{
  return z_shell_main_kv(argc, argv);
}
