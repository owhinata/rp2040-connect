/* SPDX-License-Identifier: Apache-2.0 */
/* The "ntp" builtin command: the Zephyr build's SHELL_CMD_REGISTER(ntp, ...)
 * of src/time_sync.c.  The command set ("status") and its handler
 * (cmd_ntp_status()) are in nuttx/apps/imu/time_sync.c, next to the state
 * they show, as in the Zephyr build; this runs them on nuttx/apps/subcmd,
 * which prints the Zephyr shell's help and argument errors.
 */

#include <nuttx/config.h>

#include "subcmd.h"

/* nuttx/apps/imu/time_sync.c */

extern const struct subcmd ntp_root;

int main(int argc, char *argv[])
{
  return subcmd_main(&ntp_root, argc, argv);
}
