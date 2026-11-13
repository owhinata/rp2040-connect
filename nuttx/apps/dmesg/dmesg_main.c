/* SPDX-License-Identifier: Apache-2.0 */
/* "dmesg": print the RAM log.  cmd_dmesg() of the Zephyr build's
 * zephyr/app/usb-console/src/log_ram.c: the log is printed in pieces of up
 * to 96 bytes so that no second copy of it is needed, the position counts
 * bytes from boot (log_ram_piece()), which keeps it valid when new messages
 * push old ones out while printing, and printing stops after 2 * 4096
 * bytes in case logging keeps up with it.  Arguments are ignored.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "log_ram.h"

int main(int argc, FAR char *argv[])
{
  char piece[97];
  uint64_t position = 0;
  bool first = true;
  bool any = false;
  size_t printed;

  (void)argc;
  (void)argv;

  /* Bounded in case logging keeps up with printing. */

  for (printed = 0; printed < 2U * LOG_RAM_BUFFER_SIZE; )
    {
      size_t n = log_ram_piece(&position, &first, piece, sizeof(piece) - 1);

      if (n == 0)
        {
          break;
        }

      piece[n] = '\0';
      fputs(piece, stdout);
      position += n;
      printed += n;
      any = true;
    }

  if (!any)
    {
      puts("No logs.");
    }

  fflush(stdout);
  return 0;
}
