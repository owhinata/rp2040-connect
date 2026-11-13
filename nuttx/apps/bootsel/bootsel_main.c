/****************************************************************************
 * nuttx/apps/bootsel/bootsel_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * "bootsel": reboot into the RP2040 USB bootloader (BOOTSEL, the RPI-RP2
 * drive) so that a new UF2 can be copied without pressing RESET twice.  A
 * development command, used by nuttx/script/flash.sh; not in the Zephyr
 * build.
 *
 * Only "bootsel" with no argument reboots.  With any argument ("--help",
 * "-h", anything) it prints the usage and returns: the Zephyr "kernel
 * reboot" ignores its arguments and reboots even for "--help", which is
 * easy to trigger by accident.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* After the output has been handed to the USB host (tcdrain), give the
 * program on the host a moment to read it before the device disappears.
 */

#define BOOTSEL_SETTLE_US  200000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  printf("Usage: %s\n"
         "Reboot into the RP2040 USB bootloader (BOOTSEL, drive RPI-RP2).\n"
         "Takes no arguments; with any argument it only prints this.\n",
         progname);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc > 1)
    {
      show_usage(argv[0]);
      return (strcmp(argv[1], "-h") == 0 ||
              strcmp(argv[1], "--help") == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

  printf("Rebooting into BOOTSEL (USB drive RPI-RP2)\n");
  fflush(stdout);

  /* Wait until the serial driver has handed everything to the USB host
   * (cdcacm: until all IN requests have completed; at most 10 s).  On a
   * descriptor that is not a tty this fails at once; the delay below still
   * applies.
   */

  tcdrain(STDOUT_FILENO);
  usleep(BOOTSEL_SETTLE_US);

  nano_reset_usb_boot();

  /* Not reached. */

  return EXIT_FAILURE;
}
