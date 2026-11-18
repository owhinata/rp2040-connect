/* SPDX-License-Identifier: MIT */
/* The "bootsel" command: the FreeRTOS build's cmd_bootsel()
 * (freertos/app/usb-console/src/commands.c), with the same message.  It
 * restarts into the boot ROM's USB loader (the RPI-RP2 drive), as holding
 * the board's REC pad low at reset does, through the board's
 * nano_reset_usb_boot() (<arch/board/nano_reset.h>).  Arguments are
 * ignored.  The message has 100 ms to reach the host (USB or Telnet) first,
 * as in the FreeRTOS build (RESTART_DELAY_MS).
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>

#include <arch/board/nano_reset.h>

#define BOOTSEL_DELAY_MS 100

int main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	printf("Entering BOOTSEL (the RPI-RP2 drive)\n");
	fflush(stdout);
	usleep(BOOTSEL_DELAY_MS * 1000);
	nano_reset_usb_boot();
}
