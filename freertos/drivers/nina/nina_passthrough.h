/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode of the FreeRTOS build: the USB CDC ACM port
 * relayed byte for byte to the NINA-W102's UART (the ESP32's UART0, which
 * its ROM bootloader uses), with the host's DTR and RTS on the module's
 * GPIO0 and RESETN, so that esptool on the PC reads and writes the module's
 * flash through the board, as WiFiNINA's SerialNINAPassthrough sketch does
 * without replacing this firmware.  nina_passthrough.c explains it.
 *
 * "nina passthrough confirm" (this build's subcommand of "nina") leaves a
 * request in the watchdog's scratch registers and restarts the board.
 * main() asks nina_passthrough_requested() first thing; when it says so,
 * main() starts nothing of the normal firmware (no task, no SPI, no Wi-Fi)
 * and runs nina_passthrough_run() instead, which never returns: the mode
 * ends with a restart into the normal firmware.
 */

#ifndef FREERTOS_DRIVERS_NINA_NINA_PASSTHROUGH_H
#define FREERTOS_DRIVERS_NINA_NINA_PASSTHROUGH_H

#include <stdbool.h>

/* Whether the last run asked for the mode; clears the request, so that any
 * later start (the RESET button, a watchdog restart, a fatal error) is a
 * normal one.  Call before fault_boot(), which clears the same registers.
 */
bool nina_passthrough_requested(void);

/* The mode, before the scheduler: never returns. */
void __attribute__((noreturn)) nina_passthrough_run(void);

#endif /* FREERTOS_DRIVERS_NINA_NINA_PASSTHROUGH_H */
