/****************************************************************************
 * boards/arduino-nano-rp2040-connect/include/nano_reset.h
 *
 * SPDX-License-Identifier: MIT
 *
 ****************************************************************************/

/* The board's restarts other than NuttX's board_reset() (the "reboot"
 * command, boards/arm/rp2040/common/src/rp2040_reset.c, a SYSRESETREQ):
 * src/nano_reset.c.  The board code and the applications (a flat build)
 * call them through <arch/board/nano_reset.h>.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_RESET_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Restarts into the boot ROM's USB loader (BOOTSEL: the RPI-RP2 drive and
 * the PICOBOOT interface), as holding the board's BOOTSEL (REC) pad low at
 * reset does: the ROM's reset_usb_boot() with no activity LED and both
 * interfaces on, the Pico SDK's reset_usb_boot(0, 0) (FreeRTOS build's
 * "bootsel").
 */

void nano_reset_usb_boot(void) noreturn_function;

/* Restarts the board at once through the watchdog, as the Pico SDK's
 * watchdog_reboot(0, 0, 0): everything but the oscillators is reset, the
 * watchdog's scratch registers 0..3 are kept, scratch 4 is cleared (the
 * boot ROM starts the flash's image, not a vector left there).
 */

void nano_watchdog_reboot(void) noreturn_function;

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_RESET_H */
