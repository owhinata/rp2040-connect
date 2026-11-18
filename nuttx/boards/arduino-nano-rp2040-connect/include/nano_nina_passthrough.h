/****************************************************************************
 * boards/arduino-nano-rp2040-connect/include/nano_nina_passthrough.h
 *
 * SPDX-License-Identifier: MIT
 *
 ****************************************************************************/

/* The NINA passthrough mode (CONFIG_NANO_NINA_PASSTHROUGH,
 * src/nano_nina_passthrough.c): the USB CDC ACM port relayed byte for byte
 * to the NINA-W102's UART (the ESP32's UART0, which its ROM bootloader
 * uses), with the host's DTR and RTS on the module's GPIO0 and RESETN, so
 * that esptool on the PC reads and writes the module's flash through the
 * board, as WiFiNINA's SerialNINAPassthrough sketch does, without replacing
 * this firmware.  The FreeRTOS build's mode (freertos/drivers/nina/
 * nina_passthrough.c) with the same decisions (nina_passthrough_logic.h of
 * common/drivers/nina).
 *
 * "nina passthrough confirm" (nuttx/apps/nina) calls
 * nano_nina_passthrough_request(), which leaves a request in the watchdog's
 * scratch registers and restarts the board.  The board's start sees it
 * (rp2040_boardinitialize()) and clears it; then nano_bringup() sets up
 * the USB device only (no SPI1, NINA, I2C, sensor, network device or
 * key-value flash), and the init task (nuttx/apps/init) runs
 * nano_nina_passthrough_run() instead of everything else (no nsh, Telnet,
 * Wi-Fi, MQTT or time synchronization).  The mode ends with a restart into
 * the normal firmware.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_NINA_PASSTHROUGH_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_NINA_PASSTHROUGH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Whether this start runs the mode: the last run asked for it. */

bool nano_nina_passthrough_active(void);

/* Leaves the request for the next start and restarts the board through the
 * watchdog (nano_watchdog_reboot()).  The caller waits for its last message
 * to reach the host first.
 */

void nano_nina_passthrough_request(void) noreturn_function;

/* The mode, in the init task: never returns. */

void nano_nina_passthrough_run(void) noreturn_function;

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_NINA_PASSTHROUGH_H */
