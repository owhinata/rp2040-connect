/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode of the Zephyr build (CONFIG_NANO_NINA_PASSTHROUGH):
 * the USB CDC ACM port relayed byte for byte to the NINA-W102's UART (the
 * ESP32's UART0, which its ROM bootloader uses), with the host's DTR and
 * RTS on the module's GPIO0 and RESETN, so that esptool on the PC reads
 * and writes the module's flash through the board, as WiFiNINA's
 * SerialNINAPassthrough sketch does, without replacing this firmware.  The
 * FreeRTOS build has the same mode (freertos/drivers/nina/
 * nina_passthrough.c); both take their decisions from
 * common/drivers/nina/nina_passthrough_logic.h.  nina_passthrough.c
 * explains this one.
 *
 * "nina passthrough confirm" (nina_passthrough_cmd(), a subcommand of the
 * shared "nina", common/drivers/nina/nina_shell.c) leaves a request in the
 * watchdog's scratch registers and restarts the board.  The next start
 * sees it before the drivers start; Zephyr then starts as usual, but the
 * NINA's SPI link reports no bus (nano_nina_bus_ready()), so that nothing
 * touches the module's lines, and main() runs nina_passthrough_run()
 * instead of the application, which never returns: the mode ends with a
 * restart into the normal firmware.
 */

#ifndef ZEPHYR_APP_DRIVERS_NINA_NINA_PASSTHROUGH_H
#define ZEPHYR_APP_DRIVERS_NINA_NINA_PASSTHROUGH_H

#include <stdbool.h>
#include <stddef.h>

#ifdef CONFIG_NANO_NINA_PASSTHROUGH

#include <zephyr/shell/shell.h>

/* Whether this start is the mode's (the request was there; it is cleared,
 * so that the next start, by the RESET button or any restart, is a normal
 * one).
 */
bool nina_passthrough_active(void);

/* The mode, from main() instead of the application: never returns. */
__attribute__((noreturn)) void nina_passthrough_run(void);

/* "nina passthrough confirm": the request, then the restart. */
int nina_passthrough_cmd(const struct shell *sh, size_t argc, char **argv);

/* The subcommand's entry in the shared "nina" set (nina_shell.c). */
#define NINA_PASSTHROUGH_SHELL_CMD                                                                 \
	SHELL_CMD_ARG(passthrough, NULL,                                                           \
		      "Restart into the NINA passthrough for esptool: passthrough confirm\n"       \
		      "(USB <-> the NINA's UART; leave it by closing the port at 300 bps, "        \
		      "or with RESET)",                                                            \
		      nina_passthrough_cmd, 2, 0),

#else

static inline bool nina_passthrough_active(void)
{
	return false;
}

#define NINA_PASSTHROUGH_SHELL_CMD

#endif /* CONFIG_NANO_NINA_PASSTHROUGH */

#endif /* ZEPHYR_APP_DRIVERS_NINA_NINA_PASSTHROUGH_H */
