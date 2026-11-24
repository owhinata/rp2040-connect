/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode: the board's USB serial port relayed byte for
 * byte to the NINA-W102's UART (the ESP32's UART0, which its ROM bootloader
 * uses), with the host's DTR and RTS on the module's GPIO0 and RESETN, so
 * that esptool on the PC reads and writes the module's flash through the
 * board, as WiFiNINA's SerialNINAPassthrough sketch (mbed core) does,
 * without replacing the board's firmware.
 *
 * The UART's pins are those of the NINA's SPI link (nina_bus.h: GPIO8 is
 * SPI1's MISO and UART1's TX, GPIO9 the chip select and UART1's RX), so the
 * link must not run: the mode is a start of its own.  "nina passthrough
 * confirm" (common/shell/nina_shell.c; the NuttX build's nuttx/apps/nina)
 * calls nina_passthrough_request(), which leaves a request in four words
 * that survive a restart (the RP2040's watchdog scratch registers 0..3)
 * and restarts the board.  The next start asks nina_passthrough_check()
 * before anything else: it reads and clears the request, so the RESET
 * button, any later restart and a fatal error start the normal firmware
 * again.  With the request the build starts nothing of the normal firmware
 * that uses the module, the SPI link or the USB port, and runs
 * nina_passthrough_run() instead.  Nothing is written to the flash.
 *
 * nina_passthrough.c is the mode itself, on two boundaries:
 *
 *   nina_passthrough_bus.h  the module's side: the request's words, the
 *                           restart, RESETN and GPIO0, the UART with its
 *                           interrupt, the LED (each build's chip code:
 *                           common/drivers/nina/port/zephyr,
 *                           common/drivers/nina/port/pico_sdk, the NuttX board's src)
 *   nina_passthrough_os.h   the host's side: the time, the writes to the
 *                           USB serial port, the log, the UART interrupt's
 *                           part of the OS (the OS bindings,
 *                           common/drivers/nina/<os>/nina_passthrough_os.c)
 *
 * The OS binding runs the mode in its own way (Zephyr: the USB stack's
 * messages, the CDC ACM callback and a 1 ms tick on work queues; FreeRTOS:
 * one loop around TinyUSB without the scheduler; NuttX: cdcacm's callback
 * in the USB interrupt and three threads) and calls the functions below at
 * its events.  nina_passthrough.c explains the mode.  The host test is
 * common/tests/nina_passthrough.c.
 */

#ifndef NINA_PASSTHROUGH_H_
#define NINA_PASSTHROUGH_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Closing the port while its rate is this one ends the mode (the board
 * restarts normally), as the "1200 bps touch" of Arduino boards does: for
 * example "stty -F /dev/ttyACM0 300".  esptool never uses it.
 */
#define NINA_PASSTHROUGH_EXIT_BAUD 300u

/* The buffer from the module to the host: 178 ms at 921600 bps. */
#define NINA_PASSTHROUGH_RING_SIZE 16384u

/* The request ------------------------------------------------------------*/

/* Whether the last run asked for the mode; clears the request, so that any
 * later start is a normal one.  Once, first thing at the start.
 */
bool nina_passthrough_check(void);

/* Whether this start runs the mode (nina_passthrough_check() found the
 * request).
 */
bool nina_passthrough_active(void);

/* Leaves the request for the next start and restarts the board.  The
 * caller gives its last message time to reach the host first.
 */
__attribute__((noreturn)) void nina_passthrough_request(void);

/* The mode, instead of the normal firmware: never returns (it ends with a
 * restart into the normal firmware).  The OS binding's: Zephyr's main(),
 * usb_console_early_init() on the Pico SDK, NuttX's init task.
 */
__attribute__((noreturn)) void nina_passthrough_run(void);

/* For the OS bindings -----------------------------------------------------*/

/* The module held in reset (the lines of a closed port), the UART's rate
 * at 115200 bps.  Before nina_passthrough_begin(), as early as the build
 * can.
 */
void nina_passthrough_hold(void);

/* RING (NINA_PASSTHROUGH_RING_SIZE bytes) as the buffer to the host, the
 * UART with its interrupt, the module's lines, the LED; the log's first
 * line.
 */
void nina_passthrough_begin(uint8_t *ring);

/* The UART's interrupt (the bus's handler calls it): its receive FIFO into
 * the buffer, then nina_passthrough_os_uart_irq().
 */
void nina_passthrough_uart_isr(void);

/* Host to module: as many of the host's LEN bytes at DATA as the UART's
 * transmit FIFO takes now.  Returns their number.  One caller at a time.
 */
uint32_t nina_passthrough_to_module(const uint8_t *data, uint32_t len);

/* Module to host: the buffer to nina_passthrough_os_write() until it is empty or
 * the host takes no more.  Returns true when some bytes went.  The
 * buffer's consumer: one caller at a time, not from the UART's interrupt.
 */
bool nina_passthrough_to_host(void);

/* The buffer emptied without the host (USB not configured): the bytes are
 * counted as discarded.  The consumer, as nina_passthrough_to_host().
 */
void nina_passthrough_discard(void);

/* Whether bytes from the module wait in the buffer (from any context). */
bool nina_passthrough_waiting(void);

/* The host's DTR and RTS, with RATE the rate of its line coding at the
 * time: the module's lines follow, and closing the port at
 * NINA_PASSTHROUGH_EXIT_BAUD asks for the way out (nina_passthrough_exit_requested()).
 * Returns true when this change closed the port (DTR set, then clear).
 */
bool nina_passthrough_host_lines(bool dtr, bool rts, uint32_t rate);

/* The host's line coding at RATE: the UART follows (8N1 always), but not
 * to NINA_PASSTHROUGH_EXIT_BAUD or out of its range.
 */
void nina_passthrough_host_rate(uint32_t rate);

/* Whether the port was closed at NINA_PASSTHROUGH_EXIT_BAUD. */
bool nina_passthrough_exit_requested(void);

/* The module's lines at the time (GPIO0 is released 750 ms after a start
 * into the module's own firmware) and the LED (5 Hz).  Returns the
 * milliseconds until it has something to do again (at least 1).
 */
uint32_t nina_passthrough_tick(void);

/* A line of the log with the counts, after WHY. */
void nina_passthrough_log_counts(const char *why);

/* The UART's interrupt off: nothing moves from the module any more. */
void nina_passthrough_stop(void);

/* The way out, in a thread (or the loop), after nina_passthrough_stop():
 * the module reset into its own firmware (GPIO0 high, RESETN low for
 * 10 ms, 750 ms, GPIO0 released), the LED off, 100 ms, the restart.
 */
__attribute__((noreturn)) void nina_passthrough_leave(void);

#ifdef __cplusplus
}
#endif

#endif /* NINA_PASSTHROUGH_H_ */
