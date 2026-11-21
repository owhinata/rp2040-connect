/* SPDX-License-Identifier: MIT */
/* The module's side of the NINA passthrough mode (nina_passthrough.c): the
 * request's words and the restart, the module's RESETN and GPIO0, the UART
 * to the module's UART0, the LED.  It is the chip's and the board's, so
 * each build implements it for its chip, on the RP2040's registers
 * or its SDK: common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c,
 * common/drivers/nina/port/freertos/nina_passthrough_bus_rp2040.c,
 * common/drivers/nina/port/nuttx/nina_passthrough_bus_rp2040.c; the
 * host tests replace it with fakes.  The board (the mbed core's
 * variants/NANO_RP2040_CONNECT; nina_bus.h):
 *
 *   RP2040   in this mode                       SPI link of the normal firmware
 *   GPIO8    UART1 TX -> the module's UART0 RX  SPI1 RX (MISO)
 *   GPIO9    UART1 RX <- the module's UART0 TX  CS, active low
 *   GPIO2    the module's GPIO0 (boot strap)    IRQ
 *   GPIO3    the module's RESETN, active low    RESET
 *   GPIO6    the LED
 *
 * The functions run in the OS binding's contexts (nina_passthrough_os.h);
 * those of the receive side also in the UART's interrupt.
 */

#ifndef NINA_PASSTHROUGH_BUS_H_
#define NINA_PASSTHROUGH_BUS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The request and the restart ----------------------------------------------*/

/* Four words that a restart through nina_passthrough_bus_restart() keeps and any
 * other start does not have to (the RP2040's watchdog scratch registers
 * 0..3, which the RESET button, the chip's RUN pin, clears).  The fatal
 * error record (common/log/fault_record.h) uses the same registers.
 */
#define NINA_PASSTHROUGH_BUS_MARK_WORDS 4

void nina_passthrough_bus_mark_get(uint32_t s[NINA_PASSTHROUGH_BUS_MARK_WORDS]);
void nina_passthrough_bus_mark_set(const uint32_t s[NINA_PASSTHROUGH_BUS_MARK_WORDS]);

/* A restart of the whole chip at once, keeping the words (the Pico SDK's
 * watchdog_reboot(0, 0, 0)).
 */
__attribute__((noreturn)) void nina_passthrough_bus_restart(void);

/* The module's lines -------------------------------------------------------*/

enum nina_passthrough_bus_gpio0 {
	NINA_PASSTHROUGH_BUS_GPIO0_LOW,      /* driven low: the ROM's download mode at reset */
	NINA_PASSTHROUGH_BUS_GPIO0_HIGH,     /* driven high: a normal start at reset */
	NINA_PASSTHROUGH_BUS_GPIO0_RELEASED, /* an input with the pull-down (the module drives it) */
};

/* RESETN driven low, without pulls (the module in reset); GPIO0 an input
 * with the pull-down, or driven high (the strap of the closed port; a
 * build that holds the module long before nina_passthrough_begin() sets it
 * at once).
 */
void nina_passthrough_bus_lines_init(void);

/* RESETN high (RELEASED: the module runs) or low. */
void nina_passthrough_bus_resetn(bool released);

void nina_passthrough_bus_gpio0(enum nina_passthrough_bus_gpio0 state);

/* GPIO8 driven by the UART's TX, or an input (the module's firmware
 * drives that net as its SPI's MISO).
 */
void nina_passthrough_bus_uart_tx_connect(bool connected);

/* The UART -----------------------------------------------------------------*/

/* The UART at BAUD, 8N1, FIFOs on, no flow control, its receive interrupt
 * enabled in the UART at a few bytes in the FIFO and after a pause without
 * more; GPIO8 an input until nina_passthrough_bus_uart_tx_connect(), GPIO9 the
 * UART's RX with the pull-up (the line idles high while the module is in
 * reset).
 */
void nina_passthrough_bus_uart_init(uint32_t baud);

/* The rate, 8N1 kept; a character in flight ends first. */
void nina_passthrough_bus_uart_set_baud(uint32_t baud);

/* The UART's interrupt on, at the build's highest priority; its handler
 * calls nina_passthrough_uart_isr().  Off: nothing of it runs any more.
 */
void nina_passthrough_bus_uart_irq_on(void);
void nina_passthrough_bus_uart_irq_off(void);

/* A received word: the byte in bits 0..7 and its flags (the layout of the
 * PL011's data register).
 */
#define NINA_PASSTHROUGH_BUS_UART_FRAMING (1u << 8)
#define NINA_PASSTHROUGH_BUS_UART_BREAK   (1u << 10) /* the line held low: no data */
#define NINA_PASSTHROUGH_BUS_UART_OVERRUN (1u << 11) /* bytes lost before this one */

/* The next word of the receive FIFO into *WORD; false when it is empty. */
bool nina_passthrough_bus_uart_rx(uint32_t *word);

/* C into the transmit FIFO; false when it is full. */
bool nina_passthrough_bus_uart_tx(uint8_t c);

/* The transmit interrupt, for an OS binding that writes the rest of the
 * host's bytes from the UART's interrupt (Zephyr): on (when the FIFO falls
 * to its level), off, and whether it is on.
 */
void nina_passthrough_bus_uart_tx_irq(bool on);
bool nina_passthrough_bus_uart_tx_irq_on(void);

/* For an OS binding that writes from a thread (NuttX): waits, in that
 * thread, until the transmit FIFO has room again (or a bound of a few
 * milliseconds passes).
 */
void nina_passthrough_bus_uart_tx_wait(void);

/* The LED --------------------------------------------------------------------*/

/* An output, off. */
void nina_passthrough_bus_led_init(void);
void nina_passthrough_bus_led(bool on);

#ifdef __cplusplus
}
#endif

#endif /* NINA_PASSTHROUGH_BUS_H_ */
