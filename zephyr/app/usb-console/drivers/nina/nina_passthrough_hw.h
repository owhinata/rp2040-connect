/* SPDX-License-Identifier: MIT */
/* The RP2040 hardware of the NINA passthrough mode (nina_passthrough.c):
 * the watchdog's scratch registers and its restart, the module's lines,
 * UART1 and the LED, on the registers of the Pico SDK (hal_rpi_pico: the
 * hardware_structs and hardware_regs headers, and the hardware_gpio,
 * hardware_resets and hardware_clocks functions that Zephyr's SoC and GPIO
 * code build).  The host test (tests/passthrough.c) puts a fake of this
 * header in its place; nothing else here touches the hardware.
 *
 * The pins are the board's (arduino_nano_connect.dts and the mbed core's
 * variant, as freertos/drivers/nina/nina_passthrough.c lists them):
 *
 *   RP2040   in this mode                       normal firmware
 *   GPIO8    UART1 TX -> the module's UART0 RX  SPI1 RX (MISO)
 *   GPIO9    UART1 RX <- the module's UART0 TX  spi1's cs-gpios
 *   GPIO2    the module's GPIO0 (boot strap)    nina_w102's irq-gpios
 *   GPIO3    the module's RESETN, active low    nina_w102's reset-gpios
 *   GPIO11   unused (reset state)               SPI1 TX (MOSI)
 *   GPIO14   unused (reset state)               SPI1 SCK
 *   GPIO6    LED (led0)
 *
 * UART1 stays "disabled" in the devicetree, so that no Zephyr driver
 * claims GPIO8 and GPIO9 in the normal firmware; this mode programs it
 * itself, as the Pico SDK's uart_init() and uart_set_baudrate() do (8N1,
 * FIFOs on, no flow control).
 */

#ifndef ZEPHYR_APP_DRIVERS_NINA_NINA_PASSTHROUGH_HW_H
#define ZEPHYR_APP_DRIVERS_NINA_NINA_PASSTHROUGH_HW_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <hardware/address_mapped.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/resets.h>
#include <hardware/structs/psm.h>
#include <hardware/structs/uart.h>
#include <hardware/structs/watchdog.h>

#include "nina_passthrough_logic.h"

#define PT_HW_PIN_TX     8
#define PT_HW_PIN_RX     9
#define PT_HW_PIN_GPIO0  2
#define PT_HW_PIN_RESETN 3
#define PT_HW_PIN_MOSI   11
#define PT_HW_PIN_SCK    14
#define PT_HW_PIN_LED    6

#define PT_HW_NINA_NODE DT_NODELABEL(nina_w102)

BUILD_ASSERT(DT_GPIO_PIN_BY_IDX(DT_NODELABEL(spi1), cs_gpios, 0) == PT_HW_PIN_RX,
	     "the NINA's chip select is UART1 RX");
BUILD_ASSERT(DT_GPIO_PIN(PT_HW_NINA_NODE, irq_gpios) == PT_HW_PIN_GPIO0,
	     "the NINA's IRQ line is its GPIO0");
BUILD_ASSERT(DT_GPIO_PIN(PT_HW_NINA_NODE, reset_gpios) == PT_HW_PIN_RESETN,
	     "the NINA's reset line");
BUILD_ASSERT(DT_GPIO_PIN(DT_ALIAS(led0), gpios) == PT_HW_PIN_LED, "the LED");
BUILD_ASSERT(DT_REG_ADDR(DT_NODELABEL(uart1)) == UART1_BASE, "UART1");

#define PT_HW_UART_IRQN DT_IRQN(DT_NODELABEL(uart1))

/* The flags of a received word (UARTDR). */
#define PT_HW_DR_OVERRUN UART_UARTDR_OE_BITS
#define PT_HW_DR_BREAK   UART_UARTDR_BE_BITS
#define PT_HW_DR_FRAMING UART_UARTDR_FE_BITS

/* The request: the watchdog's scratch registers 0..3, which survive its
 * restart (not the RESET button: that is the chip's RUN pin).
 */
static inline void pt_hw_mark_get(uint32_t s[NINA_PT_MARK_WORDS])
{
	for (int i = 0; i < NINA_PT_MARK_WORDS; i++) {
		s[i] = watchdog_hw->scratch[i];
	}
}

static inline void pt_hw_mark_set(const uint32_t s[NINA_PT_MARK_WORDS])
{
	for (int i = 0; i < NINA_PT_MARK_WORDS; i++) {
		watchdog_hw->scratch[i] = s[i];
	}
}

/* The watchdog's restart of the whole chip but its oscillators, as the
 * Pico SDK's watchdog_reboot(0, 0, 0) (scratch register 4 not the boot
 * ROM's magic: a normal start from the flash).  Registers 0..3 keep the
 * request.
 */
static inline FUNC_NORETURN void pt_hw_restart(void)
{
	(void)irq_lock();
	hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
	psm_hw->wdsel = PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS);
	watchdog_hw->scratch[4] = 0;
	hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_TRIGGER_BITS);
	for (;;) {
	}
}

/* RESETN driven, without pulls (Zephyr's reset-gpios), low: the module in
 * reset; GPIO0 an input with the pull-down (irq-gpios) until driven.
 */
static inline void pt_hw_lines_init(void)
{
	gpio_init(PT_HW_PIN_RESETN);
	gpio_disable_pulls(PT_HW_PIN_RESETN);
	gpio_put(PT_HW_PIN_RESETN, 0);
	gpio_set_dir(PT_HW_PIN_RESETN, GPIO_OUT);
	gpio_init(PT_HW_PIN_GPIO0);
	gpio_pull_down(PT_HW_PIN_GPIO0);
}

static inline void pt_hw_resetn(bool released)
{
	gpio_put(PT_HW_PIN_RESETN, released);
}

static inline void pt_hw_gpio0(enum nina_pt_gpio0 state)
{
	switch (state) {
	case NINA_PT_GPIO0_LOW:
		gpio_put(PT_HW_PIN_GPIO0, 0);
		gpio_set_dir(PT_HW_PIN_GPIO0, GPIO_OUT);
		break;
	case NINA_PT_GPIO0_HIGH:
		gpio_put(PT_HW_PIN_GPIO0, 1);
		gpio_set_dir(PT_HW_PIN_GPIO0, GPIO_OUT);
		break;
	case NINA_PT_GPIO0_RELEASED:
		gpio_put(PT_HW_PIN_GPIO0, 0);
		gpio_set_dir(PT_HW_PIN_GPIO0, GPIO_IN);
		break;
	}
}

/* GPIO8 to UART1, or a plain input (SIO with the direction in). */
static inline void pt_hw_uart_tx_connect(bool connected)
{
	gpio_set_function(PT_HW_PIN_TX, connected ? GPIO_FUNC_UART : GPIO_FUNC_SIO);
}

/* The divisors for BAUD from clk_peri, as uart_set_baudrate(); when the
 * UART runs, it stops for 15 bit times at the old rate first (a character
 * in flight ends), as the SDK's LCR_H write does, and the dummy LCR_H
 * write latches the divisors.
 */
static inline void pt_hw_uart_set_baud(uint32_t baud)
{
	uart_hw_t *const hw = uart1_hw;
	uint32_t clk = clock_get_hz(clk_peri);
	uint32_t div = 8u * clk / baud + 1u;
	uint32_t ibrd = div >> 7;
	uint32_t fbrd;
	uint32_t cr = hw->cr;

	if (ibrd == 0) {
		ibrd = 1;
		fbrd = 0;
	} else if (ibrd >= 65535) {
		ibrd = 65535;
		fbrd = 0;
	} else {
		fbrd = (div & 0x7f) >> 1;
	}
	if (cr & UART_UARTCR_UARTEN_BITS) {
		uint64_t brdiv = 64u * hw->ibrd + hw->fbrd;

		hw_clear_bits(&hw->cr, UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS |
					       UART_UARTCR_RXE_BITS);
		k_busy_wait((uint32_t)(brdiv * 3750000u / clk) + 1u);
	}
	hw->ibrd = ibrd;
	hw->fbrd = fbrd;
	hw->lcr_h = hw->lcr_h;
	hw->cr = cr;
}

/* UART1 out of reset at BAUD, 8N1, FIFOs on; the receive interrupt at 4
 * bytes in the FIFO (and after 32 bit times without more), the transmit
 * one, when enabled, at 16 or fewer.  GPIO8 a SIO input until
 * pt_hw_uart_tx_connect(), GPIO9 UART1 RX with the pull-up (the line idles
 * high while the module is in reset); SPI1's MOSI and SCK back in their
 * reset state (no function, pull-down), as the FreeRTOS build leaves them.
 */
static inline void pt_hw_uart_init(uint32_t baud)
{
	uart_hw_t *const hw = uart1_hw;

	reset_block_num(RESET_UART1);
	unreset_block_num_wait_blocking(RESET_UART1);
	pt_hw_uart_set_baud(baud);
	hw->lcr_h = (3u << UART_UARTLCR_H_WLEN_LSB) | UART_UARTLCR_H_FEN_BITS;
	hw->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
	hw->ifls = (0u << UART_UARTIFLS_RXIFLSEL_LSB) | (2u << UART_UARTIFLS_TXIFLSEL_LSB);
	hw->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;

	gpio_set_function(PT_HW_PIN_MOSI, GPIO_FUNC_NULL);
	gpio_pull_down(PT_HW_PIN_MOSI);
	gpio_set_function(PT_HW_PIN_SCK, GPIO_FUNC_NULL);
	gpio_pull_down(PT_HW_PIN_SCK);
	gpio_init(PT_HW_PIN_TX);
	gpio_disable_pulls(PT_HW_PIN_TX);
	gpio_set_function(PT_HW_PIN_RX, GPIO_FUNC_UART);
	gpio_pull_up(PT_HW_PIN_RX);
}

/* No interrupt of UART1 any more. */
static inline void pt_hw_uart_irqs_off(void)
{
	uart1_hw->imsc = 0;
}

/* The transmit interrupt (the FIFO at 16 bytes or fewer) on or off. */
static inline void pt_hw_uart_tx_irq(bool on)
{
	if (on) {
		hw_set_bits(&uart1_hw->imsc, UART_UARTIMSC_TXIM_BITS);
	} else {
		hw_clear_bits(&uart1_hw->imsc, UART_UARTIMSC_TXIM_BITS);
	}
}

static inline bool pt_hw_uart_tx_irq_on(void)
{
	return (uart1_hw->imsc & UART_UARTIMSC_TXIM_BITS) != 0;
}

/* The next received word (the byte and its flags) into *DR; false when
 * the FIFO is empty.
 */
static inline bool pt_hw_uart_rx(uint32_t *dr)
{
	if (uart1_hw->fr & UART_UARTFR_RXFE_BITS) {
		return false;
	}
	*dr = uart1_hw->dr;
	return true;
}

/* C into the transmit FIFO; false when it is full. */
static inline bool pt_hw_uart_tx(uint8_t c)
{
	if (uart1_hw->fr & UART_UARTFR_TXFF_BITS) {
		return false;
	}
	uart1_hw->dr = c;
	return true;
}

static inline void pt_hw_led_init(void)
{
	gpio_init(PT_HW_PIN_LED);
	gpio_set_dir(PT_HW_PIN_LED, GPIO_OUT);
}

static inline void pt_hw_led_toggle(void)
{
	gpio_xor_mask(1u << PT_HW_PIN_LED);
}

static inline void pt_hw_led_off(void)
{
	gpio_put(PT_HW_PIN_LED, 0);
}

#endif /* ZEPHYR_APP_DRIVERS_NINA_NINA_PASSTHROUGH_HW_H */
