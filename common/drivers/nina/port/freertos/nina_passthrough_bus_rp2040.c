/* SPDX-License-Identifier: MIT */
/* The module's side of the NINA passthrough mode on the FreeRTOS build
 * (common/drivers/nina/port/nina_passthrough_bus.h), on the Pico SDK's
 * hardware_uart, hardware_gpio, hardware_irq and hardware_watchdog: the
 * watchdog's scratch registers and its restart, the module's lines, UART1
 * and the LED.
 *
 * The board (the mbed core's variants/NANO_RP2040_CONNECT: pins_arduino.h
 * SerialNina = Serial3 on D25/D26, NINA_GPIO0 D20, NINA_RESETN D24, and
 * variant.cpp's pin table; common/drivers/nina/port/nina_bus.h):
 *
 *   RP2040   UART1 in this mode                SPI link of the normal firmware
 *   GPIO8    TX  -> the module's UART0 RX      SPI1 RX (MISO)
 *   GPIO9    RX  <- the module's UART0 TX      CS, active low
 *   GPIO2    the module's GPIO0 (boot strap)   IRQ (driven only at the reset)
 *   GPIO3    the module's RESETN, active low   RESET
 *
 * The mode runs before the scheduler and before SPI1 is set up, so GPIO8
 * and GPIO9 are in their reset state when UART1 takes them.  UART1's
 * receive interrupt has the highest priority (no other interrupt delays
 * it, and nothing in this mode masks interrupts for long); its handler and
 * the receive path run from RAM (__not_in_flash_func(), and
 * NINA_PASSTHROUGH_ISR_SECTION for the core, nina.cmake).  The binding
 * (common/drivers/nina/port/freertos/nina_passthrough_os.c) polls the transmit
 * FIFO, so this build has no transmit interrupt and no
 * nina_passthrough_bus_uart_tx_wait().
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

#include "nina_passthrough.h"
#include "nina_passthrough_bus.h"

#define PT_UART       uart1
#define PT_UART_IRQ   UART1_IRQ
#define PT_PIN_TX     8 /* UART1 TX */
#define PT_PIN_RX     9 /* UART1 RX */
#define PT_PIN_GPIO0  2
#define PT_PIN_RESETN 3

_Static_assert(UART_UARTDR_FE_BITS == NINA_PASSTHROUGH_UART_FRAMING &&
		       UART_UARTDR_BE_BITS == NINA_PASSTHROUGH_UART_BREAK &&
		       UART_UARTDR_OE_BITS == NINA_PASSTHROUGH_UART_OVERRUN,
	       "a received word is UARTDR as read");

/* The request: the watchdog's scratch registers 0..3, which survive its
 * restart; fault_boot() clears them later at a normal start.
 */
void nina_passthrough_bus_mark_get(uint32_t s[NINA_PASSTHROUGH_MARK_WORDS])
{
	for (int i = 0; i < NINA_PASSTHROUGH_MARK_WORDS; i++) {
		s[i] = watchdog_hw->scratch[i];
	}
}

void nina_passthrough_bus_mark_set(const uint32_t s[NINA_PASSTHROUGH_MARK_WORDS])
{
	for (int i = 0; i < NINA_PASSTHROUGH_MARK_WORDS; i++) {
		watchdog_hw->scratch[i] = s[i];
	}
}

void nina_passthrough_bus_restart(void)
{
	watchdog_reboot(0, 0, 0);
	for (;;) {
	}
}

/* RESETN without pulls (Zephyr's reset-gpios), driven low; GPIO0 with the
 * pull-down (irq-gpios), as nina_bus_rp2040.c sets them.
 */
void nina_passthrough_bus_lines_init(void)
{
	gpio_init(PT_PIN_RESETN);
	gpio_disable_pulls(PT_PIN_RESETN);
	gpio_put(PT_PIN_RESETN, 0);
	gpio_set_dir(PT_PIN_RESETN, GPIO_OUT);
	gpio_init(PT_PIN_GPIO0);
	gpio_pull_down(PT_PIN_GPIO0);
}

void nina_passthrough_bus_resetn(bool released)
{
	gpio_put(PT_PIN_RESETN, released);
}

void nina_passthrough_bus_gpio0(enum nina_passthrough_gpio0 state)
{
	switch (state) {
	case NINA_PASSTHROUGH_GPIO0_LOW:
		gpio_put(PT_PIN_GPIO0, 0);
		gpio_set_dir(PT_PIN_GPIO0, GPIO_OUT);
		break;
	case NINA_PASSTHROUGH_GPIO0_HIGH:
		gpio_put(PT_PIN_GPIO0, 1);
		gpio_set_dir(PT_PIN_GPIO0, GPIO_OUT);
		break;
	case NINA_PASSTHROUGH_GPIO0_RELEASED:
		gpio_put(PT_PIN_GPIO0, 0);
		gpio_set_dir(PT_PIN_GPIO0, GPIO_IN);
		break;
	}
}

/* GPIO8 to the UART, or a plain input (SIO with the direction in). */
void nina_passthrough_bus_uart_tx_connect(bool connected)
{
	gpio_set_function(PT_PIN_TX, connected ? GPIO_FUNC_UART : GPIO_FUNC_SIO);
}

/* UART1, 8N1, FIFOs on, no flow control (uart_init()); GPIO8 as SIO input
 * until nina_passthrough_bus_uart_tx_connect(); GPIO9 with the pull-up.
 */
void nina_passthrough_bus_uart_init(uint32_t baud)
{
	uart_init(PT_UART, baud);
	gpio_init(PT_PIN_TX);
	gpio_disable_pulls(PT_PIN_TX);
	gpio_set_function(PT_PIN_RX, GPIO_FUNC_UART);
	gpio_pull_up(PT_PIN_RX);
}

void nina_passthrough_bus_uart_set_baud(uint32_t baud)
{
	uart_set_baudrate(PT_UART, baud);
}

static void __not_in_flash_func(uart_irq)(void)
{
	nina_passthrough_uart_isr();
}

/* The receive interrupt (uart_set_irqs_enabled(): at 4 bytes in the FIFO,
 * or after 32 bit times without more).
 */
void nina_passthrough_bus_uart_irq_on(void)
{
	irq_set_exclusive_handler(PT_UART_IRQ, uart_irq);
	irq_set_priority(PT_UART_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
	uart_set_irqs_enabled(PT_UART, true, false);
	irq_set_enabled(PT_UART_IRQ, true);
}

void nina_passthrough_bus_uart_irq_off(void)
{
	irq_set_enabled(PT_UART_IRQ, false);
}

bool __not_in_flash_func(nina_passthrough_bus_uart_rx)(uint32_t *word)
{
	uart_hw_t *hw = uart_get_hw(PT_UART);

	if (hw->fr & UART_UARTFR_RXFE_BITS) {
		return false;
	}
	*word = hw->dr;
	return true;
}

bool nina_passthrough_bus_uart_tx(uint8_t c)
{
	uart_hw_t *hw = uart_get_hw(PT_UART);

	if (hw->fr & UART_UARTFR_TXFF_BITS) {
		return false;
	}
	hw->dr = c;
	return true;
}

/* The LED (GPIO6); the normal firmware blinks it at 1 Hz. */
void nina_passthrough_bus_led_init(void)
{
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

void nina_passthrough_bus_led(bool on)
{
	gpio_put(PICO_DEFAULT_LED_PIN, on);
}
