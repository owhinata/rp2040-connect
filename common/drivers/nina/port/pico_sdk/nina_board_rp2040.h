/* SPDX-License-Identifier: MIT */
/* The board's lines to the NINA-W102 on the RP2040, for the bindings on the
 * Pico SDK (nina_bus_rp2040.c: SPI1 and the four lines; the OS's nina_os.c,
 * port/<OS>: the interrupts of READY and IRQ): the Arduino Nano RP2040 Connect's, as in
 * Zephyr's arduino_nano_connect.dts and -pinctrl.dtsi and the NuttX
 * board's board.h (BOARD_NINA_*).
 */

#ifndef NINA_BOARD_RP2040_H_
#define NINA_BOARD_RP2040_H_

#define NINA_SPI          spi1
#define NINA_SPI_RESET    RESET_SPI1
#define NINA_SPI_SCK_GPIO 14
#define NINA_SPI_TX_GPIO  11
#define NINA_SPI_RX_GPIO  8
#define NINA_CS_GPIO      9  /* active low */
#define NINA_READY_GPIO   10 /* active low, pull-up */
#define NINA_RESET_GPIO   3  /* active low */
#define NINA_IRQ_GPIO     2  /* active high, pull-down */

#endif /* NINA_BOARD_RP2040_H_ */
