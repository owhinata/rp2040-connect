/****************************************************************************
 * boards/arduino-nano-rp2040-connect/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Derived from boards/arm/rp2040/raspberrypi-pico/include/board.h
 * (NuttX 13.1.0).  The clock tree is the same as the Zephyr board
 * (zephyr/dts/arm/raspberrypi/rpi_pico/rp2040.dtsi); the pin numbers come
 * from zephyr/boards/arduino/nano_connect/arduino_nano_connect.dts.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_BOARD_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rp2040_i2cdev.h"
#include "rp2040_spidev.h"
#include "rp2040_i2sdev.h"

#include "rp2040_spisd.h"

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/compiler.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define MHZ                     1000000

#define BOARD_XOSC_FREQ         (12 * MHZ)

/* XOSC STARTUP.DELAY multiplier (in units of about 1 ms).  6 is the value
 * the Zephyr build uses (the startup-delay-multiplier default of the
 * raspberrypi,pico-xosc binding, the same as the pico-sdk default), giving
 * STARTUP.DELAY = 282 in both.
 */

#define BOARD_XOSC_STARTUPDELAY 6
#define BOARD_PLL_SYS_FREQ      (125 * MHZ)
#define BOARD_PLL_USB_FREQ      (48 * MHZ)

#define BOARD_REF_FREQ          (12 * MHZ)
#define BOARD_SYS_FREQ          (125 * MHZ)
#define BOARD_PERI_FREQ         (125 * MHZ)
#define BOARD_USB_FREQ          (48 * MHZ)
#define BOARD_ADC_FREQ          (48 * MHZ)
#define BOARD_RTC_FREQ          46875

#define BOARD_UART_BASEFREQ     BOARD_PERI_FREQ

/* The watchdog tick (which also clocks the TIMER) runs at 1 MHz. */

#define BOARD_TICK_CLOCK        (1 * MHZ)

/* Flash ********************************************************************/

/* 16 MiB AT25SF128A.  The last two 4 KiB erase sectors are kept out of the
 * image for the key-value store, at the same place as the Zephyr build's
 * storage_partition (zephyr/app/usb-console/app.overlay).  The linker
 * script uses the same numbers.
 */

#define BOARD_FLASH_BASE        0x10000000
#define BOARD_FLASH_SIZE        (16 * 1024 * 1024)
#define BOARD_KV_OFFSET         0x00ffe000  /* Offset from BOARD_FLASH_BASE */
#define BOARD_KV_SIZE           0x00002000

/* GPIO definitions *********************************************************/

/* All numbers are RP2040 GPIO numbers.  Except for UART0, nothing here is
 * configured by the board code yet: the pins are reset defaults until the
 * driver that owns them sets them up.
 */

/* UART0 (Arduino D1/D0), 115200 8N1 in the Zephyr board too.  Not the
 * console (the console is USB CDC ACM, as in the Zephyr build).  The
 * in-tree rp2040_common_earlyinitialize() gives GPIO0/1 to UART0 (Zephyr's
 * pinctrl does the same); NuttX registers /dev/ttyS0 and programs the UART
 * itself only when it is opened.  These defines must match
 * CONFIG_RP2040_UART0_TX_GPIO/RX_GPIO (the defaults, 0 and 1).
 */

#define BOARD_UART0_TX_GPIO          0
#define BOARD_UART0_RX_GPIO          1

/* LED on GPIO6 (Zephyr led0).  The pin is also D13/SCK of the Arduino
 * header, so the board code does not drive it: no ARCH_LEDS, no USERLED.
 */

#define BOARD_GPIO_LED_PIN           6

/* I2C0 (Arduino A4/A5): LSM6DSOX at 0x6a and ATECC608A at 0x60. */

#define BOARD_I2C0_SDA_GPIO          12
#define BOARD_I2C0_SCL_GPIO          13
#define BOARD_I2C0_FREQUENCY         100000 /* clock-frequency at boot */
#define BOARD_LSM6DSOX_I2C_ADDR      0x6a
#define BOARD_LSM6DSOX_INT1_GPIO     24  /* irq-gpios, active high */
#define BOARD_LSM6DSOX_INT1_FLAGS    0   /* <&gpio0 24 0>: active high, no pull */
#define BOARD_ATECC608A_I2C_ADDR     0x60

/* The properties of the LSM6DSOX node of the Zephyr build's devicetree
 * (zephyr/build/usb-console/zephyr/zephyr.dts, lsm6dsox@6a: the board's
 * arduino_nano_connect.dts and app/usb-console/app.overlay; the others are
 * the defaults of st,lsm6dso-common.yaml), for the device that
 * nuttx/drivers/lsm6dso/lsm6dso.c defines.  The values are the binding's
 * (zephyr/dt-bindings/sensor/lsm6dso.h).
 */

#define BOARD_LSM6DSOX_NODE_NAME     "lsm6dsox@6a"
#define BOARD_LSM6DSOX_ACCEL_PM      0   /* default: high performance */
#define BOARD_LSM6DSOX_ACCEL_ODR     0   /* default: off */
#define BOARD_LSM6DSOX_ACCEL_LP_FILTER 0 /* default: LPF2 off */
#define BOARD_LSM6DSOX_ACCEL_RANGE   3   /* app.overlay: LSM6DSO_DT_FS_8G */
#define BOARD_LSM6DSOX_GYRO_PM       0   /* default: high performance */
#define BOARD_LSM6DSOX_GYRO_ODR      0   /* default: off */
#define BOARD_LSM6DSOX_GYRO_RANGE    6   /* app.overlay: LSM6DSO_DT_FS_2000DPS */
#define BOARD_LSM6DSOX_DRDY_PULSED   0   /* default: latched */

/* SPI1 to the NINA-W102 Wi-Fi module. */

#define BOARD_NINA_SPI1_SCK_GPIO     14
#define BOARD_NINA_SPI1_TX_GPIO      11  /* MOSI */
#define BOARD_NINA_SPI1_RX_GPIO      8   /* MISO */
#define BOARD_NINA_CS_GPIO           9   /* cs-gpios, active low */
#define BOARD_NINA_READY_GPIO        10  /* ready-gpios, active low, pull-up */
#define BOARD_NINA_RESET_GPIO        3   /* reset-gpios, active low */
#define BOARD_NINA_IRQ_GPIO          2   /* irq-gpios, pull-down */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardearlyinitialize(void);

/****************************************************************************
 * Name: rp2040_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardinitialize(void);

/****************************************************************************
 * Name: nano_chip_id
 *
 * Description:
 *   Copy the chip ID to id: the 8-byte unique ID of the external flash
 *   (command 0x4B), raw, in the order the flash sends it.  The same bytes
 *   as hwinfo_get_device_id() in the Zephyr build; the USB serial number is
 *   these bytes as 16 upper-case hexadecimal digits.  Read once at boot
 *   (src/nano_chipid.c).
 *
 * Returned Value:
 *   OK, or -EAGAIN if the ID has not been read yet.
 *
 ****************************************************************************/

#define NANO_CHIP_ID_SIZE 8

int nano_chip_id(uint8_t id[NANO_CHIP_ID_SIZE]);

/****************************************************************************
 * Name: nano_reset_usb_boot
 *
 * Description:
 *   Reboot into the RP2040 USB bootloader (BOOTSEL, drive RPI-RP2) through
 *   the boot ROM's reset_usb_boot(0, 0) (src/nano_reset.c).  The caller
 *   flushes its output first.  Does not return.
 *
 ****************************************************************************/

void nano_reset_usb_boot(void) noreturn_function;

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_BOARD_H */
