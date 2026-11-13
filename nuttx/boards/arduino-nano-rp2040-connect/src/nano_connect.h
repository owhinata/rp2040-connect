/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_connect.h
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

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_CONNECT_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_CONNECT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Derived from boards/arm/rp2040/raspberrypi-pico/src/rp2040_pico.h
 * (NuttX 13.1.0).  The pin numbers are in include/board.h.
 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int nano_bringup(void);

/* nano_chipid.c: read the flash unique ID; from rp2040_boardinitialize(). */

void nano_chip_id_initialize(void);

/* nano_usbserial.c: build the USB serial number string from the chip ID;
 * from nano_bringup() before the USB device is registered.
 */

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
void nano_usbserial_initialize(void);
#endif

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_CONNECT_H */
