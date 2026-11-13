/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_usbserial.c
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

/* The USB serial number string: the chip ID (nano_chipid.c) as 16
 * upper-case hexadecimal digits, one byte after another in the order of
 * the ID.  This is how the Zephyr USB stack builds it from hwinfo
 * (subsys/usb/device_next/usbd_ch9.c, get_sn_from_hwid(): "0123456789ABCDEF",
 * high nibble first), so both builds show the same serial number, e.g.
 * 503533748A8C9218, in /dev/serial/by-id/.
 *
 * board_usbdev_serialstr() is called by the CDC ACM driver while it answers
 * GET_DESCRIPTOR on EP0, in interrupt context, so the string is built once
 * by nano_usbserial_initialize() before the USB device is registered.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nano_connect.h"

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "0" (NuttX's fixed serial string) until the chip ID is known. */

static char g_serialstr[2 * NANO_CHIP_ID_SIZE + 1] = "0";

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_usbserial_initialize
 *
 * Description:
 *   Build the serial number string from the chip ID.  Called from
 *   nano_bringup() before cdcacm_initialize().
 *
 ****************************************************************************/

void nano_usbserial_initialize(void)
{
  static const char hex[] = "0123456789ABCDEF";
  uint8_t id[NANO_CHIP_ID_SIZE];
  int i;

  if (nano_chip_id(id) < 0)
    {
      return;
    }

  for (i = 0; i < NANO_CHIP_ID_SIZE; i++)
    {
      g_serialstr[i * 2]     = hex[id[i] >> 4];
      g_serialstr[i * 2 + 1] = hex[id[i] & 0xf];
    }

  g_serialstr[2 * NANO_CHIP_ID_SIZE] = '\0';
}

/****************************************************************************
 * Name: board_usbdev_serialstr
 *
 * Description:
 *   Return the USB serial number string (CONFIG_BOARD_USBDEV_SERIALSTR).
 *
 ****************************************************************************/

FAR const char *board_usbdev_serialstr(void)
{
  return g_serialstr;
}

#endif /* CONFIG_BOARD_USBDEV_SERIALSTR */
