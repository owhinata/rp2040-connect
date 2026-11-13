/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_reset.c
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

/* Reboot into the RP2040 boot ROM's USB bootloader (BOOTSEL: the RPI-RP2
 * mass storage drive and the PICOBOOT interface), as pressing BOOTSEL at
 * reset would.  The ROM function reset_usb_boot(gpio_activity_pin_mask,
 * disable_interface_mask) is looked up by its code 'U','B' in the ROM
 * function table (RP2040 datasheet 2.8.3.1.5, rp2040_rom.h).  Arguments
 * (0, 0): no activity LED, both interfaces enabled.  It reboots the chip
 * through the watchdog and does not return.
 *
 * The normal reboot (nsh "reboot", board_reset() in the in-tree
 * boards/arm/rp2040/common/src/rp2040_reset.c) is not changed.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "rp2040_rom.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*rom_reset_usb_boot_fn)(uint32_t gpio_activity_pin_mask,
                                      uint32_t disable_interface_mask);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_reset_usb_boot
 *
 * Description:
 *   Reboot into BOOTSEL.  The caller flushes its output first.  Does not
 *   return.
 *
 ****************************************************************************/

void nano_reset_usb_boot(void)
{
  rom_reset_usb_boot_fn reset_usb_boot;

  /* The ROM table pointers live at the fixed ROM addresses 0x14 and 0x18;
   * GCC warns about reading such small constant addresses.
   */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
  reset_usb_boot =
    (rom_reset_usb_boot_fn)ROM_LOOKUP(ROM_RESET_USB_BOOT);
#pragma GCC diagnostic pop

  /* Nothing else may run once the ROM starts tearing the chip down. */

  up_irq_save();

  if (reset_usb_boot != NULL)
    {
      reset_usb_boot(0, 0);
    }

  /* Not reached.  If the lookup failed, fall back to a plain reset rather
   * than return to a caller that expects the board to be gone.
   */

  up_systemreset();
}
