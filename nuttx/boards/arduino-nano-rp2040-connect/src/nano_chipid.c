/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_chipid.c
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

/* The chip ID: the 64-bit unique ID of the external flash (command 0x4B,
 * "Read Unique ID"), as raw bytes in the order the flash sends them.  This
 * is what the Zephyr build returns from hwinfo_get_device_id()
 * (drivers/hwinfo/hwinfo_rpi_pico.c -> pico-sdk flash_get_unique_id():
 * rxbuf[1 + 4 dummy bytes + i], i = 0..7) and what it prints as the USB
 * serial number.  NuttX's board_uniqueid() returns a PRNG output seeded from
 * the same ID instead, so it is not used here.
 *
 * The flash command stops XIP, so it runs once, from
 * rp2040_boardinitialize(), the same place where the in-tree
 * rp2040_uniqueid_initialize() runs its own 0x4B command, before the
 * scheduler and interrupts are started.  rp2040_flash_cmd() (in-tree,
 * boards/arm/rp2040/common/src/rp2040_uniqueid.c, built with
 * CONFIG_BOARDCTL_UNIQUEID) runs from RAM and restarts XIP through a copy
 * of boot2 on its stack.  Interrupts are disabled around it anyway, as the
 * Zephyr driver does (irq_lock), so that nothing can execute from flash
 * while XIP is off.  Later callers read the cached copy.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "rp2040_uniqueid.h"
#include "nano_connect.h"

#ifndef CONFIG_BOARDCTL_UNIQUEID
#  error "nano_chipid.c needs rp2040_flash_cmd(): set CONFIG_BOARDCTL_UNIQUEID"
#endif

#if RP2040_FLASH_ID_SIZE != NANO_CHIP_ID_SIZE
#  error "RP2040_FLASH_ID_SIZE and NANO_CHIP_ID_SIZE differ"
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined (not static) in boards/arm/rp2040/common/src/rp2040_uniqueid.c,
 * but not declared in any header.
 */

void rp2040_flash_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_chip_id[NANO_CHIP_ID_SIZE];
static bool g_chip_id_valid;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_chip_id_initialize
 *
 * Description:
 *   Read the flash unique ID once and keep it.  Called from
 *   rp2040_boardinitialize().
 *
 ****************************************************************************/

void nano_chip_id_initialize(void)
{
  uint8_t txbuf[RP2040_FLASH_ID_BUFFER_SIZE];
  uint8_t rxbuf[RP2040_FLASH_ID_BUFFER_SIZE];
  irqstate_t flags;

  memset(txbuf, 0, sizeof(txbuf));
  memset(rxbuf, 0, sizeof(rxbuf));
  txbuf[0] = RP2040_FLASH_RUID_CMD;

  flags = up_irq_save();
  rp2040_flash_cmd(txbuf, rxbuf, sizeof(txbuf));
  up_irq_restore(flags);

  memcpy(g_chip_id, &rxbuf[RP2040_FLASH_ID_BUFFER_OFFSET],
         NANO_CHIP_ID_SIZE);
  g_chip_id_valid = true;
}

/****************************************************************************
 * Name: nano_chip_id
 *
 * Description:
 *   Copy the 8-byte chip ID (see the top of this file) to id.
 *
 * Returned Value:
 *   OK, or -EAGAIN before nano_chip_id_initialize() has run.
 *
 ****************************************************************************/

int nano_chip_id(uint8_t id[NANO_CHIP_ID_SIZE])
{
  if (!g_chip_id_valid)
    {
      return -EAGAIN;
    }

  memcpy(id, g_chip_id, NANO_CHIP_ID_SIZE);
  return OK;
}
