/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_boardinitialize.c
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

/* Derived from boards/arm/rp2040/raspberrypi-pico/src/
 * rp2040_boardinitialize.c (NuttX 13.1.0).  The Pico code drives the LED
 * pin high in rp2040_boardearlyinitialize(); this board does not, because
 * its LED (GPIO6) is also D13/SCK and the Zephyr build does not drive it.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp2040_gpio.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_initialize.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#include "nano_connect.h"

#ifdef CONFIG_NANO_LOG_RAM
#  include "log_ram.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardearlyinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp2040_common_earlyinitialize();
  #endif

  /* --- Place any board specific early initialization here --- */
}

/****************************************************************************
 * Name: rp2040_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp2040_common_initialize();
  #endif

  /* --- Place any board specific initialization here --- */

#ifdef CONFIG_NANO_LOG_RAM
  /* The RAM log (and syslog channel) first, so the rest of the boot can
   * log.  The TIMER (its time stamps) runs since rp2040_clockconfig().
   */

  log_ram_initialize();
#endif

  /* Read the chip ID here, like rp2040_common_initialize() reads the flash
   * ID for board_uniqueid(): the flash command stops XIP, and nothing else
   * runs yet.
   */

  nano_chip_id_initialize();
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called after up_initialize() and board_early_initialize() and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers for which board_early_initialize() is not
 *   suitable.
 *
 *   Waiting for events, use of I2C, SPI, etc are permissible in the context
 *   of board_late_initialize().  That is because board_late_initialize()
 *   will run on a temporary, internal kernel thread.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  nano_bringup();
}
#endif
