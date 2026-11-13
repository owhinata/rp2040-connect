/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_bringup.c
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

/* Derived from boards/arm/rp2040/raspberrypi-pico/src/rp2040_bringup.c
 * (NuttX 13.1.0), without the LED and button drivers (see
 * nano_boardinitialize.c).  rp2040_common_bringup() mounts /proc when
 * CONFIG_FS_PROCFS is set.  Runs from board_late_initialize(), just before
 * the init task starts (13.1.0 has no NSH_ARCHINIT/board_app_initialize).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/debug.h>
#include <stddef.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#include <arch/board/board.h>

#include "nano_connect.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_bringup
 ****************************************************************************/

int nano_bringup(void)
{
  int result = OK;
  int ret;

#ifdef CONFIG_ARCH_BOARD_COMMON

  /* A failure here must not keep the console from being registered. */

  ret = rp2040_common_bringup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rp2040_common_bringup() failed: %d\n", ret);
      result = ret;
    }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#ifdef CONFIG_CDCACM
  /* The console: USB CDC ACM, /dev/ttyACM0 (minor 0).  It is not a NuttX
   * console device (CDCACM_CONSOLE is off); the init task (nuttx/apps/init)
   * opens it and runs nsh on it.  The serial number string must be ready
   * before the host asks for the descriptors.
   */

#  ifdef CONFIG_BOARD_USBDEV_SERIALSTR
  nano_usbserial_initialize();
#  endif

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize() failed: %d\n", ret);
      result = ret;
    }
#endif

  UNUSED(ret);
  return result;
}
