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
#include <errno.h>
#include <stddef.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#include <arch/board/board.h>

#include "nano_connect.h"

#ifdef CONFIG_NANO_NINA
#  include "nano_nina.h"
#endif

#ifdef CONFIG_NANO_I2C0
#  include <nuttx/i2c/i2c_master.h>
#  include "nano_i2c.h"
#endif

#ifdef CONFIG_NANO_I2C0_DMA
#  include "nano_dma.h"
#endif

#ifdef CONFIG_NANO_LSM6DSO
#  include "nano_lsm6dso.h"
#endif

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

#ifdef CONFIG_NANO_NINA
  /* SPI1 to the NINA-W102 and its chip select.  The Zephyr build sets up
   * spi1 (and drives cs-gpios inactive) at boot; the module itself is
   * reset on the first request (nuttx/drivers/nina/nina_spi.c).
   */

  ret = nano_nina_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nano_nina_initialize() failed: %d\n", ret);
      result = ret;
    }
#endif

#ifdef CONFIG_NANO_I2C0_DMA
  /* The DMA for the I2C0 reads: reset, both interrupts.  The Zephyr build
   * brings up its DMA device before i2c0 (CONFIG_DMA_INIT_PRIORITY 40,
   * CONFIG_I2C_INIT_PRIORITY 50, both POST_KERNEL).
   */

  ret = nano_dma_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nano_dma_initialize() failed: %d\n", ret);
      result = ret;
    }
#endif

#ifdef CONFIG_NANO_I2C0
  /* I2C0: reset, pins, controller registers and interrupt, as the Zephyr
   * build's i2c0 is set up at boot (POST_KERNEL), then /dev/i2c0 for the
   * i2c tool.
   */

  {
    struct i2c_master_s *i2c = nano_i2c0_initialize();

    if (i2c == NULL)
      {
        syslog(LOG_ERR, "ERROR: nano_i2c0_initialize() failed\n");
        result = -ENODEV;
      }
#  ifdef CONFIG_I2C_DRIVER
    else
      {
        ret = i2c_register(i2c, 0);
        if (ret < 0)
          {
            syslog(LOG_ERR, "ERROR: i2c_register(0) failed: %d\n", ret);
            result = ret;
          }
      }
#  endif

#  ifdef CONFIG_NANO_LSM6DSO
    /* The LSM6DSOX (lsm6dsox@6a) on i2c0, which Zephyr initializes before
     * main() (POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY 90).  Its log lines
     * ("Initialize device lsm6dsox@6a", "chip id 0x6c") come first in
     * dmesg, as in the Zephyr build.  A failure leaves the device not
     * ready (the driver logs it) and does not stop the boot.
     */

    if (i2c != NULL)
      {
        (void)nano_lsm6dso_initialize(i2c);
      }
#  endif
  }
#endif

#ifdef CONFIG_NANO_FLASH_KV
  /* The key-value store: /dev/config on the last 8 KiB of the flash.  The
   * Zephyr build mounts its NVS in main(), after its devices (the
   * LSM6DSOX above).  Done before USB is registered: the first boot after
   * the Zephyr build may erase the area, with interrupts off for each
   * 4 KiB sector.
   */

  ret = nano_flash_kv_initialize();
  if (ret < 0)
    {
      result = ret;
    }
#endif

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
