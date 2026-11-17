/* ST Microelectronics STMEMS hal i/f
 *
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * zephyrproject-rtos/modules/hal/st/sensor/stmemsc/
 */

/* Zephyr's drivers/sensor/st/stmemsc/stmemsc_mdelay.c, with k_msleep() of
 * the compatibility layer (<zephyr/kernel.h>, which Zephyr's stmemsc.h
 * reads; this directory's stmemsc.h does not).  ST's lsm6dso_reg.c does
 * not call it.  Shared by the NuttX and FreeRTOS builds.
 */

#include <zephyr/kernel.h>

#include "stmemsc.h"

void stmemsc_mdelay(uint32_t millisec)
{
	k_msleep(millisec);
}
