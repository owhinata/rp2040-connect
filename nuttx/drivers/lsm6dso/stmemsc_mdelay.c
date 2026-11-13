/* ST Microelectronics STMEMS hal i/f
 *
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * zephyrproject-rtos/modules/hal/st/sensor/stmemsc/
 */

/* Zephyr's drivers/sensor/st/stmemsc/stmemsc_mdelay.c: k_msleep() becomes
 * lsm6dso_os_sleep_ms() (the same rounding, lsm6dso_os.h).  ST's
 * lsm6dso_reg.c does not call it.
 */

#include "lsm6dso_os.h"
#include "stmemsc.h"

void stmemsc_mdelay(uint32_t millisec)
{
	lsm6dso_os_sleep_ms((int32_t)millisec);
}
