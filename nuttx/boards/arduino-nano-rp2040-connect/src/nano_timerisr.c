/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_timerisr.c
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

/* The tickless system timer (CONFIG_NANO_TICKLESS).  Takes the place of
 * arch/arm/src/rp2040/rp2040_timerisr.c (periodic SysTick), which
 * nuttx/patches/nuttx/0004-rp2040-make-defs-alarm-arch-idle-custom.patch
 * leaves out of the build when CONFIG_ALARM_ARCH is set.  Derived from the
 * CONFIG_RP23XX_SYSTIMER_TICKLESS branch of
 * arch/arm/src/rp23xx/rp23xx_timerisr.c (NuttX 13.1.0).  SysTick is not
 * used.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/timers/arch_alarm.h>

#include "nano_connect.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   Called from clock_initialize() during start-up.  Hands ALARM0 of the
 *   RP2040 TIMER to drivers/timers/arch_alarm.c, which then drives the
 *   tickless scheduler (nxsched_process_timer() on each alarm) and provides
 *   up_timer_gettick(), up_alarm_tick_start(), up_udelay() and up_perf_*().
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  up_alarm_set_lowerhalf(nano_oneshot_initialize());
}
