/* SPDX-License-Identifier: MIT */
/* Read before common/imu/time_sync.c (-include, after the prelude of the
 * shared sources; freertos/drivers/imu/imu.cmake), for that file only.
 *
 * time_sync.c reads the RP2040 TIMER's raw registers itself, at
 *
 *   #define TIMER_BASE   DT_REG_ADDR(DT_NODELABEL(timer))
 *
 * (the layer's devicetree.h: 0x40054000), as in the Zephyr and NuttX builds.
 * The Pico SDK's hardware/regs/addressmap.h defines TIMER_BASE as well (the
 * same address, as _u(0x40054000)), and the file reads it through the
 * layer's headers (<zephyr/kernel.h>: the board's port header and
 * os_clock.h read hardware/timer.h), so the file's definition would be a
 * redefinition (a warning that no option turns off).  This header reads
 * those SDK headers first, so that every SDK inline function that uses
 * TIMER_BASE (timer_hw) is compiled with the SDK's definition, and then
 * takes the SDK's name away; the include guards keep the SDK from defining
 * it again, and the file's definition is the only one from there on.  Both
 * are the same address, so a later use of the SDK's timer_hw would read the
 * same registers.  The shared source is not changed.
 */

#ifndef FREERTOS_DRIVERS_IMU_TIME_SYNC_PRELUDE_H
#define FREERTOS_DRIVERS_IMU_TIME_SYNC_PRELUDE_H

#include "hardware/regs/addressmap.h"
#include "hardware/timer.h"

_Static_assert(TIMER_BASE == 0x40054000, "the TIMER of common/compat/zephyr/devicetree.h");

#undef TIMER_BASE

#endif /* FREERTOS_DRIVERS_IMU_TIME_SYNC_PRELUDE_H */
