/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's sensor clock (<zephyr/drivers/sensor_clock.h>): the time base of
 * the sensor API's timestamps.  The Zephyr build uses the system clock
 * (CONFIG_SENSOR_CLOCK_SYSTEM, drivers/sensor/sensor_clock_sys.c):
 * k_cycle_get_64(), the 125 MHz SysTick count, and k_cyc_to_ns_near64().
 * Here it is the board's monotonic clock, the RP2040 TIMER (nano_mono_us(),
 * 1 MHz from the crystal), so a cycle is a microsecond and a timestamp a
 * whole number of microseconds (zephyr_sensor_clock.c).  Part of the small
 * Zephyr compatibility layer of the port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_SENSOR_CLOCK_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_SENSOR_CLOCK_H

#include <stdint.h>

/* 0, or -EINVAL for a NULL pointer. */

int sensor_clock_get_cycles(uint64_t *cycles);
uint64_t sensor_clock_cycles_to_ns(uint64_t cycles);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_SENSOR_CLOCK_H */
