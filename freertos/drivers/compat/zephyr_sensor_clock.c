/* SPDX-License-Identifier: MIT */
/* The sensor clock of <zephyr/drivers/sensor_clock.h> (common/compat) on the
 * board's TIMER, nano_mono_us() (1 MHz, as the NuttX build's
 * nuttx/drivers/compat/zephyr_sensor_clock.c; the Zephyr build uses
 * drivers/sensor/sensor_clock_sys.c on its system clock).  The LSM6DSO's
 * FIFO stream stamps its batches with it.  Part of the FreeRTOS
 * implementation of the Zephyr compatibility layer (freertos/drivers/compat).
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/sensor_clock.h>

#include "os_clock.h"

int sensor_clock_get_cycles(uint64_t *cycles)
{
	if (cycles == NULL) {
		return -EINVAL;
	}

	*cycles = nano_mono_us();
	return 0;
}

uint64_t sensor_clock_cycles_to_ns(uint64_t cycles)
{
	return cycles * 1000U;
}
