/* SPDX-License-Identifier: MIT */
/* The sensor clock of <zephyr/drivers/sensor_clock.h> on the layer's cycle
 * counter, k_cycle_get_64(): the board's TIMER, rp2040_timer_us() (1 MHz; the
 * Zephyr build uses drivers/sensor/sensor_clock_sys.c on its system clock).
 * The LSM6DSO's FIFO stream stamps its batches with it.  Part of the small
 * Zephyr compatibility layer of the port, its OS-independent part
 * (common/compat).
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/kernel.h>

int sensor_clock_get_cycles(uint64_t *cycles)
{
	if (cycles == NULL) {
		return -EINVAL;
	}

	*cycles = k_cycle_get_64();
	return 0;
}

uint64_t sensor_clock_cycles_to_ns(uint64_t cycles)
{
	return cycles * 1000U;
}
