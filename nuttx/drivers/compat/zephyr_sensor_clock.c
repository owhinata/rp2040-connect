/* SPDX-License-Identifier: Apache-2.0 */
/* The sensor clock of <zephyr/drivers/sensor_clock.h> on the board's
 * TIMER (Zephyr: drivers/sensor/sensor_clock_sys.c on SysTick).  Part of
 * the small Zephyr compatibility layer of the port (nuttx/drivers/compat).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <arch/board/nano_timer.h>
#include <zephyr/drivers/sensor_clock.h>

int sensor_clock_get_cycles(uint64_t *cycles)
{
  if (cycles == NULL)
    {
      return -EINVAL;
    }

  *cycles = nano_mono_us();
  return 0;
}

uint64_t sensor_clock_cycles_to_ns(uint64_t cycles)
{
  return cycles * 1000U;
}
