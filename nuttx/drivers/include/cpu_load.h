/* SPDX-License-Identifier: Apache-2.0 */
/* CPU load from idle windows: the board's up_idle()
 * (boards/arduino-nano-rp2040-connect/src/nano_idle.c), the equivalent of
 * Zephyr's CONFIG_CPU_LOAD with CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK
 * (<zephyr/sys/cpu_load.h>, lib/os/cpu_load/cpu_load_backend_idle.c).
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_CPU_LOAD_H
#define __NUTTX_DRIVERS_INCLUDE_CPU_LOAD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* The load since the last call with reset=true (since boot before the
 * first one), in tenths of a percent: 1000 * (total - idle) / total,
 * rounded down, both in microseconds of the TIMER.  0 when no time has
 * passed.  Idle time is the time the CPU spends waiting for an interrupt
 * in the idle loop; the window closes when the interrupt arrives, before
 * its handler runs, so interrupt handlers count as load even when they
 * interrupt idle.  reset=true starts a new measurement window at this
 * call.
 */

int cpu_load_get(bool reset);

#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_DRIVERS_INCLUDE_CPU_LOAD_H */
