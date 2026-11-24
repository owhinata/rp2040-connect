/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's <zephyr/sys/cpu_load.h>: cpu_load_get(bool reset), which the
 * port's board provides with the same meaning (the load since the last
 * reset, idle windows closed at interrupt entry; common/compat/port/cpu_load.h,
 * on NuttX boards/.../src/rp2040_idle_cpuload.c).  Part of the small Zephyr
 * compatibility layer of the port (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_CPU_LOAD_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_CPU_LOAD_H

/* common/compat/port/cpu_load.h (angle brackets: not this directory). */

#include <cpu_load.h>

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_CPU_LOAD_H */
