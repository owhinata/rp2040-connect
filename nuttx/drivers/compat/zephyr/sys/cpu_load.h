/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's <zephyr/sys/cpu_load.h>: cpu_load_get(bool reset), which the
 * port's board provides with the same meaning (the load since the last
 * reset, idle windows closed at interrupt entry; nuttx/drivers/include/
 * cpu_load.h, boards/.../src/nano_idle.c).  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_CPU_LOAD_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_CPU_LOAD_H

/* $(TOPDIR)/external/include/cpu_load.h (angle brackets: not this
 * directory).
 */

#include <cpu_load.h>

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_CPU_LOAD_H */
