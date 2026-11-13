/* SPDX-License-Identifier: Apache-2.0 */
/* snprintk() of Zephyr's <zephyr/sys/printk.h>.  The Zephyr build has
 * CONFIG_PICOLIBC=y, where that header defines snprintk() and vsnprintk()
 * as snprintf() and vsnprintf() of the C library; so does this one.
 * printk() itself is not provided (the port logs with app_log.h).  Part of
 * the small Zephyr compatibility layer of the port (nuttx/drivers/compat,
 * see nuttx/drivers/README.md).  Included by <zephyr/kernel.h>, as in
 * Zephyr.
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_PRINTK_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_PRINTK_H

#include <stdio.h>

#define snprintk(...) snprintf(__VA_ARGS__)
#define vsnprintk(str, size, fmt, ap) vsnprintf(str, size, fmt, ap)

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_PRINTK_H */
