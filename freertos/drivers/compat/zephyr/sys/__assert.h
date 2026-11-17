/* SPDX-License-Identifier: MIT */
/* Zephyr's __ASSERT() and __ASSERT_NO_MSG() (<zephyr/sys/__assert.h>).  The
 * Zephyr build has CONFIG_ASSERT off, so they compile to nothing there; so
 * they do here (the expression is not evaluated; sizeof keeps the variables
 * it names used).  The NuttX build has DEBUGASSERT() (also nothing in its
 * configuration).  Part of the FreeRTOS implementation of the small Zephyr
 * compatibility layer of the port (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H

#define __ASSERT_NO_MSG(test) ((void)sizeof((test) ? 1 : 0))
#define __ASSERT(test, ...)   ((void)sizeof((test) ? 1 : 0))

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H */
