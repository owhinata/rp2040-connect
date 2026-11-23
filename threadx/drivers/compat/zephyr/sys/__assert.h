/* SPDX-License-Identifier: MIT */
/* Zephyr's __ASSERT() and __ASSERT_NO_MSG() (<zephyr/sys/__assert.h>).  The
 * Zephyr build has CONFIG_ASSERT off, so they compile to nothing there; so
 * they do here, as in the FreeRTOS build (the expression is not evaluated;
 * sizeof keeps the variables it names used).  Part of the ThreadX
 * implementation of the Zephyr compatibility layer (threadx/drivers/compat).
 */

#ifndef THREADX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H
#define THREADX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H

#define __ASSERT_NO_MSG(test) ((void)sizeof((test) ? 1 : 0))
#define __ASSERT(test, ...)   ((void)sizeof((test) ? 1 : 0))

#endif /* THREADX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H */
