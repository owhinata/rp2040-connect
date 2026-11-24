/* SPDX-License-Identifier: MIT */
/* Zephyr's __ASSERT() and __ASSERT_NO_MSG() (<zephyr/sys/__assert.h>).  The
 * Zephyr build has CONFIG_ASSERT off, so they compile to nothing there; so
 * they do here (the expression is not evaluated; sizeof keeps the variables
 * it names used).  The NuttX build has DEBUGASSERT() (also nothing in its
 * configuration).  Part of the Zephyr compatibility layer for an OS without
 * an assertion of its own in the layer (common/compat/port/generic).
 */

#ifndef COMMON_COMPAT_PORT_GENERIC_ZEPHYR_SYS___ASSERT_H
#define COMMON_COMPAT_PORT_GENERIC_ZEPHYR_SYS___ASSERT_H

#define __ASSERT_NO_MSG(test) ((void)sizeof((test) ? 1 : 0))
#define __ASSERT(test, ...)   ((void)sizeof((test) ? 1 : 0))

#endif /* COMMON_COMPAT_PORT_GENERIC_ZEPHYR_SYS___ASSERT_H */
