/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's __ASSERT() and __ASSERT_NO_MSG() (<zephyr/sys/__assert.h>).
 * The Zephyr build has CONFIG_ASSERT off, so they compile to nothing
 * there.  Here they are NuttX's DEBUGASSERT() (also nothing without
 * CONFIG_DEBUG_ASSERTIONS, as in the imu configuration); host tests have
 * assert() (common/tests/zephyr_fakes/zephyr/sys/__assert.h).  Part of the
 * NuttX implementation of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H

#include <assert.h>
#define __ASSERT_NO_MSG(test)  DEBUGASSERT(test)
#define __ASSERT(test, ...)    DEBUGASSERT(test)

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H */
