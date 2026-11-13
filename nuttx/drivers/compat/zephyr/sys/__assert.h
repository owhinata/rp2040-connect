/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's __ASSERT() and __ASSERT_NO_MSG() (<zephyr/sys/__assert.h>).
 * The Zephyr build has CONFIG_ASSERT off, so they compile to nothing
 * there.  Here they are NuttX's DEBUGASSERT() (also nothing without
 * CONFIG_DEBUG_ASSERTIONS, as in the imu configuration), and assert() in a
 * host test.  Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H

#ifdef __NuttX__
#  include <assert.h>
#  define __ASSERT_NO_MSG(test)  DEBUGASSERT(test)
#  define __ASSERT(test, ...)    DEBUGASSERT(test)
#else
#  include <assert.h>
#  define __ASSERT_NO_MSG(test)  assert(test)
#  define __ASSERT(test, ...)    assert(test)
#endif

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H */
