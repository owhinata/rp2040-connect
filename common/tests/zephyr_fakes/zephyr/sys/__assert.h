/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's __ASSERT() and __ASSERT_NO_MSG() (<zephyr/sys/__assert.h>) for
 * host tests: assert().  The Zephyr build has CONFIG_ASSERT off, so they
 * compile to nothing there; on NuttX they are DEBUGASSERT()
 * (nuttx/drivers/compat/zephyr/sys/__assert.h).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H

#include <assert.h>
#define __ASSERT_NO_MSG(test)  assert(test)
#define __ASSERT(test, ...)    assert(test)

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS___ASSERT_H */
