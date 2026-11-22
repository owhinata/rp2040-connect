/* SPDX-License-Identifier: MIT */
/* The per-OS <zephyr/sys/__assert.h> for the host check of the generic
 * kernel part: Zephyr's __ASSERT() and __ASSERT_NO_MSG() on the host's
 * assert().
 */

#ifndef COMMON_TESTS_KERNEL_GENERIC_HOST_ZEPHYR_SYS___ASSERT_H
#define COMMON_TESTS_KERNEL_GENERIC_HOST_ZEPHYR_SYS___ASSERT_H

#include <assert.h>

#define __ASSERT_NO_MSG(test) assert(test)
#define __ASSERT(test, ...)   assert(test)

#endif /* COMMON_TESTS_KERNEL_GENERIC_HOST_ZEPHYR_SYS___ASSERT_H */
