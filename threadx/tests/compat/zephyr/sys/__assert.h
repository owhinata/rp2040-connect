/* SPDX-License-Identifier: MIT */
/* The per-OS <zephyr/sys/__assert.h> of the layer for the ThreadX host
 * test (the FreeRTOS build's is freertos/drivers/compat/zephyr/sys/__assert.h):
 * Zephyr's __ASSERT() and __ASSERT_NO_MSG() on the host's assert().
 */

#ifndef THREADX_TESTS_COMPAT_ZEPHYR_SYS___ASSERT_H
#define THREADX_TESTS_COMPAT_ZEPHYR_SYS___ASSERT_H

#include <assert.h>

#define __ASSERT_NO_MSG(test) assert(test)
#define __ASSERT(test, ...)   assert(test)

#endif /* THREADX_TESTS_COMPAT_ZEPHYR_SYS___ASSERT_H */
