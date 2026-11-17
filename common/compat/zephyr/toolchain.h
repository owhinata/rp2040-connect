/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/toolchain.h> (toolchain/gcc.h, Zephyr
 * v4.5.0) that the copies of Zephyr's NVS and settings
 * (common/compat/subsys, <zephyr/kvss/nvs.h>) use, with Zephyr's GCC
 * definitions: __packed and __weak; BUILD_ASSERT() and the other macros of
 * toolchain/common.h that the port uses are in <zephyr/sys/util.h>.  Part
 * of the small Zephyr compatibility layer of the port (common/compat).
 */

#ifndef __COMMON_COMPAT_ZEPHYR_TOOLCHAIN_H
#define __COMMON_COMPAT_ZEPHYR_TOOLCHAIN_H

#include <zephyr/sys/util.h>

#ifndef __packed
#  define __packed            __attribute__((__packed__))
#endif

#ifndef __weak
#  define __weak              __attribute__((__weak__))
#endif

#endif /* __COMMON_COMPAT_ZEPHYR_TOOLCHAIN_H */
