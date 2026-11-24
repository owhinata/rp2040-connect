/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/toolchain.h> (toolchain/gcc.h, Zephyr
 * v4.5.0) that the copies of Zephyr's NVS and settings
 * (common/compat/subsys, <zephyr/kvss/nvs.h>) and of its fnmatch()
 * (common/compat/subsys/portability) use, with Zephyr's GCC definitions:
 * __packed, __weak, __fallthrough and CODE_UNREACHABLE; BUILD_ASSERT() and
 * the other macros of
 * toolchain/common.h that the port uses are in <zephyr/sys/util.h>.  Part
 * of the small Zephyr compatibility layer of the port (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_TOOLCHAIN_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_TOOLCHAIN_H

#include <zephyr/sys/util.h>

#ifndef __packed
#  define __packed            __attribute__((__packed__))
#endif

#ifndef __weak
#  define __weak              __attribute__((__weak__))
#endif

#ifndef __fallthrough
#  define __fallthrough       __attribute__((fallthrough))
#endif

#ifndef CODE_UNREACHABLE
#  define CODE_UNREACHABLE    __builtin_unreachable()
#endif

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_TOOLCHAIN_H */
