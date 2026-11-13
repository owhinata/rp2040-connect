/* SPDX-License-Identifier: Apache-2.0 */
/* The few macros of Zephyr's <zephyr/sys/util.h> (and util_macro.h) that the
 * ported sources use, with Zephyr's definitions, so that those sources keep
 * their text.  Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md); it is not Zephyr's
 * header and has nothing else.
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_UTIL_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_UTIL_H

#include <stddef.h>
#include <stdint.h>

#ifndef BIT
#  define BIT(n)              (1UL << (n))
#endif

#ifndef BIT_MASK
#  define BIT_MASK(n)         (BIT(n) - 1UL)
#endif

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(array)   (sizeof(array) / sizeof((array)[0]))
#endif

/* <sys/param.h> of NuttX defines MIN and MAX the same way. */

#ifndef MIN
#  define MIN(a, b)           (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a, b)           (((a) > (b)) ? (a) : (b))
#endif

#ifndef CLAMP
#  define CLAMP(val, low, high) \
  (((val) <= (low)) ? (low) : MIN(val, high))
#endif

#ifndef ARG_UNUSED
#  define ARG_UNUSED(x)       (void)(x)
#endif

#ifndef CONTAINER_OF
#  define CONTAINER_OF(ptr, type, field) \
  ((type *)(((char *)(ptr)) - offsetof(type, field)))
#endif

#ifndef __maybe_unused
#  define __maybe_unused      __attribute__((__unused__))
#endif

#ifndef USEC_PER_SEC
#  define USEC_PER_SEC        1000000
#endif

/* Used by the RTIO and sensor parts (rtio.h, sensor.h). */

/* 32-bit masks (BITS_PER_LONG is 32 on the RP2040; written with unsigned
 * int so that a 64-bit host test gets the same values).
 */

#ifndef GENMASK
#  define GENMASK(h, l) \
  (((~0U) - (1U << (l)) + 1) & (~0U >> (31 - (h))))
#endif

#ifndef FIELD_GET
#  define FIELD_GET(mask, value) \
  (((value) & (mask)) / ((mask) & ~((mask) << 1)))
#endif

#ifndef FIELD_PREP
#  define FIELD_PREP(mask, value) \
  ((((uint32_t)(value)) * ((mask) & ~((mask) << 1))) & (mask))
#endif

#ifndef DIV_ROUND_UP
#  define DIV_ROUND_UP(n, d)  (((n) + (d) - 1) / (d))
#endif

/* Round up to a multiple of the word size (4 bytes on the RP2040). */

#ifndef WB_UP
#  define WB_UP(x)            (((x) + sizeof(void *) - 1) & \
                               ~(sizeof(void *) - 1))
#endif

#ifndef _CONCAT
#  define _DO_CONCAT(x, y)    x ## y
#  define _CONCAT(x, y)       _DO_CONCAT(x, y)
#endif

#ifndef CONCAT
#  define CONCAT(x, y)        _CONCAT(x, y)
#endif

#ifndef __aligned
#  define __aligned(x)        __attribute__((__aligned__(x)))
#endif

#ifndef BUILD_ASSERT
#  define BUILD_ASSERT(expr, ...) _Static_assert(expr, "" __VA_ARGS__)
#endif

/* ilog2() of a 32-bit value: Zephyr's 31 - __builtin_clz(x) at run time,
 * which is -1 for 0 on Arm (CLZ of 0 is 32); spelled out so that a host
 * test gets the same.
 */

#ifndef ilog2
#  define ilog2(x) \
  ((uint32_t)(x) == 0 ? -1 : 31 - __builtin_clz((uint32_t)(x)))
#endif

/* IS_ENABLED(CONFIG_X): 1 if CONFIG_X is defined as 1 (a NuttX "y"), else
 * 0, also when it is not defined at all; usable in C expressions and #if.
 * Zephyr's util_macro.h / util_internal.h.
 */

#define IS_ENABLED(config_macro)        Z_IS_ENABLED1(config_macro)
#define Z_IS_ENABLED1(config_macro)     Z_IS_ENABLED2(_XXXX##config_macro)
#define _XXXX1                          _YYYY,
#define Z_IS_ENABLED2(one_or_two_args)  Z_IS_ENABLED3(one_or_two_args 1, 0)
#define Z_IS_ENABLED3(ignore_this, val, ...) val

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_UTIL_H */
