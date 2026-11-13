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
