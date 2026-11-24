/* SPDX-License-Identifier: Apache-2.0 */
/* The few macros of Zephyr's <zephyr/sys/util.h> (and util_macro.h) that the
 * ported sources use, with Zephyr's definitions, so that those sources keep
 * their text.  Part of the small Zephyr compatibility layer of the port
 * (common/compat, see common/compat/README.md); it is not Zephyr's
 * header and has nothing else.
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_UTIL_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_UTIL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>  /* as Zephyr's sys/util.h, which its sources rely on */

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

/* 64-bit masks (zephyr/sys/util.h; for the Q value print format of
 * <zephyr/dsp/print_format.h>).
 */

#ifndef GENMASK64
#  define GENMASK64(h, l) \
  (((~0ULL) - (1ULL << (l)) + 1) & (~0ULL >> (64 - 1 - (h))))
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

/* <zephyr/toolchain/common.h> */

#ifndef STRINGIFY
#  define Z_STRINGIFY(x)      #x
#  define STRINGIFY(s)        Z_STRINGIFY(s)
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

/* Used by the copies of Zephyr's NVS and settings (common/compat/subsys):
 * KB() (a size_t, as Zephyr's), ZTESTABLE_STATIC (static outside ztest).
 */

#ifndef KB
#  define KB(x)               (((size_t)(x)) << 10)
#endif

#ifndef ZTESTABLE_STATIC
#  define ZTESTABLE_STATIC    static
#endif

/* The hexadecimal conversions of Zephyr's lib/utils/hex.c (the copy in
 * common/compat/lib/utils/hex.c), which the "kv" command uses.
 */

int char2hex(char c, uint8_t *x);
int hex2char(uint8_t x, char *c);
size_t bin2hex(const uint8_t *buf, size_t buflen, char *hex, size_t hexlen);
size_t hex2bin(const char *hex, size_t hexlen, uint8_t *buf, size_t buflen);

/* sys_gcd(): the greatest common divisor, unsigned (Zephyr's, which the
 * copy of Zephyr's getopt_long.c in common/compat/lib/utils/getopt uses).
 */

#define sys_gcd(a, b) \
  ((((__typeof__(a))-1) < 0) ? sys_gcd_s(a, b) : sys_gcd_u(a, b))

static inline uint32_t sys_gcd_u(uint32_t a, uint32_t b)
{
  uint32_t c;

  if (a == 0)
    {
      return b;
    }

  if (b == 0)
    {
      return a;
    }

  c = a % b;
  while (c != 0)
    {
      a = b;
      b = c;
      c = a % b;
    }

  return b;
}

static inline uint32_t sys_gcd_s(int32_t a, int32_t b)
{
  return sys_gcd_u(a < 0 ? -(uint32_t)a : (uint32_t)a,
                   b < 0 ? -(uint32_t)b : (uint32_t)b);
}

/* IS_ENABLED(CONFIG_X): 1 if CONFIG_X is defined as 1 (a NuttX "y"), else
 * 0, also when it is not defined at all; usable in C expressions and #if.
 * Zephyr's util_macro.h / util_internal.h.
 */

#define IS_ENABLED(config_macro)        Z_IS_ENABLED1(config_macro)
#define Z_IS_ENABLED1(config_macro)     Z_IS_ENABLED2(_XXXX##config_macro)
#define _XXXX1                          _YYYY,
#define Z_IS_ENABLED2(one_or_two_args)  Z_IS_ENABLED3(one_or_two_args 1, 0)
#define Z_IS_ENABLED3(ignore_this, val, ...) val

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_UTIL_H */
