/* SPDX-License-Identifier: Apache-2.0 */
/* The big-endian accessors of Zephyr's <zephyr/sys/byteorder.h> that the
 * ported code uses, with Zephyr's definitions: values at any alignment,
 * independent of the CPU's byte order.  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat, see
 * nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_BYTEORDER_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_BYTEORDER_H

#include <stdint.h>

static inline void sys_put_be16(uint16_t val, uint8_t dst[2])
{
  dst[0] = val >> 8;
  dst[1] = val;
}

static inline void sys_put_be32(uint32_t val, uint8_t dst[4])
{
  sys_put_be16(val >> 16, dst);
  sys_put_be16(val, &dst[2]);
}

static inline uint16_t sys_get_be16(const uint8_t src[2])
{
  return ((uint16_t)src[0] << 8) | src[1];
}

static inline uint32_t sys_get_be32(const uint8_t src[4])
{
  return ((uint32_t)sys_get_be16(&src[0]) << 16) | sys_get_be16(&src[2]);
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_BYTEORDER_H */
