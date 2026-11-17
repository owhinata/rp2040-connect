/* SPDX-License-Identifier: MIT */
/* The register read of Zephyr's <zephyr/sys/sys_io.h> that the shared
 * sources use (common/imu/time_sync.c: the TIMER's raw registers), as
 * Zephyr defines it on Arm (zephyr/include/zephyr/arch/common/sys_io.h): one
 * volatile 32-bit load, without a barrier.  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat, see
 * nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_SYS_IO_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_SYS_IO_H

#include <stdint.h>

typedef uintptr_t mem_addr_t;

static inline uint32_t sys_read32(mem_addr_t addr)
{
  return *(volatile uint32_t *)addr;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_SYS_IO_H */
