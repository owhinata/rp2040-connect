/* SPDX-License-Identifier: MIT */
/* What the files of the generic kernel part of the Zephyr compatibility
 * layer share (zephyr_kernel.c, zephyr_mutex.c); not for the layer's users.
 * Include after <zephyr/kernel.h>.
 */

#ifndef COMMON_COMPAT_INTERNAL_ZEPHYR_OS_INTERNAL_H
#define COMMON_COMPAT_INTERNAL_ZEPHYR_OS_INTERNAL_H

#include "zephyr_kernel_os.h"

/* The OS's wait of a Zephyr timeout (z_os_wait(), z_os_mutex_lock()):
 * K_NO_WAIT (and any count of 0 or less) 0, K_FOREVER Z_OS_TICKS_FOREVER,
 * else ticks + 1, Zephyr's end of a relative wait (z_add_timeout(): the
 * current tick + ticks + 1; the OS ends a wait of n at the current tick +
 * n), at most Z_OS_TICKS_FOREVER - 1 (about 49.7 days with a 32-bit tick of
 * 1 ms).
 */
z_os_ticks_t z_wait_ticks(k_timeout_t timeout);

#endif /* COMMON_COMPAT_INTERNAL_ZEPHYR_OS_INTERNAL_H */
