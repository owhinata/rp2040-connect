/* SPDX-License-Identifier: MIT */
/* The OS part of <zephyr/sys/atomic.h> of the Zephyr compatibility layer
 * (common/compat/include/zephyr/sys/atomic.h, which includes this header before it
 * defines its names), the generic kernel part's default: none, for an OS
 * whose headers define neither atomic_t nor atomic_*() (FreeRTOS.h and
 * FreeRTOS's own atomic.h, with Atomic_*() names, which is not used;
 * ThreadX's tx_api.h; the Pico SDK's headers).  An OS whose headers do
 * puts its own zephyr_os_atomic.h before this directory on the include path.
 */

#ifndef COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_ATOMIC_H
#define COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_ATOMIC_H

#endif /* COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_ATOMIC_H */
