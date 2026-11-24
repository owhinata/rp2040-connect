/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of <zephyr/sys/atomic.h> of the Zephyr compatibility
 * layer (common/compat/include/zephyr/sys/atomic.h, which includes this header
 * before it defines its names).  NuttX's <nuttx/atomic.h> (pulled in by
 * most kernel headers) has its own atomic_t and atomic_set(), atomic_add(),
 * atomic_or()... with other meanings.  It is included here first and its
 * macros undefined, so that the layer's header takes the names over for
 * the file: atomic_t is zephyr_atomic_t.  A file that uses both must
 * include the NuttX headers before any header of the compatibility layer.
 * Part of the NuttX implementation of the layer (common/compat/port/nuttx).
 */

#ifndef __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_OS_ATOMIC_H
#define __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_OS_ATOMIC_H

#include <nuttx/config.h>
#include <nuttx/atomic.h>
#undef atomic_set
#undef atomic_add
#undef atomic_sub
#undef atomic_and
#undef atomic_or
#undef atomic_xor

#endif /* __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_OS_ATOMIC_H */
