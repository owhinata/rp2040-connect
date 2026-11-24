/* SPDX-License-Identifier: MIT */
/* The variables of the ThreadX primitives of the generic kernel part of the
 * Zephyr compatibility layer (zephyr_kernel_os_impl.h): the depth of the
 * thread's lock, z_os_task_lock(), and the interrupt mask it saved at the
 * outermost level.  Part of the ThreadX implementation of the layer
 * (common/compat/port/threadx).
 */

#include "zephyr_kernel_os_impl.h"

UINT z_tx_lock_depth;
UINT z_tx_lock_state;
