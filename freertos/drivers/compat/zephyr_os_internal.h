/* SPDX-License-Identifier: MIT */
/* What the files of the FreeRTOS part of the Zephyr compatibility layer share
 * (zephyr_kernel.c, zephyr_mutex.c); not for the layer's users.  Include
 * after <zephyr/kernel.h>.  Part of the FreeRTOS implementation of the layer
 * (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_INTERNAL_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_INTERNAL_H

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

/* The FreeRTOS block time of a Zephyr timeout: K_NO_WAIT (and any count of 0
 * or less) 0, K_FOREVER portMAX_DELAY (for ever, INCLUDE_vTaskSuspend), else
 * ticks + 1, Zephyr's end of a relative wait (z_add_timeout(): the current
 * tick + ticks + 1; FreeRTOS ends a block time of n at the current tick + n),
 * at most portMAX_DELAY - 1 (about 49.7 days with the board's 32-bit tick of
 * 1 ms).
 */
TickType_t z_os_wait_ticks(k_timeout_t timeout);

/* k_yield() for the file's own sleeps (zephyr_kernel.c). */
void z_os_yield(void);

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_INTERNAL_H */
