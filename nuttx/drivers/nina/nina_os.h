/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services nina_spi.c uses, in the terms of the Zephyr build's
 * drivers/nina/nina_wifi.c.  nina_os.c implements them on NuttX; the host
 * test (nuttx/tests/nina_spi.c) replaces them with fakes.
 */

#ifndef __NUTTX_DRIVERS_NINA_NINA_OS_H
#define __NUTTX_DRIVERS_NINA_NINA_OS_H

#include <stdint.h>

/* k_mutex_lock(&nina_lock, K_FOREVER) / k_mutex_unlock(): a recursive
 * mutex with priority inheritance.  nina_os_lock() returns 0, or a
 * negative errno if the wait was cancelled (never: NuttX's mutex wait only
 * fails like that with pthread cancellation points, which this build does
 * not use; a signal does not end the wait).
 */

int nina_os_lock(void);
void nina_os_unlock(void);

/* k_sleep(K_USEC(us)) and k_sleep(K_MSEC(ms)): the time rounded up to
 * ticks, plus one tick.  Not ended by signals.
 */

void nina_os_sleep_us(uint32_t us);
void nina_os_sleep_ms(uint32_t ms);

/* k_uptime_get(): the kernel clock in milliseconds, rounded down. */

int64_t nina_os_uptime_ms(void);

#endif /* __NUTTX_DRIVERS_NINA_NINA_OS_H */
