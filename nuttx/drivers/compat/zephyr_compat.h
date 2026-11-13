/* SPDX-License-Identifier: Apache-2.0 */
/* The start of the Zephyr compatibility layer (nuttx/drivers/compat), for
 * the board's bring-up, which does not include the layer's Zephyr headers
 * (they take over some of NuttX's names, see <zephyr/sys/atomic.h>).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_COMPAT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_COMPAT_H

/* Starts the system work queue ("sysworkq"), the RTIO work queue
 * ("RTIO WQ 0", zephyr_rtio_workq.c) and the gpio0 device: what Zephyr
 * starts at boot (POST_KERNEL) before the sensors.  The board calls it in
 * its bring-up, before the LSM6DSOX.  0 or a negative errno.
 */

int zephyr_compat_initialize(void);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_COMPAT_H */
