/* SPDX-License-Identifier: Apache-2.0 */
/* The devicetree nodes of the Zephyr build that the ported code names
 * (zephyr/build/usb-console/zephyr/zephyr.dts), as the devices of
 * <zephyr/device.h>: DT_NODELABEL(label) and DT_BUS(node) for use in
 * DEVICE_DT_GET().  The properties of the nodes are in the board's
 * <arch/board/board.h> (BOARD_I2C0_*, BOARD_LSM6DSOX_*).  Part of the small
 * Zephyr compatibility layer of the port (nuttx/drivers/compat).
 *
 *   label     node                                 defined in
 *   i2c0      /soc/i2c@40044000 ("i2c@40044000")   board src/nano_lsm6dso.c
 *   lsm6dsox  /soc/i2c@40044000/lsm6dsox@6a        drivers/lsm6dso/lsm6dso.c
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DEVICETREE_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DEVICETREE_H

#define DT_NODELABEL(label)   label

/* The parent bus of a node: DT_BUS(node) -> DT_BUS_<node>. */

#define DT_BUS(node_id)       Z_DT_BUS(node_id)
#define Z_DT_BUS(node_id)     DT_BUS_##node_id
#define DT_BUS_lsm6dsox       i2c0

struct device;
DEVICE_DT_DECLARE(i2c0);
DEVICE_DT_DECLARE(lsm6dsox);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DEVICETREE_H */
