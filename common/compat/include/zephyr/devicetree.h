/* SPDX-License-Identifier: Apache-2.0 */
/* The devicetree nodes of the Zephyr build that the ported code names
 * (zephyr/build/usb-console/zephyr/zephyr.dts), as the devices of
 * <zephyr/device.h>: DT_NODELABEL(label) and DT_BUS(node) for use in
 * DEVICE_DT_GET(), and DT_HAS_COMPAT_ON_BUS_STATUS_OKAY() for the LSM6DSO
 * driver's buses.  The
 * other properties of the nodes are in the board's <arch/board/board.h>
 * (BOARD_I2C0_*, BOARD_LSM6DSOX_*).  Part of the small Zephyr compatibility
 * layer of the port (common/compat).
 *
 *   label     node                                 defined in
 *   gpio0     /soc/gpio@40014000/gpio-port@0       compat/zephyr_gpio.c
 *   i2c0      /soc/i2c@40044000 ("i2c@40044000")   board src/rp2040_lsm6dso.c
 *                                                  (FreeRTOS: drivers/i2c_dw/
 *                                                  port/freertos/
 *                                                  i2c_dw_instance.h)
 *   i2c1      /soc/i2c@40048000 ("i2c@40048000")   FreeRTOS with
 *                                                  CONFIG_I2C_DW_I2C1 only:
 *                                                  drivers/i2c_dw/
 *                                                  port/freertos/
 *                                                  i2c_dw_instance.h (not in
 *                                                  zephyr_device.c's list)
 *   lsm6dsox  /soc/i2c@40044000/lsm6dsox@6a        drivers/lsm6dso/
 *                                                  lsm6dso_instance.h
 *   storage_partition  partition@ffe000 of /soc/flash-controller@18000000
 *                                                  (<zephyr/storage/flash_map.h>,
 *                                                  compat/zephyr_flash_kv.c)
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_DEVICETREE_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_DEVICETREE_H

#include <zephyr/sys/util.h>

#define DT_NODELABEL(label)   label

/* The node of a compatible: its name, as a label's. */
#define DT_COMPAT_GET_ANY_STATUS_OKAY(compat) compat

/* /chosen: none of the properties that the ported code asks for is set
 * (zephyr,settings-partition: the copy of Zephyr's settings_nvs.c then uses
 * the partition storage_partition of <zephyr/storage/flash_map.h>, as in
 * the Zephyr build, whose app.overlay chooses none either).
 */

#define DT_HAS_CHOSEN(prop)   0

/* The parent bus of a node: DT_BUS(node) -> DT_BUS_<node>. */

#define DT_BUS(node_id)       Z_DT_BUS(node_id)
#define Z_DT_BUS(node_id)     DT_BUS_##node_id
#define DT_BUS_lsm6dsox       i2c0

/* DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(compat, bus): 1 if a node with the
 * compatible is on a bus of that type, else 0; usable in #if.  Zephyr's
 * definition, on the macros DT_COMPAT_<compat>_BUS_<bus> that the
 * devicetree defines as 1: one st,lsm6dso node, on an I2C bus (lsm6dsox);
 * no st,lsm6dso32 node and nothing on SPI.
 */

#define DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(compat, bus) \
  IS_ENABLED(Z_DT_CAT4(DT_COMPAT_, compat, _BUS_, bus))
#define Z_DT_CAT4(a1, a2, a3, a4) a1 ## a2 ## a3 ## a4
#define DT_COMPAT_st_lsm6dso_BUS_i2c 1

struct device;
DEVICE_DT_DECLARE(gpio0);
DEVICE_DT_DECLARE(i2c0);
DEVICE_DT_DECLARE(i2c1);
DEVICE_DT_DECLARE(lsm6dsox);

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_DEVICETREE_H */
