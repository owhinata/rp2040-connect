/* SPDX-License-Identifier: Apache-2.0 */
/* <zephyr/sys/iterable_sections.h> for the sources that include it without
 * using it in this build: the sensor shell (nuttx/apps/sensor) walks the
 * sensor_info section only with CONFIG_SENSOR_INFO, which the Zephyr build
 * does not set.  STRUCT_SECTION_FOREACH() and the other section macros are
 * not provided (the port has no linker sections of Zephyr's).  Part of the
 * small Zephyr compatibility layer of the port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_ITERABLE_SECTIONS_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_ITERABLE_SECTIONS_H

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_ITERABLE_SECTIONS_H */
