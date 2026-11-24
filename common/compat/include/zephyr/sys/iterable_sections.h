/* SPDX-License-Identifier: Apache-2.0 */
/* <zephyr/sys/iterable_sections.h> for the sources that include it without
 * using it in this build: the sensor shell (nuttx/apps/sensor) walks the
 * sensor_info section only with CONFIG_SENSOR_INFO, which the Zephyr build
 * does not set, and the copy of Zephyr's settings
 * (common/compat/subsys/settings) walks the section of the static settings
 * handlers (SETTINGS_STATIC_HANDLER_DEFINE()), which nothing in the port
 * defines.  The port has no linker sections of Zephyr's: a section is
 * always empty here, STRUCT_SECTION_FOREACH() runs its body never, and
 * STRUCT_SECTION_ITERABLE() is not provided (an object put into a section
 * would not be found).  Part of the small Zephyr compatibility layer of the
 * port (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_ITERABLE_SECTIONS_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_ITERABLE_SECTIONS_H

#define STRUCT_SECTION_FOREACH(struct_type, iterator) \
  for (struct struct_type *iterator = (struct struct_type *)0; \
       iterator != (struct struct_type *)0; iterator = (struct struct_type *)0)

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_ITERABLE_SECTIONS_H */
