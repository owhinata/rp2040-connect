/* SPDX-License-Identifier: MIT */
/* The Zephyr build's Kconfig values (zephyr/build/usb-console/zephyr/.config)
 * that the copies of Zephyr's NVS and settings (common/compat/subsys) read,
 * included before each of them (-include, as the Zephyr build reads its
 * configuration into every file).  They decide the bytes on the flash:
 * with other values the store would not be the Zephyr build's.
 *
 *   option                                   .config    here
 *   CONFIG_NVS                               y          1
 *   CONFIG_NVS_LOOKUP_CACHE                  not set    not defined
 *   CONFIG_NVS_DATA_CRC                      not set    not defined
 *   CONFIG_NVS_INIT_BAD_MEMORY_REGION        not set    not defined
 *   CONFIG_NVS_LOG_LEVEL                     3          3
 *   CONFIG_SETTINGS                          y          1
 *   CONFIG_SETTINGS_NVS                      y          1
 *   CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT     1          1
 *   CONFIG_SETTINGS_NVS_SECTOR_COUNT         8          8 (the area holds 2)
 *   CONFIG_SETTINGS_NVS_NAME_CACHE           not set    not defined
 *   CONFIG_SETTINGS_DYNAMIC_HANDLERS         y          1
 *   CONFIG_SETTINGS_RUNTIME                  not set    not defined
 *   CONFIG_SETTINGS_SAVE_SINGLE_SUBTREE_...  not set    not defined
 *   CONFIG_SETTINGS_LOG_LEVEL                3          3
 *   CONFIG_MULTITHREADING                    y          1
 *   CONFIG_ZTEST                             not set    not defined
 *
 * The undefined ones are tested with #if in places (nvs.h, settings_nvs.c),
 * which is 0 as in Zephyr; build them without -Wundef.  Part of the small
 * Zephyr compatibility layer of the port (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_KV_CONFIG_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_KV_CONFIG_H

#define CONFIG_NVS                            1
#define CONFIG_NVS_LOG_LEVEL                  3
#define CONFIG_SETTINGS                       1
#define CONFIG_SETTINGS_NVS                   1
#define CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT  1
#define CONFIG_SETTINGS_NVS_SECTOR_COUNT      8
#define CONFIG_SETTINGS_DYNAMIC_HANDLERS      1
#define CONFIG_SETTINGS_LOG_LEVEL             3
#define CONFIG_MULTITHREADING                 1

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_KV_CONFIG_H */
