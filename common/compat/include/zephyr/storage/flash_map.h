/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/storage/flash_map.h> (Zephyr v4.5.0) that
 * the copy of Zephyr's settings on NVS
 * (common/compat/subsys/settings/src/settings_nvs.c) uses, with Zephyr's
 * names and types, for the one partition of the Zephyr build's devicetree
 * that the port has (zephyr/app/usb-console/app.overlay):
 *
 *   storage_partition: partition@ffe000, reg = <0xffe000 0x2000>, on the
 *   flash controller of <zephyr/drivers/flash.h> (4096-byte sectors).
 *
 * flash_area_open() is where the port checks what the area holds before
 * NVS mounts it (zephyr_flash_kv.c, flash_kv_layout()): data that is
 * neither Zephyr's NVS nor erased is logged once and the area is not
 * opened (-EILSEQ), so that the settings, and every key-value and Wi-Fi
 * credential call after them, fail without writing anything; the "kv
 * wipe" command erases it (kv_store_wipe.c).  Not in Zephyr, whose NVS
 * mounts whatever is there (see flash_kv.h).  -ENODEV while the OS's
 * boundary is not ready (Zephyr: the flash device is not ready).  Part of
 * the small Zephyr compatibility layer of the port (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_STORAGE_FLASH_MAP_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_STORAGE_FLASH_MAP_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

struct flash_area
{
  uint8_t fa_id;
  uint16_t pad16;
  off_t fa_off;                 /* from the start of the flash device */
  size_t fa_size;
  const struct device *fa_dev;
};

struct flash_sector
{
  ptrdiff_t fs_off;             /* from the start of the partition */
  size_t fs_size;
};

/* PARTITION_ID(label): the partitions of the devicetree. */

#define PARTITION_ID(label)                     Z_PARTITION_ID(label)
#define Z_PARTITION_ID(label)                   Z_PARTITION_ID_##label
#define Z_PARTITION_ID_storage_partition        0

int flash_area_open(uint8_t id, const struct flash_area **fa);

static inline void flash_area_close(const struct flash_area *fa)
{
  (void)fa;
}

/* The sectors of the partition: up to *COUNT of them into SECTORS; -ENOMEM
 * if there are more (SECTORS filled, *COUNT unchanged), as Zephyr's.
 */

int flash_area_get_sectors(int fa_id, uint32_t *count,
                           struct flash_sector *sectors);

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_STORAGE_FLASH_MAP_H */
