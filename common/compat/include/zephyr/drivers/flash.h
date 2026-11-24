/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/drivers/flash.h> (Zephyr v4.5.0) that the
 * copy of Zephyr's NVS (common/compat/subsys/kvss/nvs/nvs.c) uses, with
 * Zephyr's names, types and the behaviour of the Zephyr build's driver
 * (zephyr/drivers/flash/flash_rpi_pico.c with the board's devicetree):
 *
 *   write_block_size 1, erase_value 0xff, explicit erase,
 *   pages (erase units) of 4096 bytes over the 16 MiB flash,
 *   offsets from the start of the flash (0 is XIP address 0x10000000).
 *
 * The only device is the flash controller of the key-value area
 * (zephyr_flash_kv.c, which also provides the flash map of
 * <zephyr/storage/flash_map.h>).  Unlike the Zephyr build's driver, which
 * reads, writes and erases anywhere in the flash, it refuses (-EINVAL,
 * with a log line) every request that does not lie entirely inside the
 * key-value area, the last 8 KiB (FLASH_KV_OFFSET, FLASH_KV_SIZE of
 * flash_kv.h): the firmware is never touched.  Reads, page programs
 * and sector erases go to the OS's boundary (flash_kv_os.h).  Threads
 * only.  Part of the small Zephyr compatibility layer of the port
 * (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_DRIVERS_FLASH_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_DRIVERS_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <zephyr/device.h>

struct flash_parameters
{
  /* Minimal write alignment and size */

  const size_t write_block_size;

  struct
    {
      bool no_explicit_erase : 1;
      bool erase_value_readback_unsupported : 1;
    } caps;

  /* Value of erased flash */

  uint8_t erase_value;
};

struct flash_pages_info
{
  off_t start_offset;  /* from the start of the flash */
  size_t size;
  uint32_t index;
};

/* The flash controller (Zephyr: DEVICE_DT_GET(DT_NODELABEL(ssi))). */

extern const struct device z_flash_kv_device;

/* 0 or a negative errno: -EINVAL outside the key-value area, else the
 * OS boundary's error.  LEN 0 does nothing and returns 0.
 */

int flash_read(const struct device *dev, off_t offset, void *data,
               size_t len);

/* Programs LEN bytes at OFFSET (programming only clears bits), one
 * 256-byte page at a time with 0xff in the page around the data.
 */

int flash_write(const struct device *dev, off_t offset, const void *data,
                size_t len);

/* Erases whole 4096-byte sectors, one at a time. */

int flash_erase(const struct device *dev, off_t offset, size_t size);

/* flash_erase() for this device (explicit erase). */

int flash_flatten(const struct device *dev, off_t offset, size_t size);

const struct flash_parameters *flash_get_parameters(const struct device *dev);
size_t flash_get_write_block_size(const struct device *dev);
int flash_get_page_info_by_offs(const struct device *dev, off_t offset,
                                struct flash_pages_info *info);

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_DRIVERS_FLASH_H */
