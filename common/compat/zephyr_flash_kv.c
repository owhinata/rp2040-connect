/* SPDX-License-Identifier: MIT */
/* The flash controller of <zephyr/drivers/flash.h> and the flash map of
 * <zephyr/storage/flash_map.h> for the key-value area (the last 8 KiB of
 * the flash, flash_kv.h), on which the copies of Zephyr's NVS and
 * settings (common/compat/subsys) keep the store as in the Zephyr build.
 *
 * The parameters are those of the Zephyr build's driver and devicetree
 * (zephyr/drivers/flash/flash_rpi_pico.c, rp2040.dtsi, app.overlay):
 * write block 1, erase value 0xff, 4096-byte pages, storage_partition at
 * 0xffe000 of 0x2000 bytes.  With CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT 1
 * and CONFIG_SETTINGS_NVS_SECTOR_COUNT 8 (zephyr_kv_config.h), the copy of
 * settings_nvs.c makes NVS of 2 sectors of 4096 bytes at 0xffe000, as the
 * Zephyr build does, so the bytes on the flash are the same.
 *
 * Every request is checked against the area (flash_kv_range_ok()) before it
 * goes to the OS's boundary, and the OS checks it again.  A write is split
 * into 256-byte page programs, each page 0xff around the data, and an
 * erase into 4096-byte sector erases: the interrupts are off for one page
 * or one sector at a time (the Zephyr build's driver keeps them off for
 * the whole flash_write() or flash_erase() call; NVS writes one entry's
 * data, at most 2 pages, or one ATE per call, and erases one sector per
 * call).  Writes and erases hold kv_flash_lock, which serializes the page
 * buffer and the calls to the boundary (NVS and the settings already
 * serialize their callers).
 *
 * flash_area_open() first checks what the area holds
 * (flash_kv_layout()), once: anything but Zephyr's NVS or an erased
 * area is reported in the log and makes flash_area_open() fail with
 * -EILSEQ, so the settings are not initialized and nothing is written.
 * flash_kv_wipe() (the "kv wipe" command, kv_store_wipe.c) erases the
 * area and lets the next flash_area_open() check it again.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>

#include "flash_kv.h"
#include "flash_kv_os.h"

LOG_MODULE_REGISTER(flash_kv, LOG_LEVEL_INF);

const struct device z_flash_kv_device = {
	.name = "flash-controller@18000000",
};

static const struct flash_parameters kv_flash_parameters = {
	.write_block_size = 1,
	.erase_value = FLASH_KV_ERASE_VALUE,
};

static const struct flash_area kv_storage_partition = {
	.fa_id = PARTITION_ID(storage_partition),
	.fa_off = FLASH_KV_OFFSET,
	.fa_size = FLASH_KV_SIZE,
	.fa_dev = &z_flash_kv_device,
};

static K_MUTEX_DEFINE(kv_flash_lock);
static uint8_t kv_page[FLASH_KV_PAGE_SIZE] __aligned(4);

/* The result of the check of the area: unchecked until the first
 * flash_area_open(), again after flash_kv_wipe().
 */

static bool kv_checked;
static enum flash_kv_layout_e kv_layout;

static bool kv_request_ok(const struct device *dev, off_t offset, size_t len,
			  uint32_t align, const char *what)
{
	if (dev == &z_flash_kv_device && offset >= 0 && len <= FLASH_KV_SIZE &&
	    flash_kv_range_ok((uint32_t)offset, (uint32_t)len, FLASH_KV_OFFSET,
			      FLASH_KV_END, align)) {
		return true;
	}
	LOG_ERR("Refused flash %s at 0x%08lx, %lu bytes", what,
		(unsigned long)offset, (unsigned long)len);
	return false;
}

int flash_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	if (len == 0) {
		return 0;
	}
	if (!kv_request_ok(dev, offset, len, 1, "read")) {
		return -EINVAL;
	}
	return flash_kv_os_read((uint32_t)offset, data, (uint32_t)len);
}

static int kv_program_page(void *arg, uint32_t page_offset, const uint8_t *page)
{
	ARG_UNUSED(arg);
	return flash_kv_os_program(page_offset, page);
}

int flash_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	int rc;

	if (len == 0) {
		return 0;
	}
	if (!kv_request_ok(dev, offset, len, 1, "write")) {
		return -EINVAL;
	}
	k_mutex_lock(&kv_flash_lock, K_FOREVER);
	rc = flash_kv_write_pages((uint32_t)offset, data, (uint32_t)len, kv_page,
				  kv_program_page, NULL);
	k_mutex_unlock(&kv_flash_lock);
	return rc;
}

int flash_erase(const struct device *dev, off_t offset, size_t size)
{
	int rc = 0;

	if (size == 0) {
		return 0;
	}
	if (!kv_request_ok(dev, offset, size, FLASH_KV_SECTOR_SIZE, "erase")) {
		return -EINVAL;
	}
	k_mutex_lock(&kv_flash_lock, K_FOREVER);
	for (size_t done = 0; done < size && rc == 0; done += FLASH_KV_SECTOR_SIZE) {
		rc = flash_kv_os_erase((uint32_t)offset + done);
	}
	k_mutex_unlock(&kv_flash_lock);
	return rc;
}

int flash_flatten(const struct device *dev, off_t offset, size_t size)
{
	return flash_erase(dev, offset, size);
}

const struct flash_parameters *flash_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &kv_flash_parameters;
}

size_t flash_get_write_block_size(const struct device *dev)
{
	return flash_get_parameters(dev)->write_block_size;
}

int flash_get_page_info_by_offs(const struct device *dev, off_t offset,
				struct flash_pages_info *info)
{
	if (dev != &z_flash_kv_device || offset < 0 ||
	    (uint32_t)offset >= FLASH_KV_FLASH_SIZE) {
		return -EINVAL;
	}
	info->index = (uint32_t)offset / FLASH_KV_SECTOR_SIZE;
	info->start_offset = (off_t)(info->index * FLASH_KV_SECTOR_SIZE);
	info->size = FLASH_KV_SECTOR_SIZE;
	return 0;
}

int flash_kv_layout(enum flash_kv_layout_e *layout)
{
	unsigned int nvs_sectors = 0, nuttx_blocks = 0;
	uint8_t tail[32];
	bool erased = true;
	int rc;

	for (uint32_t sector = 0; sector < FLASH_KV_SECTORS; sector++) {
		uint32_t end = FLASH_KV_OFFSET + (sector + 1) * FLASH_KV_SECTOR_SIZE;
		const uint8_t *nvs_first = &tail[16], *nvs_close = &tail[24];

		rc = flash_kv_os_read(end - sizeof(tail), tail, sizeof(tail));
		if (rc) {
			return rc;
		}
		if (flash_kv_nvs_ate_valid(
			    nvs_first, crc8_ccitt(0xff, nvs_first, FLASH_KV_NVS_ATE_CRC_LEN),
			    FLASH_KV_SECTOR_SIZE - 2 * FLASH_KV_NVS_ATE_SIZE, false) ||
		    flash_kv_nvs_ate_valid(nvs_close,
					   crc8_ccitt(0xff, nvs_close, FLASH_KV_NVS_ATE_CRC_LEN),
					   FLASH_KV_SECTOR_SIZE - FLASH_KV_NVS_ATE_SIZE, true)) {
			nvs_sectors++;
		}
		if (flash_kv_nuttx_ate_plausible(
			    &tail[0], flash_kv_nuttx_crc8(&tail[0], FLASH_KV_NUTTX_ATE_CRC_LEN),
			    FLASH_KV_SECTOR_SIZE, false) ||
		    flash_kv_nuttx_ate_plausible(
			    &tail[16], flash_kv_nuttx_crc8(&tail[16], FLASH_KV_NUTTX_ATE_CRC_LEN),
			    FLASH_KV_SECTOR_SIZE, true)) {
			nuttx_blocks++;
		}
	}
	for (uint32_t off = 0; off < FLASH_KV_SIZE && erased && nvs_sectors == 0;
	     off += sizeof(tail)) {
		rc = flash_kv_os_read(FLASH_KV_OFFSET + off, tail, sizeof(tail));
		if (rc) {
			return rc;
		}
		for (size_t i = 0; i < sizeof(tail); i++) {
			if (tail[i] != FLASH_KV_ERASE_VALUE) {
				erased = false;
				break;
			}
		}
	}
	*layout = flash_kv_layout_classify(nvs_sectors, nuttx_blocks, erased);
	return 0;
}

static int kv_check(void)
{
	int rc;

	if (kv_checked) {
		return 0;
	}
	rc = flash_kv_layout(&kv_layout);
	if (rc) {
		LOG_ERR("Could not read the key-value area: %d", rc);
		return rc;
	}
	kv_checked = true;
	switch (kv_layout) {
	case FLASH_KV_LAYOUT_NUTTX:
		LOG_ERR("Key-value area 0x%06x-0x%06x holds the NuttX build's earlier format",
			FLASH_KV_OFFSET, FLASH_KV_END - 1);
		break;
	case FLASH_KV_LAYOUT_FOREIGN:
		LOG_ERR("Key-value area 0x%06x-0x%06x holds data that is not Zephyr's NVS",
			FLASH_KV_OFFSET, FLASH_KV_END - 1);
		break;
	default:
		return 0;
	}
	LOG_ERR("Not used; \"kv wipe confirm\" erases it (all keys and Wi-Fi credentials)");
	return 0;
}

int flash_area_open(uint8_t id, const struct flash_area **fa)
{
	int rc;

	if (id != PARTITION_ID(storage_partition)) {
		return -ENOENT;
	}
	if (!flash_kv_os_ready()) {
		return -ENODEV;
	}
	k_mutex_lock(&kv_flash_lock, K_FOREVER);
	rc = kv_check();
	if (rc == 0 && kv_layout != FLASH_KV_LAYOUT_NVS && kv_layout != FLASH_KV_LAYOUT_EMPTY) {
		rc = -EILSEQ;
	}
	k_mutex_unlock(&kv_flash_lock);
	if (rc) {
		return rc;
	}
	*fa = &kv_storage_partition;
	return 0;
}

int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors)
{
	const struct flash_area *fa;
	uint32_t n = 0;
	int rc = flash_area_open((uint8_t)fa_id, &fa);

	if (rc < 0 || fa_id < 0 || fa_id > UINT8_MAX) {
		return -EINVAL;
	}
	for (size_t off = 0; off < fa->fa_size; off += FLASH_KV_SECTOR_SIZE) {
		if (n >= *count) {
			flash_area_close(fa);
			return -ENOMEM;
		}
		sectors[n].fs_off = (ptrdiff_t)off;
		sectors[n].fs_size = FLASH_KV_SECTOR_SIZE;
		n++;
	}
	flash_area_close(fa);
	*count = n;
	return 0;
}

int flash_kv_wipe(void)
{
	int rc;

	if (!flash_kv_os_ready()) {
		return -ENODEV;
	}
	rc = flash_erase(&z_flash_kv_device, FLASH_KV_OFFSET, FLASH_KV_SIZE);
	k_mutex_lock(&kv_flash_lock, K_FOREVER);
	kv_checked = false;
	k_mutex_unlock(&kv_flash_lock);
	if (rc) {
		LOG_ERR("Could not erase the key-value area: %d", rc);
	} else {
		LOG_WRN("Key-value area erased");
	}
	return rc;
}
