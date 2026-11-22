/* SPDX-License-Identifier: MIT */
/* The OS boundary of the key-value area (common/compat/flash_kv_os.h) on the
 * RP2040: reads, 256-byte page programs and 4 KiB sector erases of the last
 * 8 KiB of the 16 MiB flash (flash offsets 0xffe000-0xffffff, XIP
 * 0x10ffe000-0x10ffffff, two sectors), the Zephyr build's
 * storage_partition.  Shared by the NuttX and FreeRTOS builds, which have
 * no flash driver of their own for the area (the Zephyr build uses its
 * flash driver and does not compile this file); each writes the flash
 * operations that stop XIP (rp2040_flash_kv_os.h).  The copies of Zephyr's
 * NVS and settings (common/compat/subsys) keep the store in the area in the
 * Zephyr build's format through the flash device of
 * common/compat/zephyr_flash_kv.c, which calls these functions.
 *
 * Safety: an erase or a program outside the area would destroy the
 * firmware.  Every request is checked three times before the flash is
 * touched: in zephyr_flash_kv.c (against the constants of flash_kv.h),
 * here (kv_flash(): against the constants and against the bounds that
 * rp2040_flash_kv_set_bounds() took from the linker script), and again by
 * the build with the interrupts off right before the flash is touched
 * (rp2040_flash_kv_os_program(), _erase()).  The bounds are set only after
 * the linker symbols __kv_start and __kv_end have been found equal to the
 * constants, the image found to end before the area and the build's flash
 * operations found ready; until then every erase and program is refused
 * and flash_kv_os_ready() is false.  Nothing here erases at the start.
 *
 * The page to program is copied into RAM first (kv_page): the caller's
 * page may be anywhere, and the boot ROM reads it while XIP is off.
 * Reads go through the uncached XIP alias (XIP_NOCACHE_NOALLOC_BASE,
 * 0x13000000), which shows what the flash holds right after a program or
 * an erase; the Zephyr build's driver reads the cached alias, which the
 * boot ROM's flash_flush_cache() at the end of each operation also keeps up
 * to date.  This file runs with XIP on and may be in the flash.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_log.h"
#include "flash_kv.h"
#include "flash_kv_os.h"
#include "rp2040_flash_kv.h"
#include "rp2040_flash_kv_os.h"

LOG_MODULE_REGISTER(flash_kv, LOG_LEVEL_INF);

/* The build's linker script (rp2040_flash_kv_os.h). */
extern const uint8_t __kv_start[];
extern const uint8_t __kv_end[];

/* The page being programmed (see above).  The calls come one at a time
 * (flash_kv_os.h).
 */
static uint8_t kv_page[FLASH_KV_PAGE_SIZE] __attribute__((aligned(4)));

/* The area's bounds as flash offsets once the linker script has been
 * checked; 0 and 0 refuse everything.
 */
static uint32_t kv_lo;
static uint32_t kv_hi;

/* Checks a program (page != NULL, count 256) or erase (page == NULL, count
 * 4096) request at flash offset `offset` against the constants and against
 * the bounds, and hands it to the build with the bounds for its last check.
 */
static int kv_flash(uint32_t offset, const uint8_t *page, uint32_t count)
{
	uint32_t align = page != NULL ? FLASH_KV_PAGE_SIZE : FLASH_KV_SECTOR_SIZE;
	uint32_t lo = kv_lo;
	uint32_t hi = kv_hi;

	if (count != align ||
	    !flash_kv_range_ok(offset, count, FLASH_KV_OFFSET, FLASH_KV_END, align) ||
	    !flash_kv_range_ok(offset, count, lo, hi, align)) {
		LOG_ERR("Refused flash %s at 0x%08lx, %lu bytes", page != NULL ? "write" : "erase",
			(unsigned long)offset, (unsigned long)count);
		return -EINVAL;
	}
	if (page != NULL) {
		return rp2040_flash_kv_os_program(offset, page, lo, hi);
	}
	return rp2040_flash_kv_os_erase(offset, lo, hi);
}

/* The boundary of common/compat/flash_kv_os.h. */

int flash_kv_os_initialize(void)
{
	return rp2040_flash_kv_set_bounds((uintptr_t)__kv_start, (uintptr_t)__kv_end,
					  rp2040_flash_kv_os_image_end());
}

bool flash_kv_os_ready(void)
{
	return kv_lo != 0;
}

int flash_kv_os_read(uint32_t offset, void *buf, uint32_t len)
{
	if (!flash_kv_range_ok(offset, len, FLASH_KV_OFFSET, FLASH_KV_END, 1)) {
		LOG_ERR("Refused flash read at 0x%08lx, %lu bytes", (unsigned long)offset,
			(unsigned long)len);
		return -EINVAL;
	}
	memcpy(buf, (const void *)(uintptr_t)(RP2040_FLASH_KV_XIP_NOCACHE_BASE + offset), len);
	return 0;
}

int flash_kv_os_program(uint32_t page_offset, const uint8_t page[FLASH_KV_PAGE_SIZE])
{
	memcpy(kv_page, page, FLASH_KV_PAGE_SIZE);
	return kv_flash(page_offset, kv_page, FLASH_KV_PAGE_SIZE);
}

int flash_kv_os_erase(uint32_t sector_offset)
{
	return kv_flash(sector_offset, NULL, FLASH_KV_SECTOR_SIZE);
}

/* The start (rp2040_flash_kv.h). */

int rp2040_flash_kv_set_bounds(uintptr_t kv_start, uintptr_t kv_end, uintptr_t image_end)
{
	int rc;

	kv_lo = 0;
	kv_hi = 0;
	if (kv_start != RP2040_FLASH_KV_XIP_BASE + FLASH_KV_OFFSET ||
	    kv_end != RP2040_FLASH_KV_XIP_BASE + FLASH_KV_END ||
	    image_end <= RP2040_FLASH_KV_XIP_BASE || image_end > kv_start) {
		LOG_ERR("Key-value area: linker script disagrees (0x%08lx-0x%08lx, image end "
			"0x%08lx)",
			(unsigned long)kv_start, (unsigned long)kv_end, (unsigned long)image_end);
		return -EFAULT;
	}
	rc = rp2040_flash_kv_os_init();
	if (rc < 0) {
		return rc;
	}
	kv_lo = FLASH_KV_OFFSET;
	kv_hi = FLASH_KV_END;
	return 0;
}
