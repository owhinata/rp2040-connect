/* SPDX-License-Identifier: MIT */
/* The OS boundary of the key-value area (common/compat/flash_kv_os.h) on
 * the Pico SDK: reads, 256-byte page programs and 4 KiB sector erases of
 * the last 8 KiB of the 16 MiB flash (flash offsets 0xffe000-0xffffff, XIP
 * 0x10ffe000-0x10ffffff, two sectors), the Zephyr build's
 * storage_partition.  The copies of Zephyr's NVS and settings
 * (common/compat/subsys) keep the store there in the Zephyr build's format
 * through the flash device of common/compat/zephyr_flash_kv.c, which calls
 * these functions.  The image never reaches the area: the linker script
 * leaves it out of the FLASH region and checks the image's end
 * (freertos/drivers/kv/ld, kv.cmake).
 *
 * Safety: an erase or a program outside the area would destroy the
 * firmware.  Every request is checked three times before the flash is
 * touched: in zephyr_flash_kv.c (against the constants of flash_kv.h),
 * here (kv_flash(): against the constants and against the bounds that
 * flash_kv_os_rp2040_set_bounds() took from the linker script), and again
 * with the interrupts off right before flash_range_program() or
 * flash_range_erase() is called (kv_flash_op()).  The bounds are set only
 * after the linker symbols __kv_start and __kv_end have been found equal to
 * the constants and the image found to end before the area; until then
 * every erase and program is refused and flash_kv_os_ready() is false.
 * The board's flash (PICO_FLASH_SIZE_BYTES of the board header) and the
 * SDK's page and sector sizes are checked against the constants when this
 * file is compiled.
 *
 * Erasing and programming stop XIP: from the boot ROM's
 * connect_internal_flash() and flash_exit_xip() until the RAM copy of
 * boot2 has restarted it, nothing may be fetched from the flash.  The SDK's
 * flash_range_erase() and flash_range_program() (hardware_flash/flash.c)
 * do that whole sequence themselves, from RAM (__no_inline_not_in_flash_func:
 * they and the helpers they call while XIP is off, flash_enable_xip_via_boot2()
 * and the copy of boot2 in boot2_copyout[], are in RAM; the ROM functions
 * are looked up inline before XIP stops), and end with the ROM's
 * flash_flush_cache(), which flushes the XIP cache and releases the chip
 * select.  They do not disable the interrupts: a handler in the flash (all
 * of them here: SysTick, PendSV, the USB, the SIO FIFO's) would fault while
 * XIP is off.  So they are called from flash_safe_execute() of pico_flash,
 * which with PICO_FLASH_ASSUME_CORE1_SAFE=1 (kv.cmake) disables this core's
 * interrupts (save_and_disable_interrupts(), PRIMASK: every interrupt,
 * SysTick and PendSV included) and does not try to stop core 1
 * (pico_flash/flash.c, use_irq_only()).  Without that option it always
 * fails on this build (PICO_ERROR_NOT_PERMITTED: FreeRTOS on the RP2040
 * with pico_multicore linked, which the kernel's port links).  Core 1 is
 * never started (configNUMBER_OF_CORES 1: the port's xPortStartScheduler()
 * does not launch it), so it stays in the boot ROM, which does not fetch
 * from the flash.  FreeRTOS's own critical section (taskENTER_CRITICAL(),
 * vPortEnterCritical() on one core) is the same "cpsid i" with a nesting
 * count; the SDK's helper is used because pico_flash's helper is what the
 * SDK provides for this and what an SDK library writing the flash would
 * use too.  No DMA channel of the port reads from the flash.
 *
 * The interrupts are off for one 256-byte page program (about 0.4-0.5 ms)
 * or one 4 KiB sector erase (34-40 ms measured on the NuttX build) at a
 * time, as on NuttX (the Zephyr build's driver keeps them off for one
 * flash_write() or flash_erase() call; NVS writes one entry's data, at most
 * two pages, or one ATE per call and erases one sector per call).  What
 * that costs on this build: the 1 kHz SysTick is pending once and handled
 * when the interrupts come back, so the FreeRTOS tick (and with it the
 * compatibility layer's timeouts, sleeps and the time slices) stands still
 * for up to about 40 ticks per erase; the tick hook then counts the lost
 * ticks at once, from the TIMER (tick.c, tick_catchup.h), so the waits end
 * at the end of the erase instead of that much later.  k_uptime_*() and
 * time_us_64() come from the TIMER, which keeps counting.  A fatal error
 * while XIP is off cannot run the fault exit (fault.c is in the flash):
 * the core locks up without a record until the board is reset.
 *
 * Reads go through the uncached XIP alias (XIP_NOCACHE_NOALLOC_BASE,
 * 0x13000000), as on NuttX; the Zephyr build's driver reads the cached
 * alias, which flash_flush_cache() also keeps up to date.  Part of the
 * FreeRTOS build (freertos/drivers/kv; the NuttX build's is
 * nuttx/boards/arduino-nano-rp2040-connect/src/rp2040_flash_kv_os.c).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "FreeRTOS.h"

#include <zephyr/logging/log.h>

#include "flash_kv.h"
#include "flash_kv_os.h"
#include "flash_kv_os_rp2040.h"

LOG_MODULE_REGISTER(flash_kv, LOG_LEVEL_INF);

#if !defined(PICO_FLASH_SIZE_BYTES) || PICO_FLASH_SIZE_BYTES != FLASH_KV_FLASH_SIZE
#error "the board's flash is not the 16 MiB of the key-value area's constants"
#endif

#if FLASH_PAGE_SIZE != FLASH_KV_PAGE_SIZE || FLASH_SECTOR_SIZE != FLASH_KV_SECTOR_SIZE
#error "the SDK's flash page or sector size differs from flash_kv.h"
#endif

#if !PICO_FLASH_ASSUME_CORE1_SAFE
#error "flash_safe_execute() needs PICO_FLASH_ASSUME_CORE1_SAFE=1 on this build"
#endif

#if configNUMBER_OF_CORES != 1
#error "flash_kv_os_rp2040.c does not stop the other core while XIP is off"
#endif

/* The area as flash offsets, [KV_LO, KV_HI). */
#define KV_LO FLASH_KV_OFFSET
#define KV_HI FLASH_KV_END

/* flash_safe_execute()'s timeout for stopping the other core: not used when
 * only the interrupts are disabled (PICO_FLASH_ASSUME_CORE1_SAFE).
 */
#define KV_SAFE_TIMEOUT_MS 100

/* The SDK's symbols: the end of the image in the flash (section_flash_end.incl,
 * declared as in the SDK), and the area (ld/section_extra_post_end.incl).
 */
extern char __flash_binary_end;
extern const uint8_t __kv_start[];
extern const uint8_t __kv_end[];

/* The page being programmed: the ROM reads it while XIP is off, so it must
 * not be in the flash; the caller's page is copied here first.  The calls
 * come one at a time (flash_kv_os.h).
 */
static uint8_t kv_page[FLASH_KV_PAGE_SIZE] __aligned(4);

/* The area's bounds as flash offsets once the linker script has been
 * checked; 0 and 0 refuse everything.
 */
static uint32_t kv_lo;
static uint32_t kv_hi;

/* One program (page != NULL, count 256) or erase (page == NULL, count 4096)
 * at flash offset `offset`, run by flash_safe_execute() with the
 * interrupts off.
 */
struct kv_flash_req {
	uint32_t offset;
	const uint8_t *page;
	uint32_t count;
	int ret;
};

/* Runs with the interrupts off and XIP still on; flash_range_program() and
 * flash_range_erase() stop XIP and restart it themselves.  The last check:
 * the bounds from the linker script, and that the interrupts are off
 * (save_and_disable_interrupts() returns the PRIMASK it found; restoring it
 * leaves them off).  Refuses without touching the flash.
 */
static void kv_flash_op(void *arg)
{
	struct kv_flash_req *req = arg;
	uint32_t lo = kv_lo;
	uint32_t hi = kv_hi;
	uint32_t primask = save_and_disable_interrupts();

	restore_interrupts(primask);
	if ((primask & 1u) == 0 || lo == 0 || lo >= hi || req->offset < lo || req->offset >= hi ||
	    req->count == 0 || req->count > hi - req->offset) {
		req->ret = -EINVAL;
		return;
	}
	if (req->page != NULL) {
		flash_range_program(req->offset, req->page, req->count);
	} else {
		flash_range_erase(req->offset, req->count);
	}
	req->ret = 0;
}

/* Checks a program (page != NULL) or erase (page == NULL) request against
 * both copies of the area's bounds and runs it with the interrupts off.
 */
static int kv_flash(uint32_t offset, const uint8_t *page, uint32_t count)
{
	uint32_t align = page != NULL ? FLASH_KV_PAGE_SIZE : FLASH_KV_SECTOR_SIZE;
	struct kv_flash_req req = {
		.offset = offset,
		.page = page,
		.count = count,
		.ret = -EINVAL,
	};
	int rc;

	if (count != align || !flash_kv_range_ok(offset, count, KV_LO, KV_HI, align) ||
	    !flash_kv_range_ok(offset, count, kv_lo, kv_hi, align)) {
		LOG_ERR("Refused flash %s at 0x%08lx, %lu bytes", page != NULL ? "write" : "erase",
			(unsigned long)offset, (unsigned long)count);
		return -EINVAL;
	}
	rc = flash_safe_execute(kv_flash_op, &req, KV_SAFE_TIMEOUT_MS);
	if (rc != PICO_OK) {
		LOG_ERR("flash_safe_execute() failed: %d", rc);
		return -EIO;
	}
	if (req.ret != 0) {
		LOG_ERR("Refused flash %s at 0x%08lx with the interrupts off",
			page != NULL ? "write" : "erase", (unsigned long)offset);
	}
	return req.ret;
}

/* The boundary of common/compat/flash_kv_os.h. */

bool flash_kv_os_ready(void)
{
	return kv_lo != 0;
}

int flash_kv_os_read(uint32_t offset, void *buf, uint32_t len)
{
	if (!flash_kv_range_ok(offset, len, KV_LO, KV_HI, 1)) {
		LOG_ERR("Refused flash read at 0x%08lx, %lu bytes", (unsigned long)offset,
			(unsigned long)len);
		return -EINVAL;
	}
	memcpy(buf, (const void *)(uintptr_t)(XIP_NOCACHE_NOALLOC_BASE + offset), len);
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

/* The start (flash_kv_os_rp2040.h, flash_kv_os.h). */

int flash_kv_os_rp2040_set_bounds(uintptr_t kv_start, uintptr_t kv_end, uintptr_t image_end)
{
	if (kv_start != XIP_BASE + KV_LO || kv_end != XIP_BASE + KV_HI || image_end <= XIP_BASE ||
	    image_end > kv_start) {
		LOG_ERR("Key-value area: linker script disagrees (0x%08lx-0x%08lx, image end "
			"0x%08lx)",
			(unsigned long)kv_start, (unsigned long)kv_end, (unsigned long)image_end);
		kv_lo = 0;
		kv_hi = 0;
		return -EFAULT;
	}
	kv_lo = KV_LO;
	kv_hi = KV_HI;
	return 0;
}

int flash_kv_os_initialize(void)
{
	return flash_kv_os_rp2040_set_bounds((uintptr_t)__kv_start, (uintptr_t)__kv_end,
					     (uintptr_t)&__flash_binary_end);
}
