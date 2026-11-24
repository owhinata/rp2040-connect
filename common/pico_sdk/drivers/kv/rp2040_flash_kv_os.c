/* SPDX-License-Identifier: MIT */
/* The flash operations of the key-value area on the Pico SDK
 * (common/chip/rp2040/include/rp2040_flash_kv_os.h): one 256-byte page
 * program or one 4 KiB sector erase with XIP off and the interrupts off,
 * for the shared part, common/chip/rp2040/rp2040_flash_kv.c, which
 * implements the OS boundary of the area (common/compat/flash_kv_os.h)
 * with them and checks every request against the area and the linker
 * script's bounds first.  The last check, with the interrupts off right
 * before flash_range_program() or flash_range_erase() is called, is here
 * (kv_flash_op()).  The image never reaches the area: the linker script
 * leaves it out of the FLASH region and checks the image's end
 * (common/pico_sdk/drivers/kv/ld, kv.cmake).  The board's flash
 * (PICO_FLASH_SIZE_BYTES of the board header) and the SDK's page and
 * sector sizes are checked against the constants when this file is
 * compiled.
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
 * of them here: the kernel's tick and switch, the USB, the SIO FIFO's) would fault while
 * XIP is off.  So they are called from flash_safe_execute() of pico_flash,
 * which with PICO_FLASH_ASSUME_CORE1_SAFE=1 (kv.cmake) disables this core's
 * interrupts (save_and_disable_interrupts(), PRIMASK: every interrupt,
 * SysTick and PendSV included) and does not try to stop core 1
 * (pico_flash/flash.c, use_irq_only()).  Without that option it fails on
 * an OS that links pico_multicore (PICO_ERROR_NOT_PERMITTED; the FreeRTOS
 * kernel's RP2040 port links it).  The OS runs on core 0 only and never
 * starts core 1 (FreeRTOS: configNUMBER_OF_CORES 1, checked in its
 * FreeRTOSConfig.h), so core 1 stays in the boot ROM, which does not fetch
 * from the flash.  An OS's own critical section on one core is the same
 * "cpsid i"; the SDK's helper is used because pico_flash's helper is what
 * the SDK provides for this and what an SDK library writing the flash
 * would use too.  No DMA channel of the port reads from the flash.
 *
 * The interrupts are off for one 256-byte page program (about 0.4-0.5 ms)
 * or one 4 KiB sector erase (34-40 ms measured on the NuttX build) at a
 * time, as on NuttX (the Zephyr build's driver keeps them off for one
 * flash_write() or flash_erase() call; NVS writes one entry's data, at most
 * two pages, or one ATE per call and erases one sector per call).  What
 * that costs with a periodic 1 kHz tick (SysTick): it is pending once and
 * handled when the interrupts come back, so the kernel's tick (and with it
 * the compatibility layer's timeouts, sleeps and the time slices) stands
 * still for up to about 40 ticks per erase; the OS's tick hook then counts
 * the lost ticks at once, from the TIMER (tick_catchup.h of
 * common/compat/kernel_generic), so the waits end at the end of the erase
 * instead of that much later.  k_uptime_*() and
 * time_us_64() come from the TIMER, which keeps counting.  A fatal error
 * while XIP is off cannot run the fault exit (fault.c is in the flash):
 * the core locks up without a record until the board is reset.
 *
 * Shared by the builds on the Pico SDK (common/pico_sdk/drivers/kv; the
 * NuttX build's is
 * nuttx/boards/arduino-nano-rp2040-connect/src/rp2040_flash_kv_os.c).
 */

#include <errno.h>
#include <stdint.h>

#include "pico.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"

#include "app_log.h"
#include "flash_kv.h"
#include "rp2040_flash_kv_os.h"

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

/* flash_safe_execute()'s timeout for stopping the other core: not used when
 * only the interrupts are disabled (PICO_FLASH_ASSUME_CORE1_SAFE).
 */
#define KV_SAFE_TIMEOUT_MS 100

/* The SDK's end of the image in the flash (section_flash_end.incl, declared
 * as in the SDK).
 */
extern char __flash_binary_end;

/* One program (page != NULL, count 256) or erase (page == NULL, count 4096)
 * at flash offset `offset`, to lie inside [lo, hi), run by
 * flash_safe_execute() with the interrupts off.
 */
struct kv_flash_req {
	uint32_t offset;
	const uint8_t *page;
	uint32_t count;
	uint32_t lo;
	uint32_t hi;
	int ret;
};

/* Runs with the interrupts off and XIP still on; flash_range_program() and
 * flash_range_erase() stop XIP and restart it themselves.  The last check:
 * the bounds, and that the interrupts are off (save_and_disable_interrupts()
 * returns the PRIMASK it found; restoring it leaves them off).  Refuses
 * without touching the flash.
 */
static void kv_flash_op(void *arg)
{
	struct kv_flash_req *req = arg;
	uint32_t lo = req->lo;
	uint32_t hi = req->hi;
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

/* Runs a program (page != NULL) or erase (page == NULL) request with the
 * interrupts off.
 */
static int kv_flash(uint32_t offset, const uint8_t *page, uint32_t count, uint32_t lo, uint32_t hi)
{
	struct kv_flash_req req = {
		.offset = offset,
		.page = page,
		.count = count,
		.lo = lo,
		.hi = hi,
		.ret = -EINVAL,
	};
	int rc;

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

/* The flash operations of rp2040_flash_kv_os.h. */

int rp2040_flash_kv_os_init(void)
{
	/* hardware_flash looks the boot ROM's functions up and copies boot2
	 * into RAM itself, at each operation.
	 */
	return 0;
}

uintptr_t rp2040_flash_kv_os_image_end(void)
{
	return (uintptr_t)&__flash_binary_end;
}

int rp2040_flash_kv_os_program(uint32_t page_offset, const uint8_t *page, uint32_t lo, uint32_t hi)
{
	return kv_flash(page_offset, page, FLASH_KV_PAGE_SIZE, lo, hi);
}

int rp2040_flash_kv_os_erase(uint32_t sector_offset, uint32_t lo, uint32_t hi)
{
	return kv_flash(sector_offset, NULL, FLASH_KV_SECTOR_SIZE, lo, hi);
}
