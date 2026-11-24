/* SPDX-License-Identifier: MIT */
/* chip/rp2040/rp2040_flash_kv.c, the OS boundary of the key-value area
 * (compat/port/flash_kv_os.h) that the NuttX and FreeRTOS builds share, on a
 * fake of the build's flash operations (rp2040_flash_kv_os.h, defined
 * here): the fake records every call, checks that it is a page or sector
 * of the area with the area's bounds and the shared part's RAM copy of the
 * page, and applies it to 8 KiB of memory mapped at the area's uncached XIP
 * address (0x13ffe000), where the shared part reads.
 *
 *  - Before the bounds are set, and after every failed start: not ready,
 *    every program and erase refused before the build is called.
 *  - The start: the linker script's symbols (the test's own, which are not
 *    the area: -EFAULT), every bound off by one at either side, the image's
 *    end at the limits, the build's start failing, a failed start after a
 *    good one.
 *  - Every program and erase offset from 8 KiB below to 8 KiB above the
 *    area: accepted exactly on the area's pages and sectors, refused
 *    without a call otherwise; offsets near 0 and near 2^32.
 *  - Reads at both ends of the area, across them and empty, and their
 *    data; the build's errors returned as they are; the messages.
 */
#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "flash_kv.h"
#include "flash_kv_os.h"
#include "rp2040_flash_kv.h"
#include "rp2040_flash_kv_os.h"

#define LO      FLASH_KV_OFFSET
#define HI      FLASH_KV_END
#define XIP     RP2040_FLASH_KV_XIP_BASE
#define PAGE    FLASH_KV_PAGE_SIZE
#define SECTOR  FLASH_KV_SECTOR_SIZE
#define LOG_LEN 8192

_Static_assert(LO == 0x00ffe000u && HI == 0x01000000u, "the area");
_Static_assert(XIP == 0x10000000u && RP2040_FLASH_KV_XIP_NOCACHE_BASE == 0x13000000u, "XIP");

/* The linker script's symbols that flash_kv_os_initialize() reads: the
 * test's own, at addresses that are not the area's.
 */
const uint8_t __kv_start[1];
const uint8_t __kv_end[1];

/* The area as the uncached XIP alias shows it. */
static uint8_t *flash;

/* The fake build ------------------------------------------------------------*/

static int init_ret;
static unsigned long inits, programs, erases;
static uintptr_t image_end = XIP + 0x40000;
static int op_ret;
static const uint8_t *caller_page;
static const uint8_t *last_page;

int rp2040_flash_kv_os_init(void)
{
	inits++;
	return init_ret;
}

uintptr_t rp2040_flash_kv_os_image_end(void)
{
	return image_end;
}

/* What the shared part may hand the build: inside the area, aligned, with
 * the area's bounds.
 */
static void check_op(uint32_t offset, uint32_t size, uint32_t lo, uint32_t hi)
{
	assert(lo == LO && hi == HI);
	assert(offset >= LO && offset < HI && offset % size == 0 && size <= HI - offset);
}

int rp2040_flash_kv_os_program(uint32_t page_offset, const uint8_t *page, uint32_t lo, uint32_t hi)
{
	check_op(page_offset, PAGE, lo, hi);
	/* The shared part's copy, always the same buffer, with the bytes. */
	assert(page != caller_page);
	assert(last_page == NULL || page == last_page);
	assert(caller_page == NULL || memcmp(page, caller_page, PAGE) == 0);
	last_page = page;
	programs++;
	if (op_ret != 0) {
		return op_ret;
	}
	for (uint32_t i = 0; i < PAGE; i++) {
		flash[page_offset - LO + i] &= page[i];
	}
	return 0;
}

int rp2040_flash_kv_os_erase(uint32_t sector_offset, uint32_t lo, uint32_t hi)
{
	check_op(sector_offset, SECTOR, lo, hi);
	erases++;
	if (op_ret != 0) {
		return op_ret;
	}
	memset(&flash[sector_offset - LO], 0xff, SECTOR);
	return 0;
}

/* The log (compat/include/app_log.h) ---------------------------------------------*/

/* What was logged since log_clear(); what does not fit is dropped. */
static char log_buf[LOG_LEN];
static size_t log_len;

static void log_clear(void)
{
	log_buf[0] = '\0';
	log_len = 0;
}

static void log_add(const char *fmt, ...)
{
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(log_buf + log_len, sizeof(log_buf) - log_len, fmt, ap);
	va_end(ap);
	if (n > 0 && (size_t)n < sizeof(log_buf) - log_len) {
		log_len += (size_t)n;
	} else {
		log_buf[log_len] = '\0';
	}
}

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	char line[160];

	(void)func;
	assert(strcmp(module, "flash_kv") == 0);
	vsnprintf(line, sizeof(line), fmt, ap);
	log_add("<%d> %s\n", level, line);
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	app_log_vprintf(level, module, func, fmt, ap);
	va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	(void)last_ms;
	(void)skipped;
	(void)rate_ms;
	*skipped_out = 0;
	return true;
}

static bool logged(const char *text)
{
	return strstr(log_buf, text) != NULL;
}

/* The tests ------------------------------------------------------------------*/

static uint8_t test_page[PAGE];

/* No program and no erase reaches the build. */
static void expect_refused(void)
{
	unsigned long before = programs + erases;

	assert(flash_kv_os_program(LO, test_page) == -EINVAL);
	assert(flash_kv_os_program(HI - PAGE, test_page) == -EINVAL);
	assert(flash_kv_os_erase(LO) == -EINVAL);
	assert(flash_kv_os_erase(HI - SECTOR) == -EINVAL);
	assert(programs + erases == before);
}

static void test_not_ready(void)
{
	assert(!flash_kv_os_ready());
	expect_refused();
	assert(logged("Refused flash write at 0x00ffe000, 256 bytes"));
	assert(logged("Refused flash erase at 0x00ffe000, 4096 bytes"));
	assert(logged("Refused flash write at 0x00ffff00, 256 bytes"));
	assert(logged("Refused flash erase at 0x00fff000, 4096 bytes"));

	/* The test's linker symbols are not the area. */
	log_clear();
	assert(flash_kv_os_initialize() == -EFAULT);
	assert(logged("linker script disagrees"));
	assert(inits == 0 && !flash_kv_os_ready());
	expect_refused();
}

static void test_bounds(void)
{
	const uintptr_t start = XIP + LO;
	const uintptr_t end = XIP + HI;
	const uintptr_t img = XIP + 0x40000;
	static const struct {
		uintptr_t kv_start, kv_end, image_end;
	} bad[] = {
		{XIP + LO - 1, XIP + HI, XIP + 0x40000},
		{XIP + LO + 1, XIP + HI, XIP + 0x40000},
		{XIP + LO - SECTOR, XIP + HI, XIP + 0x40000},
		{XIP + LO + SECTOR, XIP + HI, XIP + 0x40000},
		{XIP + LO, XIP + HI - 1, XIP + 0x40000},
		{XIP + LO, XIP + HI + 1, XIP + 0x40000},
		{XIP + LO, XIP + HI - SECTOR, XIP + 0x40000},
		{XIP + LO, XIP + HI, XIP + LO + 1}, /* the image reaches the area */
		{XIP + LO, XIP + HI, XIP + HI},
		{XIP + LO, XIP + HI, XIP},     /* no image */
		{XIP + LO, XIP + HI, XIP - 1}, /* not in the flash */
		{XIP + LO, XIP + HI, 0},
		{0, 0, 0},
		{LO, HI, 0x40000}, /* flash offsets instead of XIP addresses */
		{XIP + LO, XIP + LO, XIP + 0x40000},
		{XIP + HI, XIP + LO, XIP + 0x40000},
	};

	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		log_clear();
		assert(rp2040_flash_kv_set_bounds(bad[i].kv_start, bad[i].kv_end,
						  bad[i].image_end) == -EFAULT);
		assert(logged("Key-value area: linker script disagrees"));
		assert(inits == 0 && !flash_kv_os_ready());
		expect_refused();
	}

	/* The build's start fails: not ready, its error returned. */
	init_ret = -ENOSYS;
	assert(rp2040_flash_kv_set_bounds(start, end, img) == -ENOSYS);
	assert(inits == 1 && !flash_kv_os_ready());
	expect_refused();
	init_ret = 0;

	/* The image may end anywhere in the flash up to the area. */
	assert(rp2040_flash_kv_set_bounds(start, end, XIP + 1) == 0);
	assert(rp2040_flash_kv_set_bounds(start, end, start) == 0);
	assert(rp2040_flash_kv_set_bounds(start, end, img) == 0);
	assert(inits == 4 && flash_kv_os_ready());

	/* A failed start after a good one refuses everything again. */
	assert(rp2040_flash_kv_set_bounds(start, end, start + 1) == -EFAULT);
	assert(!flash_kv_os_ready());
	expect_refused();
	init_ret = -EIO;
	assert(rp2040_flash_kv_set_bounds(start, end, img) == -EIO);
	assert(!flash_kv_os_ready());
	expect_refused();
	init_ret = 0;
	assert(rp2040_flash_kv_set_bounds(start, end, img) == 0);
	assert(flash_kv_os_ready());
}

/* Every offset from 8 KiB below the area to 8 KiB above it, and offsets
 * near 0 and 2^32: accepted exactly on the area's pages and sectors, each
 * reaching the build once.
 */
static void test_ranges(void)
{
	static const uint32_t far[] = {0,
				       1,
				       PAGE,
				       SECTOR,
				       LO - 1,
				       0x00f00000u,
				       0x01001000u,
				       0x10ffe000u,
				       0x13ffe000u,
				       UINT32_MAX - SECTOR + 1,
				       UINT32_MAX - PAGE + 1,
				       UINT32_MAX};

	caller_page = test_page;
	memset(test_page, 0xff, sizeof(test_page));
	for (uint32_t a = LO - FLASH_KV_SIZE; a < HI + FLASH_KV_SIZE; a++) {
		bool page_ok = a >= LO && a < HI && a % PAGE == 0;
		bool sector_ok = a >= LO && a < HI && a % SECTOR == 0;
		unsigned long p = programs, e = erases;

		assert(flash_kv_os_program(a, test_page) == (page_ok ? 0 : -EINVAL));
		assert(programs == p + (page_ok ? 1 : 0));
		assert(flash_kv_os_erase(a) == (sector_ok ? 0 : -EINVAL));
		assert(erases == e + (sector_ok ? 1 : 0));
	}
	assert(programs == FLASH_KV_SIZE / PAGE && erases == FLASH_KV_SECTORS);
	log_clear();
	for (size_t i = 0; i < sizeof(far) / sizeof(far[0]); i++) {
		unsigned long before = programs + erases;

		assert(flash_kv_os_program(far[i], test_page) == -EINVAL);
		assert(flash_kv_os_erase(far[i]) == -EINVAL);
		assert(programs + erases == before);
	}
	assert(logged("Refused flash write at 0x00000000, 256 bytes"));
	assert(logged("Refused flash write at 0xffffff00, 256 bytes"));
	assert(logged("Refused flash erase at 0x01001000, 4096 bytes"));
	assert(logged("Refused flash erase at 0xfffff000, 4096 bytes"));
	log_clear();
	assert(flash_kv_os_program(LO - PAGE, test_page) == -EINVAL);
	assert(flash_kv_os_program(HI, test_page) == -EINVAL);
	assert(flash_kv_os_program(LO + 1, test_page) == -EINVAL);
	assert(flash_kv_os_erase(LO - SECTOR) == -EINVAL);
	assert(flash_kv_os_erase(HI) == -EINVAL);
	assert(flash_kv_os_erase(LO + PAGE) == -EINVAL);
	assert(logged("<1> Refused flash write at 0x00ffdf00, 256 bytes\n"));
	assert(logged("<1> Refused flash write at 0x01000000, 256 bytes\n"));
	assert(logged("<1> Refused flash write at 0x00ffe001, 256 bytes\n"));
	assert(logged("<1> Refused flash erase at 0x00ffd000, 4096 bytes\n"));
	assert(logged("<1> Refused flash erase at 0x01000000, 4096 bytes\n"));
	assert(logged("<1> Refused flash erase at 0x00ffe100, 4096 bytes\n"));
}

/* The page goes through the shared part's copy; programming clears bits;
 * reads see the flash and are refused outside the area.
 */
static void test_data(void)
{
	static const struct {
		uint32_t offset, len;
		bool ok;
	} reads[] = {
		{LO, 1, true},
		{LO, FLASH_KV_SIZE, true},
		{LO + 1, FLASH_KV_SIZE - 1, true},
		{HI - 1, 1, true},
		{LO - 1, 1, false},
		{LO - 1, 2, false},
		{HI - 1, 2, false},
		{HI, 1, false},
		{LO, FLASH_KV_SIZE + 1, false},
		{LO + 1, FLASH_KV_SIZE, false},
		{LO, 0, false},
		{0, 1, false},
		{LO, UINT32_MAX, false},
		{UINT32_MAX, 1, false},
	};
	static uint8_t buf[FLASH_KV_SIZE + 1];
	uint8_t mine[PAGE];

	log_clear();
	assert(flash_kv_os_erase(LO) == 0 && flash_kv_os_erase(LO + SECTOR) == 0);
	for (uint32_t i = 0; i < PAGE; i++) {
		mine[i] = (uint8_t)(i * 7 + 1);
	}
	caller_page = mine;
	assert(flash_kv_os_program(HI - PAGE, mine) == 0);
	assert(memcmp(&flash[FLASH_KV_SIZE - PAGE], mine, PAGE) == 0);
	mine[0] = 0xf0;
	assert(flash_kv_os_program(HI - PAGE, mine) == 0);
	assert(flash[FLASH_KV_SIZE - PAGE] == (0x01 & 0xf0));

	for (size_t i = 0; i < sizeof(reads) / sizeof(reads[0]); i++) {
		memset(buf, 0x5a, sizeof(buf));
		if (!reads[i].ok) {
			assert(flash_kv_os_read(reads[i].offset, buf, reads[i].len) == -EINVAL);
			assert(buf[0] == 0x5a);
			continue;
		}
		assert(flash_kv_os_read(reads[i].offset, buf, reads[i].len) == 0);
		assert(memcmp(buf, &flash[reads[i].offset - LO], reads[i].len) == 0);
		assert(buf[reads[i].len] == 0x5a);
	}
	assert(logged("Refused flash read at 0x00ffdfff, 1 bytes"));
	assert(logged("Refused flash read at 0x00ffffff, 2 bytes"));

	/* The build's errors, as they are. */
	op_ret = -EIO;
	assert(flash_kv_os_program(LO, mine) == -EIO);
	assert(flash_kv_os_erase(LO) == -EIO);
	op_ret = 0;
}

int main(void)
{
	void *area = mmap((void *)(uintptr_t)(RP2040_FLASH_KV_XIP_NOCACHE_BASE + LO), FLASH_KV_SIZE,
			  PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE,
			  -1, 0);

	if (area != (void *)(uintptr_t)(RP2040_FLASH_KV_XIP_NOCACHE_BASE + LO)) {
		fprintf(stderr, "rp2040_flash_kv: cannot map the area at 0x%08x\n",
			RP2040_FLASH_KV_XIP_NOCACHE_BASE + LO);
		return 1;
	}
	flash = area;
	memset(flash, 0xff, FLASH_KV_SIZE);

	test_not_ready();
	test_bounds();
	test_ranges();
	test_data();
	printf("rp2040_flash_kv: %lu programs, %lu erases\n", programs, erases);
	return 0;
}
