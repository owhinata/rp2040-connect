/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's key-value store on the FreeRTOS
 * POSIX port: the flash boundary of the board (freertos/drivers/kv/
 * nano_flash_kv.c) on fakes of the Pico SDK's flash functions
 * (pico_fake/, implemented here on a RAM copy of the 8 KiB area), under the
 * shared store as the board builds it (common/imu/kv_store.c,
 * common/compat/kv_store_wipe.c, zephyr_flash_kv.c, the copies of Zephyr's
 * settings and NVS) and the FreeRTOS part of the compatibility layer
 * (k_mutex: the settings' K_MUTEX_DEFINE(settings_lock), all zero; NVS's
 * lock; the recursive lock of "kv wipe").
 *
 * - Before the bounds are set, and with wrong linker bounds: not ready,
 *   nothing erased or programmed.
 * - The boundary's refusals: below, above and across the area, unaligned,
 *   a failing flash_safe_execute(), and interrupts that are not off when
 *   the operation runs; none reaches the flash.  Every operation that does
 *   runs with the interrupts off, inside the area and aligned (the fakes
 *   abort otherwise).
 * - A page program copies the caller's page (the fake checks that the SDK
 *   gets a different buffer) and only clears bits; reads see the flash.
 * - Data that is neither Zephyr's NVS nor erased: the store fails with
 *   -EILSEQ, the area is not touched, the log says so; "kv wipe" checks
 *   its argument, then erases and the store works.
 * - Set, get, delete, mounting again, garbage collection (many writes),
 *   two tasks writing at once, and "kv wipe" of a full store.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"

#include "kv_store.h"
#include "kv_store_os.h"
#include "nano_flash_kv.h"
#include "nano_flash_kv_os.h"
#include "priorities.h"
#include "settings_priv.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 16384
#define WRITER_STACK 16384
#define WRITES       60
#define LOG_SIZE     16384

volatile int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* settings_init.c */
extern bool settings_subsys_initialized;

/* The linker's symbols, which nano_flash_kv_initialize() would read; the
 * test calls nano_flash_kv_set_bounds() with its own values instead.
 */
const uint8_t __kv_start[1];
const uint8_t __kv_end[1];
char __flash_binary_end;

#define KV_XIP_START (0x10000000u + NANO_KV_OFFSET)
#define KV_XIP_END   (0x10000000u + NANO_KV_END)

/* The fakes of the Pico SDK (pico_fake/) ---------------------------------*/

uint8_t kv_test_flash[NANO_KV_SIZE];

static uint32_t primask;
static unsigned long programs, erases;
static const uint8_t *last_caller_page;

/* flash_safe_execute(): 0 runs the function with the interrupts off (the
 * SDK's with PICO_FLASH_ASSUME_CORE1_SAFE=1), 1 fails, 2 runs it with the
 * interrupts on.
 */
static int safe_mode;

uint32_t save_and_disable_interrupts(void)
{
	uint32_t was = primask;

	primask = 1;
	return was;
}

void restore_interrupts(uint32_t status)
{
	primask = status;
}

int flash_safe_execute(void (*func)(void *), void *param, uint32_t enter_exit_timeout_ms)
{
	uint32_t was;

	(void)enter_exit_timeout_ms;
	if (safe_mode == 1) {
		return PICO_ERROR_NOT_PERMITTED;
	}
	if (safe_mode == 2) {
		func(param);
		return PICO_OK;
	}
	was = save_and_disable_interrupts();
	func(param);
	restore_interrupts(was);
	return PICO_OK;
}

static void check_op(uint32_t offs, size_t count, uint32_t align)
{
	assert(primask == 1);
	assert(offs >= NANO_KV_OFFSET && offs < NANO_KV_END);
	assert(count == align && offs % align == 0);
	assert(count <= NANO_KV_END - offs);
}

void flash_range_erase(uint32_t flash_offs, size_t count)
{
	check_op(flash_offs, count, FLASH_SECTOR_SIZE);
	memset(&kv_test_flash[flash_offs - NANO_KV_OFFSET], 0xff, count);
	erases++;
}

void flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count)
{
	check_op(flash_offs, count, FLASH_PAGE_SIZE);
	assert(data != last_caller_page);
	for (size_t i = 0; i < count; i++) {
		kv_test_flash[flash_offs - NANO_KV_OFFSET + i] &= data[i];
	}
	programs++;
}

/* The log and the shell's output ------------------------------------------*/

static char log_buf[LOG_SIZE];
static char out_buf[1024];

static void append(char *buf, size_t size, const char *fmt, va_list ap)
{
	size_t n = strlen(buf);

	vsnprintf(buf + n, size - n, fmt, ap);
}

static void append_text(char *buf, size_t size, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	append(buf, size, fmt, ap);
	va_end(ap);
}

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	static const char *const names[] = {"", "err", "wrn", "inf", "dbg"};

	(void)func;
	append_text(log_buf, sizeof(log_buf), "<%s> %s: ", names[level], module);
	append(log_buf, sizeof(log_buf), fmt, ap);
	append_text(log_buf, sizeof(log_buf), "\n");
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

void z_shell_backend_vfprintf(const struct shell *sh, enum shell_vt100_color color, const char *fmt,
			      va_list args)
{
	(void)sh;
	(void)color;
	append(out_buf, sizeof(out_buf), fmt, args);
}

/* The tests ----------------------------------------------------------------*/

static bool all_erased(void)
{
	for (size_t i = 0; i < sizeof(kv_test_flash); i++) {
		if (kv_test_flash[i] != 0xff) {
			return false;
		}
	}
	return true;
}

static bool flash_has(const char *text)
{
	return memmem(kv_test_flash, sizeof(kv_test_flash), text, strlen(text)) != NULL;
}

static void expect_value(const char *key, const char *want)
{
	char value[KV_STORE_VALUE_MAX];
	ssize_t len = kv_store_get(key, value, sizeof(value));

	assert(len == (ssize_t)strlen(want));
	assert(memcmp(value, want, (size_t)len) == 0);
}

/* The boundary alone: refusals before the bounds and outside the area. */
static void test_boundary(void)
{
	uint8_t page[NANO_KV_PAGE_SIZE], buf[16];

	memset(page, 0x00, sizeof(page));
	last_caller_page = page;

	/* Not ready: no erase or program (refused by the check before the
	 * interrupts go off, which the log tells apart from the last one), the
	 * store sees no device.
	 */
	assert(!nano_flash_kv_os_ready());
	assert(nano_flash_kv_os_program(NANO_KV_OFFSET, page) == -EINVAL);
	assert(nano_flash_kv_os_erase(NANO_KV_OFFSET) == -EINVAL);
	assert(strstr(log_buf, "Refused flash write at 0x00ffe000, 256 bytes") != NULL);
	assert(strstr(log_buf, "Refused flash erase at 0x00ffe000, 4096 bytes") != NULL);
	assert(strstr(log_buf, "with the interrupts off") == NULL);
	assert(kv_store_init() == -ENODEV);

	/* Wrong linker bounds. */
	assert(nano_flash_kv_set_bounds(KV_XIP_START - 0x1000, KV_XIP_END, 0x10020000) == -EFAULT);
	assert(nano_flash_kv_set_bounds(KV_XIP_START, KV_XIP_END - 1, 0x10020000) == -EFAULT);
	assert(nano_flash_kv_set_bounds(KV_XIP_START, KV_XIP_END, KV_XIP_START + 1) == -EFAULT);
	assert(nano_flash_kv_set_bounds(KV_XIP_START, KV_XIP_END, 0x10000000) == -EFAULT);
	assert(!nano_flash_kv_os_ready());
	assert(nano_flash_kv_os_erase(NANO_KV_OFFSET) == -EINVAL);
	assert(strstr(log_buf, "linker script disagrees") != NULL);

	/* Right ones: the image may end right at the area. */
	assert(nano_flash_kv_set_bounds(KV_XIP_START, KV_XIP_END, KV_XIP_START) == 0);
	assert(nano_flash_kv_os_ready());

	/* Outside, across and unaligned. */
	assert(nano_flash_kv_os_program(NANO_KV_OFFSET - NANO_KV_PAGE_SIZE, page) == -EINVAL);
	assert(nano_flash_kv_os_program(NANO_KV_END, page) == -EINVAL);
	assert(nano_flash_kv_os_program(NANO_KV_OFFSET + 1, page) == -EINVAL);
	assert(nano_flash_kv_os_program(0, page) == -EINVAL);
	assert(nano_flash_kv_os_program(UINT32_MAX - 0xff, page) == -EINVAL);
	assert(nano_flash_kv_os_erase(NANO_KV_OFFSET - NANO_KV_SECTOR_SIZE) == -EINVAL);
	assert(nano_flash_kv_os_erase(NANO_KV_END) == -EINVAL);
	assert(nano_flash_kv_os_erase(NANO_KV_OFFSET + NANO_KV_PAGE_SIZE) == -EINVAL);
	assert(nano_flash_kv_os_erase(0) == -EINVAL);
	assert(nano_flash_kv_os_read(NANO_KV_OFFSET - 1, buf, 2) == -EINVAL);
	assert(nano_flash_kv_os_read(NANO_KV_END - 1, buf, 2) == -EINVAL);
	assert(nano_flash_kv_os_read(NANO_KV_OFFSET, buf, 0) == -EINVAL);

	/* flash_safe_execute() fails, or leaves the interrupts on. */
	safe_mode = 1;
	assert(nano_flash_kv_os_erase(NANO_KV_OFFSET) == -EIO);
	safe_mode = 2;
	assert(nano_flash_kv_os_erase(NANO_KV_OFFSET) == -EINVAL);
	assert(nano_flash_kv_os_program(NANO_KV_OFFSET, page) == -EINVAL);
	assert(strstr(log_buf, "with the interrupts off") != NULL);
	safe_mode = 0;
	assert(programs == 0 && erases == 0 && all_erased());

	/* A page in the last sector: only clears bits, read back as written. */
	memset(page, 0xff, sizeof(page));
	page[0] = 0x5a;
	page[255] = 0x0f;
	assert(nano_flash_kv_os_program(NANO_KV_END - NANO_KV_PAGE_SIZE, page) == 0);
	page[0] = 0xa5;
	page[255] = 0xf0;
	assert(nano_flash_kv_os_program(NANO_KV_END - NANO_KV_PAGE_SIZE, page) == 0);
	assert(nano_flash_kv_os_read(NANO_KV_END - NANO_KV_PAGE_SIZE, buf, 1) == 0);
	assert(buf[0] == 0x00);
	assert(nano_flash_kv_os_read(NANO_KV_END - 1, buf, 1) == 0);
	assert(buf[0] == 0x00);
	assert(programs == 2);
	assert(nano_flash_kv_os_erase(NANO_KV_END - NANO_KV_SECTOR_SIZE) == 0);
	assert(erases == 1 && all_erased());
	assert(primask == 0);
	last_caller_page = NULL;
}

/* Data that is not Zephyr's NVS, then "kv wipe". */
static void test_foreign(void)
{
	static const struct shell sh;
	char *no[] = {"wipe", "no", NULL};
	char *confirm[] = {"wipe", "confirm", NULL};
	unsigned long before, erased;
	char value[8];

	memcpy(&kv_test_flash[100], "FAT16   ", 8);
	before = programs + erases;
	log_buf[0] = '\0';
	assert(kv_store_init() == -EILSEQ);
	assert(kv_store_get("imu.port", value, sizeof(value)) == -EILSEQ);
	assert(kv_store_set("imu.port", "18830", 5) == -EILSEQ);
	assert(programs + erases == before);
	assert(memcmp(&kv_test_flash[100], "FAT16   ", 8) == 0);
	assert(strstr(log_buf, "holds data that is not Zephyr's NVS") != NULL);
	assert(strstr(log_buf, "\"kv wipe confirm\" erases it") != NULL);

	out_buf[0] = '\0';
	assert(kv_store_cmd_wipe(&sh, 2, no) == -EINVAL);
	assert(strstr(out_buf, "Type \"kv wipe confirm\"") != NULL);
	assert(programs + erases == before);

	erased = erases;
	out_buf[0] = '\0';
	assert(kv_store_cmd_wipe(&sh, 2, confirm) == 0);
	assert(strcmp(out_buf, "Erased\n") == 0);
	assert(erases >= erased + NANO_KV_SECTORS);
	assert(kv_store_init() == 0);
	assert(strstr(log_buf, "<inf> fs_nvs: 2 Sectors of 4096 bytes") != NULL);
}

static void test_store(void)
{
	char value[KV_STORE_VALUE_MAX];

	assert(kv_store_set("imu.port", "18830", 5) == 0);
	assert(kv_store_set("imu.cal.gyro.70000", "\x01\x02\x03", 3) == 0);
	expect_value("imu.port", "18830");
	expect_value("imu.cal.gyro.70000", "\x01\x02\x03");
	assert(flash_has("app_kv/imu.port"));
	assert(flash_has("18830"));
	assert(kv_store_get("absent", value, sizeof(value)) == -ENOENT);
	assert(kv_store_delete("imu.cal.gyro.70000") == 0);
	assert(kv_store_get("imu.cal.gyro.70000", value, sizeof(value)) == -ENOENT);

	/* Mount again on what is there (the settings' state as after a
	 * start), as "kv wipe" does after its erase.
	 */
	settings_lock_take();
	settings_subsys_initialized = false;
	assert(settings_subsys_init() == 0);
	settings_lock_release();
	expect_value("imu.port", "18830");
	assert(kv_store_get("imu.cal.gyro.70000", value, sizeof(value)) == -ENOENT);
}

/* Two tasks write their own keys at once (each set goes through the
 * settings' lock and NVS's); with the values of 100 bytes the sectors fill
 * and garbage collection erases them over and over.
 */
static SemaphoreHandle_t writers_done;

static void writer(void *arg)
{
	const char *key = arg;
	char value[100], got[KV_STORE_VALUE_MAX];

	for (int i = 0; i < WRITES; i++) {
		memset(value, 'a' + i % 26, sizeof(value));
		snprintf(value, sizeof(value), "%s %d", key, i);
		value[strlen(value)] = '.';
		assert(kv_store_set(key, value, sizeof(value)) == 0);
		assert(kv_store_get(key, got, sizeof(got)) == (ssize_t)sizeof(value));
		assert(memcmp(got, value, sizeof(value)) == 0);
		vTaskDelay(1);
	}
	xSemaphoreGive(writers_done);
	vTaskDelete(NULL);
}

static void test_writers(void)
{
	unsigned long before = erases;

	writers_done = xSemaphoreCreateCounting(2, 0);
	assert(xTaskCreate(writer, "writer a", WRITER_STACK / sizeof(StackType_t), "w.a",
			   NANO_PRIO_SHELL, NULL) == pdPASS);
	assert(xTaskCreate(writer, "writer b", WRITER_STACK / sizeof(StackType_t), "w.b",
			   NANO_PRIO_SHELL, NULL) == pdPASS);
	for (int i = 0; i < 2; i++) {
		assert(xSemaphoreTake(writers_done, portMAX_DELAY) == pdTRUE);
	}
	assert(erases > before);
	expect_value("imu.port", "18830");
}

static void test_wipe(void)
{
	char value[8];

	/* NVS mounts again on the erased area and writes its first entry (a
	 * garbage collection marker), but no key is left.
	 */
	assert(kv_store_wipe() == 0);
	assert(!flash_has("app_kv/") && !flash_has("18830") && !flash_has("w.a 59"));
	assert(kv_store_get("imu.port", value, sizeof(value)) == -ENOENT);
	assert(kv_store_get("w.a", value, sizeof(value)) == -ENOENT);
	assert(kv_store_set("imu.port", "18830", 5) == 0);
	expect_value("imu.port", "18830");
	assert(primask == 0);
}

static void runner(void *arg)
{
	(void)arg;
	test_boundary();
	test_foreign();
	test_store();
	test_writers();
	test_wipe();
	printf("kv: %lu programs, %lu erases\n", programs, erases);
	fflush(stdout);
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	memset(kv_test_flash, 0xff, sizeof(kv_test_flash));
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_MAIN,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
