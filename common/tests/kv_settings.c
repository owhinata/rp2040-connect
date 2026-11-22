/* SPDX-License-Identifier: MIT */
/* Host test of the key-value store of the builds on the compatibility
 * layer: common/imu/kv_store.c (the Zephyr build's) on the copies of
 * Zephyr's settings and NVS (common/compat/subsys), the flash device and
 * flash map of the key-value area (common/compat/zephyr_flash_kv.c) and
 * "kv wipe" (common/compat/kv_store_wipe.c), on a fake flash
 * (kv_flash_fake.c: 8 KiB, AND programs, sector erases, power loss).
 * Each boot runs in a child process, so the static state of the settings
 * and NVS starts fresh while the flash is kept.
 *
 *  - the geometry and the bytes on the flash, against Zephyr's NVS format
 *    computed here independently (CRC-8 bit by bit),
 *  - reads, writes, deletes, the key and value limits of kv_store.h,
 *    remounts, garbage collection, a full store,
 *  - data that is neither Zephyr's NVS nor erased: not used, not
 *    touched, reported; "kv wipe" erases it,
 *  - the flash device's refusals outside the area,
 *  - a power loss at every program and erase of a sequence of writes that
 *    goes through garbage collection: after the next boot every key holds
 *    its value from before or after the write in flight.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>

#include "kv_flash_fake.h"
#include "kv_store.h"
#include "kv_store_os.h"
#include "nvs_priv.h"

extern bool settings_subsys_initialized;

/* The log (common/compat/log/app_log.h): one line "<lvl> module: text" each. */

static char log_buf[8192];

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt,
		     va_list ap)
{
	static const char *const names[] = {"", "err", "wrn", "inf", "dbg"};
	size_t n = strlen(log_buf);

	(void)func;
	n += snprintf(log_buf + n, sizeof(log_buf) - n, "<%s> %s: ", names[level], module);
	n += vsnprintf(log_buf + n, sizeof(log_buf) - n, fmt, ap);
	snprintf(log_buf + n, sizeof(log_buf) - n, "\n");
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

/* kv_store.c's z_shell_main_kv() (the commands are tested by
 * nuttx/tests/kv_cmd.c on the shell).
 */
int z_shell_cmd_main(const struct shell_static_entry *root, int argc, char **argv)
{
	(void)root;
	(void)argc;
	(void)argv;
	abort();
}

/* Boots ------------------------------------------------------------------*/

/* Run FN in a child process: its exit status (0: passed). */
static int boot(void (*fn)(void))
{
	pid_t pid;
	int status;

	fflush(stdout);
	fflush(stderr);
	pid = fork();
	assert(pid >= 0);
	if (pid == 0) {
		log_buf[0] = '\0';
		fn();
		_exit(0);
	}
	assert(waitpid(pid, &status, 0) == pid);
	return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
}

#define BOOT(fn) assert(boot(fn) == 0)

static uint8_t *area(uint32_t offset)
{
	return &kv_fake->flash[offset];
}

/* Zephyr's NVS format, computed here -------------------------------------*/

/* CRC-8, polynomial 0x07, MSB first: crc8_ccitt() of Zephyr's sys/crc.h. */
static uint8_t ref_crc8(uint8_t crc, const uint8_t *p, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		crc ^= p[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

/* The 8 bytes of an ATE: id, offset, len (little-endian), part 0xff, CRC. */
static void ref_ate(uint8_t out[8], uint16_t id, uint16_t offset, uint16_t len)
{
	out[0] = id;
	out[1] = id >> 8;
	out[2] = offset;
	out[3] = offset >> 8;
	out[4] = len;
	out[5] = len >> 8;
	out[6] = 0xff;
	out[7] = ref_crc8(0xff, out, 7);
}

static bool is_ate(uint32_t sector, uint32_t slot, uint16_t id, uint16_t offset, uint16_t len)
{
	uint8_t want[8];

	ref_ate(want, id, offset, len);
	return memcmp(area(sector * 4096 + slot), want, 8) == 0;
}

static bool erased(uint32_t from, uint32_t to)
{
	for (uint32_t i = from; i < to; i++) {
		if (kv_fake->flash[i] != 0xff) {
			return false;
		}
	}
	return true;
}

/* Tests ------------------------------------------------------------------*/

static void test_crc(void)
{
	uint8_t buf[64];

	assert(crc8_ccitt(0x00, "123456789", 9) == 0xf4);
	for (int n = 0; n < 1000; n++) {
		for (size_t i = 0; i < sizeof(buf); i++) {
			buf[i] = (uint8_t)rand();
		}
		assert(crc8_ccitt(0xff, buf, n % 64) == ref_crc8(0xff, buf, n % 64));
	}
	assert(sizeof(struct nvs_ate) == 8 && offsetof(struct nvs_ate, crc8) == 7);
}

/* The NVS that the settings mount: the Zephyr build's geometry. */
static void boot_geometry(void)
{
	struct nvs_fs *fs = NULL;

	assert(settings_subsys_init() == 0);
	assert(settings_storage_get((void **)&fs) == 0 && fs);
	assert(fs->offset == 0xffe000 && fs->sector_size == 4096 && fs->sector_count == 2);
	assert(fs->flash_device == &z_flash_kv_device);
	assert(fs->flash_parameters->write_block_size == 1);
	assert(fs->flash_parameters->erase_value == 0xff);
	/* Zephyr's mount messages (an empty area: the gc-done ATE first). */
	assert(strcmp(log_buf, "<inf> fs_nvs: 2 Sectors of 4096 bytes\n"
			       "<inf> fs_nvs: alloc wra: 0, fe8\n"
			       "<inf> fs_nvs: data wra: 0, 0\n") == 0);
	/* Idempotent. */
	assert(settings_subsys_init() == 0);
}

static void boot_set_port(void)
{
	assert(kv_store_set("imu.port", "18830", 5) == 0);
}

static void boot_set_session(void)
{
	assert(kv_store_set("imu.session", "\0\0\0\1", 4) == 0);
}

static void boot_same_session(void)
{
	unsigned long programs = kv_fake->programs;

	assert(settings_subsys_init() == 0);
	/* The value already stored: nothing written (nvs_write() compares). */
	assert(kv_store_set("imu.session", "\0\0\0\1", 4) == 0);
	assert(kv_fake->programs == programs);
}

static void test_format(void)
{
	kv_fake_init();
	BOOT(boot_geometry);
	/* The empty area got Zephyr's gc-done ATE (id 0xffff, offset 0, len 0)
	 * in sector 0's first ATE slot; nothing else.
	 */
	assert(is_ate(0, 0xff0, 0xffff, 0, 0));
	assert(erased(0, 0xff0) && erased(0xff8, 0x2000));

	/* "app_kv/imu.port" = "18830": settings_nvs.c's largest name ID
	 * (0x8000, NVS_NAMECNT_ID) = 0x8001, the value at 0x8001 + 0x4000,
	 * the name at 0x8001 (without NUL), in that order.
	 */
	BOOT(boot_set_port);
	assert(memcmp(area(0), "\x01\x80" "18830" "app_kv/imu.port", 22) == 0);
	assert(is_ate(0, 0xfe8, 0x8000, 0, 2));
	assert(is_ate(0, 0xfe0, 0xc001, 2, 5));
	assert(is_ate(0, 0xfd8, 0x8001, 7, 15));
	assert(erased(22, 0xfd8) && erased(0x1000, 0x2000));

	/* "app_kv/imu.session": the next name ID, 0x8002. */
	BOOT(boot_set_session);
	assert(memcmp(area(22), "\x02\x80" "\0\0\0\1" "app_kv/imu.session", 24) == 0);
	assert(is_ate(0, 0xfd0, 0x8000, 22, 2));
	assert(is_ate(0, 0xfc8, 0xc002, 24, 4));
	assert(is_ate(0, 0xfc0, 0x8002, 28, 18));
	assert(erased(46, 0xfc0));
	BOOT(boot_same_session);
}

static void boot_rw(void)
{
	char key[40], value[KV_STORE_VALUE_MAX + 1], buf[KV_STORE_VALUE_MAX + 8];
	const char *bad[] = {"a b", "a/b", "a:b", "a*", "a?", "\xe3\x81\x82",
			     "a\tb", "a+b", "a=b", "$a"};

	assert(kv_store_init() == 0);
	/* Absent, then stored. */
	assert(kv_store_get("imu.host", buf, sizeof(buf)) == -ENOENT);
	assert(kv_store_set("imu.host", "10.42.0.1", 9) == 0);
	assert(kv_store_get("imu.host", buf, sizeof(buf)) == 9 && !memcmp(buf, "10.42.0.1", 9));
	/* Too small a buffer: filled, -ENOBUFS. */
	memset(buf, 0, sizeof(buf));
	assert(kv_store_get("imu.host", buf, 4) == -ENOBUFS && !memcmp(buf, "10.4", 4));
	assert(buf[4] == 0);
	/* Replaced, then deleted; deleting again succeeds. */
	assert(kv_store_set("imu.host", "x", 1) == 0);
	assert(kv_store_get("imu.host", buf, sizeof(buf)) == 1 && buf[0] == 'x');
	assert(kv_store_delete("imu.host") == 0);
	assert(kv_store_get("imu.host", buf, sizeof(buf)) == -ENOENT);
	assert(kv_store_delete("imu.host") == 0);
	assert(kv_store_delete("never.stored") == 0);
	/* Binary values. */
	assert(kv_store_set("bin", "\0\xff\0", 3) == 0);
	assert(kv_store_get("bin", buf, sizeof(buf)) == 3 && !memcmp(buf, "\0\xff\0", 3));

	/* Keys: 1 to 32 characters of letters, digits, '_', '-', '.'. */
	assert(kv_store_set("azAZ09_-.", "v", 1) == 0);
	assert(kv_store_get("azAZ09_-.", buf, sizeof(buf)) == 1);
	assert(kv_store_set("a", "v", 1) == 0);
	memset(key, 'k', 32);
	key[32] = '\0';
	assert(kv_store_set(key, "v", 1) == 0);
	assert(kv_store_get(key, buf, sizeof(buf)) == 1);
	assert(kv_store_delete(key) == 0);
	key[32] = 'k';
	key[33] = '\0';
	assert(kv_store_set(key, "v", 1) == -EINVAL);
	assert(kv_store_get(key, buf, sizeof(buf)) == -EINVAL);
	assert(kv_store_delete(key) == -EINVAL);
	assert(kv_store_set("", "v", 1) == -EINVAL);
	assert(kv_store_set(NULL, "v", 1) == -EINVAL);
	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		assert(kv_store_set(bad[i], "v", 1) == -EINVAL);
		assert(kv_store_get(bad[i], buf, sizeof(buf)) == -EINVAL);
	}
	/* "sys." is reserved for the shell only (kv_cmd.c): the store takes it. */
	assert(kv_store_set("sys.x", "v", 1) == 0);
	assert(kv_store_delete("sys.x") == 0);

	/* Values: 1 to 128 bytes. */
	memset(value, 'v', sizeof(value));
	assert(kv_store_set("big", value, KV_STORE_VALUE_MAX) == 0);
	assert(kv_store_get("big", buf, sizeof(buf)) == KV_STORE_VALUE_MAX);
	assert(kv_store_set("big", value, KV_STORE_VALUE_MAX + 1) == -EINVAL);
	assert(kv_store_set("big", value, 0) == -EINVAL);
	assert(kv_store_set("big", NULL, 1) == -EINVAL);
	assert(kv_store_get("big", NULL, 1) == -EINVAL);
	assert(kv_store_get("big", buf, 0) == -EINVAL);
	assert(kv_store_get("big", buf, sizeof(buf)) == KV_STORE_VALUE_MAX);

	/* Other settings names, beside "app_kv/" (the Wi-Fi credentials). */
	assert(settings_save_one("wifi_cred/0", "cred", 4) == 0);
	assert(settings_load_one("wifi_cred/0", buf, sizeof(buf)) == 4);
	assert(kv_store_get("wifi_cred", buf, sizeof(buf)) == -ENOENT);
}

static void boot_rw_after(void)
{
	char buf[KV_STORE_VALUE_MAX];

	assert(kv_store_get("imu.port", buf, sizeof(buf)) == 5 && !memcmp(buf, "18830", 5));
	assert(kv_store_get("bin", buf, sizeof(buf)) == 3 && !memcmp(buf, "\0\xff\0", 3));
	assert(kv_store_get("big", buf, sizeof(buf)) == KV_STORE_VALUE_MAX);
	assert(kv_store_get("imu.host", buf, sizeof(buf)) == -ENOENT);
	assert(settings_load_one("wifi_cred/0", buf, sizeof(buf)) == 4);
	assert(settings_delete("wifi_cred/0") == 0);
	assert(settings_load_one("wifi_cred/0", buf, sizeof(buf)) == 0);
}

static void test_rw(void)
{
	BOOT(boot_rw);
	BOOT(boot_rw_after);
}

/* Garbage collection: many replacements of one key move the others. */

static void boot_gc(void)
{
	uint8_t value[KV_STORE_VALUE_MAX], buf[KV_STORE_VALUE_MAX];

	for (int i = 0; i < 400; i++) {
		memset(value, i, sizeof(value));
		assert(kv_store_set("big", value, sizeof(value) - (i % 7)) == 0);
		if (i % 50 == 0) {
			assert(kv_store_get("imu.port", buf, sizeof(buf)) == 5);
		}
	}
	assert(kv_fake->erases >= 10);
}

static void boot_gc_after(void)
{
	uint8_t buf[KV_STORE_VALUE_MAX];

	assert(kv_store_get("imu.port", buf, sizeof(buf)) == 5 && !memcmp(buf, "18830", 5));
	assert(kv_store_get("imu.session", buf, sizeof(buf)) == 4 && !memcmp(buf, "\0\0\0\1", 4));
	assert(kv_store_get("big", buf, sizeof(buf)) == KV_STORE_VALUE_MAX - 399 % 7);
	assert(buf[0] == (uint8_t)399);
}

static enum nano_kv_layout_e layout(void)
{
	enum nano_kv_layout_e l;

	assert(nano_flash_kv_layout(&l) == 0);
	return l;
}

static void test_gc(void)
{
	BOOT(boot_gc);
	BOOT(boot_gc_after);
	assert(layout() == NANO_KV_LAYOUT_NVS);
}

/* A full store: -ENOSPC, and what was stored before stays. */

static void boot_full(void)
{
	uint8_t value[KV_STORE_VALUE_MAX];
	char key[16];
	int rc = 0, i;

	for (i = 0; i < 100 && rc == 0; i++) {
		snprintf(key, sizeof(key), "k%d", i);
		memset(value, i, sizeof(value));
		rc = kv_store_set(key, value, sizeof(value));
	}
	assert(rc == -ENOSPC && i > 20);
}

static void boot_full_after(void)
{
	uint8_t buf[KV_STORE_VALUE_MAX];
	char key[16];
	int n = 0;

	for (int i = 0; i < 100; i++) {
		snprintf(key, sizeof(key), "k%d", i);
		if (kv_store_get(key, buf, sizeof(buf)) == KV_STORE_VALUE_MAX) {
			assert(buf[0] == (uint8_t)i && buf[127] == (uint8_t)i);
			n++;
		} else {
			break;
		}
	}
	/* 2 sectors of 4096 bytes, one kept free for garbage collection:
	 * about 25 values of 128 bytes with their names.
	 */
	assert(n >= 20 && n < 30);
	/* A delete still works when full. */
	assert(kv_store_delete("k0") == 0);
	assert(kv_store_get("k0", buf, sizeof(buf)) == -ENOENT);
}

static void test_full(void)
{
	kv_fake_init();
	BOOT(boot_full);
	BOOT(boot_full_after);
}

/* Data that is neither Zephyr's NVS nor erased. */

static uint8_t before[NANO_KV_SIZE];

static void boot_foreign(void)
{
	char buf[8];

	assert(settings_subsys_init() == -EILSEQ);
	assert(kv_store_init() == -EILSEQ);
	assert(kv_store_get("imu.port", buf, sizeof(buf)) == -EILSEQ);
	assert(kv_store_set("imu.port", "1", 1) == -EILSEQ);
	assert(kv_store_delete("imu.port") == -EILSEQ);
	/* The settings were never registered: their own error. */
	assert(settings_save_one("wifi_cred/0", "x", 1) == -ENOENT);
	assert(settings_load_one("wifi_cred/0", buf, sizeof(buf)) == 0);
	/* Reported once. */
	assert(strstr(log_buf, "<err> flash_kv: Key-value area 0xffe000-0xffffff holds data that "
			       "is not Zephyr's NVS\n<err> flash_kv: Not used; \"kv wipe confirm\" "
			       "erases it (all keys and Wi-Fi credentials)\n") == log_buf);
	assert(strstr(strstr(log_buf, "Key-value area") + 1, "Key-value area") == NULL);
	assert(kv_fake->programs == 0 && kv_fake->erases == 0);
}

static void boot_old_nuttx(void)
{
	char buf[8];

	assert(kv_store_get("imu.port", buf, sizeof(buf)) == -EILSEQ);
	assert(strcmp(log_buf, "<err> flash_kv: Key-value area 0xffe000-0xffffff holds the NuttX "
			       "build's earlier format\n<err> flash_kv: Not used; \"kv wipe confirm\" "
			       "erases it (all keys and Wi-Fi credentials)\n") == 0);
	assert(kv_fake->programs == 0 && kv_fake->erases == 0);
}

static void boot_wipe(void)
{
	char buf[8];

	assert(kv_store_get("imu.port", buf, sizeof(buf)) == -EILSEQ);
	log_buf[0] = '\0';
	assert(kv_store_wipe() == 0);
	assert(strcmp(log_buf, "<wrn> flash_kv: Key-value area erased\n"
			       "<inf> fs_nvs: 2 Sectors of 4096 bytes\n"
			       "<inf> fs_nvs: alloc wra: 0, fe8\n"
			       "<inf> fs_nvs: data wra: 0, 0\n") == 0);
	assert(kv_fake->erases == 2);
	assert(kv_store_set("imu.port", "18830", 5) == 0);
	assert(kv_store_get("imu.port", buf, sizeof(buf)) == 5);
	assert(settings_save_one("wifi_cred/0", "x", 1) == 0);
}

static void boot_after_wipe(void)
{
	char buf[8];

	assert(kv_store_get("imu.port", buf, sizeof(buf)) == 5 && !memcmp(buf, "18830", 5));
	assert(settings_load_one("wifi_cred/0", buf, sizeof(buf)) == 1);
	/* A wipe of a store in use: everything gone, the store usable. */
	assert(kv_store_wipe() == 0);
	assert(kv_store_get("imu.port", buf, sizeof(buf)) == -ENOENT);
	assert(settings_load_one("wifi_cred/0", buf, sizeof(buf)) == 0);
	assert(kv_store_set("imu.odr", "6667", 4) == 0);
}

static void boot_after_wipe2(void)
{
	char buf[8];

	assert(kv_store_get("imu.port", buf, sizeof(buf)) == -ENOENT);
	assert(kv_store_get("imu.odr", buf, sizeof(buf)) == 4);
	/* A failed erase: the error, and the settings mounted on what is
	 * there (here the area as it was).
	 */
	kv_fake->fail_next = 1;
	assert(kv_store_wipe() == -EIO);
	assert(kv_store_get("imu.odr", buf, sizeof(buf)) == 4);
}

/* An area as the NuttX build's earlier format (NuttX's NVS) left it. */
static void nuttx_ate(uint8_t *s, uint32_t id, uint16_t offset, uint16_t len, uint16_t key_len,
		      uint8_t data_crc8)
{
	memset(s, 0xff, 16);
	s[0] = id;
	s[1] = id >> 8;
	s[2] = id >> 16;
	s[3] = id >> 24;
	s[4] = offset;
	s[5] = offset >> 8;
	s[6] = len;
	s[7] = len >> 8;
	s[8] = key_len;
	s[9] = key_len >> 8;
	s[10] = data_crc8;
	s[11] = nano_kv_nuttx_crc8(s, 11);
}

static void test_foreign(void)
{
	/* Random bytes everywhere (an OpenMV file system, say). */
	kv_fake_init();
	srand(7);
	for (size_t i = 0; i < NANO_KV_SIZE; i++) {
		kv_fake->flash[i] = (uint8_t)rand();
	}
	assert(layout() == NANO_KV_LAYOUT_FOREIGN);
	memcpy(before, kv_fake->flash, sizeof(before));
	BOOT(boot_foreign);
	assert(memcmp(before, kv_fake->flash, sizeof(before)) == 0);
	/* One stray byte in an erased area. */
	kv_fake_init();
	kv_fake->flash[1234] = 0x7f;
	assert(layout() == NANO_KV_LAYOUT_FOREIGN);
	BOOT(boot_foreign);
	assert(kv_fake->flash[1234] == 0x7f && erased(0, 1234) && erased(1235, NANO_KV_SIZE));

	/* NuttX's NVS: block 0 with a gc-done ATE and one entry, block 1
	 * closed.
	 */
	kv_fake_init();
	memcpy(area(0), "app_kv/imu.port\0" "18830", 21);
	nuttx_ate(area(4096 - 32), 0xffffffff, 0, 0, 0, 0xff);
	nuttx_ate(area(4096 - 48), 0x1234, 0, 5, 16, 0x5a);
	nuttx_ate(area(8192 - 16), 0xffffffff, 4096 - 16 * 4, 0, 0, 0xff);
	assert(layout() == NANO_KV_LAYOUT_NUTTX);
	memcpy(before, kv_fake->flash, sizeof(before));
	BOOT(boot_old_nuttx);
	assert(memcmp(before, kv_fake->flash, sizeof(before)) == 0);
	BOOT(boot_wipe);
	BOOT(boot_after_wipe);
	BOOT(boot_after_wipe2);
	assert(layout() == NANO_KV_LAYOUT_NVS);

	/* Erased: Zephyr's NVS is started on it. */
	kv_fake_init();
	assert(layout() == NANO_KV_LAYOUT_EMPTY);
}

/* The flash device refuses everything outside the area. */

static void boot_flash_api(void)
{
	const struct device *dev = &z_flash_kv_device;
	const struct flash_area *fa;
	struct flash_sector sectors[3];
	struct flash_pages_info info;
	uint32_t count;
	uint8_t buf[16] = {0};

	assert(flash_read(dev, 0xffe000, buf, 16) == 0);
	assert(flash_read(dev, 0xffdfff, buf, 2) == -EINVAL);
	assert(flash_read(dev, 0xfffff8, buf, 9) == -EINVAL);
	assert(flash_read(dev, 0, buf, 16) == -EINVAL);
	assert(flash_read(dev, -1, buf, 1) == -EINVAL);
	assert(flash_read(dev, 0x1000000, buf, 1) == -EINVAL);
	assert(flash_write(dev, 0xffdff0, buf, 32) == -EINVAL);
	assert(flash_write(dev, 0x1000, buf, 1) == -EINVAL);
	assert(flash_write(dev, 0xffffff, buf, 2) == -EINVAL);
	assert(flash_erase(dev, 0xffd000, 4096) == -EINVAL);
	assert(flash_erase(dev, 0xffd000, 8192) == -EINVAL);
	assert(flash_erase(dev, 0xffe000, 12288) == -EINVAL);
	assert(flash_erase(dev, 0xffe100, 4096) == -EINVAL);
	assert(flash_erase(dev, 0xffe000, 100) == -EINVAL);
	assert(flash_erase(dev, 0x0, 4096) == -EINVAL);
	assert(flash_flatten(dev, 0xffc000, 8192) == -EINVAL);
	assert(flash_write(NULL, 0xffe000, buf, 1) == -EINVAL);
	assert(kv_fake->programs == 0 && kv_fake->erases == 0);
	/* Zero lengths do nothing. */
	assert(flash_write(dev, 0, buf, 0) == 0 && flash_erase(dev, 0, 0) == 0);
	/* Inside: a write over a page boundary is two page programs. */
	assert(flash_write(dev, 0xffe0fc, "abcdefgh", 8) == 0);
	assert(kv_fake->programs == 2 && !memcmp(area(0xfc), "abcdefgh", 8));
	assert(erased(0, 0xfc) && erased(0x104, NANO_KV_SIZE));
	assert(flash_erase(dev, 0xffe000, 8192) == 0 && kv_fake->erases == 2);
	assert(erased(0, NANO_KV_SIZE));
	/* Parameters and pages: the Zephyr build's. */
	assert(flash_get_write_block_size(dev) == 1);
	assert(flash_get_page_info_by_offs(dev, 0xfff123, &info) == 0);
	assert(info.start_offset == 0xfff000 && info.size == 4096 && info.index == 0xfff);
	assert(flash_get_page_info_by_offs(dev, 0x1000000, &info) == -EINVAL);
	/* The flash map: storage_partition only. */
	assert(flash_area_open(PARTITION_ID(storage_partition), &fa) == 0);
	assert(fa->fa_off == 0xffe000 && fa->fa_size == 0x2000 && fa->fa_dev == dev);
	assert(flash_area_open(1, &fa) == -ENOENT);
	count = 1;
	assert(flash_area_get_sectors(PARTITION_ID(storage_partition), &count, sectors) ==
	       -ENOMEM);
	assert(count == 1 && sectors[0].fs_off == 0 && sectors[0].fs_size == 4096);
	count = 3;
	assert(flash_area_get_sectors(PARTITION_ID(storage_partition), &count, sectors) == 0);
	assert(count == 2 && sectors[1].fs_off == 4096 && sectors[1].fs_size == 4096);
	assert(strstr(log_buf, "<err> flash_kv: Refused flash erase at 0x00ffd000, 4096 bytes\n"));
	/* Not ready: the device is not there. */
	kv_fake->ready = false;
	assert(flash_area_open(PARTITION_ID(storage_partition), &fa) == -ENODEV);
	assert(settings_subsys_init() == -ENODEV);
	assert(kv_store_init() == -ENODEV);
	kv_fake->ready = true;
	assert(kv_store_init() == 0);
}

static void test_flash_api(void)
{
	kv_fake_init();
	BOOT(boot_flash_api);
}

/* Zephyr's NVS itself on an area whose two close slots hold data (what
 * the Zephyr build does with foreign data in its storage_partition, where
 * nothing checks it before): the mount fails with -EDEADLK
 * (CONFIG_NVS_INIT_BAD_MEMORY_REGION is off) and nothing is written.
 */

static void boot_raw_nvs(void)
{
	struct nvs_fs fs = {
		.offset = 0xffe000,
		.sector_size = 4096,
		.sector_count = 2,
		.flash_device = &z_flash_kv_device,
	};

	assert(nvs_mount(&fs) == -EDEADLK);
	assert(kv_fake->programs == 0 && kv_fake->erases == 0);
}

static void test_raw_nvs(void)
{
	kv_fake_init();
	srand(11);
	for (size_t i = 0; i < NANO_KV_SIZE; i++) {
		kv_fake->flash[i] = (uint8_t)rand();
	}
	memcpy(before, kv_fake->flash, sizeof(before));
	BOOT(boot_raw_nvs);
	assert(memcmp(before, kv_fake->flash, sizeof(before)) == 0);
}

/* Power loss -------------------------------------------------------------*/

#define KEYS 5

static const char *const pl_keys[KEYS] = {"imu.port", "imu.host", "cal", "big", "imu.session"};

struct step {
	int key;     /* index into pl_keys */
	int len;     /* 0: delete */
	uint8_t fill;
};

static struct step steps[220];
static int nsteps;

/* Shared with the children: how far the writer got. */
static struct progress {
	int started, done;
} *progress;

static void step_value(const struct step *s, uint8_t *value)
{
	for (int i = 0; i < s->len; i++) {
		value[i] = (uint8_t)(s->fill + i);
	}
}

static void make_steps(void)
{
	nsteps = 0;
	steps[nsteps++] = (struct step){0, 5, '1'};
	steps[nsteps++] = (struct step){1, 9, 'a'};
	steps[nsteps++] = (struct step){2, 56, 0x10};
	for (int i = 0; i < 200; i++) {
		if (i % 20 == 7) {
			steps[nsteps++] = (struct step){1, 0, 0};       /* delete */
		} else if (i % 20 == 13) {
			steps[nsteps++] = (struct step){1, 3 + i % 5, (uint8_t)i};
		} else if (i % 9 == 4) {
			steps[nsteps++] = (struct step){4, 4, (uint8_t)i};
		} else {
			steps[nsteps++] = (struct step){3, 100 + i % 29, (uint8_t)i};
		}
	}
	assert(nsteps <= (int)(sizeof(steps) / sizeof(steps[0])));
}

static void boot_writer(void)
{
	uint8_t value[KV_STORE_VALUE_MAX];

	for (int i = 0; i < nsteps; i++) {
		progress->started = i;
		if (steps[i].len) {
			step_value(&steps[i], value);
			assert(kv_store_set(pl_keys[steps[i].key], value, steps[i].len) == 0);
		} else {
			assert(kv_store_delete(pl_keys[steps[i].key]) == 0);
		}
		progress->done = i + 1;
	}
}

/* The value of KEY after the first N steps: its last step, or -1. */
static int last_step(int key, int n)
{
	int last = -1;

	for (int i = 0; i < n; i++) {
		if (steps[i].key == key) {
			last = i;
		}
	}
	return last;
}

static bool holds(int step, const uint8_t *buf, ssize_t len)
{
	uint8_t value[KV_STORE_VALUE_MAX];

	if (step < 0 || steps[step].len == 0) {
		return len == -ENOENT;
	}
	step_value(&steps[step], value);
	return len == steps[step].len && memcmp(buf, value, len) == 0;
}

static void boot_verifier(void)
{
	uint8_t buf[KV_STORE_VALUE_MAX];
	int done = progress->done;
	bool in_flight = progress->started == done && done < nsteps;
	enum nano_kv_layout_e l;

	assert(nano_flash_kv_layout(&l) == 0);
	assert(l == NANO_KV_LAYOUT_NVS || (l == NANO_KV_LAYOUT_EMPTY && done == 0));
	assert(kv_store_init() == 0);
	for (int k = 0; k < KEYS; k++) {
		ssize_t len = kv_store_get(pl_keys[k], buf, sizeof(buf));

		if (holds(last_step(k, done), buf, len)) {
			continue;
		}
		assert(in_flight && steps[done].key == k && holds(done, buf, len));
	}
	/* The store takes writes again, and keeps them. */
	assert(kv_store_set("after", "cut", 3) == 0);
	assert(kv_store_get("after", buf, sizeof(buf)) == 3);
}

static void boot_verifier2(void)
{
	uint8_t buf[8];

	assert(kv_store_get("after", buf, sizeof(buf)) == 3 && !memcmp(buf, "cut", 3));
}

static void test_power_loss(void)
{
	unsigned long total, gc_erases;
	int status;

	progress = mmap(NULL, sizeof(*progress), PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	assert(progress != MAP_FAILED);
	make_steps();

	/* Without a cut: the number of programs and erases, and that the
	 * sequence goes through garbage collection several times.
	 */
	kv_fake_init();
	BOOT(boot_writer);
	total = kv_fake->ops;
	gc_erases = kv_fake->erases;
	assert(gc_erases >= 4 && total > 300);
	progress->done = nsteps;
	progress->started = nsteps;
	BOOT(boot_verifier);

	for (unsigned long cut = 1; cut <= total; cut++) {
		kv_fake_init();
		progress->started = progress->done = 0;
		kv_fake->cut_at = cut;
		status = boot(boot_writer);
		assert(status == KV_FAKE_CUT_EXIT);
		kv_fake->cut_at = 0;
		BOOT(boot_verifier);
		BOOT(boot_verifier2);
	}
}

int main(void)
{
	kv_fake_init();
	test_crc();
	test_format();
	test_rw();
	test_gc();
	test_full();
	test_foreign();
	test_flash_api();
	test_raw_nvs();
	test_power_loss();
	puts("kv_settings ok");
	return 0;
}
