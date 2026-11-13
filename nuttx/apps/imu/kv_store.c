/* SPDX-License-Identifier: Apache-2.0 */
/* The key-value store: zephyr/app/usb-console/src/kv_store.c on NuttX.
 * The Zephyr build keeps the values in Zephyr's settings on NVS
 * (settings_save_one()/settings_load_one()/settings_delete()); this one
 * keeps them in NuttX's NVS, /dev/config (drivers/mtd/mtd_config_nvs.c on
 * the board's MTD over the last 8 KiB of the flash,
 * boards/arduino-nano-rp2040-connect/src/nano_flash_kv.c), under the same
 * names ("app_kv/" + key).  The stored format differs from the Zephyr
 * build's.  The shell command is in nuttx/apps/kv.
 *
 * What the Zephyr settings on NVS did without being asked, and what this
 * does:
 * - storing the value that is already stored writes nothing and succeeds
 *   (Zephyr: nvs_write() compares; NuttX: nvs_write() compares),
 * - deleting a key that is not there succeeds (Zephyr: settings_nvs_save()
 *   returns 0; NuttX: nvs_write() with no data returns 0),
 * - when the live entries do not fit, the write fails with -ENOSPC after
 *   garbage collection has gone through every block (both NVS),
 * - reading into a buffer that is too small fills it and reports the full
 *   length, which kv_store_get() turns into -ENOBUFS (both).
 *
 * Every call opens /dev/config, does one ioctl and closes it: file
 * descriptors belong to a task group, and the callers are the shell's
 * commands (separate tasks) and the application's threads.  The driver
 * serializes the calls (nvs_lock, a mutex with priority inheritance), as
 * Zephyr's settings_lock did.  kv_store_init() only checks that the store
 * is there; the board mounts it at boot (Zephyr: settings_subsys_init(),
 * which is idempotent, is called from every function).
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/mtd/configdata.h>

#include "kv_name.h"
#include "kv_store.h"

#ifndef CONFIG_MTD_CONFIG_NAMED
#  error "kv_store.c stores named items: set MTD_CONFIG_NAMED"
#endif

#if CONFIG_MTD_CONFIG_NAME_LEN < 40 /* KV_NAME_SIZE */
#  error "MTD_CONFIG_NAME_LEN must hold \"app_kv/\", 32 characters and NUL"
#endif

_Static_assert(sizeof(((struct config_data_s *)0)->name) >= KV_NAME_SIZE,
	       "config_data_s.name is too short for the longest name");

#define KV_DEVPATH "/dev/config"

/* One request to /dev/config.  -ENODEV if the store is not there, so that
 * it is not taken for a missing key (-ENOENT).
 */
static int kv_request(int cmd, struct config_data_s *data)
{
	int fd = open(KV_DEVPATH, O_RDWR | O_CLOEXEC);
	int ret;

	if (fd < 0) {
		return errno == ENOENT ? -ENODEV : -errno;
	}
	ret = data ? ioctl(fd, cmd, (unsigned long)(uintptr_t)data) : 0;
	if (ret < 0) {
		ret = -errno;
	}
	close(fd);
	return ret;
}

int kv_store_init(void)
{
	return kv_request(0, NULL);
}

int kv_store_set(const char *key, const void *value, size_t len)
{
	struct config_data_s data;
	int ret = kv_make_name(key, data.name, sizeof(data.name));

	if (ret) {
		return ret;
	}
	if (!value || !len || len > KV_STORE_VALUE_MAX) {
		return -EINVAL;
	}
	data.configdata = (uint8_t *)(uintptr_t)value;
	data.len = len;
	return kv_request(CFGDIOC_SETCONFIG, &data);
}

ssize_t kv_store_get(const char *key, void *value, size_t capacity)
{
	struct config_data_s data;
	int ret = kv_make_name(key, data.name, sizeof(data.name));

	if (ret) {
		return ret;
	}
	if (!value || !capacity) {
		return -EINVAL;
	}
	data.configdata = value;
	data.len = capacity;
	ret = kv_request(CFGDIOC_GETCONFIG, &data);
	if (ret < 0) {
		return ret;
	}
	/* data.len is now the stored length, even when larger than capacity
	 * (then only capacity bytes were read).
	 */
	if (data.len == 0) {
		return -ENOENT;
	}
	if (data.len > capacity) {
		return -ENOBUFS;
	}
	return data.len;
}

int kv_store_delete(const char *key)
{
	struct config_data_s data;
	int ret = kv_make_name(key, data.name, sizeof(data.name));

	if (ret) {
		return ret;
	}
	data.configdata = NULL;
	data.len = 0;
	return kv_request(CFGDIOC_DELCONFIG, &data);
}
