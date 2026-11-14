/* SPDX-License-Identifier: Apache-2.0 */
/* settings_subsys_init(), settings_save_one(), settings_delete() and
 * settings_load_one() of the Zephyr compatibility layer's
 * <zephyr/settings/settings.h>, on NuttX's NVS (/dev/config), for the
 * stored Wi-Fi credentials and the Wi-Fi profile.  Each item is the NVS item
 * of the same name; the names of the "kv" command's keys start with
 * "app_kv/" (nuttx/apps/imu/kv_name.h), so the two never meet.  The calls
 * are kv_store.c's, without its key rules and 128-byte limit.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/mtd/configdata.h>

#include <zephyr/settings/settings.h>

#ifndef CONFIG_MTD_CONFIG_NAMED
#  error "settings_nvs.c stores named items: set MTD_CONFIG_NAMED"
#endif

#define SETTINGS_DEVPATH "/dev/config"

/* One request to /dev/config; -ENODEV if the store is not there. */

static int settings_request(int cmd, struct config_data_s *data)
{
	int fd = open(SETTINGS_DEVPATH, O_RDWR | O_CLOEXEC);
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

static int settings_name(struct config_data_s *data, const char *name)
{
	size_t len;

	if (!name) {
		return -EINVAL;
	}
	len = strnlen(name, sizeof(data->name));
	if (!len || len >= sizeof(data->name)) {
		return -EINVAL;
	}
	memcpy(data->name, name, len + 1);
	return 0;
}

int settings_subsys_init(void)
{
	return settings_request(0, NULL);
}

int settings_save_one(const char *name, const void *value, size_t val_len)
{
	struct config_data_s data;
	int ret = settings_name(&data, name);

	if (ret) {
		return ret;
	}
	if (!val_len) {
		return settings_delete(name);
	}
	if (!value) {
		return -EINVAL;
	}
	data.configdata = (uint8_t *)(uintptr_t)value;
	data.len = val_len;
	return settings_request(CFGDIOC_SETCONFIG, &data);
}

int settings_delete(const char *name)
{
	struct config_data_s data;
	int ret = settings_name(&data, name);

	if (ret) {
		return ret;
	}
	data.configdata = NULL;
	data.len = 0;
	return settings_request(CFGDIOC_DELCONFIG, &data);
}

ssize_t settings_load_one(const char *name, void *buf, size_t buf_len)
{
	struct config_data_s data;
	int ret = settings_name(&data, name);

	if (ret) {
		return ret;
	}
	if (!buf || !buf_len) {
		return -EINVAL;
	}
	data.configdata = buf;
	data.len = buf_len;
	ret = settings_request(CFGDIOC_GETCONFIG, &data);
	if (ret == -ENOENT) {
		return 0;
	}
	if (ret < 0) {
		return ret;
	}
	/* data.len is the stored length, also when it is more than buf_len
	 * (then only buf_len bytes were read).
	 */
	return data.len;
}
