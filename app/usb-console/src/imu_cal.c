/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include "imu_cal.h"
#include "kv_store.h"

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));

static bool lookup(enum lsm6dso_cal_kind kind, uint32_t gain, struct lsm6dso_cal *cal)
{
	char key[IMU_CAL_KEY_MAX];
	uint8_t blob[IMU_CAL_BLOB_SIZE];
	ssize_t len;

	if (imu_cal_key(kind, gain, key, sizeof(key)) < 0) {
		return false;
	}
	len = kv_store_get(key, blob, sizeof(blob));
	return len > 0 && imu_cal_decode(blob, len, kind, gain, cal) == 0;
}

int imu_cal_init(void)
{
	if (!device_is_ready(imu)) {
		return -ENODEV;
	}
	lsm6dso_cal_set_lookup(imu, lookup);
	return 0;
}

static void print_one(const struct shell *sh, const char *name, const char *unit,
		      enum lsm6dso_cal_kind kind, uint32_t gain, bool calibrated)
{
	char key[IMU_CAL_KEY_MAX];

	(void)imu_cal_key(kind, gain, key, sizeof(key));
	shell_print(sh, "  %-6s %s (%u %s/LSB, key %s)", name,
		    calibrated ? "calibrated" : "nominal", (unsigned int)gain, unit, key);
}

int imu_cal_shell(const struct shell *sh, size_t argc, char **argv)
{
	struct lsm6dso_cal_status status;

	if (!device_is_ready(imu)) {
		shell_error(sh, "IMU not ready");
		return -ENODEV;
	}
	if (argc > 1) {
		if (strcmp(argv[1], "reload") != 0) {
			shell_error(sh, "usage: imu cal [reload]");
			return -EINVAL;
		}
		lsm6dso_cal_reload(imu);
	}
	lsm6dso_cal_status(imu, &status);
	shell_print(sh, "conversion:");
	print_one(sh, "accel:", "ug", LSM6DSO_CAL_ACCEL, status.accel_gain, status.accel_calibrated);
	print_one(sh, "gyro:", "udps", LSM6DSO_CAL_GYRO, status.gyro_gain, status.gyro_calibrated);
	return 0;
}

