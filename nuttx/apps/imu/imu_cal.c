/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX port: a copy of zephyr/app/usb-console/src/imu_cal.c with only this
 * header and the NuttX include below added. The calls keep their Zephyr
 * names on nuttx/drivers/compat, the Zephyr compatibility layer: the
 * LSM6DSOX device (DEVICE_DT_GET(), device_is_ready()), shell_print() and
 * shell_error() (stdout and stderr of the session that runs "imu cal"), and
 * snprintk(). The key-value store is the port's kv_store.c on /dev/config.
 * The init task (nuttx/apps/init) calls imu_cal_init() where the Zephyr
 * build's main() does, after kv_store_init(); "imu cal" is the subcommand
 * of the builtin "imu" command (imu_stream.c). NuttX's headers come first:
 * the layer's <zephyr/sys/atomic.h> takes over NuttX's atomic names. */
#include <nuttx/config.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
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

/* Prints one bias value, kept as LSB << 8, with two decimals. */
static void format_bias(char *text, size_t size, int32_t value)
{
	int32_t hundredths = (int32_t)(((int64_t)value * 100 +
					(value >= 0 ? 128 : -128)) / 256);

	snprintk(text, size, "%s%d.%02d", hundredths < 0 ? "-" : "",
		 (int)(abs(hundredths) / 100), (int)(abs(hundredths) % 100));
}

static void print_bias(const struct shell *sh, const struct lsm6dso_cal_status *status)
{
	char x[12], y[12], z[12];

	format_bias(x, sizeof(x), status->gyro_bias[0]);
	format_bias(y, sizeof(y), status->gyro_bias[1]);
	format_bias(z, sizeof(z), status->gyro_bias[2]);
	if (status->gyro_bias_updates > 0) {
		shell_print(sh, "  gyro bias: %s %s %s LSB (tracked at rest, %u updates, %u rejected)",
			    x, y, z, (unsigned int)status->gyro_bias_updates,
			    (unsigned int)status->gyro_bias_rejected);
	} else {
		shell_print(sh, "  gyro bias: %s %s %s LSB (%s, %u rejected)", x, y, z,
			    status->gyro_calibrated ? "stored" : "none",
			    (unsigned int)status->gyro_bias_rejected);
	}
}

int imu_cal_shell(const struct shell *sh, size_t argc, char **argv)
{
	struct lsm6dso_cal_status status;

	if (!device_is_ready(imu)) {
		shell_error(sh, "IMU not ready");
		return -ENODEV;
	}
	if (argc > 1 && strcmp(argv[1], "reload") == 0) {
		lsm6dso_cal_reload(imu);
	} else if (argc > 1 && strcmp(argv[1], "zero") == 0) {
		int ret = lsm6dso_cal_zero(imu);

		if (ret == -EBUSY) {
			shell_error(sh, "The sensor is streaming; the bias is tracked from the stream");
		} else if (ret == -EAGAIN) {
			shell_error(sh, "The sensor did not stay at rest");
		} else if (ret == -ERANGE) {
			shell_error(sh, "The reading is too far from the stored bias");
		} else if (ret) {
			shell_error(sh, "Could not read the sensor: %d", ret);
		}
		if (ret) {
			return ret;
		}
	} else if (argc > 1) {
		shell_error(sh, "usage: imu cal [reload|zero]");
		return -EINVAL;
	}
	lsm6dso_cal_status(imu, &status);
	shell_print(sh, "conversion:");
	print_one(sh, "accel:", "ug", LSM6DSO_CAL_ACCEL, status.accel_gain, status.accel_calibrated);
	print_one(sh, "gyro:", "udps", LSM6DSO_CAL_GYRO, status.gyro_gain, status.gyro_calibrated);
	print_bias(sh, &status);
	return 0;
}

