/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/shell/shell.h>

#define WHO_AM_I_REG   0x0f
#define WHO_AM_I_VALUE 0x6c
#define STABLE_PREFIX  9U

static const struct i2c_dt_spec imu_bus = I2C_DT_SPEC_GET(DT_NODELABEL(lsm6dsox));

static int verify_result(const struct shell *shell, const char *name, int ret,
			 const uint8_t *actual, const uint8_t *expected, size_t length)
{
	if (ret != 0 || memcmp(actual, expected, length) != 0) {
		shell_error(shell, "%s failed: rc=%d", name, ret);
		return -EIO;
	}
	return 0;
}

static int cmd_validate(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t reference[STABLE_PREFIX], data[65], reg = WHO_AM_I_REG;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!i2c_is_ready_dt(&imu_bus)) {
		return -ENODEV;
	}
	ret = i2c_burst_read_dt(&imu_bus, reg, reference, sizeof(reference));
	if (ret != 0 || reference[0] != WHO_AM_I_VALUE) {
		shell_error(shell, "reference read failed: rc=%d", ret);
		return -EIO;
	}
	for (size_t length = 1; length <= sizeof(data); length++) {
		memset(data, 0xa5, sizeof(data));
		ret = i2c_burst_read_dt(&imu_bus, reg, data, length);
		if (verify_result(shell, "burst", ret, data, reference,
				  MIN(length, sizeof(reference))) != 0) {
			shell_error(shell, "length=%u", (unsigned int)length);
			return -EIO;
		}
		if (length < sizeof(data) && data[length] != 0xa5) {
			shell_error(shell, "buffer overwritten: length=%u", (unsigned int)length);
			return -EIO;
		}
	}
	/* Contiguous read messages without STOP, followed by a read with STOP. */
	struct i2c_msg split[] = {
		{.buf = &reg, .len = 1, .flags = I2C_MSG_WRITE},
		{.buf = data, .len = 3, .flags = I2C_MSG_READ | I2C_MSG_RESTART},
		{.buf = data + 3, .len = 6, .flags = I2C_MSG_READ | I2C_MSG_STOP},
	};
	memset(data, 0xa5, sizeof(data));
	ret = i2c_transfer_dt(&imu_bus, split, ARRAY_SIZE(split));
	if (verify_result(shell, "split read", ret, data, reference, sizeof(reference)) != 0) {
		return -EIO;
	}
	/* Two register reads with direction changes and RESTART in one transfer. */
	struct i2c_msg repeated[] = {
		{.buf = &reg, .len = 1, .flags = I2C_MSG_WRITE},
		{.buf = data, .len = 1, .flags = I2C_MSG_READ | I2C_MSG_RESTART},
		{.buf = &reg, .len = 1, .flags = I2C_MSG_WRITE | I2C_MSG_RESTART},
		{.buf = data + 1, .len = 1, .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP},
	};
	ret = i2c_transfer_dt(&imu_bus, repeated, ARRAY_SIZE(repeated));
	if (ret != 0 || data[0] != WHO_AM_I_VALUE || data[1] != WHO_AM_I_VALUE) {
		shell_error(shell, "repeated start failed: rc=%d", ret);
		return -EIO;
	}
	shell_print(shell, "i2c validation passed: lengths=1..65 split_read repeated_start");
	return 0;
}

static int cmd_timeout(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t data[64], reg = WHO_AM_I_REG, id = 0;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (CONFIG_I2C_TRANSFER_TIMEOUT_MS > 2) {
		shell_error(shell, "requires timeout-test build (<=2 ms) and 100 kHz I2C");
		return -EINVAL;
	}
	ret = i2c_burst_read_dt(&imu_bus, reg, data, sizeof(data));
	if (ret != -ETIMEDOUT) {
		shell_error(shell, "expected -ETIMEDOUT, got %d", ret);
		return -EIO;
	}
	ret = i2c_burst_read_dt(&imu_bus, reg, &id, 1);
	if (ret != 0 || id != WHO_AM_I_VALUE) {
		shell_error(shell, "read after timeout failed: rc=%d id=0x%02x", ret, id);
		return -EIO;
	}
	shell_print(shell, "i2c timeout recovery passed");
	return 0;
}

SHELL_CMD_REGISTER(i2c_validate, NULL, "Validate I2C transfer shapes", cmd_validate);
SHELL_CMD_REGISTER(i2c_validate_timeout, NULL, "Validate I2C timeout recovery", cmd_timeout);
