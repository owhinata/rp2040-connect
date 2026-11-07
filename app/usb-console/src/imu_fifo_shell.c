/* SPDX-License-Identifier: Apache-2.0 */
#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include "../drivers/lsm6dso/lsm6dso_fifo_stats.h"

static int cmd_imu_fifo_stats(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
	struct lsm6dso_fifo_stats stats;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	lsm6dso_fifo_get_stats(dev, &stats);
	shell_print(shell, "words=%" PRIu64 " transfers=%" PRIu32,
		    stats.words_read, stats.transfers);
	shell_print(shell, "transfer_ns=%" PRIu64 " max_transfer_ns=%" PRIu64,
		    stats.transfer_time_ns, stats.max_transfer_time_ns);
	shell_print(shell, "i2c_errors=%" PRIu32 " buffer_too_small=%" PRIu32
		    " overrun_observations=%" PRIu32,
		    stats.i2c_errors, stats.buffer_too_small, stats.overrun_observations);
	return 0;
}

SHELL_CMD_REGISTER(imu_fifo_stats, NULL, "Show IMU FIFO counters", cmd_imu_fifo_stats);
