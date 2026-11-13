/* SPDX-License-Identifier: Apache-2.0 */
/* STUB, replaced by the port of src/imu_cal.c (T22 of
 * nuttx/PORTING_PLAN.md).
 *
 * imu_cal_init() and imu_cal_shell() of imu_cal.h. imu_cal_init() does
 * nothing and succeeds: the LSM6DSO driver keeps its nominal conversion
 * (the RAW stream of "imu start" does not use it). "imu cal" prints an
 * error and fails with -ENOSYS; the Zephyr build shows the conversion in
 * use there. imu_cal_key() and imu_cal_decode() are the real ones
 * (imu_cal_codec.c, unchanged copy) but are not built yet.
 */
#include <nuttx/config.h>
#include <errno.h>
#include <zephyr/shell/shell.h>

#include "imu_cal.h"

int imu_cal_init(void)
{
	return 0;
}

int imu_cal_shell(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;
	shell_error(sh, "Calibration is not available in this build yet");
	return -ENOSYS;
}
