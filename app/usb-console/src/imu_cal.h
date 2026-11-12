/* SPDX-License-Identifier: Apache-2.0 */
#ifndef APP_IMU_CAL_H_
#define APP_IMU_CAL_H_

#include <stddef.h>
#include <stdint.h>
#include "lsm6dso_cal.h"

/* Calibration values are kept in the KV store, one entry per sensor and
 * sensitivity:
 *
 *   imu.cal.accel.<ug per LSB>     imu.cal.gyro.<udps per LSB>
 *
 * An entry applies only while the sensor runs at that sensitivity, so a range
 * that was never calibrated falls back to the nominal conversion.  The value
 * is 56 bytes, little-endian:
 *
 *   u8  version (1)
 *   u8  kind (0 accel, 1 gyro)
 *   u16 reserved (0)
 *   u32 sensitivity, as in the key
 *   i32 m[3][3]   row-major, see struct lsm6dso_cal
 *   i32 bias[3]
 */
#define IMU_CAL_VERSION   1
#define IMU_CAL_BLOB_SIZE 56
#define IMU_CAL_KEY_MAX   32

/* Writes the KV key for a sensor and sensitivity. Returns the length, or a
 * negative errno if it does not fit.
 */
int imu_cal_key(enum lsm6dso_cal_kind kind, uint32_t gain, char *key, size_t capacity);
/* Decodes a stored value. Returns 0, or a negative errno if the value is not a
 * calibration for this sensor and sensitivity.
 */
int imu_cal_decode(const uint8_t *blob, size_t len, enum lsm6dso_cal_kind kind, uint32_t gain,
		   struct lsm6dso_cal *cal);

/* Makes the IMU driver use the stored calibration. Call after kv_store_init(). */
int imu_cal_init(void);

struct shell;
/* "imu cal [reload]": shows the conversion in use, after reading the KV store again
 * when asked.
 */
int imu_cal_shell(const struct shell *sh, size_t argc, char **argv);

#endif
