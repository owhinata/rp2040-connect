/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the calibration (nuttx/apps/imu/imu_cal.c, the copy of the
 * Zephyr build's src/imu_cal.c) on the ported LSM6DSO driver and its fake
 * sensor (tests/lsm6dso_driver.c, included below; its own tests run first)
 * and a fake key-value store.  It checks the path from a stored value to
 * the driver's output and the "imu cal" text:
 *
 * - imu_cal_init(): the lookup reads imu.cal.accel.<gain> and
 *   imu.cal.gyro.<gain> at once; nothing stored, a value of the wrong
 *   length, version, kind or sensitivity, or one too long for the buffer
 *   leaves the nominal conversion; -ENODEV while the IMU is not ready.
 * - sensor_channel_get() gives M * (raw - bias) of the stored value, not the
 *   nominal conversion.
 * - a range change reads the key of the new sensitivity (nominal without
 *   one) and back; "reload" reads the store again (a deleted key is
 *   nominal again, a tracked bias is dropped).
 * - "imu cal" lines as in the Zephyr build (README: "conversion:", the
 *   accel and gyro lines with gain, unit and key, the bias with two
 *   decimals rounded half away from zero, "stored", "none", "tracked at
 *   rest"), the usage error, "zero" at rest, too far from the stored bias,
 *   while the FIFO is owned and on an I2C error, and "IMU not ready".  The
 *   messages go to stderr, the conversion to stdout, as shell_error() and
 *   shell_print() of the compatibility layer.
 */
#define main lsm6dso_driver_tests
#include "lsm6dso_driver.c"
#undef main

#include <unistd.h>

#include "imu_cal.h"
#include "kv_store.h"

/* Fake key-value store ------------------------------------------------------*/

static struct {
	char key[KV_STORE_KEY_MAX + 1];
	uint8_t value[KV_STORE_VALUE_MAX];
	size_t len;
} kv[4];
static int kv_reads;

static void kv_put(const char *key, const uint8_t *value, size_t len)
{
	for (size_t i = 0; i < ARRAY_SIZE(kv); i++) {
		if (kv[i].len == 0 || strcmp(kv[i].key, key) == 0) {
			strcpy(kv[i].key, key);
			memcpy(kv[i].value, value, len);
			kv[i].len = len;
			return;
		}
	}
	assert(0);
}

static void kv_clear(void)
{
	memset(kv, 0, sizeof(kv));
}

ssize_t kv_store_get(const char *key, void *value, size_t capacity)
{
	kv_reads++;
	for (size_t i = 0; i < ARRAY_SIZE(kv); i++) {
		if (kv[i].len > 0 && strcmp(kv[i].key, key) == 0) {
			if (kv[i].len > capacity) {
				return -ENOBUFS;
			}
			memcpy(value, kv[i].value, kv[i].len);
			return (ssize_t)kv[i].len;
		}
	}
	return -ENOENT;
}

/* Helpers -------------------------------------------------------------------*/

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

/* The 56-byte value of imu_cal.h. */
static void encode(uint8_t blob[IMU_CAL_BLOB_SIZE], enum lsm6dso_cal_kind kind, uint32_t gain,
		   const struct lsm6dso_cal *cal)
{
	memset(blob, 0, IMU_CAL_BLOB_SIZE);
	blob[0] = IMU_CAL_VERSION;
	blob[1] = kind == LSM6DSO_CAL_ACCEL ? 0 : 1;
	put_le32(blob + 4, gain);
	for (int i = 0; i < 9; i++) {
		put_le32(blob + 8 + 4 * i, (uint32_t)cal->m[i / 3][i % 3]);
	}
	for (int i = 0; i < 3; i++) {
		put_le32(blob + 44 + 4 * i, (uint32_t)cal->bias[i]);
	}
}

static void store(enum lsm6dso_cal_kind kind, uint32_t gain, const struct lsm6dso_cal *cal)
{
	char key[IMU_CAL_KEY_MAX];
	uint8_t blob[IMU_CAL_BLOB_SIZE];

	assert(imu_cal_key(kind, gain, key, sizeof(key)) > 0);
	encode(blob, kind, gain, cal);
	kv_put(key, blob, sizeof(blob));
}

static char out_text[2048], err_text[2048];

static void read_back(FILE *f, char *text, size_t size)
{
	size_t n;

	rewind(f);
	n = fread(text, 1, size - 1, f);
	text[n] = '\0';
	fclose(f);
}

/* Runs "imu cal ARGS..." and keeps what it wrote to stdout and stderr. */
static int cal(int argc, char **argv)
{
	FILE *out = tmpfile(), *err = tmpfile();
	int saved_out, saved_err, ret;

	assert(out && err);
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	dup2(fileno(out), 1);
	dup2(fileno(err), 2);
	ret = imu_cal_shell(NULL, (size_t)argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	read_back(out, out_text, sizeof(out_text));
	read_back(err, err_text, sizeof(err_text));
	return ret;
}

static int cal0(void)
{
	char *argv[] = {"cal", NULL};

	return cal(1, argv);
}

static int cal1(const char *arg)
{
	char *argv[] = {"cal", (char *)arg, NULL};

	return cal(2, argv);
}

static void set_range(enum sensor_channel chan, int32_t val1, int32_t val2)
{
	struct sensor_value v = {.val1 = val1, .val2 = val2};

	assert(sensor_attr_set(imu(), chan, SENSOR_ATTR_FULL_SCALE, &v) == 0);
}

static void set_odr(int32_t hz)
{
	struct sensor_value v = {.val1 = hz};

	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
}

/* The driver's output for raw readings, against the conversion cal. */
static void check_output(enum sensor_channel chan, const int16_t raw[3],
			 const struct lsm6dso_cal *want)
{
	struct sensor_value v[3];
	int32_t out[3];

	assert(sensor_sample_fetch(imu()) == 0);
	assert(sensor_channel_get(imu(), chan, v) == 0);
	lsm6dso_cal_apply(want, raw, out);
	for (int i = 0; i < 3; i++) {
		assert(micro(&v[i]) == lsm6dso_cal_to_micro(out[i]));
	}
}

/* Tests ---------------------------------------------------------------------*/

static const char nominal_text[] =
	"conversion:\n"
	"  accel: nominal (244 ug/LSB, key imu.cal.accel.244)\n"
	"  gyro:  nominal (70000 udps/LSB, key imu.cal.gyro.70000)\n"
	"  gyro bias: 0.00 0.00 0.00 LSB (none, 0 rejected)\n";

static void test_nominal(void)
{
	kv_clear();
	kv_reads = 0;
	assert(imu_cal_init() == 0);
	assert(kv_reads == 2);
	assert(cal0() == 0);
	assert(strcmp(out_text, nominal_text) == 0 && err_text[0] == '\0');
}

static void test_stored(void)
{
	static const int16_t accel[3] = {120, -340, 4100};
	static const int16_t gyro[3] = {9, -3, 1};
	struct lsm6dso_cal a, g, nominal;
	struct lsm6dso_cal_status status;

	/* A scale a bit off nominal, misalignment terms and a bias. */
	lsm6dso_cal_nominal(&a, LSM6DSO_CAL_ACCEL, 244);
	a.m[0][0] += a.m[0][0] / 100;
	a.m[0][1] = 3000;
	a.m[2][1] = -2500;
	a.m[2][2] -= a.m[2][2] / 50;
	a.bias[0] = 20 * 256;
	a.bias[2] = -35 * 256 - 64;
	lsm6dso_cal_nominal(&g, LSM6DSO_CAL_GYRO, 70000);
	g.m[1][1] += 1234;
	g.bias[0] = 1958;   /* 7.648 LSB -> 7.65 */
	g.bias[1] = -1088;  /* -4.25 */
	g.bias[2] = -207;   /* -0.8086 -> -0.81 */
	store(LSM6DSO_CAL_ACCEL, 244, &a);
	store(LSM6DSO_CAL_GYRO, 70000, &g);
	/* Stored but not read again yet. */
	assert(cal0() == 0 && strcmp(out_text, nominal_text) == 0);
	assert(cal1("reload") == 0);
	assert(strcmp(out_text,
		      "conversion:\n"
		      "  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)\n"
		      "  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)\n"
		      "  gyro bias: 7.65 -4.25 -0.81 LSB (stored, 0 rejected)\n") == 0);
	assert(err_text[0] == '\0');
	/* The driver's output uses the stored values. */
	set_raw(LSM6DSO_OUTX_L_A, accel[0], accel[1], accel[2]);
	set_raw(LSM6DSO_OUTX_L_G, gyro[0], gyro[1], gyro[2]);
	check_output(SENSOR_CHAN_ACCEL_XYZ, accel, &a);
	check_output(SENSOR_CHAN_GYRO_XYZ, gyro, &g);
	{
		struct sensor_value v[3];
		int32_t out[3];

		lsm6dso_cal_nominal(&nominal, LSM6DSO_CAL_ACCEL, 244);
		lsm6dso_cal_apply(&nominal, accel, out);
		assert(sensor_channel_get(imu(), SENSOR_CHAN_ACCEL_XYZ, v) == 0);
		assert(micro(&v[0]) != lsm6dso_cal_to_micro(out[0]));
	}
	/* Another accel range: its own key, nominal without one; back again. */
	set_range(SENSOR_CHAN_ACCEL_XYZ, 156, 906400); /* 16 g */
	assert(cal0() == 0);
	assert(strstr(out_text, "  accel: nominal (488 ug/LSB, key imu.cal.accel.488)\n"));
	assert(strstr(out_text, "  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)\n"));
	lsm6dso_cal_nominal(&nominal, LSM6DSO_CAL_ACCEL, 488);
	check_output(SENSOR_CHAN_ACCEL_XYZ, accel, &nominal);
	set_range(SENSOR_CHAN_ACCEL_XYZ, 78, 453200); /* 8 g */
	assert(cal0() == 0);
	assert(strstr(out_text, "  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)\n"));
	check_output(SENSOR_CHAN_ACCEL_XYZ, accel, &a);
	/* Gyro range: nominal and no stored bias; back to 2000 dps. */
	set_range(SENSOR_CHAN_GYRO_XYZ, 17, 453293); /* 1000 dps */
	assert(cal0() == 0);
	assert(strstr(out_text, "  gyro:  nominal (35000 udps/LSB, key imu.cal.gyro.35000)\n"
				"  gyro bias: 0.00 0.00 0.00 LSB (none, 0 rejected)\n"));
	set_range(SENSOR_CHAN_GYRO_XYZ, 34, 906585); /* 2000 dps */
	assert(cal0() == 0);
	assert(strstr(out_text, "  gyro bias: 7.65 -4.25 -0.81 LSB (stored, 0 rejected)\n"));

	/* zero at rest, within 1 dps of the stored bias: tracked. */
	set_raw(LSM6DSO_OUTX_L_A, 0, 0, 4100);
	set_raw(LSM6DSO_OUTX_L_G, 10, -5, 2);
	assert(cal1("zero") == 0 && err_text[0] == '\0');
	assert(strstr(out_text, "  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)\n"
				"  gyro bias: 10.00 -5.00 2.00 LSB "
				"(tracked at rest, 1 updates, 0 rejected)\n"));
	lsm6dso_cal_status(imu(), &status);
	assert(status.gyro_bias[0] == 10 * 256);
	/* Far from the stored bias: refused, nothing on stdout. */
	set_raw(LSM6DSO_OUTX_L_G, 3000, 0, 0);
	assert(cal1("zero") == -ERANGE);
	assert(out_text[0] == '\0');
	assert(strcmp(err_text, "The reading is too far from the stored bias\n") == 0);
	assert(cal0() == 0);
	lsm6dso_cal_status(imu(), &status);
	assert(status.gyro_bias_rejected > 0);
	assert(strstr(out_text, "  gyro bias: 10.00 -5.00 2.00 LSB (tracked at rest, 1 updates, "));
	/* An I2C error. */
	nack_at = transfers + 2;
	assert(cal1("zero") == -EIO);
	nack_at = -1;
	assert(out_text[0] == '\0' && strcmp(err_text, "Could not read the sensor: -5\n") == 0);
	/* While a stream owns the sensor. */
	set_odr(6667);
	assert(lsm6dso_fifo_start_locked_driver(imu(), 32, true, true) == 0);
	assert(cal1("zero") == -EBUSY);
	assert(out_text[0] == '\0');
	assert(strcmp(err_text,
		      "The sensor is streaming; the bias is tracked from the stream\n") == 0);
	/* Showing and reloading still work while it streams. */
	assert(cal0() == 0 && strstr(out_text, "tracked at rest"));
	assert(lsm6dso_fifo_stop_locked_driver(imu()) == 0);
	set_odr(0);
	/* reload drops the tracked bias. */
	assert(cal1("reload") == 0);
	assert(strstr(out_text, "  gyro bias: 7.65 -4.25 -0.81 LSB (stored, 0 rejected)\n"));

	/* Positive bias rounding, and half away from zero. */
	g.bias[0] = 128;   /* 0.5 */
	g.bias[1] = 2;     /* 0.0078 -> 0.01 */
	g.bias[2] = -1;    /* -0.0039 -> -0.00 */
	store(LSM6DSO_CAL_GYRO, 70000, &g);
	assert(cal1("reload") == 0);
	assert(strstr(out_text, "  gyro bias: 0.50 0.01 0.00 LSB (stored, 0 rejected)\n"));

	/* A deleted key is nominal again after reload (the Zephyr build's way to
	 * see the uncalibrated values: kv delete, imu cal reload).
	 */
	kv_clear();
	store(LSM6DSO_CAL_GYRO, 70000, &g);
	assert(cal1("reload") == 0);
	assert(strstr(out_text, "  accel: nominal (244 ug/LSB, key imu.cal.accel.244)\n"));
	lsm6dso_cal_nominal(&nominal, LSM6DSO_CAL_ACCEL, 244);
	set_raw(LSM6DSO_OUTX_L_A, accel[0], accel[1], accel[2]);
	check_output(SENSOR_CHAN_ACCEL_XYZ, accel, &nominal);
}

static void test_bad_values(void)
{
	struct lsm6dso_cal a;
	uint8_t blob[IMU_CAL_BLOB_SIZE + 1];

	lsm6dso_cal_nominal(&a, LSM6DSO_CAL_ACCEL, 244);
	a.bias[0] = 256;
	/* Wrong version, kind, sensitivity, reserved bytes, length. */
	for (int c = 0; c < 6; c++) {
		kv_clear();
		encode(blob, LSM6DSO_CAL_ACCEL, 244, &a);
		size_t len = IMU_CAL_BLOB_SIZE;

		switch (c) {
		case 0: blob[0] = 2; break;
		case 1: blob[1] = 1; break;
		case 2: put_le32(blob + 4, 488); break;
		case 3: blob[2] = 1; break;
		case 4: len = IMU_CAL_BLOB_SIZE - 1; break;
		case 5: blob[IMU_CAL_BLOB_SIZE] = 0; len = IMU_CAL_BLOB_SIZE + 1; break;
		}
		kv_put("imu.cal.accel.244", blob, len);
		assert(cal1("reload") == 0);
		assert(strcmp(out_text, nominal_text) == 0);
	}
	/* The right value under the gyro's key is not taken for the gyro. */
	kv_clear();
	encode(blob, LSM6DSO_CAL_ACCEL, 244, &a);
	kv_put("imu.cal.gyro.70000", blob, IMU_CAL_BLOB_SIZE);
	assert(cal1("reload") == 0 && strcmp(out_text, nominal_text) == 0);
	kv_clear();
}

static void test_errors(void)
{
	char *argv[] = {"cal", "zero", "now", NULL};

	assert(cal1("foo") == -EINVAL);
	assert(out_text[0] == '\0' && strcmp(err_text, "usage: imu cal [reload|zero]\n") == 0);
	/* Three words reach the handler only past the shell's count check; the
	 * handler looks at the first.
	 */
	set_raw(LSM6DSO_OUTX_L_G, 0, 0, 0);
	assert(cal(3, argv) == 0);
	/* Not ready. */
	imu()->state->initialized = false;
	assert(imu_cal_init() == -ENODEV);
	assert(cal0() == -ENODEV);
	assert(out_text[0] == '\0' && strcmp(err_text, "IMU not ready\n") == 0);
	assert(cal1("zero") == -ENODEV && strcmp(err_text, "IMU not ready\n") == 0);
	imu()->state->initialized = true;
	assert(imu_cal_init() == 0);
}

int main(void)
{
	assert(lsm6dso_driver_tests() == 0);
	/* The driver test leaves the bus at 100 kHz and the ODRs off. */
	set_range(SENSOR_CHAN_ACCEL_XYZ, 78, 453200);
	set_range(SENSOR_CHAN_GYRO_XYZ, 34, 906585);
	test_nominal();
	test_stored();
	test_bad_values();
	test_errors();
	return 0;
}
