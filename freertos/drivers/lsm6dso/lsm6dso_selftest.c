/* SPDX-License-Identifier: MIT */
/* The board's check of the LSM6DSOX for the "selftest" command (a test
 * feature: built with -DSELFTEST=ON only,
 * freertos/drivers/lsm6dso/lsm6dso.cmake), through Zephyr's sensor API, the
 * driver's FIFO calls (<zephyr/drivers/sensor/lsm6dso.h>) and RTIO on the
 * device lsm6dsox, with the sensor's registers read back on I2C0:
 *
 *   ready   lsm6dsox, i2c0 and gpio0 are ready after the start; the FIFO is
 *           idle (no stream)
 *   fetch   both sensors at 104 Hz for 100 ms, then sensor_sample_fetch()
 *           and sensor_channel_get(): the acceleration's magnitude is
 *           7..12.5 m/s^2 and each rate below 0.35 rad/s (20 dps) with the
 *           board at rest; the die temperature through the driver is
 *           -ENOTSUP (CONFIG_LSM6DSO_ENABLE_TEMP is off in the Zephyr build,
 *           so the driver has no temperature there either), and OUT_TEMP
 *           (0x20) read on I2C0 gives 0..70 C; both sensors off again
 *   stream  the IMU service's setup (I2C0 at fast plus, both sensors at
 *           6667 Hz, the watermark 128 words) and a FIFO stream
 *           (SENSOR_DT_STREAM_IODEV(), sensor_stream()) for 500 ms, then
 *           lsm6dso_fifo_request_stop() and the final completion
 *           (-ECANCELED, within 2 s): the batches decode
 *           (lsm6dso_fifo_buffer_words(), lsm6dso_fifo_next()); 0.7..1.2
 *           times 6667 samples a second of each sensor (plus a watermark);
 *           the FIFO statistics (which start again with each stream) count
 *           the words that came, a new generation, no I2C error, overrun,
 *           short buffer, stop error or fault, and the stream is no longer
 *           active; no completion was
 *           dropped and the submission is released; INT1 (GPIO24) has
 *           interrupted (counted by a second gpio0 callback); and the sensor
 *           is back at rest: GPIO24's interrupt off (PROC0_INTE), FIFO in
 *           bypass mode (FIFO_CTRL4), INT1 not routed (INT1_CTRL), and,
 *           after both sensors are switched off as at the start, their
 *           rates 0 (CTRL1_XL, CTRL2_G)
 *
 * It ends with the sensor as the start leaves it (both off, the watermark as
 * it was) and I2C0 at the standard speed.  The counts and times are printed.
 * Not a host test: it needs the board.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/structs/io_bank0.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/lsm6dso.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

#include "lsm6dso_selftest.h"

#define INT1_GPIO 24

#define REG_FIFO_CTRL4 0x0a /* FIFO_MODE: bits 2..0, 0 is bypass */
#define REG_INT1_CTRL  0x0d /* INT1_FIFO_TH: bit 3 */
#define REG_CTRL1_XL   0x10 /* ODR_XL: bits 7..4 */
#define REG_CTRL2_G    0x11 /* ODR_G: bits 7..4 */
#define REG_OUT_TEMP_L 0x20 /* 16 bits, 256 LSB/C, 0 at 25 C */

#define FETCH_HZ        104
#define FETCH_SETTLE_MS 100
#define STREAM_HZ       6667
#define STREAM_WM       128 /* the IMU service's default watermark */
#define STREAM_MS       500
#define STOP_MS         2000

/* The IMU service's stream: one multishot read of the FIFO watermark
 * trigger, in a context of 4 submissions, 16 completions and 128 blocks of
 * 64 bytes (common/imu/imu_stream.c).
 */
SENSOR_DT_STREAM_IODEV(st_iodev, DT_NODELABEL(lsm6dsox),
		       {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE});
RTIO_DEFINE_WITH_MEMPOOL(st_rtio, 4, 16, 128, 64, 4);

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
static const struct device *const bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct i2c_dt_spec imu_i2c = {
	.bus = DEVICE_DT_GET(DT_NODELABEL(i2c0)),
	.addr = 0x6a,
};

static int failures;

static void check_eq(long long got, long long want, int line, const char *what)
{
	if (got != want) {
		printf("  FAIL line %d: %s: %lld (0x%llx), expected %lld (0x%llx)\n", line, what,
		       got, (unsigned long long)got, want, (unsigned long long)want);
		failures++;
	}
}

static void check_range(long long got, long long min, long long max, int line, const char *what)
{
	if (got < min || got > max) {
		printf("  FAIL line %d: %s: %lld, expected %lld..%lld\n", line, what, got, min,
		       max);
		failures++;
	}
}

#define CHECK(cond)                check_eq((cond) ? 1 : 0, 1, __LINE__, #cond)
#define CHECK_EQ(got, want)        check_eq((long long)(got), (long long)(want), __LINE__, #got)
#define CHECK_RANGE(got, min, max) check_range((long long)(got), (min), (max), __LINE__, #got)

/* A value in thousandths, rounded, for the printed lines (no floating-point
 * printf needed).
 */
static long milli(double v)
{
	return (long)(v * 1000.0 + (v >= 0.0 ? 0.5 : -0.5));
}

static void print_milli(const char *name, double v)
{
	long m = milli(v);
	unsigned long a = (unsigned long)(m < 0 ? -m : m);

	printf(" %s %s%lu.%03lu", name, m < 0 ? "-" : "", a / 1000, a % 1000);
}

static int set_odr(int32_t hz)
{
	struct sensor_value odr = {.val1 = hz};
	int ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	if (ret == 0) {
		ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
				      &odr);
	}
	return ret;
}

static int read_reg(uint8_t reg)
{
	uint8_t value = 0;
	int ret = i2c_burst_read_dt(&imu_i2c, reg, &value, 1);

	return ret != 0 ? ret : value;
}

static void test_ready(void)
{
	struct lsm6dso_fifo_stats stats;

	CHECK(device_is_ready(imu));
	CHECK(device_is_ready(bus));
	CHECK(device_is_ready(gpio0));
	lsm6dso_fifo_get_stats(imu, &stats);
	CHECK(!stats.active);
}

static void test_fetch(void)
{
	struct sensor_value accel[3], gyro[3];
	double a2 = 0.0, wmax = 0.0;
	uint8_t raw[2] = {0};
	int16_t temp;

	CHECK_EQ(set_odr(FETCH_HZ), 0);
	k_msleep(FETCH_SETTLE_MS);
	CHECK_EQ(sensor_sample_fetch(imu), 0);
	CHECK_EQ(sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel), 0);
	CHECK_EQ(sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro), 0);
	CHECK_EQ(sensor_sample_fetch_chan(imu, SENSOR_CHAN_DIE_TEMP), -ENOTSUP);
	CHECK_EQ(i2c_burst_read_dt(&imu_i2c, REG_OUT_TEMP_L, raw, sizeof(raw)), 0);
	CHECK_EQ(set_odr(0), 0);

	printf("  accel m/s^2");
	for (int i = 0; i < 3; i++) {
		double v = sensor_value_to_double(&accel[i]);

		a2 += v * v;
		print_milli(i == 0 ? "x" : i == 1 ? "y" : "z", v);
	}
	printf("\n  gyro rad/s");
	for (int i = 0; i < 3; i++) {
		double v = sensor_value_to_double(&gyro[i]);

		wmax = v > wmax ? v : -v > wmax ? -v : wmax;
		print_milli(i == 0 ? "x" : i == 1 ? "y" : "z", v);
	}
	temp = (int16_t)(raw[0] | (raw[1] << 8));
	printf("\n  OUT_TEMP 0x%04x:", (unsigned int)(uint16_t)temp);
	print_milli("C", 25.0 + temp / 256.0);
	printf("\n");

	/* |a|^2 in (m/s^2)^2: 7^2..12.5^2 */
	CHECK_RANGE(milli(a2), 49000, 156250);
	CHECK_RANGE(milli(wmax), 0, 350);
	CHECK_RANGE(milli(25.0 + temp / 256.0), 0, 70000);
}

/* The stream ---------------------------------------------------------------*/

struct stream_run {
	uint32_t batches;
	uint32_t bad_batches;
	uint32_t bufferless;
	uint64_t words;
	uint32_t accel;
	uint32_t gyro;
	bool terminal;
	int terminal_rc;
};

static atomic_t int1_count;

static void int1_counted(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	atomic_inc(&int1_count);
}

static void count_batch(struct stream_run *run, const uint8_t *buffer, uint32_t length)
{
	struct lsm6dso_fifo_sample sample;
	const uint8_t *words;
	size_t size, cursor = 0;

	if (lsm6dso_fifo_buffer_words(buffer, length, &words, &size) != 0) {
		run->bad_batches++;
		return;
	}
	run->batches++;
	run->words += size / LSM6DSO_FIFO_WORD_SIZE;
	while (lsm6dso_fifo_next(words, size, &cursor, &sample) > 0) {
		run->accel += sample.kind == LSM6DSO_FIFO_ACCEL;
		run->gyro += sample.kind == LSM6DSO_FIFO_GYRO;
	}
}

/* As the IMU service's consume_completions(). */
static void consume(struct stream_run *run)
{
	struct rtio_cqe *cqe;

	while ((cqe = rtio_cqe_consume(&st_rtio)) != NULL) {
		uint8_t *buffer = NULL;
		uint32_t length = 0;
		int rc = cqe->result;
		int buffer_rc = rtio_cqe_get_mempool_buffer(&st_rtio, cqe, &buffer, &length);

		rtio_cqe_release(&st_rtio, cqe);
		if (rc < 0) {
			run->terminal = true;
			run->terminal_rc = rc;
		} else if (buffer_rc != 0 || buffer == NULL || length == 0) {
			run->bufferless++;
		} else {
			count_batch(run, buffer, length);
		}
		if (buffer != NULL) {
			rtio_release_buffer(&st_rtio, buffer, length);
		}
	}
}

/* Waits for completions until the deadline (k_uptime_get()) or, with
 * until_terminal, the final one.
 */
static void wait_completions(struct stream_run *run, int64_t deadline, bool until_terminal)
{
	for (;;) {
		struct k_poll_event event;
		int64_t remaining;

		consume(run);
		remaining = deadline - k_uptime_get();
		if ((until_terminal && run->terminal) || remaining <= 0) {
			break;
		}
		k_poll_event_init(&event, K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
				  st_rtio.consume_sem);
		(void)k_poll(&event, 1, K_MSEC(remaining));
	}
	consume(run);
}

static void test_stream(void)
{
	struct lsm6dso_fifo_stats before, after;
	struct gpio_callback int1_cb;
	struct stream_run run = {0};
	rtio_sqe_handle_t handle = RTIO_SQE_HANDLE_INVALID;
	uint16_t watermark = lsm6dso_fifo_get_watermark(imu);
	uint32_t dropped = (uint32_t)atomic_get(&st_rtio.xcqcnt);
	int64_t start, stop, end;
	long long expected;
	int ret;

	/* Completions left over from an earlier run are not ours. */
	consume(&run);
	run = (struct stream_run){0};

	CHECK_EQ(i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)), 0);
	CHECK_EQ(set_odr(STREAM_HZ), 0);
	CHECK_EQ(lsm6dso_fifo_set_watermark(imu, STREAM_WM), 0);

	atomic_clear(&int1_count);
	gpio_init_callback(&int1_cb, int1_counted, BIT(INT1_GPIO));
	CHECK_EQ(gpio_add_callback(gpio0, &int1_cb), 0);

	lsm6dso_fifo_get_stats(imu, &before);
	start = k_uptime_get();
	ret = sensor_stream(&st_iodev, &st_rtio, NULL, &handle);
	CHECK_EQ(ret, 0);
	if (ret == 0) {
		wait_completions(&run, start + STREAM_MS, false);
		CHECK(!run.terminal);
		stop = k_uptime_get();
		CHECK_EQ(lsm6dso_fifo_request_stop(imu, &st_rtio, handle), 0);
		wait_completions(&run, stop + STOP_MS, true);
		end = k_uptime_get();
	} else {
		stop = end = start;
	}
	lsm6dso_fifo_get_stats(imu, &after);

	CHECK_EQ(gpio_remove_callback(gpio0, &int1_cb), 0);
	CHECK_EQ(set_odr(0), 0);
	CHECK_EQ(lsm6dso_fifo_set_watermark(imu, watermark), 0);

	printf("  %lld ms, stop %lld ms: %u batches, %llu words, accel %u, gyro %u, INT1 %ld, "
	       "end %d\n",
	       (long long)(stop - start), (long long)(end - stop), (unsigned int)run.batches,
	       (unsigned long long)run.words, (unsigned int)run.accel, (unsigned int)run.gyro,
	       (long)atomic_get(&int1_count), run.terminal_rc);
	printf("  FIFO: generation %u, words_read %llu, transfers %u, max transfer %llu us\n",
	       (unsigned int)after.generation, (unsigned long long)after.words_read,
	       (unsigned int)after.transfers,
	       (unsigned long long)(after.max_transfer_time_ns / 1000U));

	CHECK(run.terminal);
	CHECK_EQ(run.terminal_rc, -ECANCELED);
	CHECK_EQ(run.bad_batches, 0);
	CHECK_EQ(run.bufferless, 0);
	CHECK(run.batches > 0);
	expected = (long long)STREAM_HZ * (stop - start) / 1000;
	CHECK_RANGE(run.accel, expected * 7 / 10, expected * 12 / 10 + STREAM_WM);
	CHECK_RANGE(run.gyro, expected * 7 / 10, expected * 12 / 10 + STREAM_WM);
	CHECK(atomic_get(&int1_count) > 0);
	CHECK_EQ(after.generation, before.generation + 1);
	CHECK_EQ(after.words_read, run.words); /* the statistics start with the stream */
	CHECK_EQ(after.i2c_errors, 0);
	CHECK_EQ(after.overrun_observations, 0);
	CHECK_EQ(after.buffer_too_small, 0);
	CHECK_EQ(after.stop_error, 0);
	CHECK(!after.fault);
	CHECK(!after.active);
	CHECK_EQ((uint32_t)atomic_get(&st_rtio.xcqcnt) - dropped, 0);
	CHECK(rtio_iodev_sqe_from_handle(&st_rtio, handle) == NULL);

	/* At rest again. */
	CHECK_EQ(io_bank0_hw->proc0_irq_ctrl.inte[INT1_GPIO / 8] >> (4 * (INT1_GPIO % 8)) & 0xfu,
		 0);
	CHECK_EQ(read_reg(REG_FIFO_CTRL4) & 0x07, 0);
	CHECK_EQ(read_reg(REG_INT1_CTRL) & 0x08, 0);
	CHECK_EQ(read_reg(REG_CTRL1_XL) & 0xf0, 0);
	CHECK_EQ(read_reg(REG_CTRL2_G) & 0xf0, 0);
}

static const struct {
	const char *name;
	void (*fn)(void);
} tests[] = {
	/* clang-format off */
	{"ready", test_ready},
	{"fetch", test_fetch},
	{"stream", test_stream},
	/* clang-format on */
};

int lsm6dso_selftest_run(void)
{
	size_t count = sizeof(tests) / sizeof(tests[0]);
	int failed = 0;

	for (size_t i = 0; i < count; i++) {
		failures = 0;
		printf("lsm6dso %s\n", tests[i].name);
		if (i > 0 && !device_is_ready(imu)) {
			printf("  skipped (lsm6dsox not ready)\n");
			failed++;
			continue;
		}
		tests[i].fn();
		printf("  %s\n", failures == 0 ? "ok" : "FAILED");
		failed += failures != 0;
	}
	(void)i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
	printf("selftest: lsm6dso: %d of %u tests failed\n", failed, (unsigned int)count);
	return failed;
}
