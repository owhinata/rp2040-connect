/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the ported LSM6DSO driver (nuttx/drivers/lsm6dso/lsm6dso.c)
 * on ST's real lsm6dso_reg.c, the stmemsc glue and the Zephyr
 * compatibility layer (compat/zephyr_i2c.c, zephyr_device.c), against a
 * fake LSM6DSOX behind a fake NuttX I2C master (register banks,
 * auto-increment, software reset, FIFO, timestamp) and fake kernel
 * services.  It checks what the driver must do as the Zephyr build's
 * drivers/lsm6dso/lsm6dso.c does:
 *
 * - init: the log lines and their order, the register writes in order
 *   (user bank, I3C off, software reset and its 100 us wait, power modes,
 *   ranges, ODRs, FIFO bypass, BDU), the registers after it, the
 *   sensitivities; the error paths and their lines; device_is_ready().
 * - every transfer: address 0x6a, the bus speed (100 kHz from the
 *   devicetree, then what i2c_configure() sets), a register read as
 *   write (NOSTOP) + read, a register write as one message.
 * - sample_fetch()/channel_get(): the registers read, the conversion
 *   (signs of val1/val2, one axis or three) against lsm6dso_cal.c and
 *   against the physical value.
 * - attr_set(): ODR and full scale, the rounding of m/s^2 to g and of
 *   rad/s to dps, the refused values and channels, -EBUSY while the FIFO
 *   is owned.
 * - the FIFO functions: start (registers, stats), read (status, burst,
 *   counters), pause, stop (bypass), and lsm6dso_timestamp_read().
 * - lsm6dso_cal_zero(): the 104 Hz run, the 200 ms settling, switching
 *   the sensors off again.
 * - the FIFO stream (lsm6dso_stream.c, lsm6dso_fifo_irq.c, lsm6dso_compat.c
 *   and the decoder, on the compatibility layer's RTIO, work queues and
 *   gpio0, with the fake kernel of tests/zephyr_fakes and a fake INT1 that
 *   follows the fake sensor's watermark flag): the stream's start
 *   (registers, INT1 edge interrupt), batches on the INT1 edge and on the
 *   20 ms poll, a second stream refused (-EBUSY), another trigger refused
 *   (-ENOTSUP), the graceful stop (pause, the last partial batch, then
 *   -ECANCELED and the registers back), a cancellation, an overrun
 *   (-EOVERFLOW), a single read through the RTIO work queue (the
 *   fallback) and the decoders.
 */

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/i2c/i2c_master.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

#include "lsm6dso.h"
#include "nano_lsm6dso.h"
#include "lsm6dso_attributes.h"
#include "lsm6dso_fifo.h"
#include "lsm6dso_fifo_hw.h"
#include "lsm6dso_fifo_stats.h"
#include "lsm6dso_fifo_decoder.h"
#include "nano_gpio.h"
#include "zephyr_kernel_host.h"

/* Fake log ---------------------------------------------------------------*/

static char log_text[8192];

void test_log(int level, const char *module, const char *fmt, ...)
{
	static const char *const names[] = {"", "err", "wrn", "inf", "dbg"};
	char line[256];
	va_list ap;
	int n;

	n = snprintf(line, sizeof(line), "<%s> %s: ", names[level], module);
	va_start(ap, fmt);
	vsnprintf(line + n, sizeof(line) - n, fmt, ap);
	va_end(ap);
	assert(strlen(log_text) + strlen(line) + 2 < sizeof(log_text));
	strcat(log_text, line);
	strcat(log_text, "\n");
}

/* Fake kernel services ---------------------------------------------------*/

static char events[16384];
static int64_t slept_us;

static void event(const char *fmt, ...)
{
	char e[64];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(e, sizeof(e), fmt, ap);
	va_end(ap);
	assert(strlen(events) + strlen(e) + 2 < sizeof(events));
	strcat(events, e);
	strcat(events, " ");
}

void lsm6dso_os_mutex_init(lsm6dso_mutex_t *mutex)
{
	mutex->depth = 0;
}

void lsm6dso_os_mutex_lock(lsm6dso_mutex_t *mutex)
{
	mutex->depth++;
}

void lsm6dso_os_mutex_unlock(lsm6dso_mutex_t *mutex)
{
	assert(mutex->depth > 0);
	mutex->depth--;
}

lsm6dso_spinlock_key_t lsm6dso_os_spin_lock(lsm6dso_spinlock_t *lock)
{
	assert(!lock->locked);
	lock->locked = 1;
	return 7;
}

void lsm6dso_os_spin_unlock(lsm6dso_spinlock_t *lock, lsm6dso_spinlock_key_t key)
{
	assert(lock->locked && key == 7);
	lock->locked = 0;
}

void lsm6dso_os_busy_wait_us(uint32_t us)
{
	event("busy%u", us);
}

void lsm6dso_os_sleep_ms(int32_t ms)
{
	event("sleep%dms", ms);
	slept_us += (int64_t)ms * 1000;
}

void lsm6dso_os_sleep_us(int32_t us)
{
	slept_us += us;
}

static uint32_t cycles;

uint32_t lsm6dso_os_cycle_get_32(void)
{
	return cycles += 250;
}

uint64_t lsm6dso_os_cyc_to_ns_floor64(uint32_t c)
{
	return (uint64_t)c * 1000U;
}

/* Fake LSM6DSOX ------------------------------------------------------------*/

enum { USER, SHUB, EMB };
static uint8_t regs[3][256];
static uint8_t fifo_data[LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE];
static size_t fifo_words;
static bool fifo_overrun;
static uint32_t timestamp, timestamp_step = 3;
static int nack_at = -1, transfers, bad_who_am_i;
static uint32_t expect_frequency = 100000;
static bool log_writes;

static int bank(void)
{
	uint8_t access = regs[USER][LSM6DSO_FUNC_CFG_ACCESS];

	return access & 0x80 ? EMB : access & 0x40 ? SHUB : USER;
}

static void power_on_defaults(void)
{
	uint8_t keep9 = regs[USER][LSM6DSO_CTRL9_XL] & 0x02; /* I3C_disable survives */

	memset(regs[USER], 0, sizeof(regs[USER]));
	regs[USER][LSM6DSO_PIN_CTRL] = 0x3f;
	regs[USER][LSM6DSO_WHO_AM_I] = bad_who_am_i ? 0x6b : 0x6c;
	regs[USER][LSM6DSO_CTRL3_C] = 0x04;
	regs[USER][LSM6DSO_CTRL9_XL] = 0xe0 | keep9;
	regs[USER][LSM6DSO_INTERNAL_FREQ_FINE] = 0xfd;
}

static uint8_t read_byte(int b, uint8_t reg)
{
	if (reg == LSM6DSO_FUNC_CFG_ACCESS) {
		b = USER; /* the same register in every bank */
	}
	if (b == USER && reg >= LSM6DSO_TIMESTAMP0 && reg <= LSM6DSO_TIMESTAMP3) {
		return (uint8_t)(timestamp >> (8 * (reg - LSM6DSO_TIMESTAMP0)));
	}
	if (b == USER && reg == LSM6DSO_FIFO_STATUS1) {
		return (uint8_t)fifo_words;
	}
	if (b == USER && reg == LSM6DSO_FIFO_STATUS2) {
		uint16_t wtm = regs[USER][LSM6DSO_FIFO_CTRL1] |
			       (uint16_t)(regs[USER][LSM6DSO_FIFO_CTRL2] & 1) << 8;

		return (uint8_t)((fifo_words >> 8) & 3) | (fifo_words >= wtm ? 0x80 : 0) |
		       (fifo_overrun ? 0x48 : 0);
	}
	return regs[b][reg];
}

static void write_byte(int b, uint8_t reg, uint8_t value)
{
	if (reg == LSM6DSO_FUNC_CFG_ACCESS) {
		b = USER; /* the same register in every bank */
	}
	if (log_writes && !(b == USER && reg == LSM6DSO_FUNC_CFG_ACCESS)) {
		event("%c%02x=%02x", "USE"[b], reg, value);
	}
	if (b == USER && reg == LSM6DSO_CTRL3_C && (value & 1)) {
		power_on_defaults(); /* SW_RESET, which clears itself */
		return;
	}
	if (b == USER && reg == LSM6DSO_WHO_AM_I) {
		return;
	}
	regs[b][reg] = value;
}

static void update_int1(void);

static int fake_transfer_regs(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);

static int fake_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)
{
	int ret = fake_transfer_regs(dev, msgs, count);

	update_int1();
	return ret;
}

static int fake_transfer_regs(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)
{
	(void)dev;
	assert(count == 1 || count == 2);
	for (int i = 0; i < count; i++) {
		assert(msgs[i].addr == 0x6a);
		assert(msgs[i].frequency == expect_frequency);
	}
	if (transfers++ == nack_at) {
		return -EIO;
	}
	if (count == 1) {
		/* i2c_write_dt(): one write message with STOP, register then data. */
		uint8_t reg = msgs[0].buffer[0];
		int b = bank();

		assert(msgs[0].flags == 0 && msgs[0].length >= 2);
		for (ssize_t i = 1; i < msgs[0].length; i++) {
			write_byte(i == 1 ? b : bank(), reg, msgs[0].buffer[i]);
			if (regs[USER][LSM6DSO_CTRL3_C] & 0x04) {
				reg++;
			}
		}
		return 0;
	}
	/* i2c_burst_read_dt(): the register (no STOP), then a read with RESTART. */
	assert(msgs[0].flags == I2C_M_NOSTOP && msgs[0].length == 1);
	assert(msgs[1].flags == I2C_M_READ && msgs[1].length >= 1);
	uint8_t reg = msgs[0].buffer[0];
	int b = bank();

	if (b == USER && reg == LSM6DSO_FIFO_DATA_OUT_TAG) {
		size_t want = (size_t)msgs[1].length;

		assert(want % LSM6DSO_FIFO_WORD_SIZE == 0);
		assert(want <= fifo_words * LSM6DSO_FIFO_WORD_SIZE);
		memcpy(msgs[1].buffer, fifo_data, want);
		memmove(fifo_data, fifo_data + want, fifo_words * LSM6DSO_FIFO_WORD_SIZE - want);
		fifo_words -= want / LSM6DSO_FIFO_WORD_SIZE;
		return 0;
	}
	for (ssize_t i = 0; i < msgs[1].length; i++) {
		msgs[1].buffer[i] = read_byte(b, reg);
		if (regs[USER][LSM6DSO_CTRL3_C] & 0x04) {
			reg++;
		}
	}
	if (b == USER && msgs[0].buffer[0] == LSM6DSO_TIMESTAMP0) {
		timestamp += timestamp_step;
	}
	return 0;
}

static const struct i2c_ops_s fake_ops = {.transfer = fake_transfer};
static struct i2c_master_s fake_master = {.ops = &fake_ops};

/* Fake INT1 (GPIO24) and the board's GPIO bank ---------------------------*/

/* INT1 is high while the FIFO threshold is routed to it (INT1_CTRL
 * INT1_FIFO_TH) and the FIFO holds the watermark or more.  A rising edge
 * calls the compatibility layer's handler if the edge is enabled (as the
 * bank interrupt would), unless the test hides it.
 */
static bool int1_level, int1_hide_edges;
static uint32_t int1_events;
static int int1_configured, int1_edges;

int nano_gpio_configure_input(unsigned int pin, bool pull_up, bool pull_down)
{
	assert(pin == 24 && !pull_up && !pull_down);
	int1_configured++;
	return 0;
}

uint32_t nano_gpio_get_all(void)
{
	return int1_level ? 1U << 24 : 0;
}

int nano_gpio_irq_configure(unsigned int pin, uint32_t events)
{
	assert(pin == 24);
	int1_events = events;
	return 0;
}

static void update_int1(void)
{
	uint16_t wtm = regs[USER][LSM6DSO_FIFO_CTRL1] |
		       (uint16_t)(regs[USER][LSM6DSO_FIFO_CTRL2] & 1) << 8;
	bool level = (regs[USER][LSM6DSO_INT1_CTRL] & 0x08) && wtm && fifo_words >= wtm;
	bool rise = level && !int1_level;

	int1_level = level;
	if (rise && (int1_events & NANO_GPIO_IRQ_EDGE_RISE) && !int1_hide_edges) {
		int1_edges++;
		nano_gpio_irq_handler(24);
	}
}

/* The sensor clock: 1 ms per reading. */
static uint64_t clock_cycles = 5000000;

int sensor_clock_get_cycles(uint64_t *c)
{
	*c = clock_cycles += 1000;
	return 0;
}

uint64_t sensor_clock_cycles_to_ns(uint64_t c)
{
	return c * 1000U;
}

/* Helpers ------------------------------------------------------------------*/

static const struct device *imu(void)
{
	return DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
}

static void reset_logs(void)
{
	log_text[0] = '\0';
	events[0] = '\0';
}

static void set_raw(uint8_t reg, int16_t x, int16_t y, int16_t z)
{
	int16_t v[3] = {x, y, z};

	for (int i = 0; i < 3; i++) {
		regs[USER][reg + 2 * i] = (uint8_t)v[i];
		regs[USER][reg + 2 * i + 1] = (uint8_t)((uint16_t)v[i] >> 8);
	}
}

static int64_t micro(const struct sensor_value *v)
{
	/* val1 and val2 carry the same sign. */
	assert(!(v->val1 > 0 && v->val2 < 0) && !(v->val1 < 0 && v->val2 > 0));
	assert(v->val2 > -1000000 && v->val2 < 1000000);
	return (int64_t)v->val1 * 1000000 + v->val2;
}

/* The conversion of the copy of lsm6dso_cal.c, as channel_get must give. */
static int64_t expected_micro(enum lsm6dso_cal_kind kind, uint32_t gain, const int16_t raw[3],
			      int axis)
{
	struct lsm6dso_cal cal;
	int32_t out[3];

	lsm6dso_cal_nominal(&cal, kind, gain);
	lsm6dso_cal_apply(&cal, raw, out);
	return lsm6dso_cal_to_micro(out[axis]);
}

static void check_init_ok(void)
{
	assert(strcmp(log_text, "<inf> LSM6DSO: Initialize device lsm6dsox@6a\n"
				"<inf> LSM6DSO: chip id 0x6c\n") == 0);
}

/* Tests ----------------------------------------------------------------------*/

static void test_bus(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6dsox)));

	assert(!device_is_ready(bus) && !device_is_ready(imu()));
	assert(i2c_configure(bus, I2C_SPEED_SET(I2C_SPEED_FAST)) == -ENODEV);
	assert(i2c_compat_bind(bus, NULL, 100000) == -EINVAL);
	assert(i2c_compat_bind(bus, &fake_master, 200000) == -EINVAL);
	assert(i2c_compat_bind(bus, &fake_master, 100000) == 0);
	assert(device_is_ready(bus) && i2c_compat_frequency(bus) == 100000);
	assert(strcmp(bus->name, "i2c@40044000") == 0);
	assert(strcmp(imu()->name, "lsm6dsox@6a") == 0);
	/* Zephyr's speed classes, not NuttX's Hz values of the same names. */
	assert(I2C_SPEED_FAST_PLUS == 3 && I2C_SPEED_SET(I2C_SPEED_FAST_PLUS) == 6);
	assert(i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_HIGH)) == -EINVAL);
	assert(i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_ULTRA)) == -EINVAL);
	assert(i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_DT)) == -EINVAL);
	assert(i2c_compat_frequency(bus) == 100000);
}

static void test_init_errors(void)
{
	/* The first transfer (the bank) fails. */
	power_on_defaults();
	reset_logs();
	transfers = 0;
	nack_at = 0;
	assert(nano_lsm6dso_initialize(&fake_master) == -EIO);
	assert(!device_is_ready(imu()) && imu()->state->init_res == EIO);
	assert(strcmp(log_text, "<inf> LSM6DSO: Initialize device lsm6dsox@6a\n"
				"<err> LSM6DSO: Failed to set user bank\n"
				"<err> LSM6DSO: failed to initialize chip\n") == 0);
	/* WHO_AM_I read fails. */
	reset_logs();
	transfers = 0;
	nack_at = 1;
	assert(nano_lsm6dso_initialize(&fake_master) == -EIO);
	assert(strstr(log_text, "<err> LSM6DSO: Failed reading chip id\n"
				"<err> LSM6DSO: failed to initialize chip\n"));
	/* A wrong chip. */
	nack_at = -1;
	bad_who_am_i = 1;
	power_on_defaults();
	reset_logs();
	assert(nano_lsm6dso_initialize(&fake_master) == -EIO);
	assert(strcmp(log_text, "<inf> LSM6DSO: Initialize device lsm6dsox@6a\n"
				"<inf> LSM6DSO: chip id 0x6b\n"
				"<err> LSM6DSO: Invalid chip id 0x6b\n"
				"<err> LSM6DSO: failed to initialize chip\n") == 0);
	assert(!device_is_ready(imu()));
	bad_who_am_i = 0;
	/* Each later step: every failing transfer ends init with -EIO and a line. */
	for (int at = 2; at < 200; at++) {
		power_on_defaults();
		reset_logs();
		transfers = 0;
		nack_at = at;
		int ret = nano_lsm6dso_initialize(&fake_master);

		if (ret == 0) {
			/* Past the last transfer of init. */
			assert(at > 10);
			break;
		}
		assert(ret == -EIO);
		assert(strstr(log_text, "<err> LSM6DSO: failed to initialize chip\n"));
		assert(!device_is_ready(imu()));
	}
	nack_at = -1;
}

static void test_init(void)
{
	struct lsm6dso_cal_status status;

	/* Leftovers of an earlier run: the reset must clear them. */
	power_on_defaults();
	regs[USER][LSM6DSO_FUNC_CFG_ACCESS] = 0x80;  /* left in the embedded bank */
	regs[USER][LSM6DSO_CTRL1_XL] = 0xa0;
	regs[USER][LSM6DSO_CTRL10_C] = 0x20;
	regs[USER][LSM6DSO_INT1_CTRL] = 0x08;
	regs[USER][LSM6DSO_FIFO_CTRL4] = 0x46;
	reset_logs();
	log_writes = true;
	assert(nano_lsm6dso_initialize(&fake_master) == 0);
	log_writes = false;
	check_init_ok();
	assert(device_is_ready(imu()));
	/* The writes in order (bank switches left out; xl/gy ODR setters write
	 * their register after reading the embedded-function state).
	 */
	assert(strcmp(events,
		      "U18=e2 U62=00 U12=05 busy100 U14=00 U15=00 U10=0c U10=0c "
		      "U16=00 U11=0c U11=0c U0a=00 U12=44 ") == 0);
	/* The registers after init (user bank). */
	assert(regs[USER][LSM6DSO_FUNC_CFG_ACCESS] == 0x00);
	assert(regs[USER][LSM6DSO_CTRL1_XL] == 0x0c);  /* 8 g, ODR off */
	assert(regs[USER][LSM6DSO_CTRL2_G] == 0x0c);   /* 2000 dps, ODR off */
	assert(regs[USER][LSM6DSO_CTRL3_C] == 0x44);   /* BDU, IF_INC */
	assert(regs[USER][LSM6DSO_CTRL4_C] == 0x00);
	assert(regs[USER][LSM6DSO_CTRL5_C] == 0x00);
	assert(regs[USER][LSM6DSO_CTRL6_C] == 0x00);
	assert(regs[USER][LSM6DSO_CTRL7_G] == 0x00);
	assert(regs[USER][LSM6DSO_CTRL8_XL] == 0x00);
	assert(regs[USER][LSM6DSO_CTRL9_XL] == 0xe2);  /* DEN_XYZ, I3C_disable */
	assert(regs[USER][LSM6DSO_CTRL10_C] == 0x00);
	for (unsigned int r = LSM6DSO_FIFO_CTRL1; r <= LSM6DSO_INT2_CTRL; r++) {
		assert(regs[USER][r] == 0x00);
	}
	assert(regs[USER][LSM6DSO_I3C_BUS_AVB] == 0x00);
	lsm6dso_cal_status(imu(), &status);
	assert(status.accel_gain == 244 && status.gyro_gain == 70000);
	assert(!status.accel_calibrated && !status.gyro_calibrated);
}

static void test_fetch(void)
{
	static const int16_t accel[3] = {123, -4567, 4100};
	static const int16_t gyro[3] = {-1, 0, 30000};
	struct sensor_value v[3];

	set_raw(LSM6DSO_OUTX_L_A, accel[0], accel[1], accel[2]);
	set_raw(LSM6DSO_OUTX_L_G, gyro[0], gyro[1], gyro[2]);
	assert(sensor_sample_fetch(imu()) == 0);
	assert(sensor_channel_get(imu(), SENSOR_CHAN_ACCEL_XYZ, v) == 0);
	for (int i = 0; i < 3; i++) {
		int64_t m = micro(&v[i]);
		double exact = accel[i] * 244e-6 * 9.80665 * 1e6;

		assert(m == expected_micro(LSM6DSO_CAL_ACCEL, 244, accel, i));
		assert(fabs((double)m - exact) <= fabs(exact) * 1e-5 + 2);
	}
	/* 4100 LSB at 0.244 mg: 1.0004 g, 9.8106 m/s^2; -4567 LSB: -10.928 m/s^2. */
	assert(v[2].val1 == 9 && v[2].val2 / 1000 == 810);
	assert(v[1].val1 == -10 && v[1].val2 / 1000 == -928);
	memset(v, 0x55, sizeof(v));
	assert(sensor_channel_get(imu(), SENSOR_CHAN_ACCEL_Y, v) == 0);
	assert(micro(&v[0]) == expected_micro(LSM6DSO_CAL_ACCEL, 244, accel, 1));
	assert(v[1].val1 == 0x55555555); /* one value only */
	assert(sensor_channel_get(imu(), SENSOR_CHAN_GYRO_XYZ, v) == 0);
	for (int i = 0; i < 3; i++) {
		int64_t m = micro(&v[i]);
		double exact = gyro[i] * 70000e-6 * M_PI / 180 * 1e6;

		assert(m == expected_micro(LSM6DSO_CAL_GYRO, 70000, gyro, i));
		assert(fabs((double)m - exact) <= fabs(exact) * 1e-5 + 2);
	}
	/* -1 LSB at 70 mdps: -0.0012217 rad/s, val1 0 and val2 negative. */
	assert(v[0].val1 == 0 && v[0].val2 < 0 && v[0].val2 > -1300);
	assert(sensor_channel_get(imu(), SENSOR_CHAN_GYRO_Z, v) == 0);
	assert(v[0].val1 == 36); /* 2100 dps = 36.65 rad/s */
	assert(sensor_channel_get(imu(), SENSOR_CHAN_DIE_TEMP, v) == -ENOTSUP);
	assert(sensor_channel_get(imu(), SENSOR_CHAN_MAGN_XYZ, v) == -ENOTSUP);
	/* Per channel fetches. */
	set_raw(LSM6DSO_OUTX_L_A, 1, 2, 3);
	assert(sensor_sample_fetch_chan(imu(), SENSOR_CHAN_GYRO_XYZ) == 0);
	assert(sensor_channel_get(imu(), SENSOR_CHAN_ACCEL_X, v) == 0);
	assert(micro(&v[0]) == expected_micro(LSM6DSO_CAL_ACCEL, 244, accel, 0));
	assert(sensor_sample_fetch_chan(imu(), SENSOR_CHAN_ACCEL_XYZ) == 0);
	assert(sensor_channel_get(imu(), SENSOR_CHAN_ACCEL_X, v) == 0);
	assert(v[0].val1 == 0 && v[0].val2 / 10 == 239); /* 1 LSB = 2393 um/s^2 */
	assert(sensor_sample_fetch_chan(imu(), SENSOR_CHAN_DIE_TEMP) == -ENOTSUP);
	assert(sensor_sample_fetch_chan(imu(), SENSOR_CHAN_ACCEL_X) == -ENOTSUP);
	/* An I2C error on the accel or the gyro read. */
	nack_at = transfers;
	assert(sensor_sample_fetch(imu()) == -EIO);
	nack_at = transfers + 1;
	assert(sensor_sample_fetch(imu()) == -EIO);
	nack_at = -1;
	/* 100 in a row. */
	for (int i = 0; i < 100; i++) {
		assert(sensor_sample_fetch(imu()) == 0);
	}
}

static void test_attr(void)
{
	struct sensor_value v = {0};
	struct lsm6dso_cal_status status;

	reset_logs();
	/* ODR: the first table entry at or above the frequency. */
	static const struct { int32_t hz; uint8_t odr; } odrs[] = {
		{0, 0}, {1, 1}, {12, 1}, {13, 2}, {26, 2}, {52, 3}, {100, 4}, {104, 4},
		{105, 5}, {209, 6}, {416, 6}, {417, 6}, {833, 7}, {1667, 8}, {3333, 9},
		{6667, 10},
	};
	for (size_t i = 0; i < ARRAY_SIZE(odrs); i++) {
		v.val1 = odrs[i].hz;
		assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ,
				       SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
		assert(regs[USER][LSM6DSO_CTRL1_XL] == (uint8_t)(odrs[i].odr << 4 | 0x0c));
		assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ,
				       SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
		assert(regs[USER][LSM6DSO_CTRL2_G] == (uint8_t)(odrs[i].odr << 4 | 0x0c));
	}
	v.val1 = 6668;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) ==
	       -EINVAL);
	assert(regs[USER][LSM6DSO_CTRL1_XL] == 0xac);
	/* The fraction is not looked at (val1 only). */
	v.val1 = 104;
	v.val2 = 999999;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	assert(regs[USER][LSM6DSO_CTRL1_XL] == 0x4c);
	/* Accel full scale in m/s^2, rounded to g. */
	static const struct { int32_t g; uint8_t fs; uint32_t gain; } ranges[] = {
		{2, 0, 61}, {4, 2, 122}, {8, 3, 244}, {16, 1, 488},
	};
	for (size_t i = 0; i < ARRAY_SIZE(ranges); i++) {
		sensor_g_to_ms2(ranges[i].g, &v);
		assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v) == 0);
		assert((regs[USER][LSM6DSO_CTRL1_XL] & 0x0c) == ranges[i].fs << 2);
		lsm6dso_cal_status(imu(), &status);
		assert(status.accel_gain == ranges[i].gain);
	}
	v.val1 = 39;          /* 3.98 g rounds to 4 */
	v.val2 = 0;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v) == 0);
	lsm6dso_cal_status(imu(), &status);
	assert(status.accel_gain == 122);
	v.val1 = 29;          /* 2.96 g rounds to 3: no such range */
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v) == -EINVAL);
	/* Gyro full scale in rad/s, rounded to dps. */
	static const struct { int32_t dps; uint8_t fs; uint32_t gain; } granges[] = {
		{125, 1, 4375}, {250, 0, 8750}, {500, 2, 17500}, {1000, 4, 35000}, {2000, 6, 70000},
	};
	for (size_t i = 0; i < ARRAY_SIZE(granges); i++) {
		sensor_degrees_to_rad(granges[i].dps, &v);
		assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v) == 0);
		assert((regs[USER][LSM6DSO_CTRL2_G] & 0x0e) == granges[i].fs << 1);
		lsm6dso_cal_status(imu(), &status);
		assert(status.gyro_gain == granges[i].gain);
	}
	sensor_degrees_to_rad(300, &v);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v) == -EINVAL);
	/* Refused attributes and channels. */
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OFFSET, &v) == -ENOTSUP);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OFFSET, &v) == -ENOTSUP);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_X, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) ==
	       -ENOTSUP);
	assert(strcmp(log_text, "<wrn> LSM6DSO: attr_set() not supported on this channel.\n") == 0);
	/* The FIFO attributes of the stream (CONFIG_SENSOR_ASYNC_API). */
	v.val1 = 32;
	v.val2 = 0;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ALL,
			       (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK, &v) == 0);
	assert(sensor_attr_get(imu(), SENSOR_CHAN_ALL,
			       (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK, &v) == 0);
	assert(v.val1 == 32 && v.val2 == 0);
	assert(sensor_attr_get(imu(), SENSOR_CHAN_ALL,
			       (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION,
			       &v) == 0);
	assert(v.val1 == 8 && v.val2 == 0);
	v.val1 = 257;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ALL,
			       (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK, &v) ==
	       -EINVAL);
	v.val1 = 7;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ALL,
			       (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION,
			       &v) == -EINVAL);
	v.val1 = 32;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ,
			       (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK, &v) ==
	       -EINVAL);
	assert(sensor_attr_get(imu(), SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) ==
	       -ENOTSUP);
	assert(sensor_attr_get(imu(), SENSOR_CHAN_ACCEL_XYZ,
			       (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK, &v) ==
	       -ENOTSUP);
	assert(sensor_trigger_set(imu(), NULL, NULL) == -ENOSYS);
	/* An I2C error. */
	v.val1 = 104;
	nack_at = transfers + 4; /* the CTRL1_XL write, after the FSM/MLC checks */
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) ==
	       -EIO);
	nack_at = -1;
	/* Back to the boot ranges. */
	sensor_g_to_ms2(8, &v);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v) == 0);
	sensor_degrees_to_rad(2000, &v);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v) == 0);
}

static void push_word(uint8_t tag, int16_t x, int16_t y, int16_t z)
{
	uint8_t *w = fifo_data + fifo_words * LSM6DSO_FIFO_WORD_SIZE;
	int16_t v[3] = {x, y, z};

	w[0] = (uint8_t)(tag << 3);
	for (int i = 0; i < 3; i++) {
		w[1 + 2 * i] = (uint8_t)v[i];
		w[2 + 2 * i] = (uint8_t)((uint16_t)v[i] >> 8);
	}
	fifo_words++;
}

static int64_t fake_clock(void)
{
	static int64_t now;

	return now += 1000;
}

static void test_fifo(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6dsox)));
	struct lsm6dso_fifo_status status;
	struct lsm6dso_fifo_stats stats;
	struct sensor_value v = {.val1 = 6667};
	static uint8_t buf[LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE];
	uint32_t tick;
	int64_t before, after;

	/* imu start: 1 MHz, then the ODRs. */
	assert(i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)) == 0);
	expect_frequency = 1000000;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	/* Not started: everything but start is -EIO. */
	assert(lsm6dso_fifo_read_locked_driver(imu(), buf, 512, &status) == -EIO);
	assert(lsm6dso_fifo_status_locked_driver(imu(), &status) == -EIO);
	assert(lsm6dso_fifo_pause_batch_locked_driver(imu()) == -EIO);
	assert(lsm6dso_fifo_stop_locked_driver(imu()) == 0);
	assert(lsm6dso_fifo_start_locked_driver(imu(), 0, true, true) == -EINVAL);
	assert(lsm6dso_fifo_start_locked_driver(imu(), 512, true, true) == -EINVAL);
	assert(lsm6dso_fifo_start_locked_driver(imu(), 32, false, false) == -EINVAL);
	assert(lsm6dso_fifo_start_locked_driver(imu(), 32, true, true) == 0);
	assert(regs[USER][LSM6DSO_FIFO_CTRL1] == 32 && (regs[USER][LSM6DSO_FIFO_CTRL2] & 0x81) == 0);
	assert(regs[USER][LSM6DSO_FIFO_CTRL3] == 0xaa);      /* both batched at 6667 Hz */
	assert(regs[USER][LSM6DSO_FIFO_CTRL4] == 0x86);      /* timestamps every 8, stream */
	assert(regs[USER][LSM6DSO_CTRL10_C] == 0x20);        /* timestamp counter on */
	lsm6dso_fifo_get_stats(imu(), &stats);
	assert(stats.active && !stats.fault && stats.generation == 1);
	assert(stats.accel_hz == 6667 && stats.gyro_hz == 6667 && stats.freq_fine == -3);
	assert(stats.accel_ug_per_lsb == 244 && stats.gyro_udps_per_lsb == 70000);
	assert(lsm6dso_fifo_start_locked_driver(imu(), 32, true, true) == -EBUSY);
	/* Owned: the single reads and attributes are refused. */
	assert(sensor_sample_fetch(imu()) == -EBUSY);
	assert(sensor_channel_get(imu(), SENSOR_CHAN_ACCEL_XYZ, &v) == -EBUSY);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) ==
	       -EBUSY);
	assert(lsm6dso_cal_zero(imu()) == -EBUSY);
	/* A read: status, then one burst of all words. */
	for (int i = 0; i < 40; i++) {
		push_word(i & 1 ? 0x01 : 0x02, (int16_t)i, 0, 4100);
	}
	assert(lsm6dso_fifo_read_locked_driver(imu(), buf, 39, &status) == -ENOBUFS);
	assert(lsm6dso_fifo_read_locked_driver(imu(), buf, 512, &status) == 0);
	assert(status.words == 40 && status.watermark && !status.overrun);
	assert(fifo_words == 0 && buf[7] == 0x01 << 3);
	lsm6dso_fifo_get_stats(imu(), &stats);
	assert(stats.transfers == 1 && stats.words_read == 40 && stats.buffer_too_small == 1);
	assert(stats.transfer_time_ns == 250000 && stats.max_transfer_time_ns == 250000);
	assert(lsm6dso_fifo_read_locked_driver(imu(), buf, 512, &status) == 0 && status.words == 0);
	/* The timestamp register between two clock readings. */
	timestamp = 0x12345678;
	timestamp_step = 3;
	assert(lsm6dso_timestamp_read(imu(), NULL, &tick, &before, &after) == -EINVAL);
	assert(lsm6dso_timestamp_read(imu(), fake_clock, &tick, &before, &after) == 0);
	assert(tick == 0x1234567b && before == 1000 && after == 2000);
	timestamp_step = 401;
	assert(lsm6dso_timestamp_read(imu(), fake_clock, &tick, &before, &after) == -EAGAIN);
	timestamp_step = 3;
	/* Pause, drain, stop. */
	assert(lsm6dso_fifo_pause_batch_locked_driver(imu()) == 0);
	assert(regs[USER][LSM6DSO_FIFO_CTRL3] == 0x00 && regs[USER][LSM6DSO_FIFO_CTRL4] == 0x86);
	assert(lsm6dso_fifo_stop_locked_driver(imu()) == 0);
	assert(regs[USER][LSM6DSO_FIFO_CTRL4] == 0x80);
	lsm6dso_fifo_record_stop_error(imu(), -5);
	lsm6dso_fifo_get_stats(imu(), &stats);
	assert(!stats.active && stats.stop_error == -5);
	/* A failing status read faults the FIFO until it is stopped. */
	assert(lsm6dso_fifo_start_locked_driver(imu(), 16, true, false) == 0);
	assert(regs[USER][LSM6DSO_FIFO_CTRL3] == 0x0a);
	nack_at = transfers;
	assert(lsm6dso_fifo_status_locked_driver(imu(), &status) == -EIO);
	nack_at = -1;
	assert(lsm6dso_fifo_status_locked_driver(imu(), &status) == -EIO);
	lsm6dso_fifo_get_stats(imu(), &stats);
	assert(stats.fault && stats.i2c_errors == 1 && stats.generation == 2);
	assert(lsm6dso_fifo_stop_locked_driver(imu()) == 0);
	lsm6dso_fifo_get_stats(imu(), &stats);
	assert(!stats.active && !stats.fault);
	/* Back to 100 kHz, ODRs off. */
	assert(i2c_configure(bus, I2C_SPEED_SET(I2C_SPEED_STANDARD)) == 0);
	expect_frequency = 100000;
	v.val1 = 0;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
}

static void test_cal_zero(void)
{
	struct lsm6dso_cal_status status;

	/* Both off: run at 104 Hz (ODR index 4) and switch off again. */
	set_raw(LSM6DSO_OUTX_L_A, 0, 0, 4100);
	set_raw(LSM6DSO_OUTX_L_G, 10, -5, 2);
	reset_logs();
	log_writes = true;
	slept_us = 0;
	assert(lsm6dso_cal_zero(imu()) == 0);
	log_writes = false;
	/* ODR 104 Hz on both, 200 ms settling, then off again. */
	assert(strncmp(events, "U10=4c U11=4c sleep200ms ", 25) == 0);
	assert(strstr(events, "U10=0c U11=0c ") != NULL);
	assert(regs[USER][LSM6DSO_CTRL1_XL] == 0x0c && regs[USER][LSM6DSO_CTRL2_G] == 0x0c);
	lsm6dso_cal_status(imu(), &status);
	assert(status.gyro_bias_updates == 1);
	assert(status.gyro_bias[0] == 10 * 256 && status.gyro_bias[1] == -5 * 256 &&
	       status.gyro_bias[2] == 2 * 256);
	assert(slept_us > 200000 && slept_us < 200000 + 3 * 1000000);
	/* At rest but 210 dps from the bias: the windows are rejected. */
	set_raw(LSM6DSO_OUTX_L_G, 3000, 0, 0);
	assert(lsm6dso_cal_zero(imu()) == -ERANGE);
	lsm6dso_cal_status(imu(), &status);
	assert(status.gyro_bias_updates == 1 && status.gyro_bias_rejected > 0);
	assert(status.gyro_bias[0] == 10 * 256); /* the earlier bias stays */
	assert(regs[USER][LSM6DSO_CTRL1_XL] == 0x0c && regs[USER][LSM6DSO_CTRL2_G] == 0x0c);
}


/* The FIFO stream ---------------------------------------------------------*/

/* imu_stream.c:62-64 */
SENSOR_DT_STREAM_IODEV(test_stream_iodev, DT_NODELABEL(lsm6dsox),
	{ SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE });
SENSOR_DT_STREAM_IODEV(test_full_iodev, DT_NODELABEL(lsm6dsox),
	{ SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_INCLUDE });
SENSOR_DT_READ_IODEV(test_read_iodev, DT_NODELABEL(lsm6dsox),
	{ SENSOR_CHAN_ACCEL_XYZ, 0 }, { SENSOR_CHAN_GYRO_XYZ, 0 });
RTIO_DEFINE_WITH_MEMPOOL(test_rtio, 4, 16, 128, 64, 4);

static void push_pairs(int pairs)
{
	for (int i = 0; i < pairs; i++) {
		push_word(0x02, (int16_t)(100 + i), 0, 4100);  /* accel */
		push_word(0x01, (int16_t)i, -1, 2);            /* gyro */
	}
	update_int1();
}

/* Takes the next completion: its result, and the FIFO words of its
 * buffer (-1 if it has none).
 */
static int next_cqe(int *words)
{
	struct rtio_cqe *cqe = rtio_cqe_consume(&test_rtio);
	uint8_t *buf = NULL;
	uint32_t len = 0;
	const uint8_t *w;
	size_t size;
	int result;

	assert(cqe != NULL);
	result = cqe->result;
	*words = -1;
	if (rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &buf, &len) == 0 && buf) {
		const struct sensor_decoder_api *decoder;
		uint16_t frames = 0;

		assert(lsm6dso_fifo_buffer_words(buf, len, &w, &size) == 0);
		*words = (int)(size / LSM6DSO_FIFO_WORD_SIZE);
		assert(sensor_get_decoder(imu(), &decoder) == 0);
		assert(decoder == &lsm6dso_combined_decoder);
		assert(decoder->get_frame_count(buf, (struct sensor_chan_spec){
				SENSOR_CHAN_ACCEL_XYZ, 0}, &frames) == 0);
		assert(frames == *words / 2);
		assert(decoder->has_trigger(buf, SENSOR_TRIG_FIFO_WATERMARK) ==
		       (*words >= 32));
	}
	rtio_cqe_release(&test_rtio, cqe);
	rtio_release_buffer(&test_rtio, buf, len);
	return result;
}

static void test_stream(void)
{
	struct lsm6dso_data *d = imu()->data;
	struct sensor_value v = {.val1 = 6667};
	rtio_sqe_handle_t h, h2;
	struct lsm6dso_fifo_stats stats;
	const char *name;
	int prio, words;
	size_t stack;

	/* The stream's work queue, started at init. */
	assert(test_work_queues() == 2);
	(void)test_work_queue(1, &name, &prio, &stack);
	assert(strcmp(name, "imu_fifo_worker") == 0 && prio == 108 && stack == 2048);
	/* imu start: 1 MHz, the ODRs, the watermark (32 here). */
	assert(i2c_configure(DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6dsox))),
			     I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)) == 0);
	expect_frequency = 1000000;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	assert(lsm6dso_fifo_set_watermark(imu(), 32) == 0);
	fifo_words = 0;

	/* sensor_stream(): lsm6dso_submit() in the caller, the worker starts the
	 * FIFO, INT1 and the 20 ms poll.
	 */
	assert(sensor_stream(&test_stream_iodev, &test_rtio, NULL, &h) == 0);
	assert(d->stream_sqe != NULL && !d->stream_active);
	assert(test_work_run_all() == 1);
	assert(d->stream_active && d->stream_irq_active && int1_configured == 1);
	assert(int1_events == NANO_GPIO_IRQ_EDGE_RISE);
	assert(regs[USER][LSM6DSO_FIFO_CTRL1] == 32 && regs[USER][LSM6DSO_FIFO_CTRL3] == 0xaa);
	assert(regs[USER][LSM6DSO_FIFO_CTRL4] == 0x86 && regs[USER][LSM6DSO_INT1_CTRL] == 0x08);
	assert(d->stream_work.timeout.armed && d->stream_work.timeout.ticks == 200);
	assert(lsm6dso_fifo_set_watermark(imu(), 64) == -EBUSY);
	assert(rtio_cqe_consume(&test_rtio) == NULL);

	/* The INT1 edge: the work on the system work queue wakes the worker
	 * (the poll is cut short), which delivers one batch of all 40 words
	 * and goes back to the poll.
	 */
	push_pairs(20);
	assert(int1_edges == 1 && int1_level);
	assert(test_work_run_all() == 3);   /* INT1 work, the batch, the empty check */
	assert(fifo_words == 0 && !int1_level);
	assert(d->stream_work.timeout.armed && d->stream_work.timeout.ticks == 200);
	assert(next_cqe(&words) == 0 && words == 40);
	assert(rtio_cqe_consume(&test_rtio) == NULL);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == d->stream_sqe);

	/* A missed edge: the 20 ms poll finds the watermark. */
	int1_hide_edges = true;
	push_pairs(17);
	int1_hide_edges = false;
	assert(test_work_run_all() == 0);
	test_timeout_fire(&d->stream_work.timeout);
	assert(test_work_run_all() == 2);
	assert(next_cqe(&words) == 0 && words == 34);
	/* Below the watermark nothing is delivered. */
	push_pairs(3);
	test_timeout_fire(&d->stream_work.timeout);
	assert(test_work_run_all() == 1 && rtio_cqe_consume(&test_rtio) == NULL);

	/* A second stream while this one runs, and another trigger. */
	assert(sensor_stream(&test_stream_iodev, &test_rtio, NULL, &h2) == 0);
	assert(next_cqe(&words) == -EBUSY && words == -1);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h2) == NULL);
	assert(sensor_stream(&test_full_iodev, &test_rtio, NULL, &h2) == 0);
	assert(next_cqe(&words) == -ENOTSUP);

	/* The graceful stop: pause batching, deliver the last 6 words, then
	 * stop (bypass, INT1 off) and end with -ECANCELED.
	 */
	assert(lsm6dso_fifo_request_stop(imu(), &test_rtio, h + 1) == -ENOENT);
	assert(lsm6dso_fifo_request_stop(imu(), &test_rtio, h) == 0);
	assert(test_work_run_all() == 2);
	assert(next_cqe(&words) == 0 && words == 6);
	assert(next_cqe(&words) == -ECANCELED && words == -1);
	assert(rtio_cqe_consume(&test_rtio) == NULL);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == NULL && d->stream_sqe == NULL);
	assert(!d->stream_active && !d->stream_irq_active && int1_events == 0);
	assert(regs[USER][LSM6DSO_FIFO_CTRL3] == 0x00 && regs[USER][LSM6DSO_FIFO_CTRL4] == 0x80);
	assert(regs[USER][LSM6DSO_INT1_CTRL] == 0x00);
	assert(!d->stream_work.timeout.armed);
	lsm6dso_fifo_get_stats(imu(), &stats);
	assert(!stats.active && stats.stop_error == 0 && stats.words_read == 80);

	/* A cancellation: noticed by the worker at its next run. */
	assert(sensor_stream(&test_stream_iodev, &test_rtio, NULL, &h) == 0);
	assert(test_work_run_all() == 1 && d->stream_active);
	push_pairs(5);
	assert(rtio_sqe_cancel(&test_rtio, h) == 0);
	test_timeout_fire(&d->stream_work.timeout);
	assert(test_work_run_all() == 1);
	assert(next_cqe(&words) == -ECANCELED && words == -1);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == NULL && !d->stream_active);
	assert(regs[USER][LSM6DSO_FIFO_CTRL4] == 0x80 && regs[USER][LSM6DSO_INT1_CTRL] == 0x00);
	/* Batching was not paused on this path (as in Zephyr). */
	assert(regs[USER][LSM6DSO_FIFO_CTRL3] == 0xaa);
	fifo_words = 0;

	/* An overrun ends the stream with -EOVERFLOW. */
	assert(sensor_stream(&test_stream_iodev, &test_rtio, NULL, &h) == 0);
	assert(test_work_run_all() == 1);
	fifo_overrun = true;
	push_pairs(16);
	assert(test_work_run_all() == 2);
	fifo_overrun = false;
	assert(next_cqe(&words) == -EOVERFLOW && words == -1);
	assert(!d->stream_active && regs[USER][LSM6DSO_INT1_CTRL] == 0x00);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == NULL);
	fifo_words = 0;
	update_int1();
	assert(atomic_get(&test_rtio.xcqcnt) == 0);
	for (int i = 0; i < 128; i++) {
		assert(test_rtio.block_pool->bitmap[i / 32] == 0);
	}

	/* A single read: the fallback on the RTIO work queue. */
	set_raw(LSM6DSO_OUTX_L_A, 0, 0, 4100);
	set_raw(LSM6DSO_OUTX_L_G, 10, -5, 2);
	assert(sensor_read_async_mempool(&test_read_iodev, &test_rtio, (void *)7) == 0);
	assert(rtio_cqe_consume(&test_rtio) == NULL);
	assert(test_work_run_all() == 1);
	{
		struct rtio_cqe *cqe = rtio_cqe_consume(&test_rtio);
		const struct sensor_decoder_api *decoder;
		struct sensor_three_axis_data out;
		uint8_t *buf;
		uint32_t len, fit = 0;
		int64_t z;

		assert(cqe && cqe->result == 0 && cqe->userdata == (void *)7);
		assert(rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &buf, &len) == 0 && buf);
		rtio_cqe_release(&test_rtio, cqe);
		assert(sensor_get_decoder(imu(), &decoder) == 0);
		assert(decoder->decode(buf, (struct sensor_chan_spec){SENSOR_CHAN_ACCEL_XYZ, 0},
				       &fit, 1, &out) == 1);
		assert(out.header.base_timestamp_ns == clock_cycles * 1000);
		z = ((int64_t)out.readings[0].z << out.shift) * 1000000 / ((int64_t)1 << 31);
		{
			const int16_t raw[3] = {0, 0, 4100};

			assert(llabs(z - expected_micro(LSM6DSO_CAL_ACCEL, 244, raw, 2)) <= 1);
		}
		rtio_release_buffer(&test_rtio, buf, len);
	}

	/* Back to 100 kHz, ODRs off. */
	assert(i2c_configure(DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6dsox))),
			     I2C_SPEED_SET(I2C_SPEED_STANDARD)) == 0);
	expect_frequency = 100000;
	v.val1 = 0;
	assert(sensor_attr_set(imu(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
	assert(sensor_attr_set(imu(), SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v) == 0);
}

int main(void)
{
	/* What the board starts before the LSM6DSOX (nano_bringup.c). */
	assert(zephyr_compat_initialize() == 0);
	assert(device_init(DEVICE_DT_GET(DT_NODELABEL(gpio0))) == 0);
	test_bus();
	test_init_errors();
	test_init();
	test_fetch();
	test_attr();
	test_fifo();
	test_cal_zero();
	test_stream();
	return 0;
}
