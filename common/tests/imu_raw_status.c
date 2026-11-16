/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <string.h>
#include "imu_raw_status.h"

int main(void)
{
	char out[512];
	struct imu_raw_status s = {
		.session_id = {0x01, 0xab, [15] = 0xff}, .state = IMU_RAW_STATE_MEASURING,
		.epoch_id = 2, .uptime_ms = 1234, .accel_acquired = UINT64_MAX, .gyro_acquired = 5,
		.next_batch = 6, .timestamp_words = 7, .fifo_overrun_observations = 8, .i2c_errors = 9,
		.raw_error = -71, .measure_error = 0, .dropped_batches = 10, .dropped_accel = 11,
		.dropped_gyro = 12, .published = 13, .connects = 14, .failures = 15,
		.queue_high_watermark = 16,
	};
	const char *expected_status =
		"{\"v\":1,\"session\":\"01ab00000000000000000000000000ff\",\"state\":\"measuring\","
		"\"epoch\":2,\"uptime_ms\":1234,\"accel\":18446744073709551615,\"gyro\":5,"
		"\"next_batch\":6,\"timestamp_words\":7,\"fifo_overruns\":8,\"i2c_errors\":9,"
		"\"raw_error\":-71,\"error\":0,\"dropped_batches\":10,\"dropped_accel\":11,"
		"\"dropped_gyro\":12,\"published\":13,\"connects\":14,\"failures\":15,"
		"\"queue_high\":16,\"sync\":\"unsynchronized\"}";
	int n = imu_raw_status_encode(out, sizeof(out), &s);

	assert(n == (int)strlen(expected_status) && !strcmp(out, expected_status));
	/* Exact fit needs room for the terminator; one byte less must fail. */
	assert(imu_raw_status_encode(out, (size_t)n + 1, &s) == n);
	assert(imu_raw_status_encode(out, (size_t)n, &s) == -ENOSPC);
	/* Time point: tick and board clock without server time, then with it. */
	s.tick_valid = true;
	s.tick = 5000000000ULL;
	s.tick_mono_us = 123456789;
	s.tick_window_us = 140;
	assert(imu_raw_status_encode(out, sizeof(out), &s) > 0 &&
	       strstr(out, "\"queue_high\":16,\"tick\":5000000000,\"tick_mono_us\":123456789,"
			   "\"tick_window_us\":140,\"sync\":\"unsynchronized\"}"));
	s.unix_valid = true;
	s.unix_us = 1794370000123456LL;
	s.unix_estimate_us = 1794370000125000LL;
	s.sync_delay_us = 5300;
	n = imu_raw_status_encode(out, sizeof(out), &s);
	assert(n > 0 && n < 640 &&
	       strstr(out, "\"tick_window_us\":140,\"unix_us\":1794370000123456,"
			   "\"unix_estimate_us\":1794370000125000,\"sync_delay_us\":5300,"
			   "\"sync\":\"ntp\"}"));
	/* Server time without a tick reading is not a usable point. */
	s.tick_valid = false;
	assert(imu_raw_status_encode(out, sizeof(out), &s) > 0 &&
	       strstr(out, "\"queue_high\":16,\"sync\":\"unsynchronized\"}"));
	s.state = IMU_RAW_STATE_IDLE;
	assert(imu_raw_status_encode(out, sizeof(out), &s) > 0 && strstr(out, "\"state\":\"idle\""));
	s.state = IMU_RAW_STATE_STOPPED;
	assert(imu_raw_status_encode(out, sizeof(out), &s) > 0 && strstr(out, "\"state\":\"stopped\""));
	s.state = (enum imu_raw_state)3;
	assert(imu_raw_status_encode(out, sizeof(out), &s) == -EINVAL);
	assert(imu_raw_status_encode(NULL, sizeof(out), &s) == -EINVAL);

	struct imu_raw_info info = {
		.session_id = {0x10}, .epoch_id = 1, .accel_odr_hz = 6667, .gyro_odr_hz = 6667,
		.accel_ug_per_lsb = 61, .gyro_udps_per_lsb = 70000, .fifo_watermark = 128,
		.timestamp_decimation = 8, .freq_fine = -3,
	};
	const char *expected_info =
		"{\"v\":1,\"session\":\"10000000000000000000000000000000\",\"epoch\":1,"
		"\"device\":\"rp2040\",\"sensor\":\"LSM6DSO\",\"payload\":\"IMU1\",\"payload_version\":2,"
		"\"raw\":\"int16le\","
		"\"axes\":\"xyz\",\"accel_odr_hz\":6667,\"gyro_odr_hz\":6667,\"accel_ug_per_lsb\":61,"
		"\"gyro_udps_per_lsb\":70000,\"fifo_watermark\":128,\"timestamp_decimation\":8,"
		"\"time\":\"sensor_tick\",\"tick_hz_nominal\":40000,\"freq_fine\":-3}";

	n = imu_raw_info_encode(out, sizeof(out), &info, "rp2040");
	assert(n == (int)strlen(expected_info) && !strcmp(out, expected_info));
	assert(imu_raw_info_encode(out, 16, &info, "rp2040") == -ENOSPC);
	info.timestamp_decimation = 0;
	assert(imu_raw_info_encode(out, sizeof(out), &info, "rp2040") > 0 &&
	       strstr(out, "\"timestamp_decimation\":0,\"time\":\"unavailable\"}"));
	/* A device ID that would need JSON escaping is rejected, not emitted. */
	assert(imu_raw_info_encode(out, sizeof(out), &info, "a\"b") == -EINVAL);
	return 0;
}
