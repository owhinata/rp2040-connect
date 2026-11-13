/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "lsm6dso_zero.h"

#define ONE     (1 << LSM6DSO_ZERO_BIAS_FRAC)
/* A rate near the one in use, so that the noise below averages as on the sensor. */
#define RATE    6400U
#define SEGMENT (RATE / LSM6DSO_ZERO_SEGMENTS)
#define WINDOW  RATE

static uint32_t seed = 1;

/* Roughly Gaussian noise with the given standard deviation. */
static int noise(int sigma)
{
	int sum = 0;

	for (int i = 0; i < 12; i++) {
		seed = seed * 1664525U + 1013904223U;
		sum += (int)(seed >> 16 & 0xff) - 128;
	}
	return sum * sigma / 256;
}

static struct lsm6dso_zero start(bool calibrated, const int32_t *reference)
{
	struct lsm6dso_zero zero = {0};
	struct lsm6dso_zero_limits limits;

	lsm6dso_zero_limits(&limits, 70000, 244, calibrated);
	/* 0.3 dps, 2 mg, and 1 or 5 dps at these sensitivities. */
	assert(limits.gyro_spread == 300000 * ONE / 70000);
	assert(limits.accel_spread == 2000 * ONE / 244);
	assert(limits.gyro_range == (calibrated ? 1000000 : 5000000) * ONE / 70000);
	lsm6dso_zero_reset(&zero, &limits, reference);
	lsm6dso_zero_set_rate(&zero, RATE);
	assert(zero.segment_samples == SEGMENT);
	return zero;
}

/* Feeds count sample pairs; returns how many of them changed the bias. */
static int feed(struct lsm6dso_zero *zero, int count, const int gyro[3], const int accel[3],
		int gyro_noise, int accel_noise)
{
	int changed = 0;

	for (int i = 0; i < count; i++) {
		int16_t g[3], a[3];

		for (int axis = 0; axis < 3; axis++) {
			g[axis] = (int16_t)(gyro[axis] + noise(gyro_noise));
			a[axis] = (int16_t)(accel[axis] + noise(accel_noise));
		}
		lsm6dso_zero_accel(zero, a);
		changed += lsm6dso_zero_gyro(zero, g);
	}
	return changed;
}

static const int level[3] = { 0, -500, 4098 };

static void test_rest(void)
{
	struct lsm6dso_zero zero = start(false, NULL);
	const int bias[3] = { 8, -3, -1 };

	/* Nothing before a full window; one update per window after that. */
	assert(feed(&zero, WINDOW - 1, bias, level, 25, 20) == 0 && zero.updates == 0);
	assert(feed(&zero, 1, bias, level, 25, 20) == 1 && zero.updates == 1);
	assert(feed(&zero, 9 * WINDOW, bias, level, 25, 20) == 9 && zero.updates == 10);
	for (int axis = 0; axis < 3; axis++) {
		/* Noise of 25 LSB per sample leaves well under 1 LSB after ten windows. */
		assert(abs(zero.bias[axis] - bias[axis] * ONE) < ONE);
	}
	assert(zero.rejected == 0);
}

static void test_blend(void)
{
	struct lsm6dso_zero zero = start(false, NULL);
	const int first[3] = { 10, 10, 10 }, second[3] = { 30, 30, 30 };

	/* Plain average while warming up... */
	assert(feed(&zero, WINDOW, first, level, 0, 0) == 1 && zero.bias[0] == 10 * ONE);
	assert(feed(&zero, WINDOW, second, level, 0, 0) == 1 && zero.bias[0] == 20 * ONE);
	/* ...then a twentieth of each new window. */
	assert(feed(&zero, 18 * WINDOW, second, level, 0, 0) == 18 && zero.updates == 20);
	assert(feed(&zero, 7 * WINDOW / 8, first, level, 0, 0) == 0);
	{
		int32_t before = zero.bias[0];

		assert(feed(&zero, WINDOW, first, level, 0, 0) == 1);
		assert(zero.bias[0] == before + (10 * ONE - before) / 20);
	}
}

static void test_motion(void)
{
	struct lsm6dso_zero zero = start(false, NULL);

	/* A steady turn changes the acceleration from segment to segment. */
	for (int i = 0; i < 5 * LSM6DSO_ZERO_SEGMENTS; i++) {
		const int gyro[3] = { 30, 0, 0 };
		const int accel[3] = { 0, -500 + 12 * i, 4098 };

		assert(feed(&zero, SEGMENT, gyro, accel, 25, 20) == 0);
	}
	/* A changing rate with a level acceleration. */
	for (int i = 0; i < 5 * LSM6DSO_ZERO_SEGMENTS; i++) {
		const int gyro[3] = { 0, (i % 4) * 8, 0 };

		assert(feed(&zero, SEGMENT, gyro, level, 25, 20) == 0);
	}
	assert(zero.updates == 0 && zero.rejected == 0);
	/* Rest right after motion is picked up within two windows. */
	{
		const int bias[3] = { 8, -3, -1 };

		assert(feed(&zero, 2 * WINDOW, bias, level, 25, 20) >= 1);
	}
}

static void test_reference(void)
{
	const int32_t stored[3] = { 8 * ONE, -3 * ONE, -1 * ONE };
	struct lsm6dso_zero zero = start(true, stored);
	const int near[3] = { 20, -3, -1 };  /* 12 LSB = 0.84 dps from the stored bias */
	const int far[3] = { 24, -3, -1 };   /* 16 LSB = 1.12 dps: a slow turn about gravity */

	assert(feed(&zero, WINDOW, far, level, 0, 0) == 0);
	assert(zero.updates == 0 && zero.rejected == 1);
	assert(feed(&zero, WINDOW, near, level, 0, 0) == 1 && zero.bias[0] == 20 * ONE);
	/* Without a calibration the same reading is accepted around zero. */
	zero = start(false, NULL);
	assert(feed(&zero, WINDOW, far, level, 0, 0) == 1 && zero.rejected == 0);
}

static void test_inputs(void)
{
	struct lsm6dso_zero zero = start(false, NULL);
	const int bias[3] = { 8, -3, -1 };
	const int16_t g[3] = { 8, -3, -1 };

	/* No acceleration: a tilt could not be told from a bias. */
	for (unsigned int i = 0; i < 3 * WINDOW; i++) {
		assert(!lsm6dso_zero_gyro(&zero, g));
	}
	assert(zero.updates == 0);
	/* A new rate restarts the window but keeps the estimate. */
	assert(feed(&zero, WINDOW, bias, level, 0, 0) == 1);
	assert(feed(&zero, WINDOW / 2, bias, level, 0, 0) == 0);
	lsm6dso_zero_set_rate(&zero, 104);
	assert(zero.segment_samples == 13 && zero.updates == 1 && zero.bias[0] == 8 * ONE);
	assert(feed(&zero, 8 * 13 - 1, bias, level, 0, 0) == 0);
	assert(feed(&zero, 1, bias, level, 0, 0) == 1);
	/* Too slow to average, or off. */
	lsm6dso_zero_set_rate(&zero, 12);
	assert(zero.segment_samples == 0);
	assert(feed(&zero, 1000, bias, level, 0, 0) == 0);
	lsm6dso_zero_set_rate(&zero, 0);
	assert(feed(&zero, 1000, bias, level, 0, 0) == 0 && zero.updates == 2);
	/* Reset forgets the estimate and keeps the rate. */
	lsm6dso_zero_set_rate(&zero, RATE);
	{
		struct lsm6dso_zero_limits limits;

		lsm6dso_zero_limits(&limits, 70000, 244, false);
		lsm6dso_zero_reset(&zero, &limits, NULL);
		assert(zero.updates == 0 && zero.bias[0] == 0 && zero.segment_samples == SEGMENT);
		/* An unknown sensitivity gives limits nothing can pass. */
		lsm6dso_zero_limits(&limits, 0, 244, false);
		assert(limits.gyro_spread == 0 && limits.gyro_range == 0);
	}
}

int main(void)
{
	test_rest();
	test_blend();
	test_motion();
	test_reference();
	test_inputs();
	return 0;
}
