/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TEST_FAKE_LSM6DSO_REG_H_
#define TEST_FAKE_LSM6DSO_REG_H_
#include <stdint.h>
typedef struct { int unused; } stmdev_ctx_t;
typedef int lsm6dso_bdr_xl_t;
typedef int lsm6dso_bdr_gy_t;
typedef struct { uint8_t fifo_th, drdy_xl; } lsm6dso_pin_int1_route_t;
#define LSM6DSO_BYPASS_MODE 0
#define LSM6DSO_STREAM_MODE 6
#define LSM6DSO_XL_NOT_BATCHED 0
#define LSM6DSO_GY_NOT_BATCHED 0
#define LSM6DSO_TEMP_NOT_BATCHED 0
#define LSM6DSO_NO_DECIMATION 0
#define LSM6DSO_CMP_DISABLE 0
#define LSM6DSO_FIFO_STATUS1 0x3a
#define LSM6DSO_FIFO_DATA_OUT_TAG 0x78
int32_t lsm6dso_fifo_mode_set(const stmdev_ctx_t *ctx, int value);
int32_t lsm6dso_pin_int1_route_get(const stmdev_ctx_t *ctx, lsm6dso_pin_int1_route_t *route);
int32_t lsm6dso_pin_int1_route_set(const stmdev_ctx_t *ctx, lsm6dso_pin_int1_route_t route);
int32_t lsm6dso_fifo_xl_batch_set(const stmdev_ctx_t *ctx, int value);
int32_t lsm6dso_fifo_gy_batch_set(const stmdev_ctx_t *ctx, int value);
int32_t lsm6dso_fifo_temp_batch_set(const stmdev_ctx_t *ctx, int value);
typedef int lsm6dso_odr_ts_batch_t;
int32_t lsm6dso_fifo_timestamp_decimation_set(const stmdev_ctx_t *ctx, int value);
int32_t lsm6dso_timestamp_set(const stmdev_ctx_t *ctx, uint8_t value);
int32_t lsm6dso_fifo_stop_on_wtm_set(const stmdev_ctx_t *ctx, uint8_t value);
int32_t lsm6dso_compression_algo_set(const stmdev_ctx_t *ctx, int value);
int32_t lsm6dso_fifo_watermark_set(const stmdev_ctx_t *ctx, uint16_t value);
#endif
