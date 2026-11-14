/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's sensor API (<zephyr/drivers/sensor.h>, Zephyr
 * v4.5.0) that the ported LSM6DSO driver and its callers use: struct
 * sensor_value, the channels, attributes and triggers (the same numbers as
 * Zephyr's), the driver's API table, sensor_attr_set()/_get(),
 * sensor_sample_fetch()/_chan(), sensor_channel_get(), sensor_trigger_set()
 * and the unit conversions; and the asynchronous API on RTIO
 * (CONFIG_SENSOR_ASYNC_API of the Zephyr build): channel specifications,
 * the decoder API and the decoded data types (sensor_data_types.h), stream
 * triggers, SENSOR_DT_READ_IODEV(), SENSOR_DT_STREAM_IODEV(),
 * sensor_get_decoder(), sensor_stream(), sensor_read(),
 * sensor_read_async_mempool(), sensor_processing_with_callback(), and the
 * default iodev and decoder (sensor_default_rtio.c, a copy of Zephyr's
 * drivers/sensor/default_rtio_sensor.c).  The functions and conversions
 * are Zephyr's, written out.  Part of the small Zephyr compatibility layer
 * of the port (nuttx/drivers/compat, see nuttx/drivers/README.md).
 *
 * The print formats of the decoded data (PRIsensor_*_data, with the Q
 * value format of <zephyr/dsp/print_format.h>) are Zephyr's, for the
 * sensor shell (nuttx/apps/sensor).
 *
 * Not here: the other decoded data types (game rotation vector,
 * occurrence), and the device definition macros (lsm6dso.c defines its
 * device with DEVICE_DT_DEFINE() of <zephyr/device.h>).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_SENSOR_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_SENSOR_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/dsp/types.h>
#include <zephyr/rtio/rtio.h>

/* A value: val1 + val2 / 1000000, both with the sign of the value. */

struct sensor_value
{
  int32_t val1;   /* integer part */
  int32_t val2;   /* fractional part, millionths */
};

enum sensor_channel
{
  SENSOR_CHAN_ACCEL_X,          /* m/s^2 */
  SENSOR_CHAN_ACCEL_Y,
  SENSOR_CHAN_ACCEL_Z,
  SENSOR_CHAN_ACCEL_XYZ,
  SENSOR_CHAN_GYRO_X,           /* rad/s */
  SENSOR_CHAN_GYRO_Y,
  SENSOR_CHAN_GYRO_Z,
  SENSOR_CHAN_GYRO_XYZ,
  SENSOR_CHAN_MAGN_X,
  SENSOR_CHAN_MAGN_Y,
  SENSOR_CHAN_MAGN_Z,
  SENSOR_CHAN_MAGN_XYZ,
  SENSOR_CHAN_DIE_TEMP,
  SENSOR_CHAN_AMBIENT_TEMP,
  SENSOR_CHAN_PRESS,
  SENSOR_CHAN_PROX,
  SENSOR_CHAN_HUMIDITY,
  SENSOR_CHAN_AMBIENT_LIGHT,
  SENSOR_CHAN_LIGHT,
  SENSOR_CHAN_IR,
  SENSOR_CHAN_RED,
  SENSOR_CHAN_GREEN,
  SENSOR_CHAN_BLUE,
  SENSOR_CHAN_ALTITUDE,
  SENSOR_CHAN_PM_1_0_CF,
  SENSOR_CHAN_PM_2_5_CF,
  SENSOR_CHAN_PM_10_CF,
  SENSOR_CHAN_PM_1_0,
  SENSOR_CHAN_PM_2_5,
  SENSOR_CHAN_PM_10,
  SENSOR_CHAN_PM_0_3_COUNT,
  SENSOR_CHAN_PM_0_5_COUNT,
  SENSOR_CHAN_PM_1_0_COUNT,
  SENSOR_CHAN_PM_2_5_COUNT,
  SENSOR_CHAN_PM_5_COUNT,
  SENSOR_CHAN_PM_10_COUNT,
  SENSOR_CHAN_DISTANCE,
  SENSOR_CHAN_CO2,
  SENSOR_CHAN_O2,
  SENSOR_CHAN_VOC,
  SENSOR_CHAN_GAS_RES,
  SENSOR_CHAN_FLOW_RATE,
  SENSOR_CHAN_VOLUME,
  SENSOR_CHAN_VOLTAGE,
  SENSOR_CHAN_VSHUNT,
  SENSOR_CHAN_CURRENT,
  SENSOR_CHAN_POWER,
  SENSOR_CHAN_RESISTANCE,
  SENSOR_CHAN_ROTATION,
  SENSOR_CHAN_POS_DX,
  SENSOR_CHAN_POS_DY,
  SENSOR_CHAN_POS_DZ,
  SENSOR_CHAN_POS_DXYZ,
  SENSOR_CHAN_RPM,
  SENSOR_CHAN_FREQUENCY,
  SENSOR_CHAN_GAUGE_VOLTAGE,
  SENSOR_CHAN_GAUGE_AVG_CURRENT,
  SENSOR_CHAN_GAUGE_STDBY_CURRENT,
  SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT,
  SENSOR_CHAN_GAUGE_TEMP,
  SENSOR_CHAN_GAUGE_STATE_OF_CHARGE,
  SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY,
  SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY,
  SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY,
  SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY,
  SENSOR_CHAN_GAUGE_AVG_POWER,
  SENSOR_CHAN_GAUGE_STATE_OF_HEALTH,
  SENSOR_CHAN_GAUGE_TIME_TO_EMPTY,
  SENSOR_CHAN_GAUGE_TIME_TO_FULL,
  SENSOR_CHAN_GAUGE_CYCLE_COUNT,
  SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE,
  SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE,
  SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
  SENSOR_CHAN_GAME_ROTATION_VECTOR,
  SENSOR_CHAN_GRAVITY_VECTOR,
  SENSOR_CHAN_GBIAS_XYZ,
  SENSOR_CHAN_ENCODER_COUNT,
  SENSOR_CHAN_ENCODER_REVOLUTIONS,
  SENSOR_CHAN_ALL,
  SENSOR_CHAN_COMMON_COUNT,
  SENSOR_CHAN_PRIV_START = SENSOR_CHAN_COMMON_COUNT,
  SENSOR_CHAN_MAX = INT16_MAX,
};

enum sensor_trigger_type
{
  SENSOR_TRIG_TIMER,
  SENSOR_TRIG_DATA_READY,
  SENSOR_TRIG_DELTA,
  SENSOR_TRIG_NEAR_FAR,
  SENSOR_TRIG_THRESHOLD,
  SENSOR_TRIG_TAP,
  SENSOR_TRIG_DOUBLE_TAP,
  SENSOR_TRIG_FREEFALL,
  SENSOR_TRIG_MOTION,
  SENSOR_TRIG_STATIONARY,
  SENSOR_TRIG_FIFO_WATERMARK,
  SENSOR_TRIG_FIFO_FULL,
  SENSOR_TRIG_TILT,
  SENSOR_TRIG_OVERFLOW,
  SENSOR_TRIG_COMMON_COUNT,
  SENSOR_TRIG_PRIV_START = SENSOR_TRIG_COMMON_COUNT,
  SENSOR_TRIG_MAX = INT16_MAX,
};

struct sensor_trigger
{
  enum sensor_trigger_type type;
  enum sensor_channel chan;
};

enum sensor_attribute
{
  SENSOR_ATTR_SAMPLING_FREQUENCY,
  SENSOR_ATTR_LOWER_THRESH,
  SENSOR_ATTR_UPPER_THRESH,
  SENSOR_ATTR_SLOPE_TH,
  SENSOR_ATTR_SLOPE_DUR,
  SENSOR_ATTR_HYSTERESIS,
  SENSOR_ATTR_OVERSAMPLING,
  SENSOR_ATTR_FULL_SCALE,
  SENSOR_ATTR_OFFSET,
  SENSOR_ATTR_CALIB_TARGET,
  SENSOR_ATTR_CONFIGURATION,
  SENSOR_ATTR_CALIBRATION,
  SENSOR_ATTR_FEATURE_MASK,
  SENSOR_ATTR_ALERT,
  SENSOR_ATTR_FF_DUR,
  SENSOR_ATTR_BATCH_DURATION,
  SENSOR_ATTR_GAIN,
  SENSOR_ATTR_RESOLUTION,
  SENSOR_ATTR_CHIP_ID,
  SENSOR_ATTR_COMMON_COUNT,
  SENSOR_ATTR_PRIV_START = SENSOR_ATTR_COMMON_COUNT,
  SENSOR_ATTR_MAX = INT16_MAX,
};

typedef void (*sensor_trigger_handler_t)(const struct device *dev,
                                         const struct sensor_trigger *trigger);
typedef int (*sensor_attr_set_t)(const struct device *dev,
                                 enum sensor_channel chan,
                                 enum sensor_attribute attr,
                                 const struct sensor_value *val);
typedef int (*sensor_attr_get_t)(const struct device *dev,
                                 enum sensor_channel chan,
                                 enum sensor_attribute attr,
                                 struct sensor_value *val);
typedef int (*sensor_trigger_set_t)(const struct device *dev,
                                    const struct sensor_trigger *trig,
                                    sensor_trigger_handler_t handler);
typedef int (*sensor_sample_fetch_t)(const struct device *dev,
                                     enum sensor_channel chan);
typedef int (*sensor_channel_get_t)(const struct device *dev,
                                    enum sensor_channel chan,
                                    struct sensor_value *val);

/* The asynchronous part of the table (the FIFO stream's driver fills it). */

struct sensor_decoder_api;
struct rtio_iodev_sqe;
typedef int (*sensor_get_decoder_t)(const struct device *dev,
                                    const struct sensor_decoder_api **api);
typedef void (*sensor_submit_t)(const struct device *sensor,
                                struct rtio_iodev_sqe *sqe);

struct sensor_driver_api
{
  sensor_attr_set_t attr_set;           /* optional */
  sensor_attr_get_t attr_get;           /* optional */
  sensor_trigger_set_t trigger_set;     /* optional */
  sensor_sample_fetch_t sample_fetch;   /* mandatory */
  sensor_channel_get_t channel_get;     /* mandatory */
  sensor_get_decoder_t get_decoder;     /* optional */
  sensor_submit_t submit;               /* optional */
};

/* The calls (Zephyr's z_impl_sensor_*(); there are no system calls here). */

static inline int sensor_attr_set(const struct device *dev,
                                  enum sensor_channel chan,
                                  enum sensor_attribute attr,
                                  const struct sensor_value *val)
{
  const struct sensor_driver_api *api = DEVICE_API_GET(sensor, dev);

  if (api->attr_set == NULL)
    {
      return -ENOSYS;
    }

  return api->attr_set(dev, chan, attr, val);
}

static inline int sensor_attr_get(const struct device *dev,
                                  enum sensor_channel chan,
                                  enum sensor_attribute attr,
                                  struct sensor_value *val)
{
  const struct sensor_driver_api *api = DEVICE_API_GET(sensor, dev);

  if (api->attr_get == NULL)
    {
      return -ENOSYS;
    }

  return api->attr_get(dev, chan, attr, val);
}

static inline int sensor_trigger_set(const struct device *dev,
                                     const struct sensor_trigger *trig,
                                     sensor_trigger_handler_t handler)
{
  const struct sensor_driver_api *api = DEVICE_API_GET(sensor, dev);

  if (api->trigger_set == NULL)
    {
      return -ENOSYS;
    }

  return api->trigger_set(dev, trig, handler);
}

static inline int sensor_sample_fetch(const struct device *dev)
{
  return DEVICE_API_GET(sensor, dev)->sample_fetch(dev, SENSOR_CHAN_ALL);
}

static inline int sensor_sample_fetch_chan(const struct device *dev,
                                           enum sensor_channel type)
{
  return DEVICE_API_GET(sensor, dev)->sample_fetch(dev, type);
}

static inline int sensor_channel_get(const struct device *dev,
                                     enum sensor_channel chan,
                                     struct sensor_value *val)
{
  return DEVICE_API_GET(sensor, dev)->channel_get(dev, chan, val);
}

/* The asynchronous API (CONFIG_SENSOR_ASYNC_API) -----------------------*/

/* A 3-axis channel: ACCEL_XYZ, GYRO_XYZ, MAGN_XYZ or POS_DXYZ. */

#define SENSOR_CHANNEL_3_AXIS(chan) \
  ((chan) == SENSOR_CHAN_ACCEL_XYZ || (chan) == SENSOR_CHAN_GYRO_XYZ || \
   (chan) == SENSOR_CHAN_MAGN_XYZ || (chan) == SENSOR_CHAN_POS_DXYZ)

struct sensor_chan_spec
{
  uint16_t chan_type;   /* enum sensor_channel */
  uint16_t chan_idx;
};

static inline bool sensor_chan_spec_eq(struct sensor_chan_spec chan_spec0,
                                       struct sensor_chan_spec chan_spec1)
{
  return chan_spec0.chan_type == chan_spec1.chan_type &&
         chan_spec0.chan_idx == chan_spec1.chan_idx;
}

/* zephyr/drivers/sensor_data_types.h */

struct sensor_data_header
{
  uint64_t base_timestamp_ns;
  uint16_t reading_count;
};

struct sensor_three_axis_data
{
  struct sensor_data_header header;
  int8_t shift;
  struct sensor_three_axis_sample_data
    {
      uint32_t timestamp_delta;
      union
        {
          q31_t values[3];
          q31_t v[3];
          struct
            {
              q31_t x;
              q31_t y;
              q31_t z;
            };
        };
    } readings[1];
};

struct sensor_q31_data
{
  struct sensor_data_header header;
  int8_t shift;
  struct sensor_q31_sample_data
    {
      uint32_t timestamp_delta;
      union
        {
          q31_t value;
          q31_t temperature;
        };
    } readings[1];
};

struct sensor_byte_data
{
  struct sensor_data_header header;
  struct sensor_byte_sample_data
    {
      uint32_t timestamp_delta;
      union
        {
          uint8_t value;
          struct
            {
              uint8_t is_near : 1;
              uint8_t padding : 7;
            };
        };
    } readings[1];
};

struct sensor_uint64_data
{
  struct sensor_data_header header;
  struct sensor_uint64_sample_data
    {
      uint32_t timestamp_delta;
      uint64_t value;
    } readings[1];
};

/* The print formats of sensor_data_types.h: "%" PRIsensor_*_data with
 * PRIsensor_*_data_arg(data, reading index).
 */

#define PRIsensor_three_axis_data PRIu64 "ns, (%" PRIq(6) ", %" PRIq(6) \
  ", %" PRIq(6) ")"

#define PRIsensor_three_axis_data_arg(data_, readings_offset_) \
  (data_).header.base_timestamp_ns + \
    (data_).readings[(readings_offset_)].timestamp_delta, \
  PRIq_arg((data_).readings[(readings_offset_)].x, 6, (data_).shift), \
  PRIq_arg((data_).readings[(readings_offset_)].y, 6, (data_).shift), \
  PRIq_arg((data_).readings[(readings_offset_)].z, 6, (data_).shift)

#define PRIsensor_q31_data PRIu64 "ns (%" PRIq(6) ")"

#define PRIsensor_q31_data_arg(data_, readings_offset_) \
  (data_).header.base_timestamp_ns + \
    (data_).readings[(readings_offset_)].timestamp_delta, \
  PRIq_arg((data_).readings[(readings_offset_)].value, 6, (data_).shift)

#define PRIsensor_byte_data(field_name_) \
  PRIu64 "ns (" STRINGIFY(field_name_) " = %" PRIu8 ")"

#define PRIsensor_byte_data_arg(data_, readings_offset_, field_name_) \
  (data_).header.base_timestamp_ns + \
    (data_).readings[(readings_offset_)].timestamp_delta, \
  (data_).readings[(readings_offset_)].field_name_

#define PRIsensor_uint64_data PRIu64 "ns (%" PRIu64 ")"

#define PRIsensor_uint64_data_arg(data_, readings_offset_) \
  (data_).header.base_timestamp_ns + \
    (data_).readings[(readings_offset_)].timestamp_delta, \
  (data_).readings[(readings_offset_)].value

/* Decodes the buffers of a sensor's RTIO reads. */

struct sensor_decoder_api
{
  int (*get_frame_count)(const uint8_t *buffer,
                         struct sensor_chan_spec chan_spec,
                         uint16_t *frame_count);
  int (*get_size_info)(struct sensor_chan_spec channel, size_t *base_size,
                       size_t *frame_size);
  int (*decode)(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
                uint32_t *fit, uint16_t max_count, void *data_out);
  bool (*has_trigger)(const uint8_t *buffer,
                      enum sensor_trigger_type trigger);
};

struct sensor_decode_context
{
  const struct sensor_decoder_api *decoder;
  const uint8_t *buffer;
  struct sensor_chan_spec channel;
  uint32_t fit;
};

#define SENSOR_DECODE_CONTEXT_INIT(decoder_, buffer_, channel_type_, \
                                   channel_index_) \
  { \
    .decoder = (decoder_), \
    .buffer = (buffer_), \
    .channel = {.chan_type = (channel_type_), .chan_idx = (channel_index_)}, \
    .fit = 0, \
  }

static inline int sensor_decode(struct sensor_decode_context *ctx,
                                void *out, uint16_t max_count)
{
  return ctx->decoder->decode(ctx->buffer, ctx->channel, &ctx->fit,
                              max_count, out);
}

int sensor_natively_supported_channel_size_info(struct sensor_chan_spec channel,
                                                size_t *base_size,
                                                size_t *frame_size);

enum sensor_stream_data_opt
{
  SENSOR_STREAM_DATA_INCLUDE = 0,
  SENSOR_STREAM_DATA_NOP = 1,
  SENSOR_STREAM_DATA_DROP = 2,
};

struct sensor_stream_trigger
{
  enum sensor_trigger_type trigger;
  enum sensor_stream_data_opt opt;
};

#define SENSOR_STREAM_TRIGGER_PREP(_trigger, _opt) \
  { \
    .trigger = (_trigger), .opt = (_opt), \
  }

/* The data of a sensor's iodev. */

struct sensor_read_config
{
  const struct device *sensor;
  const bool is_streaming;
  union
    {
      struct sensor_chan_spec *const channels;
      struct sensor_stream_trigger *const triggers;
    };
  size_t count;
  const size_t max;
};

#define SENSOR_DT_READ_IODEV(name, dt_node, ...) \
  static struct sensor_chan_spec _CONCAT(__channel_array_, name)[] = \
    {__VA_ARGS__}; \
  static struct sensor_read_config _CONCAT(__sensor_read_config_, name) = \
    { \
      .sensor = DEVICE_DT_GET(dt_node), \
      .is_streaming = false, \
      .channels = _CONCAT(__channel_array_, name), \
      .count = ARRAY_SIZE(_CONCAT(__channel_array_, name)), \
      .max = ARRAY_SIZE(_CONCAT(__channel_array_, name)), \
    }; \
  RTIO_IODEV_DEFINE(name, &__sensor_iodev_api, \
                    _CONCAT(&__sensor_read_config_, name))

#define SENSOR_DT_STREAM_IODEV(name, dt_node, ...) \
  static struct sensor_stream_trigger _CONCAT(__trigger_array_, name)[] = \
    {__VA_ARGS__}; \
  static struct sensor_read_config _CONCAT(__sensor_read_config_, name) = \
    { \
      .sensor = DEVICE_DT_GET(dt_node), \
      .is_streaming = true, \
      .triggers = _CONCAT(__trigger_array_, name), \
      .count = ARRAY_SIZE(_CONCAT(__trigger_array_, name)), \
      .max = ARRAY_SIZE(_CONCAT(__trigger_array_, name)), \
    }; \
  RTIO_IODEV_DEFINE(name, &__sensor_iodev_api, \
                    &_CONCAT(__sensor_read_config_, name))

/* The default decoder and iodev API (sensor_default_rtio.c). */

extern const struct sensor_decoder_api __sensor_default_decoder;
extern const struct rtio_iodev_api __sensor_iodev_api;

/* The frame of the default (single read) decoder. */

struct __attribute__((__packed__)) sensor_data_generic_header
{
  uint64_t timestamp_ns;
  uint32_t num_channels;
  int8_t shift;
  int8_t _padding[sizeof(struct sensor_chan_spec) - 1];
  struct sensor_chan_spec channels[0];
};

static inline int sensor_get_decoder(const struct device *dev,
                                     const struct sensor_decoder_api **decoder)
{
  const struct sensor_driver_api *api = DEVICE_API_GET(sensor, dev);

  if (api->get_decoder == NULL)
    {
      *decoder = &__sensor_default_decoder;
      return 0;
    }

  return api->get_decoder(dev, decoder);
}

/* Starts a stream: one multishot read with the context's pool, submitted
 * now in the caller's context.  *handle (if not NULL) names it for
 * rtio_sqe_cancel().  0 or -ENOMEM (no free submission).
 */

static inline int sensor_stream(const struct rtio_iodev *iodev,
                                struct rtio *ctx, void *userdata,
                                rtio_sqe_handle_t *handle)
{
  struct rtio_sqe *sqe = rtio_sqe_acquire(ctx);

  if (sqe == NULL)
    {
      return -ENOMEM;
    }

  if (handle != NULL)
    {
      *handle = rtio_sqe_handle(ctx, sqe);
    }

  rtio_sqe_prep_read_multishot(sqe, iodev, RTIO_PRIO_NORM, userdata);
  rtio_submit(ctx, 0);
  return 0;
}

/* A blocking single read into buf. */

static inline int sensor_read(const struct rtio_iodev *iodev,
                              struct rtio *ctx, uint8_t *buf, size_t buf_len)
{
  struct rtio_sqe *sqe = rtio_sqe_acquire(ctx);

  if (sqe == NULL)
    {
      return -ENOMEM;
    }

  rtio_sqe_prep_read(sqe, iodev, RTIO_PRIO_NORM, buf, buf_len, buf);
  rtio_submit(ctx, 0);

  struct rtio_cqe *cqe = rtio_cqe_consume_block(ctx);
  int res = cqe->result;

  __ASSERT(cqe->userdata == buf,
           "consumed non-matching completion for sensor read into buffer");

  rtio_cqe_release(ctx, cqe);
  return res;
}

/* A single read into the context's pool; the completion comes later. */

static inline int sensor_read_async_mempool(const struct rtio_iodev *iodev,
                                            struct rtio *ctx, void *userdata)
{
  struct rtio_sqe *sqe = rtio_sqe_acquire(ctx);

  if (sqe == NULL)
    {
      return -ENOMEM;
    }

  rtio_sqe_prep_read_with_pool(sqe, iodev, RTIO_PRIO_NORM, userdata);
  rtio_submit(ctx, 0);
  return 0;
}

typedef void (*sensor_processing_callback_t)(int result, uint8_t *buf,
                                             uint32_t buf_len,
                                             void *userdata);

/* Waits for a completion, calls cb with its buffer, releases the buffer. */

void sensor_processing_with_callback(struct rtio *ctx,
                                     sensor_processing_callback_t cb);

/* Unit conversions (Zephyr's, with its rounding). */

#define SENSOR_G   9806650LL   /* standard gravity, um/s^2 */
#define SENSOR_PI  3141592LL   /* pi, millionths */

/* m/s^2 to g, rounded half away from zero. */

static inline int32_t sensor_ms2_to_g(const struct sensor_value *ms2)
{
  int64_t micro_ms2 = ms2->val1 * 1000000LL + ms2->val2;

  if (micro_ms2 > 0)
    {
      return (micro_ms2 + SENSOR_G / 2) / SENSOR_G;
    }
  else
    {
      return (micro_ms2 - SENSOR_G / 2) / SENSOR_G;
    }
}

static inline void sensor_g_to_ms2(int32_t g, struct sensor_value *ms2)
{
  ms2->val1 = ((int64_t)g * SENSOR_G) / 1000000LL;
  ms2->val2 = ((int64_t)g * SENSOR_G) % 1000000LL;
}

static inline int32_t sensor_ms2_to_mg(const struct sensor_value *ms2)
{
  int64_t nano_ms2 = (ms2->val1 * 1000000LL + ms2->val2) * 1000LL;

  if (nano_ms2 > 0)
    {
      return (nano_ms2 + SENSOR_G / 2) / SENSOR_G;
    }
  else
    {
      return (nano_ms2 - SENSOR_G / 2) / SENSOR_G;
    }
}

static inline int32_t sensor_ms2_to_ug(const struct sensor_value *ms2)
{
  int64_t micro_ms2 = (ms2->val1 * INT64_C(1000000)) + ms2->val2;

  return (micro_ms2 * 1000000LL) / SENSOR_G;
}

static inline void sensor_ug_to_ms2(int32_t ug, struct sensor_value *ms2)
{
  ms2->val1 = ((int64_t)ug * SENSOR_G / 1000000LL) / 1000000LL;
  ms2->val2 = ((int64_t)ug * SENSOR_G / 1000000LL) % 1000000LL;
}

/* rad/s to degrees/s, rounded half away from zero. */

static inline int32_t sensor_rad_to_degrees(const struct sensor_value *rad)
{
  int64_t micro_rad_s = rad->val1 * 1000000LL + rad->val2;

  if (micro_rad_s > 0)
    {
      return (micro_rad_s * 180LL + SENSOR_PI / 2) / SENSOR_PI;
    }
  else
    {
      return (micro_rad_s * 180LL - SENSOR_PI / 2) / SENSOR_PI;
    }
}

static inline void sensor_degrees_to_rad(int32_t d, struct sensor_value *rad)
{
  rad->val1 = ((int64_t)d * SENSOR_PI / 180LL) / 1000000LL;
  rad->val2 = ((int64_t)d * SENSOR_PI / 180LL) % 1000000LL;
}

static inline double sensor_value_to_double(const struct sensor_value *val)
{
  return (double)val->val1 + (double)val->val2 / 1000000;
}

static inline float sensor_value_to_float(const struct sensor_value *val)
{
  return (float)val->val1 + (float)val->val2 / 1000000;
}

static inline int sensor_value_from_double(struct sensor_value *val,
                                           double inp)
{
  if (inp < (double)INT32_MIN || inp > (double)INT32_MAX)
    {
      return -ERANGE;
    }

  int32_t val1 = (int32_t)inp;
  int32_t val2 = (int32_t)((inp - (double)val1) * 1000000.0);

  val->val1 = val1;
  val->val2 = val2;
  return 0;
}

static inline int sensor_value_from_float(struct sensor_value *val,
                                          float inp)
{
  if (inp < (float)INT32_MIN || inp >= (float)INT32_MAX)
    {
      return -ERANGE;
    }

  int32_t val1 = (int32_t)inp;
  int32_t val2 = (int32_t)((inp - (float)val1) * 1000000.0f);

  val->val1 = val1;
  val->val2 = val2;
  return 0;
}

static inline int64_t sensor_value_to_deci(const struct sensor_value *val)
{
  return ((int64_t)val->val1 * 10) + val->val2 / 100000;
}

static inline int64_t sensor_value_to_centi(const struct sensor_value *val)
{
  return ((int64_t)val->val1 * 100) + val->val2 / 10000;
}

static inline int64_t sensor_value_to_milli(const struct sensor_value *val)
{
  return ((int64_t)val->val1 * 1000) + val->val2 / 1000;
}

static inline int64_t sensor_value_to_micro(const struct sensor_value *val)
{
  return ((int64_t)val->val1 * 1000000) + val->val2;
}

static inline int sensor_value_from_milli(struct sensor_value *val,
                                          int64_t milli)
{
  if (milli < ((int64_t)INT32_MIN - 1) * 1000LL ||
      milli > ((int64_t)INT32_MAX + 1) * 1000LL)
    {
      return -ERANGE;
    }

  val->val1 = (int32_t)(milli / 1000);
  val->val2 = (int32_t)(milli % 1000) * 1000;
  return 0;
}

static inline int sensor_value_from_micro(struct sensor_value *val,
                                          int64_t micro)
{
  if (micro < ((int64_t)INT32_MIN - 1) * 1000000LL ||
      micro > ((int64_t)INT32_MAX + 1) * 1000000LL)
    {
      return -ERANGE;
    }

  val->val1 = (int32_t)(micro / 1000000LL);
  val->val2 = (int32_t)(micro % 1000000LL);
  return 0;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_SENSOR_H */
