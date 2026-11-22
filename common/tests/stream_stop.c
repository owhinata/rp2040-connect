/* SPDX-License-Identifier: Apache-2.0 */
/* Exercise the production stream worker (port/zephyr/lsm6dso_rtio_stream.c) with a fake core
 * (the FIFO calls of lsm6dso_chip.h), INT1 and RTIO executor.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>
#include "lsm6dso_cal.h"
#include "lsm6dso_decoder.h"
#include "lsm6dso_reg.h"

/* Replace the Zephyr device, GPIO and RTIO dependencies and the core, not the worker itself. */
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_
#define BIT(n) (1UL << (n))
#define CONTAINER_OF(ptr, type, field) ((type *)((char *)(ptr) - offsetof(type, field)))
#define ARG_UNUSED(x) (void)(x)
#define K_NO_WAIT 0
#define K_FOREVER (-1)
#define K_MSEC(x) (x)
#define K_PRIO_PREEMPT(x) (x)
#define K_THREAD_STACK_DEFINE(name, size) char name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)
#define SENSOR_STREAM_DATA_INCLUDE 0
#define GPIO_INPUT 1
#define GPIO_INT_DISABLE 0
#define GPIO_INT_EDGE_TO_ACTIVE 2
#define LSM6DSO_DEC_1 1
#define LSM6DSO_DEC_8 2
#define LSM6DSO_DEC_32 3

typedef int atomic_t;
typedef int k_spinlock_key_t;
typedef uint32_t gpio_port_pins_t;
struct k_work { int unused; };
struct k_work_sync { int unused; };
struct k_work_delayable { struct k_work work; };
struct k_work_q { int unused; };
struct k_spinlock { int unused; };
struct device { void *data; const void *config; };
struct gpio_dt_spec { const struct device *port; uint8_t pin; };
struct gpio_callback { int unused; };
struct lsm6dso_fifo_irq {
	struct gpio_dt_spec gpio;
	struct gpio_callback callback;
	struct k_work work;
	atomic_t enabled;
	int (*handle)(void *context);
	void *context;
};
struct rtio { int unused; };
struct rtio_iodev { void *data; };
struct rtio_iodev_sqe { struct { struct rtio_iodev *iodev; } sqe; struct rtio *r; };
struct sensor_stream_trigger { int trigger, opt; };
struct sensor_read_config { bool is_streaming; size_t count; struct sensor_stream_trigger *triggers; };
struct lsm6dso_fifo_status { uint16_t words; bool watermark, full, overrun, overrun_latched; };
struct lsm6dso_config { stmdev_ctx_t ctx; struct gpio_dt_spec fifo_gpio; };
struct lsm6dso_chip {
	uint16_t accel_freq, gyro_freq;
	uint32_t acc_gain, gyro_gain;
};
/* The bus of the stream's check that the FIFO is on I2C. */
static int stmemsc_i2c_read(const void *bus, uint8_t reg, uint8_t *value, uint8_t len)
{ (void)bus; (void)reg; (void)value; (void)len; return 0; }
struct lsm6dso_data {
	const struct device *dev;
	struct k_work_delayable stream_work;
	struct k_spinlock stream_lock;
	uint16_t stream_watermark;
	struct rtio_iodev_sqe *stream_sqe;
	struct rtio *stream_rtio;
	rtio_sqe_handle_t stream_handle;
	struct lsm6dso_fifo_irq fifo_irq;
	bool stream_active, stream_irq_active, stream_stop_requested, stream_batch_paused;
	struct lsm6dso_chip chip;
};

static struct rtio test_rtio;
static struct rtio_iodev_sqe *live_sqe;
static struct device *test_device;
static unsigned int fifo_words, injected_tail, pause_calls, stop_calls, completions;
static unsigned int delivered_words;
static bool batching_paused, inject_stop, fail_pause, fail_alloc, overrun;
static int terminal_error;
static union { uint64_t align; uint8_t bytes[4096]; } receive_buffer;

static int atomic_cas(atomic_t *a, int old, int val) { if (*a != old) return 0; *a = val; return 1; }
static int k_work_reschedule_for_queue(struct k_work_q *q, struct k_work_delayable *w, int t)
{ (void)q; (void)w; (void)t; return 0; }
static int k_work_schedule_for_queue(struct k_work_q *q, struct k_work_delayable *w, int t)
{ (void)q; (void)w; (void)t; return 0; }
static void k_work_queue_start(struct k_work_q *q, char *s, size_t n, int p, void *c)
{ (void)q; (void)s; (void)n; (void)p; (void)c; }
static void k_work_init_delayable(struct k_work_delayable *w, void (*f)(struct k_work *))
{ (void)w; (void)f; }
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w)
{ return CONTAINER_OF(w, struct k_work_delayable, work); }
static k_spinlock_key_t k_spin_lock(struct k_spinlock *l) { (void)l; return 0; }
static void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t k) { (void)l; (void)k; }
static bool gpio_is_ready_dt(const struct gpio_dt_spec *g) { (void)g; return true; }
/* INT1: the routing and the GPIO interrupt do nothing. */
static int lsm6dso_chip_fifo_route_int1(struct lsm6dso_chip *c, bool enabled)
{ (void)c; (void)enabled; return 0; }
static int gpio_pin_configure_dt(const struct gpio_dt_spec *g, int flags)
{ (void)g; (void)flags; return 0; }
static int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *g, int flags)
{ (void)g; (void)flags; return 0; }
static int gpio_pin_get_dt(const struct gpio_dt_spec *g) { (void)g; return 0; }
static void gpio_init_callback(struct gpio_callback *c,
			       void (*f)(const struct device *, struct gpio_callback *,
					 gpio_port_pins_t), gpio_port_pins_t pins)
{ (void)c; (void)f; (void)pins; }
static int gpio_add_callback(const struct device *p, struct gpio_callback *c)
{ (void)p; (void)c; return 0; }
static int gpio_remove_callback(const struct device *p, struct gpio_callback *c)
{ (void)p; (void)c; return 0; }
static void k_work_init(struct k_work *w, void (*f)(struct k_work *)) { (void)w; (void)f; }
static int k_work_submit(struct k_work *w) { (void)w; return 0; }
static bool k_work_cancel_sync(struct k_work *w, struct k_work_sync *s)
{ (void)w; (void)s; return false; }
static int atomic_get(const atomic_t *a) { return *a; }
static void atomic_set(atomic_t *a, int v) { *a = v; }
static void atomic_clear(atomic_t *a) { *a = 0; }
static int lsm6dso_chip_fifo_start(struct lsm6dso_chip *c, uint16_t w, bool a, bool g)
{ (void)c; (void)w; (void)a; (void)g; return 0; }
static int lsm6dso_chip_fifo_stop(struct lsm6dso_chip *c)
{
	(void)c;
	stop_calls++;
	return 0;
}
static void lsm6dso_chip_fifo_record_stop_error(struct lsm6dso_chip *c, int error)
{ (void)c; (void)error; }
/* The decimation's calls are not used by the test (and not linked). */
int lsm6dso_chip_fifo_set_timestamp_decimation(struct lsm6dso_chip *c, unsigned int decimation);
unsigned int lsm6dso_chip_fifo_get_timestamp_decimation(struct lsm6dso_chip *c);
static int lsm6dso_chip_fifo_pause_batch(struct lsm6dso_chip *c)
{
	(void)c;
	pause_calls++;
	if (fail_pause) return -EIO;
	batching_paused = true;
	return 0;
}
static int lsm6dso_chip_fifo_status(struct lsm6dso_chip *c, struct lsm6dso_fifo_status *status)
{
	(void)c;
	*status = (struct lsm6dso_fifo_status){.words = fifo_words, .watermark = fifo_words >= 32,
						.overrun = overrun};
	return 0;
}
int lsm6dso_fifo_request_stop(const struct device *, struct rtio *, rtio_sqe_handle_t);
static int lsm6dso_chip_fifo_read(struct lsm6dso_chip *c, uint8_t *out, size_t capacity,
				  struct lsm6dso_fifo_status *status)
{
	(void)c;
	assert(capacity >= fifo_words);
	status->words = fifo_words;
	status->watermark = fifo_words >= 32;
	status->overrun = false;
	status->overrun_latched = false;
	for (unsigned int i = 0; i < fifo_words; i++) {
		memset(out + i * 7, 0, 7);
		out[i * 7] = i % 2 ? 0x08 : 0x10;
	}
	fifo_words = injected_tail;
	injected_tail = 0;
	if (inject_stop) {
		inject_stop = false;
		assert(lsm6dso_fifo_request_stop(test_device, &test_rtio, 7) == 0);
	}
	return 0;
}
static struct rtio_iodev_sqe *rtio_iodev_sqe_from_handle(struct rtio *r, rtio_sqe_handle_t h)
{ return r == &test_rtio && h == 7 ? live_sqe : NULL; }
static bool rtio_iodev_sqe_is_canceled(const struct rtio_iodev_sqe *s) { (void)s; return false; }
static rtio_sqe_handle_t rtio_sqe_handle(struct rtio *r, const void *s)
{ (void)r; (void)s; return 7; }
static int rtio_sqe_rx_buf(struct rtio_iodev_sqe *s, uint32_t min, uint32_t max,
			   uint8_t **buf, uint32_t *len)
{
	(void)s; (void)max;
	if (fail_alloc) return -ENOMEM;
	assert(min <= sizeof(receive_buffer.bytes));
	*buf = receive_buffer.bytes;
	*len = sizeof(receive_buffer.bytes);
	return 0;
}
static void lsm6dso_chip_cal_track(struct lsm6dso_chip *c, const uint8_t *words, size_t len)
{
	(void)c;
	(void)words;
	(void)len;
}

static void lsm6dso_chip_cal_snapshot(struct lsm6dso_chip *c, struct lsm6dso_cal *accel,
				      struct lsm6dso_cal *gyro)
{
	(void)c;
	lsm6dso_cal_nominal(accel, LSM6DSO_CAL_ACCEL, 244);
	lsm6dso_cal_nominal(gyro, LSM6DSO_CAL_GYRO, 70000);
}

void lsm6dso_submit_stream(const struct device *d, struct rtio_iodev_sqe *s);
static void rtio_iodev_sqe_ok(struct rtio_iodev_sqe *s, int result)
{
	struct lsm6dso_fifo_packet *p = (void *)receive_buffer.bytes;
	(void)s;
	assert(result == 0 && p->marker == LSM6DSO_FIFO_PACKET_MARKER);
	completions++;
	delivered_words += p->words;
	/* RTIO resubmits a multishot SQE before reporting the completion. */
	lsm6dso_submit_stream(test_device, s);
}
static void rtio_iodev_sqe_err(struct rtio_iodev_sqe *s, int result)
{ assert(s == live_sqe); terminal_error = result; live_sqe = NULL; }
static int sensor_clock_get_cycles(uint64_t *v) { *v = 1000000000; return 0; }
static uint64_t sensor_clock_cycles_to_ns(uint64_t v) { return v; }

#include "../drivers/lsm6dso/port/zephyr/lsm6dso_rtio_stream.c"

int main(void)
{
	struct lsm6dso_data data = { .chip = { .accel_freq = 208, .gyro_freq = 208 } };
	struct lsm6dso_config cfg = {0};
	struct device dev = { .data = &data, .config = &cfg };
	struct sensor_stream_trigger trigger = { SENSOR_TRIG_FIFO_WATERMARK,
						 SENSOR_STREAM_DATA_INCLUDE };
	struct sensor_read_config read_cfg = { true, 1, &trigger };
	struct rtio_iodev iodev = { &read_cfg };
	struct rtio_iodev_sqe sqe = { .sqe.iodev = &iodev, .r = &test_rtio };

	test_device = &dev;
	data.dev = &dev;
	data.stream_active = true;
	live_sqe = &sqe;
	lsm6dso_submit_stream(&dev, &sqe);
	assert(lsm6dso_fifo_request_stop(&dev, &test_rtio, 8) == -ENOENT);
	/* The stop request arrives during a full FIFO read. That batch must be
	 * delivered, followed by the final partial batch, before bypass mode.
	 */
	fifo_words = 32;
	injected_tail = 5;
	inject_stop = true;
	stream_work(&data.stream_work.work);
	assert(completions == 1 && delivered_words == 32 && stop_calls == 0);
	stream_work(&data.stream_work.work);
	assert(batching_paused && pause_calls == 1 && fifo_words == 0 && completions == 2 &&
	       delivered_words == 37 && stop_calls == 0);
	stream_work(&data.stream_work.work);
	assert(stop_calls == 1 && terminal_error == -ECANCELED && live_sqe == NULL);

	/* A pause failure must report an error rather than claiming a clean drain. */
	memset(&data, 0, sizeof(data));
	data.dev = &dev;
	data.stream_active = true;
	live_sqe = &sqe;
	fail_pause = true;
	terminal_error = 0;
	lsm6dso_submit_stream(&dev, &sqe);
	assert(lsm6dso_fifo_request_stop(&dev, &test_rtio, 7) == 0);
	stream_work(&data.stream_work.work);
	assert(terminal_error == -EIO && live_sqe == NULL);

	/* A full buffer or FIFO overrun must not produce a successful final CQE. */
	fail_pause = false;
	fail_alloc = true;
	fifo_words = 5;
	memset(&data, 0, sizeof(data));
	data.dev = &dev;
	data.stream_active = true;
	live_sqe = &sqe;
	terminal_error = 0;
	lsm6dso_submit_stream(&dev, &sqe);
	assert(lsm6dso_fifo_request_stop(&dev, &test_rtio, 7) == 0);
	stream_work(&data.stream_work.work);
	assert(terminal_error == -ENOMEM && live_sqe == NULL && completions == 2);

	fail_alloc = false;
	overrun = true;
	memset(&data, 0, sizeof(data));
	data.dev = &dev;
	data.stream_active = true;
	live_sqe = &sqe;
	terminal_error = 0;
	lsm6dso_submit_stream(&dev, &sqe);
	assert(lsm6dso_fifo_request_stop(&dev, &test_rtio, 7) == 0);
	stream_work(&data.stream_work.work);
	assert(terminal_error == -EOVERFLOW && live_sqe == NULL && completions == 2);

	/* An empty FIFO stops without allocating or delivering an empty batch. */
	overrun = false;
	fifo_words = 0;
	memset(&data, 0, sizeof(data));
	data.dev = &dev;
	data.stream_active = true;
	live_sqe = &sqe;
	terminal_error = 0;
	lsm6dso_submit_stream(&dev, &sqe);
	assert(lsm6dso_fifo_request_stop(&dev, &test_rtio, 7) == 0);
	stream_work(&data.stream_work.work);
	assert(terminal_error == -ECANCELED && live_sqe == NULL && completions == 2);

	/* A stop before the worker starts does not start FIFO acquisition. */
	memset(&data, 0, sizeof(data));
	data.dev = &dev;
	live_sqe = &sqe;
	terminal_error = 0;
	lsm6dso_submit_stream(&dev, &sqe);
	assert(lsm6dso_fifo_request_stop(&dev, &test_rtio, 7) == 0);
	stream_work(&data.stream_work.work);
	assert(terminal_error == -ECANCELED && live_sqe == NULL && completions == 2);
	/* Configuration has bounds and must not change a live stream. */
	assert(lsm6dso_fifo_set_watermark(&dev, 0) == -EINVAL);
	assert(lsm6dso_fifo_set_watermark(&dev, 257) == -EINVAL);
	assert(lsm6dso_fifo_set_watermark(&dev, 128) == 0);
	assert(lsm6dso_fifo_get_watermark(&dev) == 128);
	data.stream_sqe = &sqe;
	assert(lsm6dso_fifo_set_watermark(&dev, 64) == -EBUSY);
	assert(lsm6dso_fifo_get_watermark(&dev) == 128);
	data.stream_sqe = NULL;
	assert(lsm6dso_fifo_set_watermark(&dev, 256) == 0);
	return 0;
}
