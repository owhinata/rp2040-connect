/* SPDX-License-Identifier: Apache-2.0 */
/* Exercise the production stream worker with a fake FIFO and RTIO executor. */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>
#include "lsm6dso_fifo.h"
#include "lsm6dso_fifo_decoder.h"

/* Replace the Zephyr device and RTIO dependencies, not the worker itself. */
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_
#define LSM6DSO_FIFO_HW_H_
#define CONTAINER_OF(ptr, type, field) ((type *)((char *)(ptr) - offsetof(type, field)))
#define ARG_UNUSED(x) (void)(x)
#define K_NO_WAIT 0
#define K_FOREVER (-1)
#define K_MSEC(x) (x)
#define K_PRIO_PREEMPT(x) (x)
#define K_THREAD_STACK_DEFINE(name, size) char name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)
#define SENSOR_STREAM_DATA_INCLUDE 0
#define LSM6DSO_FIFO_DEFAULT_WATERMARK 32U
#define LSM6DSO_FIFO_MAX_WATERMARK 256U
#define LSM6DSO_NO_DECIMATION 0
#define LSM6DSO_DEC_1 1
#define LSM6DSO_DEC_8 2
#define LSM6DSO_DEC_32 3

typedef int atomic_t;
typedef int k_spinlock_key_t;
typedef uint32_t rtio_sqe_handle_t;
struct k_work { int unused; };
struct k_work_delayable { struct k_work work; };
struct k_work_q { int unused; };
struct k_spinlock { int unused; };
struct k_mutex { int unused; };
static inline void k_mutex_lock(struct k_mutex *lock, int timeout) { (void)lock; (void)timeout; }
static inline void k_mutex_unlock(struct k_mutex *lock) { (void)lock; }
struct gpio_dt_spec { int unused; };
struct lsm6dso_fifo_irq { int unused; };
struct device { void *data; const void *config; };
struct rtio { int unused; };
struct rtio_iodev { void *data; };
struct rtio_iodev_sqe { struct { struct rtio_iodev *iodev; } sqe; struct rtio *r; };
struct sensor_stream_trigger { int trigger, opt; };
struct sensor_read_config { bool is_streaming; size_t count; struct sensor_stream_trigger *triggers; };
struct lsm6dso_fifo_status { uint16_t words; bool watermark, full, overrun, overrun_latched; };
struct lsm6dso_config { int ctx; struct gpio_dt_spec fifo_gpio; };
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
	uint16_t accel_freq, gyro_freq;
	uint32_t acc_gain, gyro_gain;
	struct k_mutex fifo_lock;
	bool fifo_owned;
	uint8_t timestamp_decimation;
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
static int lsm6dso_fifo_route_int1(const int *ctx, bool enabled)
{ (void)ctx; (void)enabled; return 0; }
static int lsm6dso_fifo_irq_start(struct lsm6dso_fifo_irq *i, const struct gpio_dt_spec *g,
				 int (*f)(void *), void *arg)
{ (void)i; (void)g; (void)f; (void)arg; return 0; }
static int lsm6dso_fifo_irq_stop(struct lsm6dso_fifo_irq *i) { (void)i; return 0; }
static int lsm6dso_fifo_start_locked_driver(const struct device *d, uint16_t w, bool a, bool g)
{ (void)d; (void)w; (void)a; (void)g; return 0; }
static int lsm6dso_fifo_stop_locked_driver(const struct device *d)
{
	(void)d;
	stop_calls++;
	return 0;
}
static void lsm6dso_fifo_record_stop_error(const struct device *d, int error)
{ (void)d; (void)error; }
static int lsm6dso_fifo_pause_batch_locked_driver(const struct device *d)
{
	(void)d;
	pause_calls++;
	if (fail_pause) return -EIO;
	batching_paused = true;
	return 0;
}
static int lsm6dso_fifo_status_locked_driver(const struct device *d,
					       struct lsm6dso_fifo_status *status)
{
	(void)d;
	*status = (struct lsm6dso_fifo_status){.words = fifo_words, .watermark = fifo_words >= 32,
						.overrun = overrun};
	return 0;
}
int lsm6dso_fifo_request_stop(const struct device *, struct rtio *, rtio_sqe_handle_t);
static int lsm6dso_fifo_read_locked_driver(const struct device *d, uint8_t *out, size_t capacity,
					     struct lsm6dso_fifo_status *status)
{
	(void)d;
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
void lsm6dso_submit(const struct device *d, struct rtio_iodev_sqe *s);
static void rtio_iodev_sqe_ok(struct rtio_iodev_sqe *s, int result)
{
	struct lsm6dso_fifo_packet *p = (void *)receive_buffer.bytes;
	(void)s;
	assert(result == 0 && p->marker == LSM6DSO_FIFO_PACKET_MARKER);
	completions++;
	delivered_words += p->words;
	/* RTIO resubmits a multishot SQE before reporting the completion. */
	lsm6dso_submit(test_device, s);
}
static void rtio_iodev_sqe_err(struct rtio_iodev_sqe *s, int result)
{ assert(s == live_sqe); terminal_error = result; live_sqe = NULL; }
static int sensor_clock_get_cycles(uint64_t *v) { *v = 1000000000; return 0; }
static uint64_t sensor_clock_cycles_to_ns(uint64_t v) { return v; }
void lsm6dso_submit_fallback(const struct device *d, struct rtio_iodev_sqe *s)
{ (void)d; (void)s; assert(false); }

#include "../drivers/lsm6dso/lsm6dso_stream.c"

int main(void)
{
	struct lsm6dso_data data = { .accel_freq = 208, .gyro_freq = 208 };
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
	lsm6dso_submit(&dev, &sqe);
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
	lsm6dso_submit(&dev, &sqe);
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
	lsm6dso_submit(&dev, &sqe);
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
	lsm6dso_submit(&dev, &sqe);
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
	lsm6dso_submit(&dev, &sqe);
	assert(lsm6dso_fifo_request_stop(&dev, &test_rtio, 7) == 0);
	stream_work(&data.stream_work.work);
	assert(terminal_error == -ECANCELED && live_sqe == NULL && completions == 2);

	/* A stop before the worker starts does not start FIFO acquisition. */
	memset(&data, 0, sizeof(data));
	data.dev = &dev;
	live_sqe = &sqe;
	terminal_error = 0;
	lsm6dso_submit(&dev, &sqe);
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
