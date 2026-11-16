/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_FIFO_IRQ_H_
#define LSM6DSO_FIFO_IRQ_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/* The worker callback must drain below the watermark, or return an error.
 * It runs on the system workqueue, never in the GPIO interrupt handler.
 */
struct lsm6dso_fifo_irq {
	struct gpio_dt_spec gpio;
	struct gpio_callback callback;
	struct k_work work;
	atomic_t enabled;
	int (*handle)(void *context);
	void *context;
};

int lsm6dso_fifo_irq_start(struct lsm6dso_fifo_irq *irq, const struct gpio_dt_spec *gpio,
			   int (*handle)(void *context), void *context);
/* Must be called from a thread other than this IRQ's work handler. */
int lsm6dso_fifo_irq_stop(struct lsm6dso_fifo_irq *irq);

#endif
