/* SPDX-License-Identifier: Apache-2.0 */
#include "lsm6dso_fifo_irq.h"

#include <errno.h>

static void fifo_irq_work(struct k_work *work)
{
	struct lsm6dso_fifo_irq *irq = CONTAINER_OF(work, struct lsm6dso_fifo_irq, work);

	if (!atomic_get(&irq->enabled)) {
		return;
	}
	/* The callback owns error reporting and stream termination. */
	if (irq->handle(irq->context) != 0) {
		atomic_clear(&irq->enabled);
		(void)gpio_pin_interrupt_configure_dt(&irq->gpio, GPIO_INT_DISABLE);
	}
}

static void fifo_gpio_callback(const struct device *port, struct gpio_callback *callback,
			       gpio_port_pins_t pins)
{
	struct lsm6dso_fifo_irq *irq = CONTAINER_OF(callback, struct lsm6dso_fifo_irq, callback);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	if (atomic_get(&irq->enabled)) {
		(void)k_work_submit(&irq->work);
	}
}

int lsm6dso_fifo_irq_start(struct lsm6dso_fifo_irq *irq, const struct gpio_dt_spec *gpio,
			   int (*handle)(void *context), void *context)
{
	int ret;

	if (!irq || !gpio || !handle || !gpio_is_ready_dt(gpio)) {
		return -EINVAL;
	}
	irq->gpio = *gpio;
	irq->handle = handle;
	irq->context = context;
	k_work_init(&irq->work, fifo_irq_work);
	atomic_clear(&irq->enabled);
	ret = gpio_pin_configure_dt(gpio, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}
	gpio_init_callback(&irq->callback, fifo_gpio_callback, BIT(gpio->pin));
	ret = gpio_add_callback(gpio->port, &irq->callback);
	if (ret != 0) {
		return ret;
	}
	atomic_set(&irq->enabled, 1);
	ret = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		atomic_clear(&irq->enabled);
		(void)gpio_remove_callback(gpio->port, &irq->callback);
		return ret;
	}
	/* Do not lose a level that was already asserted before edge detection. */
	ret = gpio_pin_get_dt(gpio);
	if (ret < 0) {
		(void)lsm6dso_fifo_irq_stop(irq);
		return ret;
	}
	if (ret > 0) {
		(void)k_work_submit(&irq->work);
	}
	return 0;
}

int lsm6dso_fifo_irq_stop(struct lsm6dso_fifo_irq *irq)
{
	struct k_work_sync sync;
	int ret, remove_ret;

	if (!irq) {
		return -EINVAL;
	}
	atomic_clear(&irq->enabled);
	ret = gpio_pin_interrupt_configure_dt(&irq->gpio, GPIO_INT_DISABLE);
	remove_ret = gpio_remove_callback(irq->gpio.port, &irq->callback);
	(void)k_work_cancel_sync(&irq->work, &sync);
	return ret != 0 ? ret : remove_ret;
}
