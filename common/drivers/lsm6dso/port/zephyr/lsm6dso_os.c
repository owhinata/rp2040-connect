/* SPDX-License-Identifier: Apache-2.0 */
/* The functions and objects of the OS's part on Zephyr (lsm6dso_os_impl.h)
 * that are not inline: the FIFO stream's work queue, and the INT1 line's
 * GPIO interrupt, which wakes the stream through the system work queue.
 */
#include "lsm6dso_os.h"

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#define WORK_Q_STACK_SIZE 2048

/* Keep FIFO I/O off Zephyr's cooperative system workqueue so a busy or
 * failing stream cannot monopolize USB and shell processing.
 */
struct k_work_q lsm6dso_os_work_q;
K_THREAD_STACK_DEFINE(lsm6dso_os_work_q_stack, WORK_Q_STACK_SIZE);
static atomic_t work_q_started;

void lsm6dso_os_work_q_start(void)
{
	if (atomic_cas(&work_q_started, 0, 1)) {
		k_work_queue_start(&lsm6dso_os_work_q, lsm6dso_os_work_q_stack,
				   K_THREAD_STACK_SIZEOF(lsm6dso_os_work_q_stack),
				   K_PRIO_PREEMPT(10), NULL);
#if defined(CONFIG_THREAD_NAME)
		(void)k_thread_name_set(k_work_queue_thread_get(&lsm6dso_os_work_q),
					"imu_fifo_worker");
#endif
	}
}

/* INT1: the GPIO interrupt of the FIFO threshold, which the core routes to
 * the sensor's pin (lsm6dso_fifo_route_int1()), wakes the stream's work
 * queue through the system work queue, never from the interrupt handler.
 */

static void int1_work(struct k_work *work)
{
	lsm6dso_os_int1_t *int1 = CONTAINER_OF(work, lsm6dso_os_int1_t, work);

	if (!atomic_get(&int1->enabled)) {
		return;
	}
	if (lsm6dso_stream_int1(int1) != 0) {
		atomic_clear(&int1->enabled);
		(void)gpio_pin_interrupt_configure_dt(&int1->gpio, GPIO_INT_DISABLE);
	}
}

static void int1_callback(const struct device *port, struct gpio_callback *callback,
			  gpio_port_pins_t pins)
{
	lsm6dso_os_int1_t *int1 = CONTAINER_OF(callback, lsm6dso_os_int1_t, callback);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	if (atomic_get(&int1->enabled)) {
		(void)k_work_submit(&int1->work);
	}
}

int lsm6dso_os_int1_disable(lsm6dso_os_int1_t *int1)
{
	struct k_work_sync sync;
	int ret, remove_ret;

	atomic_clear(&int1->enabled);
	ret = gpio_pin_interrupt_configure_dt(&int1->gpio, GPIO_INT_DISABLE);
	remove_ret = gpio_remove_callback(int1->gpio.port, &int1->callback);
	(void)k_work_cancel_sync(&int1->work, &sync);
	return ret != 0 ? ret : remove_ret;
}

int lsm6dso_os_int1_enable(lsm6dso_os_int1_t *int1)
{
	const struct gpio_dt_spec *gpio = &int1->gpio;
	int ret;

	if (!gpio_is_ready_dt(gpio)) {
		return -EINVAL;
	}
	k_work_init(&int1->work, int1_work);
	atomic_clear(&int1->enabled);
	ret = gpio_pin_configure_dt(gpio, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}
	gpio_init_callback(&int1->callback, int1_callback, BIT(gpio->pin));
	ret = gpio_add_callback(gpio->port, &int1->callback);
	if (ret != 0) {
		return ret;
	}
	atomic_set(&int1->enabled, 1);
	ret = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		atomic_clear(&int1->enabled);
		(void)gpio_remove_callback(gpio->port, &int1->callback);
		return ret;
	}
	return 0;
}
