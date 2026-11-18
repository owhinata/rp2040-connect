/* SPDX-License-Identifier: Apache-2.0 */
/* Fakes of the Zephyr calls of drivers/nina/nano_nina.c and nina_os.c for
 * the host test tests/nina_boundary.c, which defines the functions.  The
 * devicetree's lines are numbered as on the board (CS 9, READY 10, RESET 3,
 * IRQ 2); the flags are distinct values the test can tell apart.
 */
#ifndef NINA_TEST_H_
#define NINA_TEST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint32_t gpio_flags_t;
typedef uint32_t gpio_port_pins_t;
typedef uint8_t gpio_pin_t;

struct device {
	int unused;
};
struct gpio_dt_spec {
	const struct device *port;
	gpio_pin_t pin;
	gpio_flags_t dt_flags;
};
struct gpio_callback;
typedef void (*gpio_callback_handler_t)(const struct device *port, struct gpio_callback *cb,
					gpio_port_pins_t pins);
struct gpio_callback {
	gpio_callback_handler_t handler;
	gpio_port_pins_t pin_mask;
};
struct spi_config {
	uint32_t frequency;
	unsigned int operation;
};
struct spi_buf {
	void *buf;
	size_t len;
};
struct spi_buf_set {
	const struct spi_buf *buffers;
	size_t count;
};
struct k_mutex {
	int unused;
};
struct k_sem {
	unsigned int count, limit;
};
typedef struct {
	int64_t us;
} k_timeout_t;

#define FAKE_PIN_cs_gpios    9
#define FAKE_PIN_ready_gpios 10
#define FAKE_PIN_reset_gpios 3
#define FAKE_PIN_irq_gpios   2

#define DT_NODELABEL(x)  0
#define DEVICE_DT_GET(x) (&fake_device)
#define GPIO_DT_SPEC_GET(node, prop)                                                               \
	{                                                                                          \
		&fake_device, FAKE_PIN_##prop, 0                                                   \
	}
#define GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx)                                                   \
	{                                                                                          \
		&fake_device, FAKE_PIN_##prop, 0                                                   \
	}
#define SPI_WORD_SET(n)                  (n)
#define SPI_TRANSFER_MSB                 0
#define GPIO_OUTPUT_INACTIVE             0x11
#define GPIO_OUTPUT_ACTIVE               0x12
#define GPIO_OUTPUT_HIGH                 0x13
#define GPIO_INPUT                       0x14
#define GPIO_INT_DISABLE                 0x100
#define GPIO_INT_EDGE_TO_ACTIVE          0x200
#define GPIO_INT_EDGE_TO_INACTIVE        0x400
#define K_MUTEX_DEFINE(name)             struct k_mutex name
#define K_SEM_DEFINE(name, initial, max) struct k_sem name = {(initial), (max)}
#define K_FOREVER                        ((k_timeout_t){-1})
#define K_MSEC(n)                        ((k_timeout_t){(int64_t)(n) * 1000})
#define K_USEC(n)                        ((k_timeout_t){(int64_t)(n)})
#define CONFIG_NINA_SPI_FREQUENCY        8000000
#define MIN(a, b)                        ((a) < (b) ? (a) : (b))
#define BIT(n)                           (1u << (n))
#define ARG_UNUSED(x)                    (void)(x)

extern struct device fake_device;
bool device_is_ready(const struct device *dev);
bool gpio_is_ready_dt(const struct gpio_dt_spec *spec);
int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, gpio_flags_t flags);
int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value);
int gpio_pin_get_dt(const struct gpio_dt_spec *spec);
int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *spec, gpio_flags_t flags);
int gpio_add_callback_dt(const struct gpio_dt_spec *spec, struct gpio_callback *callback);
int gpio_remove_callback_dt(const struct gpio_dt_spec *spec, struct gpio_callback *callback);
static inline void gpio_init_callback(struct gpio_callback *callback,
				      gpio_callback_handler_t handler, gpio_port_pins_t pin_mask)
{
	callback->handler = handler;
	callback->pin_mask = pin_mask;
}
int spi_transceive(const struct device *dev, const struct spi_config *cfg,
		   const struct spi_buf_set *tx, const struct spi_buf_set *rx);
int spi_write(const struct device *dev, const struct spi_config *cfg, const struct spi_buf_set *tx);
int k_mutex_lock(struct k_mutex *lock, k_timeout_t timeout);
int k_mutex_unlock(struct k_mutex *lock);
int32_t k_sleep(k_timeout_t timeout);
int64_t k_uptime_get(void);
void k_sem_reset(struct k_sem *sem);
int k_sem_take(struct k_sem *sem, k_timeout_t timeout);
void k_sem_give(struct k_sem *sem);

/* <zephyr/shell/shell.h>, for drivers/nina/nina_passthrough.h (built with
 * CONFIG_NANO_NINA_PASSTHROUGH; the test defines nina_passthrough_active()).
 */
struct shell;

#endif
