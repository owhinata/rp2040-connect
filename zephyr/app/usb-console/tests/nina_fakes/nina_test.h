/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NINA_TEST_H_
#define NINA_TEST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct device { int unused; };
struct gpio_dt_spec { int unused; };
struct spi_config { uint32_t frequency; unsigned int operation; };
struct spi_buf { void *buf; size_t len; };
struct spi_buf_set { const struct spi_buf *buffers; size_t count; };
struct k_mutex { int unused; };

#define DT_NODELABEL(x) 0
#define DEVICE_DT_GET(x) (&fake_device)
#define GPIO_DT_SPEC_GET(node, prop) {0}
#define GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx) {0}
#define SPI_WORD_SET(n) (n)
#define SPI_TRANSFER_MSB 0
#define GPIO_OUTPUT_INACTIVE 0
#define GPIO_OUTPUT_ACTIVE 1
#define GPIO_OUTPUT_HIGH 2
#define GPIO_INPUT 3
#define K_MUTEX_DEFINE(name) struct k_mutex name
#define K_FOREVER 0
#define K_MSEC(n) (n)
#define K_USEC(n) (n)
#define CONFIG_NINA_SPI_FREQUENCY 8000000
#define MIN(a, b) ((a) < (b) ? (a) : (b))

extern struct device fake_device;
bool device_is_ready(const struct device *dev);
bool gpio_is_ready_dt(const struct gpio_dt_spec *spec);
int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, int flags);
int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value);
int gpio_pin_get_dt(const struct gpio_dt_spec *spec);
int spi_transceive(const struct device *dev, const struct spi_config *cfg,
		   const struct spi_buf_set *tx, const struct spi_buf_set *rx);
int spi_write(const struct device *dev, const struct spi_config *cfg,
	      const struct spi_buf_set *tx);
void k_mutex_lock(struct k_mutex *lock, int timeout);
void k_mutex_unlock(struct k_mutex *lock);
void k_sleep(int ms);
int64_t k_uptime_get(void);

#endif
