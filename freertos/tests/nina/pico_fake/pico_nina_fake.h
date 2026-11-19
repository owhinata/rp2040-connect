/* SPDX-License-Identifier: MIT */
/* The part of the Pico SDK that freertos/drivers/nina/nina_bus.c and the
 * NINA core's FreeRTOS binding (common/drivers/nina/freertos/nina_os.c: the
 * GPIO interrupt functions for READY's and the IRQ line's edges, the TIMER)
 * use, for their host test (freertos/tests/nina/main.c, which implements
 * the functions: they record the calls and play the module's side of the
 * lines and of SPI1).  hardware/gpio.h, hardware/irq.h, hardware/resets.h,
 * hardware/spi.h and hardware/timer.h next to this one include it.  Names
 * and values as in the SDK (hardware_gpio, hardware_irq, hardware_resets,
 * hardware_spi, hardware_timer, hardware/regs/intctrl.h,
 * hardware/regs/spi.h).
 */

#ifndef FREERTOS_TESTS_NINA_PICO_FAKE_PICO_NINA_FAKE_H
#define FREERTOS_TESTS_NINA_PICO_FAKE_PICO_NINA_FAKE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;

/* hardware_gpio */
typedef enum gpio_function {
	GPIO_FUNC_SPI = 1,
	GPIO_FUNC_SIO = 5,
} gpio_function_t;

enum gpio_dir {
	GPIO_IN = 0,
	GPIO_OUT = 1,
};

enum gpio_drive_strength {
	GPIO_DRIVE_STRENGTH_2MA = 0,
	GPIO_DRIVE_STRENGTH_4MA = 1,
};

enum gpio_slew_rate {
	GPIO_SLEW_RATE_SLOW = 0,
	GPIO_SLEW_RATE_FAST = 1,
};

enum gpio_override {
	GPIO_OVERRIDE_NORMAL = 0,
};

void gpio_init(uint gpio);
void gpio_set_function(uint gpio, gpio_function_t fn);
void gpio_set_pulls(uint gpio, bool up, bool down);
void gpio_set_drive_strength(uint gpio, enum gpio_drive_strength drive);
void gpio_set_slew_rate(uint gpio, enum gpio_slew_rate slew);
void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled);
void gpio_set_input_enabled(uint gpio, bool enabled);
void gpio_set_oeover(uint gpio, uint value);
void gpio_set_outover(uint gpio, uint value);
void gpio_set_inover(uint gpio, uint value);
void gpio_set_irqover(uint gpio, uint value);
void gpio_set_dir(uint gpio, bool out);
void gpio_put(uint gpio, bool value);
bool gpio_get(uint gpio);

enum gpio_irq_level {
	GPIO_IRQ_LEVEL_LOW = 0x1u,
	GPIO_IRQ_LEVEL_HIGH = 0x2u,
	GPIO_IRQ_EDGE_FALL = 0x4u,
	GPIO_IRQ_EDGE_RISE = 0x8u,
};

uint32_t gpio_get_all(void);
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, bool enabled);
uint32_t gpio_get_irq_event_mask(uint gpio);
void gpio_acknowledge_irq(uint gpio, uint32_t event_mask);

/* hardware/regs/intctrl.h and hardware_irq */
#define IO_IRQ_BANK0 13

typedef void (*irq_handler_t)(void);

void gpio_add_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler);
void irq_set_enabled(uint num, bool enabled);

/* hardware_resets */
typedef enum reset_num {
	RESET_SPI1 = 17,
} reset_num_t;

void reset_block_num(reset_num_t block_num);
void unreset_block_num_wait_blocking(reset_num_t block_num);

/* hardware_spi */
typedef struct {
	volatile uint32_t cr0;
	volatile uint32_t cr1;
	volatile uint32_t dr;
	volatile uint32_t sr;
	volatile uint32_t cpsr;
	volatile uint32_t imsc;
	volatile uint32_t ris;
	volatile uint32_t mis;
	volatile uint32_t icr;
	volatile uint32_t dmacr;
} spi_hw_t;

typedef struct spi_inst spi_inst_t;

extern spi_inst_t *const fake_spi1;
#define spi1 fake_spi1

typedef enum {
	SPI_CPHA_0 = 0,
	SPI_CPHA_1 = 1,
} spi_cpha_t;

typedef enum {
	SPI_CPOL_0 = 0,
	SPI_CPOL_1 = 1,
} spi_cpol_t;

typedef enum {
	SPI_LSB_FIRST = 0,
	SPI_MSB_FIRST = 1,
} spi_order_t;

#define SPI_SSPCR1_SSE_BITS 0x00000002u

spi_hw_t *spi_get_hw(spi_inst_t *spi);
uint spi_set_baudrate(spi_inst_t *spi, uint baudrate);
void spi_set_format(spi_inst_t *spi, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha,
		    spi_order_t order);
int spi_write_read_blocking(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len);
int spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len);

/* hardware_timer */
uint64_t time_us_64(void);

/* hardware/address_mapped.h */
void hw_set_bits(volatile uint32_t *addr, uint32_t mask);

#endif /* FREERTOS_TESTS_NINA_PICO_FAKE_PICO_NINA_FAKE_H */
