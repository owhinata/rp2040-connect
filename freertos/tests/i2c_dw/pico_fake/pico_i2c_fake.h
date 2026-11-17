/* SPDX-License-Identifier: MIT */
/* The part of the Pico SDK that the FreeRTOS part of the I2C0 and DMA core
 * uses (freertos/drivers/i2c_dw: i2c_dw_os_impl.h, nano_i2c_instance.h,
 * nano_dma_instance.h, nano_dma_os.c), for its host test
 * (freertos/tests/i2c_dw/main.c, which implements the functions: they
 * record the calls).  pico.h and the hardware/ headers next to this one
 * include it.  Names and values as in the SDK (SYS_CLK_HZ of
 * hardware/platform_defs.h, hardware_gpio, hardware_resets, hardware_irq,
 * hardware/regs/intctrl.h, hardware_sync, hardware_dma), except I2C0_BASE:
 * the I2C0 block's registers are the array fake_i2c0_regs here.
 */

#ifndef FREERTOS_TESTS_I2C_DW_PICO_FAKE_PICO_I2C_FAKE_H
#define FREERTOS_TESTS_I2C_DW_PICO_FAKE_PICO_I2C_FAKE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;

/* hardware/platform_defs.h */
#define SYS_CLK_HZ 125000000u

/* hardware/regs/addressmap.h: the registers of I2C0 (0x100 bytes). */
extern volatile uint32_t fake_i2c0_regs[64];
#define I2C0_BASE ((uintptr_t)fake_i2c0_regs)

/* hardware_gpio */
typedef enum gpio_function {
	GPIO_FUNC_I2C = 3,
	GPIO_FUNC_SIO = 5,
} gpio_function_t;

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

/* hardware_resets */
enum reset_num_rp2040 {
	RESET_DMA = 2,
	RESET_I2C0 = 3,
};

void reset_block_num(uint32_t block_num);
void unreset_block_num_wait_blocking(uint32_t block_num);

/* hardware/regs/intctrl.h and hardware_irq */
#define DMA_IRQ_0 11
#define DMA_IRQ_1 12
#define I2C0_IRQ  23

typedef void (*irq_handler_t)(void);

void irq_set_exclusive_handler(uint num, irq_handler_t handler);
void irq_set_enabled(uint num, bool enabled);

/* hardware_sync */
uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t status);

/* hardware_dma */
void dma_claim_mask(uint32_t channel_mask);

#endif /* FREERTOS_TESTS_I2C_DW_PICO_FAKE_PICO_I2C_FAKE_H */
