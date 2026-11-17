/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the I2C0 core (common/drivers/i2c_dw/nano_i2c.c,
 * which includes this file at its end): I2C0 as the device i2c0 of the
 * Zephyr compatibility layer with Zephyr's I2C API (<zephyr/drivers/i2c.h>
 * of freertos/drivers/compat), its interrupt, reset and pins on the Pico
 * SDK, and nano_i2c0_initialize() (nano_i2c.h).
 *
 *   Zephyr (i2c_dw.c of the Zephyr build)  here
 *   i2c_dw_transfer()                      i2c_dw_transfer(): the checks
 *                                          below, then i2c_dw_transfer_msgs()
 *                                          with the speed class that
 *                                          i2c_configure() last set
 *   i2c_dw_runtime_configure()             the same: under bus_sem, -EIO
 *                                          once quarantined, otherwise
 *                                          i2c_dw_configure() at once (its
 *                                          counts and the clear of the
 *                                          interrupts, as Zephyr's)
 *   IRQ_CONNECT(), irq_enable() of         irq_set_exclusive_handler(),
 *   I2C0_IRQ (23)                          irq_set_enabled()
 *   clock_control_on(clk_sys)              nothing: clk_sys runs from the
 *                                          start (Zephyr sets bit 11 of
 *                                          CLK_SYS_CTRL, which clk_sys does
 *                                          not have; NuttX writes nothing)
 *   reset_line_toggle_dt() (Zephyr's       reset_block_num(RESET_I2C0), then
 *   reset_rpi_pico.c, on these SDK calls)  unreset_block_num_wait_blocking()
 *   pinctrl_apply_state() (Zephyr's        i2c_dw_pinctrl(): the same SDK
 *   pinctrl_rpi_pico.c)                    calls in the same order
 *   DEVICE_DT_INST_DEFINE(): k_sem_init(), nano_i2c0_initialize(), from the
 *   then i2c_dw_turn_on() at boot          application's start after
 *   (POST_KERNEL 50, after the DMA's 40)   nano_dma_initialize()
 *
 * The transfer refuses what the core does not do and Zephyr's driver
 * would do wrong or not at all: a device that is not ready (-ENODEV; in
 * Zephyr a device that failed its init is not to be called), no message
 * array, an address above 0x7f or a message with I2C_MSG_ADDR_10_BITS
 * (7-bit addressing only, as the Zephyr build uses it), a message with a
 * length and no buffer (-EINVAL).  i2c_configure() refuses
 * I2C_ADDR_10_BITS in the word (-EINVAL) and, through i2c_dw_configure(),
 * any class but standard, fast and fast plus (the RP2040's I2C has no high
 * speed); a refused class changes nothing (Zephyr's stores the minimum
 * counts, see nano_i2c.c).  The mode bits are ignored (Zephyr forces
 * controller mode).
 *
 * The speed class of a transfer is read when the transfer starts; a
 * transfer that races with an i2c_configure() of another task may run at
 * the class before it (and set the controller to it for that transfer),
 * after which the next transfer takes the configured class again.
 * (Zephyr's driver reads its class under bus_sem; the core takes bus_sem
 * itself.)  The IMU service configures the bus once, before it starts
 * reading.
 *
 * The interrupt handler runs at the Pico SDK's default priority for the
 * peripherals (0x80, the same as the DMA's and the USB's; see
 * nano_dma_instance.h), calls the core's body and, through
 * i2c_dw_os_sem_give(), the layer's k_sem_give(), which wakes the waiting
 * task with a FromISR task notification and asks for the switch at the end
 * of the handler (PendSV): the task runs right after the handler, not
 * through the timer task.  irq_set_exclusive_handler() panics if a handler
 * is already installed on I2C0_IRQ (hard_assert(), on in this Release
 * build); nothing else in this build uses I2C0 or its pins (the Pico SDK's
 * board header names GPIO12/13 as PICO_DEFAULT_I2C_SDA_PIN/_SCL_PIN, which
 * only examples and the RP2350's PSRAM code read; the SDK's start-up code
 * leaves the pins at their reset state and takes I2C0 out of reset, which
 * the reset here repeats).
 */

#ifndef FREERTOS_DRIVERS_I2C_DW_NANO_I2C_INSTANCE_H
#define FREERTOS_DRIVERS_I2C_DW_NANO_I2C_INSTANCE_H

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/addressmap.h"
#include "hardware/resets.h"

#include <zephyr/device.h>

#include "nano_i2c.h"

/* The board's i2c0 node (arduino_nano_connect.dts, -pinctrl.dtsi). */
#define I2C0_SDA_GPIO 12
#define I2C0_SCL_GPIO 13

/* The speed class that i2c_configure() last set (at first the devicetree's
 * clock-frequency, I2C_BITRATE_STANDARD, which i2c_dw_init_config() sets).
 */
static volatile uint32_t i2c0_speed;

static int i2c_dw_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			   uint16_t addr)
{
	struct i2c_dw_dev_config *const dw = dev->data;

	if (!dw->ready) {
		return -ENODEV;
	}
	if (msgs == NULL) {
		return -EINVAL;
	}
	if (num_msgs == 0) {
		return 0;
	}
	if (addr > 0x7f) {
		return -EINVAL;
	}
	for (int i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_ADDR_10_BITS) ||
		    (msgs[i].len > 0 && msgs[i].buf == NULL)) {
			return -EINVAL;
		}
	}

	return i2c_dw_transfer_msgs(dw, msgs, num_msgs, addr, i2c0_speed);
}

static int i2c_dw_runtime_configure(const struct device *dev, uint32_t config)
{
	struct i2c_dw_dev_config *const dw = dev->data;
	uint32_t speed = I2C_SPEED_GET(config);
	int ret;

	if (!dw->ready) {
		return -ENODEV;
	}
	if (config & I2C_ADDR_10_BITS) {
		return -EINVAL;
	}

	(void)i2c_dw_os_bus_lock(&dw->bus_sem);
#ifdef CONFIG_NANO_I2C0_DMA
	ret = dw->rp_dma_quarantined ? -EIO : i2c_dw_configure(dw, speed);
#else
	ret = i2c_dw_configure(dw, speed);
#endif
	if (ret == 0) {
		i2c0_speed = speed;
	}
	i2c_dw_os_bus_unlock(&dw->bus_sem);

	return ret;
}

static DEVICE_API(i2c, i2c_dw_funcs) = {
	.configure = i2c_dw_runtime_configure,
	.transfer = i2c_dw_transfer,
};

DEVICE_DT_DEFINE(i2c0, I2C_DW_NAME, NULL, &i2c_0_runtime, NULL, &i2c_dw_funcs);

/* Zephyr's i2c_dw_isr() for i2c0. */
static void i2c_dw_isr(void)
{
	i2c_dw_isr_handler(&i2c_0_runtime);
}

/* pinctrl_configure_pin() of Zephyr's RP2040 pinctrl for the board's
 * i2c0_default group: pinmux I2C0_SDA_P12, I2C0_SCL_P13 (function I2C),
 * input-enable, input-schmitt-enable; the defaults otherwise: no pulls,
 * drive-strength 4 mA, slow slew, no overrides.  The pad ends as 0x52 (IE,
 * DRIVE 4 mA, SCHMITT), the IO_BANK0 control as 0x3 (FUNCSEL I2C).
 */
static void i2c_dw_pinctrl(unsigned int gpio)
{
	gpio_init(gpio);
	gpio_set_function(gpio, GPIO_FUNC_I2C);
	gpio_set_pulls(gpio, false, false);
	gpio_set_drive_strength(gpio, GPIO_DRIVE_STRENGTH_4MA);
	gpio_set_slew_rate(gpio, GPIO_SLEW_RATE_SLOW);
	gpio_set_input_hysteresis_enabled(gpio, true);
	gpio_set_input_enabled(gpio, true);
	gpio_set_oeover(gpio, GPIO_OVERRIDE_NORMAL);
	gpio_set_outover(gpio, GPIO_OVERRIDE_NORMAL);
	gpio_set_inover(gpio, GPIO_OVERRIDE_NORMAL);
	gpio_set_irqover(gpio, GPIO_OVERRIDE_NORMAL);
}

static int i2c_dw_os_prepare(struct i2c_dw_dev_config *dw)
{
	(void)dw;

	reset_block_num(RESET_I2C0);
	unreset_block_num_wait_blocking(RESET_I2C0);

	/* pinctrl-0: the group's pins in their order. */
	i2c_dw_pinctrl(I2C0_SDA_GPIO);
	i2c_dw_pinctrl(I2C0_SCL_GPIO);

	return 0;
}

static void i2c_dw_os_irq_enable(struct i2c_dw_dev_config *dw)
{
	(void)dw;

	irq_set_exclusive_handler(I2C0_IRQ, i2c_dw_isr);
	irq_set_enabled(I2C0_IRQ, true);
}

int nano_i2c0_initialize(void)
{
	struct i2c_dw_dev_config *const dw = &i2c_0_runtime;
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	int ret;

	if (dw->ready) {
		return 0;
	}

	/* i2c_dw_initialize() */
	dw->base = I2C0_BASE;
	(void)k_sem_init(&dw->device_sync_sem, 0, K_SEM_MAX_LIMIT);
	(void)k_sem_init(&dw->bus_sem, 1, 1);
#ifdef CONFIG_NANO_I2C0_DMA
	(void)k_sem_init(&dw->rp_dma_sem, 0, K_SEM_MAX_LIMIT);
#endif

	/* pm_device_driver_init(): i2c_dw_turn_on() */
	ret = i2c_dw_turn_on(dw);
	if (ret < 0) {
		/* Zephyr's do_device_init(): the device stays not ready. */
		dev->state->init_res = (uint8_t)MIN(-ret, UINT8_MAX);
		return ret;
	}

	i2c0_speed = dw->speed;
	dw->ready = true;
	dev->state->init_res = 0;
	dev->state->initialized = true;
	return 0;
}

#endif /* FREERTOS_DRIVERS_I2C_DW_NANO_I2C_INSTANCE_H */
