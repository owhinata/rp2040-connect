/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the I2C core (common/drivers/i2c_dw/i2c_dw.c,
 * which includes this file at its end): I2C0 and, with CONFIG_I2C_DW_I2C1,
 * I2C1 as the devices i2c0 and i2c1 of the Zephyr compatibility layer with
 * Zephyr's I2C API (<zephyr/drivers/i2c.h> of freertos/drivers/compat),
 * their interrupts, resets and pins on the Pico SDK, and
 * i2c_dw_i2c0_initialize() and i2c_dw_i2c1_initialize()
 * (i2c_dw_controllers_rp2040.h).  Each instance's device, interrupt handler,
 * speed class and start-up are under its own symbol (CONFIG_I2C_DW_I2C0,
 * CONFIG_I2C_DW_I2C1, i2c_dw.cmake); the functions of the API, the pins'
 * setup, the reset and the interrupt's connection are shared and take the
 * instance's values with I2C_DW_INST() (i2c_dw.c), as Zephyr's driver has
 * one API for all its instances.
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
 *   I2C0_IRQ (23), I2C1_IRQ (24)           irq_set_enabled()
 *   clock_control_on(clk_sys)              nothing: clk_sys runs from the
 *                                          start (Zephyr sets bit 11 of
 *                                          CLK_SYS_CTRL, which clk_sys does
 *                                          not have; NuttX writes nothing)
 *   reset_line_toggle_dt() (Zephyr's       reset_block_num(RESET_I2Cn), then
 *   reset_rpi_pico.c, on these SDK calls)  unreset_block_num_wait_blocking()
 *   pinctrl_apply_state() (Zephyr's        i2c_dw_pinctrl(): the same SDK
 *   pinctrl_rpi_pico.c)                    calls in the same order
 *   DEVICE_DT_INST_DEFINE(): k_sem_init(), i2c_dw_i2c0_initialize(), then
 *   then i2c_dw_turn_on() at boot          i2c_dw_i2c1_initialize(), from the
 *   (POST_KERNEL 50, after the DMA's 40;   application's start: the same,
 *   i2c0, then i2c1)                       i2c0's after
 *                                          i2c_dw_dma_os_initialize()
 *                                          (i2c_dw_dma_os_rp2040.c: the
 *                                          DMA interrupt's shared
 *                                          handler); the semaphores are
 *                                          made there, in their own
 *                                          memory (i2c_dw_os_sem_init()
 *                                          of i2c_dw_os_impl.h)
 *
 * The transfer refuses what the core does not do and Zephyr's driver
 * would do wrong or not at all: a device that is not ready (-ENODEV; in
 * Zephyr a device that failed its init is not to be called), no message
 * array, an address above 0x7f or a message with I2C_MSG_ADDR_10_BITS
 * (7-bit addressing only, as the Zephyr build uses it), a message with a
 * length and no buffer (-EINVAL); the core refuses a message of no bytes
 * (-ENOTSUP, as the Zephyr build's driver, before anything is sent).
 * i2c_configure() refuses I2C_ADDR_10_BITS in the word (-EINVAL) and,
 * through i2c_dw_configure(), any class but standard, fast and fast plus
 * (the RP2040's I2C has no high speed); a refused class changes nothing
 * (Zephyr's stores the minimum counts, see i2c_dw.c).  The mode bits are
 * ignored (Zephyr forces controller mode).  The API table has no
 * recover_bus, as the compatibility layer has no i2c_recover_bus().
 *
 * The speed class (one for each device) of a transfer is read by the core
 * under bus_sem, as Zephyr's driver reads its class: a transfer that waits
 * for the bus while another task's i2c_configure() completes runs at the
 * class that call set.
 *
 * The interrupt handler runs at the Pico SDK's default priority for the
 * peripherals (0x80, the same as the DMA's and the USB's; see
 * i2c_dw_dma_os_rp2040.c), calls the core's body and, through
 * i2c_dw_os_sem_give(), the waiting task's notification
 * (vTaskNotifyGiveIndexedFromISR()), and asks for the switch to the woken
 * task at the end of the handler (PendSV): the task runs right
 * after the handler, not through the timer task.  irq_set_exclusive_handler() panics if a handler
 * is already installed on the line (hard_assert(), on in this Release
 * build); nothing else in this build uses I2C0, I2C1 or their pins (the
 * Pico SDK's board header names GPIO12/13 as PICO_DEFAULT_I2C_SDA_PIN/
 * _SCL_PIN, which only examples and the RP2350's PSRAM code read; GPIO18/19
 * are D6/D7 of the header, which nothing of this build sets; the SDK's
 * start-up code leaves the pins at their reset state and takes I2C0 and
 * I2C1 out of reset, which the reset here repeats).
 */

#ifndef I2C_DW_INSTANCE_H_
#define I2C_DW_INSTANCE_H_

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/addressmap.h"
#include "hardware/resets.h"

#include <zephyr/device.h>

#include "i2c_dw_controllers_rp2040.h"

/* The pins of each controller.  i2c0: the board's i2c0 node
 * (arduino_nano_connect.dts, -pinctrl.dtsi), no pulls (the board has its
 * own pull-ups on the LSM6DSOX's bus).  i2c1: free pins of the header (also
 * possible: SDA GPIO2/6/10/14/22/26, SCL GPIO3/7/11/15/23/27, of which GPIO2,
 * 3, 10, 11, 14 are the NINA's, 6/7 SPI0 and the LED, 26/27 the ADC in the
 * Zephyr build), with the internal pull-ups (nothing on the board pulls
 * them up).
 */
#define I2C0_SDA_GPIO 12
#define I2C0_SCL_GPIO 13
#define I2C0_PULL_UP  false
#define I2C1_SDA_GPIO 18 /* D6 */
#define I2C1_SCL_GPIO 19 /* D7 */
#define I2C1_PULL_UP  true

/* The speed class that i2c_configure() last set on each device (at first
 * the devicetree's clock-frequency, I2C_BITRATE_STANDARD, which
 * i2c_dw_init_config() sets).
 */
#ifdef CONFIG_I2C_DW_I2C0
static volatile uint32_t i2c0_speed;
#endif
#ifdef CONFIG_I2C_DW_I2C1
static volatile uint32_t i2c1_speed;
#endif

#define I2C_DW_OS_SPEED(dw) (*I2C_DW_INST(dw, &i2c0_speed, &i2c1_speed))

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

	return i2c_dw_transfer_msgs(dw, msgs, num_msgs, addr, &I2C_DW_OS_SPEED(dw));
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
#ifdef CONFIG_I2C_DW_RP2040_DMA
	ret = dw->rp2040_dma_quarantined ? -EIO : i2c_dw_configure(dw, speed);
#else
	ret = i2c_dw_configure(dw, speed);
#endif
	if (ret == 0) {
		I2C_DW_OS_SPEED(dw) = speed;
	}
	i2c_dw_os_bus_unlock(&dw->bus_sem);

	return ret;
}

static DEVICE_API(i2c, i2c_dw_funcs) = {
	.configure = i2c_dw_runtime_configure,
	.transfer = i2c_dw_transfer,
};

/* Each instance's device and Zephyr's i2c_dw_isr() for it. */

#ifdef CONFIG_I2C_DW_I2C0
DEVICE_DT_DEFINE(i2c0, I2C_DW_I2C0_NAME, NULL, &i2c_0_runtime, NULL, &i2c_dw_funcs);

static void i2c_dw_isr_0(void)
{
	i2c_dw_isr_handler(&i2c_0_runtime);
}
#endif

#ifdef CONFIG_I2C_DW_I2C1
DEVICE_DT_DEFINE(i2c1, I2C_DW_I2C1_NAME, NULL, &i2c_1_runtime, NULL, &i2c_dw_funcs);

static void i2c_dw_isr_1(void)
{
	i2c_dw_isr_handler(&i2c_1_runtime);
}
#endif

/* pinctrl_configure_pin() of Zephyr's RP2040 pinctrl for the pins of an
 * I2C group: pinmux I2Cn_SDA_Pxx, I2Cn_SCL_Pxx (function I2C),
 * input-enable, input-schmitt-enable, bias-pull-up if pull_up; the
 * defaults otherwise: drive-strength 4 mA, slow slew, no overrides.  i2c0's
 * pads end as 0x52 (IE, DRIVE 4 mA, SCHMITT), i2c1's as 0x5a (and PUE), the
 * IO_BANK0 control as 0x3 (FUNCSEL I2C).
 */
static void i2c_dw_pinctrl(unsigned int gpio, bool pull_up)
{
	gpio_init(gpio);
	gpio_set_function(gpio, GPIO_FUNC_I2C);
	gpio_set_pulls(gpio, pull_up, false);
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

	reset_block_num(I2C_DW_INST(dw, RESET_I2C0, RESET_I2C1));
	unreset_block_num_wait_blocking(I2C_DW_INST(dw, RESET_I2C0, RESET_I2C1));

	/* pinctrl-0: the group's pins in their order. */
	i2c_dw_pinctrl(I2C_DW_INST(dw, I2C0_SDA_GPIO, I2C1_SDA_GPIO),
		       I2C_DW_INST(dw, I2C0_PULL_UP, I2C1_PULL_UP));
	i2c_dw_pinctrl(I2C_DW_INST(dw, I2C0_SCL_GPIO, I2C1_SCL_GPIO),
		       I2C_DW_INST(dw, I2C0_PULL_UP, I2C1_PULL_UP));

	return 0;
}

static int i2c_dw_os_irq_enable(struct i2c_dw_dev_config *dw)
{
	(void)dw;

	irq_set_exclusive_handler(I2C_DW_INST(dw, I2C0_IRQ, I2C1_IRQ),
				  I2C_DW_INST(dw, i2c_dw_isr_0, i2c_dw_isr_1));
	irq_set_enabled(I2C_DW_INST(dw, I2C0_IRQ, I2C1_IRQ), true);
	return 0;
}

/* i2c_dw_initialize() and the end of Zephyr's do_device_init() for the
 * instance dw, its device dev.
 */
static inline int i2c_dw_os_initialize(const struct device *dev, struct i2c_dw_dev_config *dw)
{
	int ret;

	if (dw->ready) {
		return 0;
	}

	/* i2c_dw_initialize() */
	dw->base = I2C_DW_INST(dw, I2C0_BASE, I2C1_BASE);
	i2c_dw_os_sem_init(&dw->device_sync_sem, 0, I2C_DW_OS_SEM_MAX_LIMIT);
	i2c_dw_os_sem_init(&dw->bus_sem, 1, 1);
#ifdef CONFIG_I2C_DW_RP2040_DMA
	if (i2c_dw_is_i2c0(dw)) {
		i2c_dw_os_sem_init(&rp2040_dma_sem, 0, I2C_DW_OS_SEM_MAX_LIMIT);

		/* The DMA device, which Zephyr initializes before i2c0
		 * (POST_KERNEL 40).  Without it the reads take the interrupts.
		 */
		ret = i2c_dw_dma_os_initialize();
		if (ret < 0) {
			LOG_ERR("No DMA for the reads: %d", ret);
		}
	}
#endif

	/* pm_device_driver_init(): i2c_dw_turn_on() */
	ret = i2c_dw_turn_on(dw);
	if (ret < 0) {
		/* Zephyr's do_device_init(): the device stays not ready. */
		dev->state->init_res = (uint8_t)MIN(-ret, UINT8_MAX);
		return ret;
	}

	I2C_DW_OS_SPEED(dw) = dw->speed;
	dw->ready = true;
	dev->state->init_res = 0;
	dev->state->initialized = true;
	return 0;
}

#ifdef CONFIG_I2C_DW_I2C0
int i2c_dw_i2c0_initialize(void)
{
	return i2c_dw_os_initialize(DEVICE_DT_GET(DT_NODELABEL(i2c0)), &i2c_0_runtime);
}
#endif

#ifdef CONFIG_I2C_DW_I2C1
int i2c_dw_i2c1_initialize(void)
{
	return i2c_dw_os_initialize(DEVICE_DT_GET(DT_NODELABEL(i2c1)), &i2c_1_runtime);
}
#endif

#endif /* I2C_DW_INSTANCE_H_ */
