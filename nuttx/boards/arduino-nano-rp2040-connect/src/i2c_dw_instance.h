/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/i2c_dw_instance.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Andrei-Edward Popa
 *
 ****************************************************************************/

/* The NuttX part of the I2C0 core (common/drivers/i2c_dw/i2c_dw.c, which
 * includes this file at its end): I2C0 as a NuttX I2C master
 * (struct i2c_master_s: I2C_TRANSFER(), and I2C_RESET() with
 * CONFIG_I2C_RESET), its interrupt, reset and pins, and
 * i2c_dw_i2c0_initialize() (i2c_dw_i2c0.h).
 *
 *   Zephyr                                here
 *   i2c_transfer() (msgs, num, addr)      I2C_TRANSFER() (struct i2c_msg_s:
 *                                         i2c_dw_msg_flags() turns the
 *                                         NuttX flags into I2C_MSG_*,
 *                                         i2c_dw_os_impl.h)
 *   i2c_configure() (speed class)         the message frequency
 *                                         (i2c_dw_speed())
 *   i2c_recover_bus()                     I2C_RESET()
 *   IRQ_CONNECT(), irq_enable()           irq_attach(), up_enable_irq()
 *   reset_line_toggle_dt()                RESETS (set, clear, RESET_DONE)
 *   pinctrl_apply_state()                 i2c_dw_pinctrl() (Zephyr's
 *                                         pinctrl_rpi_pico.c steps)
 *
 * Differences of the NuttX interface (see nuttx/drivers/README.md):
 * the speed comes with each transfer (msgs[0].frequency; 100 kHz, 400 kHz
 * and 1 MHz, as Zephyr's i2c_map_dt_bitrate() takes them) instead of from
 * i2c_configure(); one address per transfer; no 10-bit addresses.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_I2C_DW_INSTANCE_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_I2C_DW_INSTANCE_H

#if I2C_DW_M_READ != I2C_M_READ || I2C_DW_M_TEN != I2C_M_TEN || \
    I2C_DW_M_NOSTOP != I2C_M_NOSTOP || I2C_DW_M_NOSTART != I2C_M_NOSTART
#  error "i2c_dw.h: the I2C_M_* values do not match NuttX's"
#endif

static int i2c_dw_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)
{
	struct i2c_dw_dev_config *const dw = (struct i2c_dw_dev_config *)dev;
	uint16_t target_address;
	uint32_t speed;
	int i;

	/* The checks of a NuttX request that the Zephyr API makes by its types:
	 * up to 255 messages, one 7-bit address and one speed per transfer.
	 */
	if (msgs == NULL || count < 0 || count > UINT8_MAX || !dw->ready) {
		return -EINVAL;
	}
	if (count == 0) {
		return 0;
	}
	target_address = msgs[0].addr;
	speed = i2c_dw_speed(msgs[0].frequency);
	if (speed == 0 || target_address > 0x7f) {
		return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		if (msgs[i].addr != target_address ||
		    msgs[i].frequency != msgs[0].frequency ||
		    (msgs[i].flags & I2C_M_TEN) || msgs[i].length < 0 ||
		    (msgs[i].length > 0 && msgs[i].buffer == NULL)) {
			return -EINVAL;
		}
	}

	return i2c_dw_transfer_msgs(dw, msgs, count, target_address, speed);
}

#ifdef CONFIG_I2C_RESET
static int i2c_dw_recovery_bus(struct i2c_master_s *dev)
{
	int ret = 0;
	struct i2c_dw_dev_config *const dw = (struct i2c_dw_dev_config *)dev;

	/* lock bus */
	ret = nxsem_wait_uninterruptible(&dw->bus_sem);
	if (ret != 0) {
		return ret;
	}
	/* do bus recovery */
	ret = i2c_recovery_bus(dw);
	/* unlock bus */
	nxsem_post(&dw->bus_sem);

	return ret;
}
#endif

static const struct i2c_ops_s funcs = {
	.transfer = i2c_dw_transfer,
#ifdef CONFIG_I2C_RESET
	.reset = i2c_dw_recovery_bus,
#endif
};

static int i2c_dw_isr(int irq, void *context, void *arg)
{
	(void)irq;
	(void)context;

	i2c_dw_isr_handler(arg);
	return 0;
}

/* pinctrl_configure_pin() of Zephyr's RP2040 pinctrl
 * (zephyr/drivers/pinctrl/pinctrl_rpi_pico.c) for the board's i2c0_default
 * group (arduino_nano_connect-pinctrl.dtsi: I2C0_SDA_P12, I2C0_SCL_P13,
 * input-enable, input-schmitt-enable; no pulls, 4 mA, slow slew, no
 * overrides: set_function() writes the whole CTRL register).
 */

static void i2c_dw_pinctrl(uint32_t gpio)
{
	rp2040_gpio_init(gpio);
	rp2040_gpio_set_function(gpio, RP2040_GPIO_FUNC_I2C);
	rp2040_gpio_set_pulls(gpio, false, false);
	rp2040_gpio_set_drive_strength(gpio, RP2040_PADS_BANK0_GPIO_DRIVE_4MA);
	rp2040_gpio_set_slew_fast(gpio, false);
	rp2040_gpio_set_input_hysteresis_enabled(gpio, true);
	modbits_reg32(RP2040_PADS_BANK0_GPIO_IE, RP2040_PADS_BANK0_GPIO_IE,
		      RP2040_PADS_BANK0_GPIO(gpio));
}

static int i2c_dw_os_prepare(struct i2c_dw_dev_config *dw)
{
	(void)dw;

	/* clk_peri runs from boot (rp2040_clockconfig()). */

	/* reset_line_toggle_dt(): reset_block_num(), then
	 * unreset_block_num_wait_blocking().
	 */
	setbits_reg32(RP2040_RESETS_RESET_I2C0, RP2040_RESETS_RESET);
	clrbits_reg32(RP2040_RESETS_RESET_I2C0, RP2040_RESETS_RESET);
	while ((getreg32(RP2040_RESETS_RESET_DONE) & RP2040_RESETS_RESET_I2C0) == 0) {
	}

	i2c_dw_pinctrl(BOARD_I2C0_SDA_GPIO);
	i2c_dw_pinctrl(BOARD_I2C0_SCL_GPIO);

	return 0;
}

static void i2c_dw_os_irq_enable(struct i2c_dw_dev_config *dw)
{
	irq_attach(RP2040_I2C0_IRQ, i2c_dw_isr, dw);
	up_enable_irq(RP2040_I2C0_IRQ);
}

struct i2c_master_s *i2c_dw_i2c0_initialize(void)
{
	struct i2c_dw_dev_config *const dw = &i2c_0_runtime;

	if (dw->ready) {
		return &dw->dev;
	}

	dw->dev.ops = &funcs;
	dw->base = RP2040_I2C0_BASE;
	nxsem_init(&dw->device_sync_sem, 0, 0);
	nxsem_init(&dw->bus_sem, 0, 1);
#ifdef CONFIG_I2C_DW_RP2040_DMA
	nxsem_init(&rp2040_dma_sem, 0, 0);
#endif

	if (i2c_dw_turn_on(dw) < 0) {
		return NULL;
	}

	dw->ready = true;
	return &dw->dev;
}

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_I2C_DW_INSTANCE_H */
