/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS part of the I2C0 and DMA core on the
 * FreeRTOS POSIX port: freertos/drivers/i2c_dw (the boundary
 * i2c_dw_os_impl.h, the device i2c0 and its Zephyr I2C API in
 * nano_i2c_instance.h, nano_dma_instance.h, nano_dma_os.c) and
 * freertos/drivers/compat/zephyr/drivers/i2c.h, under the shared core
 * (common/drivers/i2c_dw/nano_i2c.c, nano_dma.c, i2c_dw_rp2040_commands.c,
 * with the prelude, as freertos/drivers/i2c_dw/i2c_dw.cmake builds them), on
 * fakes of the Pico SDK's GPIO, reset, IRQ, sync and DMA-claim functions
 * (pico_fake/, implemented here: they record the calls) and on an array in
 * place of I2C0's registers, which a task of this test plays the
 * controller and the LSM6DSOX on (it calls the I2C0 handler that the driver
 * installed, as an interrupt handler).  DMA reads are not run (the DMA
 * registers are at fixed addresses; reads under 32 bytes, or with the DMA
 * reads off, take the interrupt path).
 *
 * - Before the start: i2c0 is not ready; its calls return -ENODEV.
 * - A block that is not a DesignWare I2C (IC_COMP_TYPE): -EIO, i2c0 not
 *   ready, its init_res EIO.
 * - The start: the DMA's channels claimed, its reset and both lines
 *   (DMA_IRQ_0, DMA_IRQ_1) on one handler, enabled; I2C0's reset, GPIO12
 *   then GPIO13 as Zephyr's pinctrl sets them (in its order), the I2C0
 *   line; the controller off, the spike lengths 1, SDA hold kept; i2c0
 *   ready.  A second call does nothing.
 * - WHO_AM_I through i2c_burst_read_dt() at 100 kHz: the commands (0x0f;
 *   then READ|STOP|RESTART), the byte, IC_TAR, the SCL counts 500/625,
 *   IC_CON with the DMA reads' RX_FIFO_FULL_HLD_CTRL, and without it once
 *   the DMA reads are off.
 * - i2c_configure(): fast plus (31/31) and fast (93/109) counts and IC_CON;
 *   high speed and 10-bit addresses refused, nothing changed.
 * - Transfers refused before the core (-EINVAL; no message: 0), touching no
 *   register.
 * - A NACK: -EIO, the core's two log lines, and the next transfer works.
 * - No interrupt: -ETIMEDOUT after the core's 1000 ms (the layer's ticks),
 *   the abort requested, the interrupts masked, and the next transfer
 *   works (the semaphore was reset).
 * - The boundary's semaphore and time functions: K_MSEC() rounding, the
 *   deadline's last try without waiting, a give from an interrupt handler.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#include "pico_i2c_fake.h"

#include "i2c_dw_os_impl.h"
#include "nano_dma.h"
#include "nano_i2c.h"
#include "nano_i2c_dma.h"
#include "priorities.h"

#define RUNNER_STACK 16384
#define HW_STACK     16384
#define LOG_SIZE     4096

volatile int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* The calls and the log ------------------------------------------------------*/

static char calls[LOG_SIZE];

static void record(const char *fmt, ...)
{
	size_t used = strlen(calls);
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(calls + used, sizeof(calls) - used, fmt, ap);
	va_end(ap);
	assert(strlen(calls) + 1 < sizeof(calls));
	strcat(calls, " ");
}

static void expect_calls(const char *expected)
{
	if (strcmp(calls, expected) != 0) {
		printf("expected: %s\n     got: %s\n", expected, calls);
		fflush(stdout);
		assert(0);
	}
	calls[0] = '\0';
}

static char log_text[LOG_SIZE];

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	size_t used = strlen(log_text);

	(void)func;
	snprintf(log_text + used, sizeof(log_text) - used, "<%d> %s: ", level, module);
	used = strlen(log_text);
	vsnprintf(log_text + used, sizeof(log_text) - used, fmt, ap);
	used = strlen(log_text);
	snprintf(log_text + used, sizeof(log_text) - used, "\n");
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	app_log_vprintf(level, module, func, fmt, ap);
	va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	(void)last_ms;
	(void)skipped;
	(void)rate_ms;
	*skipped_out = 0;
	return true;
}

static void expect_log(const char *expected)
{
	if (strcmp(log_text, expected) != 0) {
		printf("expected log: %s\n     got log: %s\n", expected, log_text);
		fflush(stdout);
		assert(0);
	}
	log_text[0] = '\0';
}

/* The fakes of the Pico SDK (pico_fake/pico_i2c_fake.h) ----------------------*/

volatile uint32_t fake_i2c0_regs[64];

static irq_handler_t handlers[32];
static int irq_masked;

void gpio_init(uint gpio)
{
	record("init(%u)", gpio);
}

void gpio_set_function(uint gpio, gpio_function_t fn)
{
	record("func(%u,%s)", gpio, fn == GPIO_FUNC_I2C ? "i2c" : "?");
}

void gpio_set_pulls(uint gpio, bool up, bool down)
{
	record("pulls(%u,%d,%d)", gpio, up, down);
}

void gpio_set_drive_strength(uint gpio, enum gpio_drive_strength drive)
{
	record("drive(%u,%s)", gpio, drive == GPIO_DRIVE_STRENGTH_4MA ? "4mA" : "?");
}

void gpio_set_slew_rate(uint gpio, enum gpio_slew_rate slew)
{
	record("slew(%u,%s)", gpio, slew == GPIO_SLEW_RATE_SLOW ? "slow" : "fast");
}

void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled)
{
	record("schmitt(%u,%d)", gpio, enabled);
}

void gpio_set_input_enabled(uint gpio, bool enabled)
{
	record("ie(%u,%d)", gpio, enabled);
}

void gpio_set_oeover(uint gpio, uint value)
{
	record("oeover(%u,%u)", gpio, value);
}

void gpio_set_outover(uint gpio, uint value)
{
	record("outover(%u,%u)", gpio, value);
}

void gpio_set_inover(uint gpio, uint value)
{
	record("inover(%u,%u)", gpio, value);
}

void gpio_set_irqover(uint gpio, uint value)
{
	record("irqover(%u,%u)", gpio, value);
}

static const char *block_name(uint32_t block)
{
	return block == RESET_DMA ? "dma" : block == RESET_I2C0 ? "i2c0" : "?";
}

void reset_block_num(uint32_t block_num)
{
	record("reset(%s)", block_name(block_num));
}

void unreset_block_num_wait_blocking(uint32_t block_num)
{
	record("unreset(%s)", block_name(block_num));
}

void irq_set_exclusive_handler(uint num, irq_handler_t handler)
{
	assert(num < 32 && handlers[num] == NULL);
	handlers[num] = handler;
	record("handler(%u)", num);
}

void irq_set_enabled(uint num, bool enabled)
{
	record("irq(%u,%d)", num, enabled);
}

uint32_t save_and_disable_interrupts(void)
{
	return (uint32_t)irq_masked++;
}

void restore_interrupts(uint32_t status)
{
	irq_masked = (int)status;
}

void dma_claim_mask(uint32_t channel_mask)
{
	record("claim(0x%x)", (unsigned int)channel_mask);
}

/* I2C0's registers and the bus ----------------------------------------------*/

#define REG(off) fake_i2c0_regs[(off) / 4]

#define IC_CON         0x00
#define IC_TAR         0x04
#define IC_DATA_CMD    0x10
#define IC_SS_SCL_HCNT 0x14
#define IC_SS_SCL_LCNT 0x18
#define IC_FS_SCL_HCNT 0x1c
#define IC_FS_SCL_LCNT 0x20
#define IC_INTR_STAT   0x2c
#define IC_INTR_MASK   0x30
#define IC_RX_TL       0x38
#define IC_ENABLE      0x6c
#define IC_STATUS      0x70
#define IC_SDA_HOLD    0x7c
#define IC_TX_ABRT_SRC 0x80
#define IC_FS_SPKLEN   0xa0
#define IC_HS_SPKLEN   0xa4
#define IC_COMP_TYPE   0xfc
#define DW_MAGIC       0x44570140u
#define STAT_TX_EMPTY  (1u << 4)
#define STAT_RX_FULL   (1u << 2)
#define STAT_TX_ABRT   (1u << 6)
#define STAT_STOP_DET  (1u << 9)
#define STATUS_TFNF    (1u << 1)
#define STATUS_RFNE    (1u << 3)
#define CMD_READ       (1u << 8)
#define CMD_STOP       (1u << 9)
#define CMD_RESTART    (1u << 10)
#define CMD_NONE       0xdead0000u
/* IC_CON as the core writes it (bit 11, BUS_CLEAR_FEATURE_CTRL, is not in
 * the RP2040's register and reads 0 there; the array keeps it).
 */
#define CON_STD_DMA    0xb63u
#define CON_STD        0x963u
#define CON_FAST_DMA   0xb65u

#define LSM6DSOX_ADDR     0x6a
#define LSM6DSOX_WHO_AM_I 0x0f
#define LSM6DSOX_ID       0x6c

/* The task that plays the controller's interrupts and the LSM6DSOX. */
enum hw_mode {
	HW_OFF,    /* no interrupt at all (a dead bus) */
	HW_DEVICE, /* the LSM6DSOX answers */
	HW_NACK,   /* nothing at the address: the address byte is not acked */
};

static volatile enum hw_mode hw_mode = HW_OFF;
static char hw_log[LOG_SIZE];
static uint8_t device_reg;

static void hw_record(const char *fmt, ...)
{
	size_t used = strlen(hw_log);
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(hw_log + used, sizeof(hw_log) - used, fmt, ap);
	va_end(ap);
	strcat(hw_log, " ");
}

static void expect_hw(const char *expected)
{
	if (strcmp(hw_log, expected) != 0) {
		printf("expected bus: %s\n     got bus: %s\n", expected, hw_log);
		fflush(stdout);
		assert(0);
	}
	hw_log[0] = '\0';
}

static void interrupt(uint32_t intr_stat, uint32_t status)
{
	REG(IC_INTR_STAT) = intr_stat;
	REG(IC_STATUS) = status;
	z_host_isr_depth++;
	handlers[I2C0_IRQ]();
	z_host_isr_depth--;
	REG(IC_INTR_STAT) = 0;
}

/* One message, armed by the core (its interrupt mask set): the controller
 * asks for the first command (TX_EMPTY); for a read, the byte arrives
 * (RX_FULL); STOP_DET after a command with STOP.  Messages of one byte.
 */
static void hw_message(void)
{
	uint32_t cmd;

	if (hw_mode == HW_NACK) {
		REG(IC_TX_ABRT_SRC) = 1; /* ADDR7BNACK */
		interrupt(STAT_TX_ABRT, STATUS_TFNF);
		REG(IC_TX_ABRT_SRC) = 0;
		hw_record("nack");
		return;
	}
	REG(IC_DATA_CMD) = CMD_NONE;
	interrupt(STAT_TX_EMPTY, STATUS_TFNF);
	cmd = REG(IC_DATA_CMD);
	hw_record("0x%x", (unsigned int)cmd);
	if (cmd == CMD_NONE) {
		return;
	}
	if (cmd & CMD_READ) {
		REG(IC_DATA_CMD) = device_reg == LSM6DSOX_WHO_AM_I ? LSM6DSOX_ID : 0;
		interrupt(STAT_RX_FULL, STATUS_TFNF | STATUS_RFNE);
	} else {
		device_reg = (uint8_t)cmd;
	}
	if (cmd & CMD_STOP) {
		interrupt(STAT_STOP_DET, STATUS_TFNF);
	}
}

static void hw_main(void *arg)
{
	(void)arg;

	for (;;) {
		vTaskDelay(1);
		if (hw_mode != HW_OFF && REG(IC_INTR_MASK) != 0) {
			hw_message();
		}
	}
}

/* The tests ------------------------------------------------------------------*/

static const struct device *const i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static const struct i2c_dt_spec imu = {
	.bus = DEVICE_DT_GET(DT_NODELABEL(i2c0)),
	.addr = LSM6DSOX_ADDR,
};

/* The commands of a WHO_AM_I read: the register address, then one read
 * with RESTART and STOP.  After a transfer that ended with a read, the
 * address also has RESTART: the core (Zephyr's i2c_dw_transfer()) adds one
 * on a change of direction from the last message of the previous transfer
 * (the controller makes a START of it on an idle bus).
 */
#define WHO_AM_I_BUS            "0xf 0x700 "
#define WHO_AM_I_BUS_AFTER_READ "0x40f 0x700 "

static int who_am_i(void)
{
	uint8_t id = 0;
	int ret = i2c_burst_read_dt(&imu, LSM6DSOX_WHO_AM_I, &id, 1);

	return ret != 0 ? ret : id;
}

static void test_before_start(void)
{
	uint8_t byte;

	assert(!i2c_is_ready_dt(&imu));
	assert(i2c_burst_read_dt(&imu, LSM6DSOX_WHO_AM_I, &byte, 1) == -ENODEV);
	assert(i2c_configure(i2c0, I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)) == -ENODEV);
	expect_calls("");
}

#define PINS(n)                                                                                    \
	"init(" #n ") func(" #n ",i2c) pulls(" #n ",0,0) drive(" #n ",4mA) slew(" #n ",slow) "     \
	"schmitt(" #n ",1) ie(" #n ",1) oeover(" #n ",0) outover(" #n ",0) inover(" #n ",0) "      \
	"irqover(" #n ",0) "

static void test_start(void)
{
	/* The reset values of the RP2040's I2C0 that the start reads. */
	REG(IC_COMP_TYPE) = 0;
	REG(IC_CON) = 0x65;
	REG(IC_SDA_HOLD) = 1;
	REG(IC_FS_SPKLEN) = 7;
	REG(IC_ENABLE) = 1;

	assert(nano_dma_initialize() == 0);
	expect_calls("claim(0xfff) reset(dma) unreset(dma) handler(11) irq(11,1) handler(12) "
		     "irq(12,1) ");
	assert(nano_dma_is_ready());

	/* Not a DesignWare block: refused after the reset and the pins. */
	assert(nano_i2c0_initialize() == -EIO);
	expect_calls("reset(i2c0) unreset(i2c0) " PINS(12) PINS(13));
	assert(!device_is_ready(i2c0));
	assert(i2c0->state->init_res == EIO);

	REG(IC_COMP_TYPE) = DW_MAGIC;
	assert(nano_i2c0_initialize() == 0);
	expect_calls("reset(i2c0) unreset(i2c0) " PINS(12) PINS(13) "handler(23) irq(23,1) ");
	assert(device_is_ready(i2c0));
	assert(i2c0->state->init_res == 0);
	assert(strcmp(i2c0->name, "i2c@40044000") == 0);
	assert((REG(IC_ENABLE) & 1) == 0);
	assert(REG(IC_FS_SPKLEN) == 1);
	assert(REG(IC_HS_SPKLEN) == 1);
	assert(REG(IC_SDA_HOLD) == 1);
	assert(REG(IC_SS_SCL_HCNT) == 0); /* written at the first transfer */

	assert(nano_dma_initialize() == 0);
	assert(nano_i2c0_initialize() == 0);
	expect_calls("");
	assert(nano_i2c_dma_enabled());
}

static void test_who_am_i(void)
{
	hw_mode = HW_DEVICE;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS);
	assert((REG(IC_TAR) & 0x3ff) == LSM6DSOX_ADDR);
	assert(REG(IC_SS_SCL_HCNT) == 500);
	assert(REG(IC_SS_SCL_LCNT) == 625);
	assert(REG(IC_CON) == CON_STD_DMA);
	assert(REG(IC_RX_TL) == 0);
	assert(REG(IC_INTR_MASK) == 0);
	assert((REG(IC_ENABLE) & 1) == 1);

	/* Two in a row: the semaphores are back at 0. */
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);

	/* The DMA reads off: IC_CON without RX_FIFO_FULL_HLD_CTRL. */
	nano_i2c_dma_set_enabled(false);
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_CON) == CON_STD);
	nano_i2c_dma_set_enabled(true);

	/* A write of two bytes: the last command carries STOP (the first, which
	 * the array does not keep, has RESTART after the read).
	 */
	static const uint8_t ctrl[2] = {0x12, 0x44};

	assert(i2c_write_dt(&imu, ctrl, sizeof(ctrl)) == 0);
	expect_hw("0x244 ");
	expect_log("");
}

static void test_configure(void)
{
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)) == 0);
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS);
	assert(REG(IC_FS_SCL_HCNT) == 31);
	assert(REG(IC_FS_SCL_LCNT) == 31);
	assert(REG(IC_CON) == CON_FAST_DMA);

	/* Refused: nothing changes. */
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_HIGH)) == -EINVAL);
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_ULTRA)) ==
	       -EINVAL);
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_ADDR_10_BITS |
					   I2C_SPEED_SET(I2C_SPEED_FAST)) == -EINVAL);
	REG(IC_FS_SCL_HCNT) = 0;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_FS_SCL_HCNT) == 31);

	assert(i2c_configure(i2c0, I2C_SPEED_SET(I2C_SPEED_FAST)) == 0);
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_FS_SCL_HCNT) == 93);
	assert(REG(IC_FS_SCL_LCNT) == 109);

	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD)) == 0);
	REG(IC_SS_SCL_HCNT) = 0;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_SS_SCL_HCNT) == 500);
	assert(REG(IC_CON) == CON_STD_DMA);
}

static void test_refused(void)
{
	const struct i2c_driver_api *api = i2c0->api;
	struct i2c_msg msg = {.buf = NULL, .len = 1, .flags = I2C_MSG_WRITE};
	uint8_t byte = 0;

	REG(IC_TAR) = 0;
	assert(i2c_read(i2c0, &byte, 1, 0x80) == -EINVAL);
	assert(i2c_transfer(i2c0, &msg, 1, LSM6DSOX_ADDR) == -EINVAL);
	msg.buf = &byte;
	msg.flags = I2C_MSG_READ | I2C_MSG_ADDR_10_BITS;
	assert(i2c_transfer(i2c0, &msg, 1, LSM6DSOX_ADDR) == -EINVAL);
	assert(i2c_transfer(i2c0, &msg, 0, LSM6DSOX_ADDR) == 0);
	assert(api->transfer(i2c0, NULL, 1, LSM6DSOX_ADDR) == -EINVAL);
	assert(api->transfer(i2c0, &msg, 0, LSM6DSOX_ADDR) == 0);
	vTaskDelay(5);
	expect_hw("");
	assert(REG(IC_TAR) == 0);
}

static void test_nack(void)
{
	uint8_t byte = 0;

	hw_mode = HW_NACK;
	assert(i2c_read(i2c0, &byte, 1, 0x10) == -EIO);
	expect_hw("nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n");
	hw_mode = HW_DEVICE;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
}

static void test_timeout(void)
{
	TickType_t start, ticks;
	int ret;

	hw_mode = HW_OFF;
	start = xTaskGetTickCount();
	ret = who_am_i();
	ticks = xTaskGetTickCount() - start;
	assert(ret == -ETIMEDOUT);
	/* 1000 ms in 1 ms ticks: 1000 + 1, then the 1 ms wait for the abort
	 * (1 + 1); a few ticks more for the host.
	 */
	assert(ticks >= 1003 && ticks <= 1020);
	assert(REG(IC_ENABLE) & 2); /* ABORT */
	assert(REG(IC_INTR_MASK) == 0);
	expect_hw("");
	expect_log("");

	REG(IC_ENABLE) &= ~2u; /* the controller has ended the abort */
	hw_mode = HW_DEVICE;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS);
}

static void give_from_isr(void *arg)
{
	z_host_isr_depth++;
	i2c_dw_os_sem_give(arg);
	z_host_isr_depth--;
}

static void giver_main(void *arg)
{
	vTaskDelay(20);
	give_from_isr(arg);
	vTaskDelete(NULL);
}

static void test_boundary(void)
{
	static struct k_sem sem;
	TickType_t start, end, ticks;

	(void)k_sem_init(&sem, 0, K_SEM_MAX_LIMIT);

	/* K_MSEC(ms) in ticks (1 ms here, 100 us on the board), plus one. */
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 10) != 0);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 10 && ticks <= 13);

	/* The deadline: now + the ticks (at least one); once passed, one try
	 * without waiting.
	 */
	start = xTaskGetTickCount();
	end = i2c_dw_os_timepoint_calc(10);
	assert(end - start >= 10 && end - start <= 11);
	assert(i2c_dw_os_timepoint_calc(0) - xTaskGetTickCount() <= 1);
	end = xTaskGetTickCount() - 5;
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_until(&sem, end) != 0);
	assert(xTaskGetTickCount() - start <= 1);
	give_from_isr(&sem);
	assert(i2c_dw_os_sem_take_until(&sem, end) == 0);

	/* A give from an interrupt handler wakes the waiting task. */
	assert(xTaskCreate(giver_main, "giver", HW_STACK / sizeof(StackType_t), &sem,
			   NANO_PRIO_SHELL + 1, NULL) == pdPASS);
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_until(&sem, i2c_dw_os_timepoint_calc(1000)) == 0);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 19 && ticks <= 30);

	/* The reset and the lock. */
	give_from_isr(&sem);
	i2c_dw_os_sem_reset(&sem);
	assert(i2c_dw_os_sem_take_until(&sem, xTaskGetTickCount()) != 0);
	(void)k_sem_init(&sem, 1, 1);
	assert(i2c_dw_os_bus_lock(&sem) == 0);
	assert(k_sem_count_get(&sem) == 0);
	i2c_dw_os_bus_unlock(&sem);
	i2c_dw_os_bus_unlock(&sem);
	assert(k_sem_count_get(&sem) == 1);

	/* The interrupt lock nests. */
	uint32_t outer = i2c_dw_os_irq_lock();
	uint32_t inner = i2c_dw_os_irq_lock();

	assert(irq_masked == 2);
	i2c_dw_os_irq_unlock(inner);
	assert(irq_masked == 1);
	i2c_dw_os_irq_unlock(outer);
	assert(irq_masked == 0);
	assert(I2C_DW_CLOCK_SPEED == 125);
}

static void runner(void *arg)
{
	(void)arg;

	test_before_start();
	test_start();
	test_who_am_i();
	test_configure();
	test_refused();
	test_nack();
	test_timeout();
	test_boundary();
	assert(irq_masked == 0);
	printf("i2c_dw: all tests passed\n");
	fflush(stdout);
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_SHELL,
			NULL) != pdPASS ||
	    xTaskCreate(hw_main, "hw", HW_STACK / sizeof(StackType_t), NULL, NANO_PRIO_SHELL + 4,
			NULL) != pdPASS) {
		return 1;
	}
	vTaskStartScheduler();
	return 1;
}
