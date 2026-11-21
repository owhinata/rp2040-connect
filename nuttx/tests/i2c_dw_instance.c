/* SPDX-License-Identifier: MIT */
/* Host test of the I2C core's NuttX binding
 * (common/drivers/i2c_dw/port/nuttx: i2c_dw_os_impl.h, i2c_dw_instance.h,
 * i2c_dw_controllers_rp2040.h) under the shared core
 * (common/drivers/i2c_dw/i2c_dw.c, included here as the board's Make.defs
 * builds it), on fakes of the NuttX headers it uses
 * (tests/i2c_dw_instance_fakes: semaphores on the host's, ticks of 1 ms, the
 * RESETS, PADS_BANK0 and GPIO calls recorded here), the board's own
 * include/board.h (the pins), arrays in place of the controllers' registers
 * and a thread that plays the controllers, a device at 0x6a on I2C0 and one
 * at 0x50 on I2C1 (it calls the handlers that irq_attach() recorded, as
 * interrupt handlers, under the interrupt lock).  The DMA boundary
 * (i2c_dw_dma_os.h) is faked here: ready, but it never has a channel, so a
 * read the DMA path would take is a fallback, counted and recorded, on the
 * interrupts.
 *
 * Built twice by script/host_tests.sh: with CONFIG_I2C_DW_I2C0 only (the
 * default configuration) and with CONFIG_I2C_DW_I2C1 too, which adds the
 * tests of I2C1.
 *
 * - I2C0's start: the DMA part first, its reset (RESETS bit 3), GPIO12 and
 *   GPIO13 without pulls, RP2040_I2C0_IRQ; the same master on a second
 *   call; a write and a read at 100 kHz (IC_TAR, the SCL counts, IC_CON with
 *   RX_FIFO_FULL_HLD_CTRL); a NACK (-EIO, the log with i2c@40044000); a read
 *   of 40 bytes as the last message asks the DMA part for a channel (one
 *   fallback).
 * - With CONFIG_I2C_DW_I2C1: before its start, nothing of I2C1 touched; a
 *   block that is not a DesignWare I2C: NULL, I2C0 unaffected; the start:
 *   no DMA part, its reset (bit 4), GPIO18 and GPIO19 with the pull-ups,
 *   RP2040_I2C1_IRQ with its own controller as the handler's argument; its
 *   own master (the same on a second call), the same operations as I2C0's;
 *   a write and a read on its registers only, IC_CON without
 *   RX_FIFO_FULL_HLD_CTRL; 400 kHz on I2C1 leaves I2C0 at 100 kHz; an
 *   unanswered address: -EIO at once with i2c@40048000 in the log, and the
 *   next transfer works; reads of 40 bytes as the last message never ask
 *   for a DMA channel; I2C_RESET(); 200 transfers on each bus from two
 *   threads at once, each bus seeing only its own.
 */

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "i2c_dw.c"

/* The calls and the log ---------------------------------------------------*/

static pthread_mutex_t g_rec_lock = PTHREAD_MUTEX_INITIALIZER;
static char g_calls[4096];
static char g_log[4096];

static void append(char *buf, size_t size, const char *fmt, va_list ap)
{
	size_t used = strlen(buf);

	vsnprintf(buf + used, size - used, fmt, ap);
	assert(strlen(buf) + 2 < size);
}

static void record(const char *fmt, ...)
{
	va_list ap;

	pthread_mutex_lock(&g_rec_lock);
	va_start(ap, fmt);
	append(g_calls, sizeof(g_calls), fmt, ap);
	va_end(ap);
	strcat(g_calls, " ");
	pthread_mutex_unlock(&g_rec_lock);
}

static void expect_text(char *buf, const char *expected, const char *what)
{
	pthread_mutex_lock(&g_rec_lock);
	if (strcmp(buf, expected) != 0) {
		printf("expected %s: %s\n     got %s: %s\n", what, expected, what, buf);
		fflush(stdout);
		assert(0);
	}
	buf[0] = '\0';
	pthread_mutex_unlock(&g_rec_lock);
}

#define expect_calls(s) expect_text(g_calls, s, "calls")
#define expect_log(s)   expect_text(g_log, s, "log")

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	size_t used;

	(void)func;
	pthread_mutex_lock(&g_rec_lock);
	used = strlen(g_log);
	snprintf(g_log + used, sizeof(g_log) - used, "<%d> %s: ", level, module);
	append(g_log, sizeof(g_log), fmt, ap);
	strcat(g_log, "\n");
	pthread_mutex_unlock(&g_rec_lock);
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

/* The fakes of the NuttX headers (tests/i2c_dw_instance_fakes) ------------*/

volatile uint32_t fake_i2c0_regs[64];
volatile uint32_t fake_i2c1_regs[64];

/* The interrupt lock: one core, so the "hardware" thread's handlers and the
 * transfers' locked sections exclude each other.
 */
static pthread_mutex_t g_irq_lock;
static __thread int g_irq_depth;

irqstate_t up_irq_save(void)
{
	pthread_mutex_lock(&g_irq_lock);
	return (irqstate_t)g_irq_depth++;
}

void up_irq_restore(irqstate_t flags)
{
	g_irq_depth = (int)flags;
	pthread_mutex_unlock(&g_irq_lock);
}

static xcpt_t g_isr[64];
static void *g_isr_arg[64];

int irq_attach(int irq, xcpt_t isr, void *arg)
{
	g_isr[irq] = isr;
	g_isr_arg[irq] = arg;
	record("attach(%d)", irq - RP2040_IRQ_EXTINT);
	return 0;
}

void up_enable_irq(int irq)
{
	record("enable(%d)", irq - RP2040_IRQ_EXTINT);
}

uint32_t getreg32(uintptr_t addr)
{
	assert(addr == RP2040_RESETS_RESET_DONE);
	return 0xffffffffu;
}

void setbits_reg32(uint32_t bits, uintptr_t addr)
{
	assert(addr == RP2040_RESETS_RESET);
	record("reset(0x%x)", (unsigned int)bits);
}

void clrbits_reg32(uint32_t bits, uintptr_t addr)
{
	assert(addr == RP2040_RESETS_RESET);
	record("unreset(0x%x)", (unsigned int)bits);
}

void modbits_reg32(uint32_t value, uint32_t mask, uintptr_t addr)
{
	unsigned int gpio = (unsigned int)((addr - RP2040_PADS_BANK0_GPIO(0)) / 4);

	assert(value == RP2040_PADS_BANK0_GPIO_IE && mask == RP2040_PADS_BANK0_GPIO_IE);
	record("ie(%u)", gpio);
}

void rp2040_gpio_init(uint32_t gpio)
{
	record("init(%u)", (unsigned int)gpio);
}

void rp2040_gpio_set_function(uint32_t gpio, uint32_t func)
{
	record("func(%u,%s)", (unsigned int)gpio, func == RP2040_GPIO_FUNC_I2C ? "i2c" : "?");
}

void rp2040_gpio_set_pulls(uint32_t gpio, int up, int down)
{
	record("pulls(%u,%d,%d)", (unsigned int)gpio, up, down);
}

void rp2040_gpio_set_drive_strength(uint32_t gpio, uint32_t drive_strength)
{
	record("drive(%u,%s)", (unsigned int)gpio,
	       drive_strength == RP2040_PADS_BANK0_GPIO_DRIVE_4MA ? "4mA" : "?");
}

void rp2040_gpio_set_slew_fast(uint32_t gpio, bool fast)
{
	record("slew(%u,%s)", (unsigned int)gpio, fast ? "fast" : "slow");
}

void rp2040_gpio_set_input_hysteresis_enabled(uint32_t gpio, bool enabled)
{
	record("schmitt(%u,%d)", (unsigned int)gpio, enabled);
}

/* The DMA boundary (i2c_dw_dma_os.h): ready, never a channel. */

int i2c_dw_dma_os_initialize(void)
{
	record("dma-init");
	return 0;
}

bool i2c_dw_dma_os_is_ready(void)
{
	return true;
}

int i2c_dw_dma_os_request_channel(void)
{
	record("dma-request");
	return -EBUSY;
}

void i2c_dw_dma_os_release_channel(uint32_t channel)
{
	(void)channel;
	assert(0);
}

int i2c_dw_dma_os_config_tx(uint32_t channel, const uint16_t *words, uintptr_t dest,
			    uint32_t count, i2c_dw_dma_os_callback_t callback, void *user_data)
{
	(void)channel, (void)words, (void)dest, (void)count, (void)callback, (void)user_data;
	assert(0);
	return -EIO;
}

int i2c_dw_dma_os_config_rx(uint32_t channel, uintptr_t source, uint8_t *bytes, uint32_t count,
			    i2c_dw_dma_os_callback_t callback, void *user_data)
{
	(void)channel, (void)source, (void)bytes, (void)count, (void)callback, (void)user_data;
	assert(0);
	return -EIO;
}

int i2c_dw_dma_os_start(uint32_t channel)
{
	(void)channel;
	assert(0);
	return -EIO;
}

int i2c_dw_dma_os_stop(uint32_t channel)
{
	(void)channel;
	assert(0);
	return -EIO;
}

bool i2c_dw_dma_os_is_busy(uint32_t channel)
{
	(void)channel;
	assert(0);
	return false;
}

void i2c_dw_dma_os_acknowledge(uint32_t channel)
{
	(void)channel;
	assert(0);
}

/* The controllers and the devices -----------------------------------------*/

#define IC_CON         0x00
#define IC_TAR         0x04
#define IC_DATA_CMD    0x10
#define IC_SS_SCL_HCNT 0x14
#define IC_SS_SCL_LCNT 0x18
#define IC_FS_SCL_HCNT 0x1c
#define IC_FS_SCL_LCNT 0x20
#define IC_INTR_STAT   0x2c
#define IC_INTR_MASK   0x30
#define IC_ENABLE      0x6c
#define IC_STATUS      0x70
#define IC_SDA_HOLD    0x7c
#define IC_TX_ABRT_SRC 0x80
#define IC_DMA_CR      0x88
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
/* IC_CON as the core writes it (bit 11, BUS_CLEAR_FEATURE_CTRL, reads 0 on
 * the RP2040; the array keeps it), with RX_FIFO_FULL_HLD_CTRL (bit 9) for
 * I2C0's DMA reads.
 */
#define CON_STD_DMA 0xb63u
#define CON_STD     0x963u
#define CON_FAST    0x965u

#define DEV0_ADDR 0x6a
#define DEV1_ADDR 0x50

enum bus_mode {
	BUS_OFF,    /* no interrupt */
	BUS_DEVICE, /* the device answers: a read gives its register + base */
	BUS_NACK,   /* nothing acks the address */
};

struct fake_bus {
	volatile uint32_t *regs;
	int irq;
	volatile enum bus_mode mode;
	uint8_t reg;     /* the device's register, the last byte written */
	uint8_t base;    /* added to the register for a read */
	char log[8192];  /* the commands of the interrupt path */
};

static struct fake_bus g_bus[2] = {
	{fake_i2c0_regs, RP2040_I2C0_IRQ, BUS_OFF, 0, 0x10, ""},
	{fake_i2c1_regs, RP2040_I2C1_IRQ, BUS_OFF, 0, 0x30, ""},
};

static void bus_record(struct fake_bus *bus, const char *fmt, ...)
{
	va_list ap;

	pthread_mutex_lock(&g_rec_lock);
	va_start(ap, fmt);
	append(bus->log, sizeof(bus->log), fmt, ap);
	va_end(ap);
	strcat(bus->log, " ");
	pthread_mutex_unlock(&g_rec_lock);
}

static void bus_interrupt(struct fake_bus *bus, uint32_t intr_stat, uint32_t status)
{
	irqstate_t flags = up_irq_save();

	bus->regs[IC_INTR_STAT / 4] = intr_stat;
	bus->regs[IC_STATUS / 4] = status;
	assert(g_isr[bus->irq] != NULL);
	g_isr[bus->irq](bus->irq, NULL, g_isr_arg[bus->irq]);
	bus->regs[IC_INTR_STAT / 4] = 0;
	up_irq_restore(flags);
}

/* One message of one byte, armed by the core (TX_EMPTY in its mask). */
static void bus_message(struct fake_bus *bus)
{
	uint32_t cmd;

	if (bus->mode == BUS_NACK) {
		bus->regs[IC_TX_ABRT_SRC / 4] = 1; /* ADDR7BNACK */
		bus_interrupt(bus, STAT_TX_ABRT, STATUS_TFNF);
		bus->regs[IC_TX_ABRT_SRC / 4] = 0;
		bus_record(bus, "nack");
		return;
	}
	bus->regs[IC_DATA_CMD / 4] = CMD_NONE;
	bus_interrupt(bus, STAT_TX_EMPTY, STATUS_TFNF);
	cmd = bus->regs[IC_DATA_CMD / 4];
	if (cmd == CMD_NONE) {
		return;
	}
	bus_record(bus, "0x%x", (unsigned int)cmd);
	if (cmd & CMD_READ) {
		bus->regs[IC_DATA_CMD / 4] = (uint8_t)(bus->reg + bus->base);
		bus_interrupt(bus, STAT_RX_FULL, STATUS_TFNF | STATUS_RFNE);
	} else {
		bus->reg = (uint8_t)cmd;
	}
	if (cmd & CMD_STOP) {
		bus_interrupt(bus, STAT_STOP_DET, STATUS_TFNF);
	}
}

static volatile bool g_hw_stop;

static void *hw_main(void *arg)
{
	(void)arg;
	while (!g_hw_stop) {
		usleep(200);
		for (int i = 0; i < 2; i++) {
			struct fake_bus *bus = &g_bus[i];

			if (bus->mode != BUS_OFF && (bus->regs[IC_INTR_MASK / 4] & STAT_TX_EMPTY)) {
				bus_message(bus);
			}
		}
	}
	return NULL;
}

static void expect_bus(int n, const char *expected)
{
	char what[16];

	snprintf(what, sizeof(what), "bus %d", n);
	expect_text(g_bus[n].log, expected, what);
}

/* A write of the register, then a read of one byte with a repeated START
 * (I2C_M_NOSTOP on the write), at freq.
 */
static int dev_read(struct i2c_master_s *dev, uint16_t addr, uint32_t freq, uint8_t reg)
{
	uint8_t value = 0;
	struct i2c_msg_s msgs[2] = {
		{.frequency = freq, .addr = addr, .flags = I2C_M_NOSTOP, .buffer = &reg, .length = 1},
		{.frequency = freq, .addr = addr, .flags = I2C_M_READ, .buffer = &value, .length = 1},
	};
	int ret = I2C_TRANSFER(dev, msgs, 2);

	return ret != 0 ? ret : value;
}

static int dev_rx(struct i2c_master_s *dev, uint16_t addr, uint8_t *buf, ssize_t len)
{
	struct i2c_msg_s msg = {
		.frequency = 100000, .addr = addr, .flags = I2C_M_READ, .buffer = buf, .length = len,
	};

	return I2C_TRANSFER(dev, &msg, 1);
}

static void reset_values(volatile uint32_t *regs, uint32_t comp_type)
{
	regs[IC_COMP_TYPE / 4] = comp_type;
	regs[IC_CON / 4] = 0x65;
	regs[IC_SDA_HOLD / 4] = 1;
	regs[IC_FS_SPKLEN / 4] = 7;
	regs[IC_ENABLE / 4] = 1;
}

static struct i2c_dw_rp2040_dma_stats g_stats;

static void expect_fallbacks(uint32_t n)
{
	struct i2c_dw_rp2040_dma_stats now;

	i2c_dw_rp2040_dma_get_stats(&now);
	assert(now.fallbacks - g_stats.fallbacks == n);
	assert(now.transfers == g_stats.transfers);
	g_stats = now;
}

#define PINS(n, up)                                                                                \
	"init(" #n ") func(" #n ",i2c) pulls(" #n "," #up ",0) drive(" #n ",4mA) slew(" #n         \
	",slow) schmitt(" #n ",1) ie(" #n ") "

/* I2C0 ----------------------------------------------------------------------*/

static struct i2c_master_s *g_i2c0;

static void test_i2c0(void)
{
	uint8_t buf[40];

	reset_values(fake_i2c0_regs, DW_MAGIC);
	g_i2c0 = i2c_dw_i2c0_initialize();
	assert(g_i2c0 != NULL);
	expect_calls("dma-init reset(0x8) unreset(0x8) " PINS(12, 0) PINS(13, 0)
		     "attach(23) enable(23) ");
	assert(g_isr_arg[RP2040_I2C0_IRQ] == (void *)g_i2c0);
	assert(i2c_dw_i2c0_initialize() == g_i2c0);
	expect_calls("");
	assert(fake_i2c0_regs[IC_FS_SPKLEN / 4] == 1 && fake_i2c0_regs[IC_HS_SPKLEN / 4] == 1);

	i2c_dw_rp2040_dma_get_stats(&g_stats);
	g_bus[0].mode = BUS_DEVICE;
	assert(dev_read(g_i2c0, DEV0_ADDR, 100000, 0x0f) == 0x1f);
	expect_bus(0, "0xf 0x700 ");
	assert((fake_i2c0_regs[IC_TAR / 4] & 0x3ff) == DEV0_ADDR);
	assert(fake_i2c0_regs[IC_SS_SCL_HCNT / 4] == 500);
	assert(fake_i2c0_regs[IC_SS_SCL_LCNT / 4] == 625);
	assert(fake_i2c0_regs[IC_CON / 4] == CON_STD_DMA);

	g_bus[0].mode = BUS_NACK;
	assert(dev_read(g_i2c0, 0x10, 100000, 0) == -EIO);
	expect_bus(0, "nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n");

	/* A read the DMA path takes: a channel asked for, none, a fallback. */
	assert(dev_rx(g_i2c0, 0x10, buf, sizeof(buf)) == -EIO);
	expect_bus(0, "nack ");
	expect_calls("dma-request ");
	expect_fallbacks(1);
	expect_log("<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n");
	g_bus[0].mode = BUS_DEVICE;
	assert(dev_read(g_i2c0, DEV0_ADDR, 100000, 0x0f) == 0x1f);
	expect_bus(0, "0x40f 0x700 ");
	expect_log("");
}

#ifdef CONFIG_I2C_DW_I2C1
/* I2C1 ----------------------------------------------------------------------*/

static struct i2c_master_s *g_i2c1;

static void test_i2c1_start(void)
{
	uint32_t i2c0_before[64];

	for (int i = 0; i < 64; i++) {
		assert(fake_i2c1_regs[i] == 0);
		i2c0_before[i] = fake_i2c0_regs[i];
	}
	assert(g_isr[RP2040_I2C1_IRQ] == NULL);

	/* Not a DesignWare block: NULL after the reset and the pins. */
	reset_values(fake_i2c1_regs, 0);
	assert(i2c_dw_i2c1_initialize() == NULL);
	expect_calls("reset(0x10) unreset(0x10) " PINS(18, 1) PINS(19, 1));
	assert(g_isr[RP2040_I2C1_IRQ] == NULL);

	reset_values(fake_i2c1_regs, DW_MAGIC);
	g_i2c1 = i2c_dw_i2c1_initialize();
	assert(g_i2c1 != NULL && g_i2c1 != g_i2c0);
	expect_calls("reset(0x10) unreset(0x10) " PINS(18, 1) PINS(19, 1) "attach(24) enable(24) ");
	assert(g_isr_arg[RP2040_I2C1_IRQ] == (void *)g_i2c1);
	assert(g_isr[RP2040_I2C1_IRQ] == g_isr[RP2040_I2C0_IRQ]);
	assert(g_i2c1->ops == g_i2c0->ops);
	assert(i2c_dw_i2c1_initialize() == g_i2c1);
	expect_calls("");
	assert(fake_i2c1_regs[IC_FS_SPKLEN / 4] == 1 && fake_i2c1_regs[IC_SDA_HOLD / 4] == 1);
	for (int i = 0; i < 64; i++) {
		assert(fake_i2c0_regs[i] == i2c0_before[i]);
	}
	expect_log("");
}

static void test_i2c1_transfers(void)
{
	uint8_t data[2] = {0x12, 0x44};
	struct i2c_msg_s write = {
		.frequency = 100000, .addr = DEV1_ADDR, .flags = 0, .buffer = data, .length = 2,
	};
	uint8_t buf[40];
	uint32_t i2c0_con = fake_i2c0_regs[IC_CON / 4];

	g_bus[1].mode = BUS_DEVICE;
	assert(dev_read(g_i2c1, DEV1_ADDR, 100000, 0x05) == 0x35);
	expect_bus(1, "0x5 0x700 ");
	expect_bus(0, "");
	assert((fake_i2c1_regs[IC_TAR / 4] & 0x3ff) == DEV1_ADDR);
	assert(fake_i2c1_regs[IC_SS_SCL_HCNT / 4] == 500);
	assert(fake_i2c1_regs[IC_CON / 4] == CON_STD);
	assert(I2C_TRANSFER(g_i2c1, &write, 1) == 0);
	expect_bus(1, "0x244 ");
	assert(fake_i2c0_regs[IC_CON / 4] == i2c0_con);

	/* 400 kHz on I2C1; I2C0 stays at 100 kHz. */
	assert(dev_read(g_i2c1, DEV1_ADDR, 400000, 0x06) == 0x36);
	expect_bus(1, "0x6 0x700 ");
	assert(fake_i2c1_regs[IC_FS_SCL_HCNT / 4] == 93 && fake_i2c1_regs[IC_FS_SCL_LCNT / 4] == 109);
	assert(fake_i2c1_regs[IC_CON / 4] == CON_FAST);
	assert(dev_read(g_i2c0, DEV0_ADDR, 100000, 0x0f) == 0x1f);
	expect_bus(0, "0x40f 0x700 ");
	assert(fake_i2c0_regs[IC_CON / 4] == CON_STD_DMA);

	/* An address nothing answers: -EIO at once (not the 1 s timeout). */
	clock_t start = clock_systime_ticks();

	g_bus[1].mode = BUS_NACK;
	assert(dev_rx(g_i2c1, 0x10, buf, 1) == -EIO);
	assert(clock_systime_ticks() - start < 500);
	expect_bus(1, "nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40048000\n<1> i2c_dw: IO Fail on i2c@40048000\n");

	/* 40 bytes as the last message: the interrupts, no DMA channel. */
	assert(dev_rx(g_i2c1, 0x10, buf, sizeof(buf)) == -EIO);
	expect_bus(1, "nack ");
	expect_calls("");
	expect_fallbacks(0);
	expect_log("<1> i2c_dw: NACK on i2c@40048000\n<1> i2c_dw: IO Fail on i2c@40048000\n");
	assert(fake_i2c1_regs[IC_DMA_CR / 4] == 0 && fake_i2c1_regs[IC_INTR_MASK / 4] == 0);

	g_bus[1].mode = BUS_DEVICE;
	assert(dev_read(g_i2c1, DEV1_ADDR, 100000, 0x07) == 0x37);
	expect_bus(1, "0x407 0x700 ");
	assert(fake_i2c1_regs[IC_CON / 4] == CON_STD);
#  ifdef CONFIG_I2C_RESET
	assert(I2C_RESET(g_i2c1) == 0);
#  endif
	expect_log("");
	expect_calls("");
}

#  define INTERLEAVED 200

struct worker {
	struct i2c_master_s *dev;
	uint16_t addr;
	uint8_t base;
	int failures;
};

static void *worker_main(void *arg)
{
	struct worker *w = arg;

	for (int i = 0; i < INTERLEAVED; i++) {
		if (dev_read(w->dev, w->addr, 100000, (uint8_t)i) != (uint8_t)(i + w->base)) {
			w->failures++;
		}
	}
	return NULL;
}

/* Each bus's log: INTERLEAVED pairs, the registers 0 to INTERLEAVED - 1 in
 * order (the first with RESTART after a read), each read with RESTART and
 * STOP.
 */
static void expect_pairs(const char *log)
{
	const char *p = log;

	for (int i = 0; i < INTERLEAVED; i++) {
		unsigned int first, read;
		int used = 0;

		assert(sscanf(p, "0x%x 0x%x %n", &first, &read, &used) == 2 && used > 0);
		assert((first & 0xff) == (unsigned int)i && (first & ~0x4ffu) == 0);
		assert(read == (CMD_READ | CMD_STOP | CMD_RESTART));
		p += used;
	}
	assert(*p == '\0');
}

static void test_i2c1_interleaved(void)
{
	struct worker w[2] = {
		{g_i2c0, DEV0_ADDR, 0x10, 0},
		{g_i2c1, DEV1_ADDR, 0x30, 0},
	};
	pthread_t t[2];

	g_bus[0].mode = BUS_DEVICE;
	g_bus[1].mode = BUS_DEVICE;
	for (int i = 0; i < 2; i++) {
		assert(pthread_create(&t[i], NULL, worker_main, &w[i]) == 0);
	}
	for (int i = 0; i < 2; i++) {
		assert(pthread_join(t[i], NULL) == 0);
		assert(w[i].failures == 0);
	}
	expect_pairs(g_bus[0].log);
	expect_pairs(g_bus[1].log);
	g_bus[0].log[0] = '\0';
	g_bus[1].log[0] = '\0';
	expect_log("");
	expect_calls("");
}
#endif /* CONFIG_I2C_DW_I2C1 */

int main(void)
{
	pthread_mutexattr_t attr;
	pthread_t hw;

	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&g_irq_lock, &attr);
	assert(pthread_create(&hw, NULL, hw_main, NULL) == 0);

	test_i2c0();
#ifdef CONFIG_I2C_DW_I2C1
	test_i2c1_start();
	test_i2c1_transfers();
	test_i2c1_interleaved();
#endif
	g_hw_stop = true;
	assert(pthread_join(hw, NULL) == 0);
	printf("i2c_dw_instance: all tests passed\n");
	return 0;
}
