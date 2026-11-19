/* SPDX-License-Identifier: MIT */
/* The part of the Pico SDK that the FreeRTOS part of the I2C0 core uses
 * (freertos/drivers/i2c_dw: i2c_dw_os_impl.h, i2c_dw_instance.h,
 * i2c_dw_dma_os.c), for its host test (freertos/tests/i2c_dw/main.c, which
 * implements the functions: they record the calls).  pico.h and the
 * hardware/ headers next to this one include it.  Names and values as in
 * the SDK (SYS_CLK_HZ of hardware/platform_defs.h, hardware_gpio,
 * hardware_resets, hardware_irq, hardware/regs/intctrl.h, hardware_sync,
 * hardware_dma, hardware/regs/dma.h, hardware/regs/dreq.h), except the
 * registers: the I2C0 block's are the array fake_i2c0_regs, the DMA's the
 * structure fake_dma_hw (with the channels' addresses as uintptr_t, a
 * host's pointers being wider than the RP2040's registers, and only the
 * registers that i2c_dw_dma_os.c uses).
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
	RESET_I2C0 = 3,
};

void reset_block_num(uint32_t block_num);
void unreset_block_num_wait_blocking(uint32_t block_num);

/* hardware/regs/intctrl.h and hardware_irq */
#define DMA_IRQ_0 11
#define DMA_IRQ_1 12
#define I2C0_IRQ  23

typedef void (*irq_handler_t)(void);

#define PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY 0x80

void irq_set_exclusive_handler(uint num, irq_handler_t handler);
void irq_add_shared_handler(uint num, irq_handler_t handler, uint8_t order_priority);
void irq_set_enabled(uint num, bool enabled);

/* hardware_sync */
uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t status);

/* hardware/address_mapped.h: the atomic set alias.  The fake writes the
 * bits into the register (main.c), and for the error flags of a channel's
 * AL1_CTRL (write 1 to clear) clears them, as the hardware does.
 */
typedef volatile uint32_t io_rw_32;

void hw_set_bits(io_rw_32 *addr, uint32_t mask);

/* hardware/regs/dreq.h, hardware/regs/dma.h */
#define DREQ_I2C0_TX 32
#define DREQ_I2C0_RX 33

#define DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS     0x80000000u
#define DMA_CH0_CTRL_TRIG_READ_ERROR_BITS    0x40000000u
#define DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS   0x20000000u
#define DMA_CH0_CTRL_TRIG_BUSY_BITS          0x01000000u
#define DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS      0x001f8000u
#define DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB       15u
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS      0x00007800u
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB       11u
#define DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS    0x00000020u
#define DMA_CH0_CTRL_TRIG_INCR_READ_BITS     0x00000010u
#define DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS     0x0000000cu
#define DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB      2u
#define DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS 0x00000002u
#define DMA_CH0_CTRL_TRIG_EN_BITS            0x00000001u

/* hardware/structs/dma.h */
#define NUM_DMA_CHANNELS 12u

typedef struct {
	uintptr_t read_addr;
	uintptr_t write_addr;
	uint32_t transfer_count;
	io_rw_32 al1_ctrl;
} dma_channel_hw_t;

typedef struct {
	io_rw_32 intr;
	io_rw_32 inte;
	io_rw_32 intf;
	io_rw_32 ints;
} dma_irq_ctrl_hw_t;

typedef struct {
	dma_channel_hw_t ch[NUM_DMA_CHANNELS];
	dma_irq_ctrl_hw_t irq_ctrl[2];
	io_rw_32 multi_channel_trigger;
	io_rw_32 abort;
} dma_hw_t;

extern dma_hw_t fake_dma_hw;
#define dma_hw (&fake_dma_hw)

/* hardware/dma.h: the claims, dma_channel_configure() and
 * dma_channel_start() record their calls (main.c); the channel
 * configuration's setters are the SDK's.  The channels' raw flags are one
 * register, INTR (irq_ctrl[0].intr; the RP2040 has no irq_ctrl[1].intr), and
 * a channel's flag on line n is INTR and INTEn (INTSn, as the hardware reads
 * it), cleared on either line by a write of 1.
 */
#define DMA_IRQ_NUM(irq_index) (DMA_IRQ_0 + (irq_index))

typedef enum dma_channel_transfer_size {
	DMA_SIZE_8 = 0,
	DMA_SIZE_16 = 1,
	DMA_SIZE_32 = 2
} dma_channel_transfer_size_t;

typedef struct {
	uint32_t ctrl;
} dma_channel_config_t;

typedef dma_channel_config_t dma_channel_config;

int dma_claim_unused_channel(bool required);
void dma_channel_unclaim(uint channel);
void dma_channel_configure(uint channel, const dma_channel_config_t *config,
			   volatile void *write_addr, const volatile void *read_addr,
			   uint32_t encoded_transfer_count, bool trigger);
void dma_channel_start(uint channel);

static inline void channel_config_set_read_increment(dma_channel_config_t *c, bool incr)
{
	c->ctrl = incr ? (c->ctrl | DMA_CH0_CTRL_TRIG_INCR_READ_BITS)
		       : (c->ctrl & ~DMA_CH0_CTRL_TRIG_INCR_READ_BITS);
}

static inline void channel_config_set_write_increment(dma_channel_config_t *c, bool incr)
{
	c->ctrl = incr ? (c->ctrl | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS)
		       : (c->ctrl & ~DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);
}

static inline void channel_config_set_dreq(dma_channel_config_t *c, uint dreq)
{
	c->ctrl = (c->ctrl & ~DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) |
		  (dreq << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);
}

static inline void channel_config_set_chain_to(dma_channel_config_t *c, uint chain_to)
{
	c->ctrl = (c->ctrl & ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) |
		  (chain_to << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);
}

static inline void channel_config_set_transfer_data_size(dma_channel_config_t *c,
							 dma_channel_transfer_size_t size)
{
	c->ctrl = (c->ctrl & ~DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) |
		  (((uint32_t)size) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);
}

static inline void channel_config_set_high_priority(dma_channel_config_t *c, bool high_priority)
{
	c->ctrl = high_priority ? (c->ctrl | DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS)
				: (c->ctrl & ~DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS);
}

static inline void channel_config_set_enable(dma_channel_config_t *c, bool enable)
{
	c->ctrl = enable ? (c->ctrl | DMA_CH0_CTRL_TRIG_EN_BITS)
			 : (c->ctrl & ~DMA_CH0_CTRL_TRIG_EN_BITS);
}

/* The SDK's default: read increment, no write increment, DREQ_FORCE (0x3f),
 * chained to itself, 32-bit transfers, enabled, not high priority (the
 * ring, byte swap, IRQ_QUIET and sniff bits stay 0).
 */
static inline dma_channel_config_t dma_channel_get_default_config(uint channel)
{
	dma_channel_config_t c = {0};

	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, 0x3f);
	channel_config_set_chain_to(&c, channel);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	channel_config_set_enable(&c, true);
	channel_config_set_high_priority(&c, false);
	return c;
}

static inline uint32_t dma_encode_transfer_count(uint transfer_count)
{
	return transfer_count;
}

static inline bool dma_channel_is_busy(uint channel)
{
	return (dma_hw->ch[channel].al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) != 0;
}

static inline void dma_irqn_set_channel_enabled(uint irq_index, uint channel, bool enabled)
{
	if (enabled) {
		dma_hw->irq_ctrl[irq_index].inte |= 1u << channel;
	} else {
		dma_hw->irq_ctrl[irq_index].inte &= ~(1u << channel);
	}
}

static inline bool dma_irqn_get_channel_status(uint irq_index, uint channel)
{
	return (dma_hw->irq_ctrl[0].intr & dma_hw->irq_ctrl[irq_index].inte & (1u << channel)) != 0;
}

static inline void dma_irqn_acknowledge_channel(uint irq_index, uint channel)
{
	(void)irq_index;
	dma_hw->irq_ctrl[0].intr &= ~(1u << channel);
}

#endif /* FREERTOS_TESTS_I2C_DW_PICO_FAKE_PICO_I2C_FAKE_H */
