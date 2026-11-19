/*
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The RP2040 DMA as the Zephyr build drives it for the I2C0 DMA reads (i2c_dw.c), in two parts:
 *
 * - the parts of Zephyr's DMA API (zephyr/include/zephyr/drivers/dma.h: struct dma_config,
 *   struct dma_block_config, dma_request_channel(), dma_release_channel(), dma_config(),
 *   dma_start(), dma_stop()) and of its RP2040 driver (zephyr/drivers/dma/dma_rpi_pico.c) that
 *   i2c_dw.c uses, as dma_rpi_pico_*() (see dma_rpi_pico.c);
 * - the parts of the Pico SDK that Zephyr's dma_rpi_pico.c and the patched i2c_dw driver call
 *   (hardware/dma.h, hardware/structs/dma.h, hardware/regs/dma.h, hardware/address_mapped.h,
 *   hardware/regs/addressmap.h), with the SDK's names, shapes and values, written out here.
 *
 * No OS header and no SDK header.  The FreeRTOS build has the SDK in the same program, and some
 * of its translation units read SDK headers too (the OS's i2c_dw_os_impl.h reads pico.h and
 * hardware/sync.h before the core; dma_rpi_pico_os.c reads hardware/dma.h before this file).
 * So the SDK's macros are defined here token for token as the SDK defines them (a second,
 * identical definition is no error, and a different one is), and the SDK's types and functions
 * only when the SDK's header that has them (its include guard) was not read first.  An SDK
 * header read after this one would fail to compile; none is.
 */

#ifndef COMMON_DRIVERS_I2C_DW_DMA_RPI_PICO_H_
#define COMMON_DRIVERS_I2C_DW_DMA_RPI_PICO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef BIT
#define BIT(nr) (1ul << (nr))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

/* zephyr/include/zephyr/drivers/dma.h: enum dma_channel_direction, enum dma_addr_adj. */
#define DMA_RPI_PICO_MEMORY_TO_MEMORY     0x0
#define DMA_RPI_PICO_MEMORY_TO_PERIPHERAL 0x1
#define DMA_RPI_PICO_PERIPHERAL_TO_MEMORY 0x2

#define DMA_RPI_PICO_ADDR_ADJ_INCREMENT 0
#define DMA_RPI_PICO_ADDR_ADJ_DECREMENT 1
#define DMA_RPI_PICO_ADDR_ADJ_NO_CHANGE 2

/*
 * zephyr/include/zephyr/dt-bindings/dma/rpi-pico-dma-common.h and rpi-pico-dma-rp2040.h: a
 * devicetree "slot" is the inverted DREQ number.
 */
#define DMA_RPI_PICO_SLOT_TO_DREQ(s) (~(s) & 0x3F)
#define DMA_RPI_PICO_DREQ_TO_SLOT    DMA_RPI_PICO_SLOT_TO_DREQ
#define DMA_RPI_PICO_SLOT_I2C0_TX    DMA_RPI_PICO_DREQ_TO_SLOT(0x20)
#define DMA_RPI_PICO_SLOT_I2C0_RX    DMA_RPI_PICO_DREQ_TO_SLOT(0x21)

/* The RP2040 has 12 channels (dma-channels = <12> in rp2040.dtsi). */
#define DMA_RPI_PICO_CHANNELS 12

/*
 * dma_callback_t: status 0 when the block completed, -EIO with a bus error (dma_rpi_pico_isr()).
 * Called from the DMA interrupt.
 */
typedef void (*dma_rpi_pico_callback_t)(void *user_data, uint32_t channel, int status);

struct dma_rpi_pico_block_config {
	uintptr_t source_address;
	uintptr_t dest_address;
	uint32_t block_size; /* bytes */
	uint16_t source_addr_adj;
	uint16_t dest_addr_adj;
};

struct dma_rpi_pico_channel_config {
	uint32_t dma_slot;
	uint32_t channel_direction;
	uint32_t channel_priority;
	uint32_t source_data_size; /* 1, 2 or 4 bytes */
	uint32_t dest_data_size;
	uint32_t block_count;
	struct dma_rpi_pico_block_config *head_block;
	dma_rpi_pico_callback_t dma_callback;
	void *user_data;
};

/*
 * dma_rpi_pico_init(): reset the DMA block and attach DMA_IRQ_0 and DMA_IRQ_1 (the OS's part,
 * dma_rpi_pico_instance.h).  From the board's start-up, before I2C0.
 */
int dma_rpi_pico_initialize(void);

/* device_is_ready(dma) */
bool dma_rpi_pico_is_ready(void);

/* dma_request_channel(dma, NULL): the lowest free channel, or -EINVAL. */
int dma_rpi_pico_request_channel(void);

/*
 * dma_release_channel(): marks the channel free (the RP2040 driver has no chan_release; the
 * hardware is left as it is).
 */
void dma_rpi_pico_release_channel(uint32_t channel);

/*
 * dma_config(), dma_start(), dma_stop(): Zephyr's dma_rpi_pico_config(), dma_rpi_pico_start()
 * and dma_rpi_pico_stop() without the device.
 */
int dma_rpi_pico_config(uint32_t channel, struct dma_rpi_pico_channel_config *dma_cfg);
int dma_rpi_pico_start(uint32_t ch);
int dma_rpi_pico_stop(uint32_t ch);

/*
 * The Pico SDK (RP2040), as far as the core uses it.  Macros: as the SDK's, token for token
 * (hardware/platform_defs.h, hardware/regs/addressmap.h, hardware/regs/dma.h).
 */
#ifndef _u
#define _u(x) x##u
#endif

#define REG_ALIAS_SET_BITS (_u(0x2) << _u(12))
#define REG_ALIAS_CLR_BITS (_u(0x3) << _u(12))
#define DMA_BASE           _u(0x50000000)

#define DMA_CH0_READ_ADDR_OFFSET   _u(0x00000000)
#define DMA_CH0_WRITE_ADDR_OFFSET  _u(0x00000004)
#define DMA_CH0_TRANS_COUNT_OFFSET _u(0x00000008)
#define DMA_CH0_CTRL_TRIG_OFFSET   _u(0x0000000c)
#define DMA_CH0_AL1_CTRL_OFFSET    _u(0x00000010)
#define DMA_INTE0_OFFSET           _u(0x00000404)
#define DMA_INTS0_OFFSET           _u(0x0000040c)
#define DMA_INTE1_OFFSET           _u(0x00000414)
#define DMA_INTS1_OFFSET           _u(0x0000041c)
#define DMA_CHAN_ABORT_OFFSET      _u(0x00000444)

#define DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS     _u(0x80000000)
#define DMA_CH0_CTRL_TRIG_READ_ERROR_BITS    _u(0x40000000)
#define DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS   _u(0x20000000)
#define DMA_CH0_CTRL_TRIG_BUSY_BITS          _u(0x01000000)
#define DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS      _u(0x001f8000)
#define DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB       _u(15)
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS      _u(0x00007800)
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB       _u(11)
#define DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS    _u(0x00000020)
#define DMA_CH0_CTRL_TRIG_INCR_READ_BITS     _u(0x00000010)
#define DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS     _u(0x0000000c)
#define DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB      _u(2)
#define DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS _u(0x00000002)
#define DMA_CH0_CTRL_TRIG_EN_BITS            _u(0x00000001)

/* hardware/address_mapped.h: the register types and the atomic set and clear aliases. */
#ifndef _HARDWARE_ADDRESS_MAPPED_H
typedef volatile uint32_t io_rw_32;
typedef const volatile uint32_t io_ro_32;
typedef volatile uint32_t io_wo_32;

static inline void hw_set_bits(io_rw_32 *addr, uint32_t mask)
{
	*(io_rw_32 *)(REG_ALIAS_SET_BITS + (uintptr_t)addr) = mask;
}

static inline void hw_clear_bits(io_rw_32 *addr, uint32_t mask)
{
	*(io_rw_32 *)(REG_ALIAS_CLR_BITS + (uintptr_t)addr) = mask;
}
#endif /* _HARDWARE_ADDRESS_MAPPED_H */

/* hardware/structs/dma.h: the DMA block's registers (RP2040 datasheet 2.5.7), dma_hw. */
#ifndef _HARDWARE_STRUCTS_DMA_H
typedef struct {
	io_rw_32 read_addr;
	io_rw_32 write_addr;
	io_rw_32 transfer_count;
	io_rw_32 ctrl_trig;
	io_rw_32 al1_ctrl;
	io_rw_32 al1_read_addr;
	io_rw_32 al1_write_addr;
	io_rw_32 al1_transfer_count_trig;
	io_rw_32 al2_ctrl;
	io_rw_32 al2_transfer_count;
	io_rw_32 al2_read_addr;
	io_rw_32 al2_write_addr_trig;
	io_rw_32 al3_ctrl;
	io_rw_32 al3_write_addr;
	io_rw_32 al3_transfer_count;
	io_rw_32 al3_read_addr_trig;
} dma_channel_hw_t;

typedef struct {
	io_rw_32 intr;
	io_rw_32 inte;
	io_rw_32 intf;
	io_rw_32 ints;
} dma_irq_ctrl_hw_t;

typedef struct {
	dma_channel_hw_t ch[12];
	uint32_t _pad0[64];
	dma_irq_ctrl_hw_t irq_ctrl[2];
	io_rw_32 timer[4];
	io_wo_32 multi_channel_trigger;
	io_rw_32 sniff_ctrl;
	io_rw_32 sniff_data;
	uint32_t _pad1;
	io_ro_32 fifo_levels;
	io_wo_32 abort;
} dma_hw_t;

#define dma_hw ((dma_hw_t *)DMA_BASE)

_Static_assert(offsetof(dma_hw_t, ch[1].read_addr) == 0x40 + DMA_CH0_READ_ADDR_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, ch[1].write_addr) == 0x40 + DMA_CH0_WRITE_ADDR_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, ch[1].transfer_count) == 0x40 + DMA_CH0_TRANS_COUNT_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, ch[1].ctrl_trig) == 0x40 + DMA_CH0_CTRL_TRIG_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, ch[1].al1_ctrl) == 0x40 + DMA_CH0_AL1_CTRL_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, irq_ctrl[0].inte) == DMA_INTE0_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, irq_ctrl[0].ints) == DMA_INTS0_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, irq_ctrl[1].inte) == DMA_INTE1_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, irq_ctrl[1].ints) == DMA_INTS1_OFFSET, "");
_Static_assert(offsetof(dma_hw_t, abort) == DMA_CHAN_ABORT_OFFSET, "");
#endif /* _HARDWARE_STRUCTS_DMA_H */

/*
 * hardware/dma.h: the channel configuration (the CTRL value) and the calls of Zephyr's
 * dma_rpi_pico.c and of the patched i2c_dw driver, without the SDK's parameter checks (off in
 * its builds).  Unsigned parameters are uint32_t where the SDK has uint.
 */
#ifndef _HARDWARE_DMA_H
typedef enum dma_channel_transfer_size {
	DMA_SIZE_8 = 0,
	DMA_SIZE_16 = 1,
	DMA_SIZE_32 = 2
} dma_channel_transfer_size_t;

typedef struct {
	uint32_t ctrl;
} dma_channel_config_t;

typedef dma_channel_config_t dma_channel_config;

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

static inline void channel_config_set_dreq(dma_channel_config_t *c, uint32_t dreq)
{
	c->ctrl = (c->ctrl & ~DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) |
		  (dreq << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);
}

static inline void channel_config_set_chain_to(dma_channel_config_t *c, uint32_t chain_to)
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

/*
 * The SDK's default: read increment, no write increment, DREQ_FORCE (0x3f), chained to itself,
 * 32-bit transfers, enabled, not high priority.  Its other setters (no ring, no byte swap,
 * IRQ_QUIET off, sniff off) clear bits that are 0 in {0} already.
 */
static inline dma_channel_config_t dma_channel_get_default_config(uint32_t channel)
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

/*
 * dma_channel_configure() with dma_channel_set_read_addr(), _set_write_addr() and
 * _set_transfer_count() (without trigger) and dma_channel_set_config() (CTRL_TRIG with trigger,
 * which starts the channel; else AL1_CTRL).
 */
static inline void dma_channel_configure(uint32_t channel, const dma_channel_config_t *config,
					 volatile void *write_addr, const volatile void *read_addr,
					 uint32_t encoded_transfer_count, bool trigger)
{
	dma_hw->ch[channel].read_addr = (uintptr_t)read_addr;
	dma_hw->ch[channel].write_addr = (uintptr_t)write_addr;
	dma_hw->ch[channel].transfer_count = encoded_transfer_count;
	if (!trigger) {
		dma_hw->ch[channel].al1_ctrl = config->ctrl;
	} else {
		dma_hw->ch[channel].ctrl_trig = config->ctrl;
	}
}

static inline bool dma_channel_is_busy(uint32_t channel)
{
	return dma_hw->ch[channel].al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS;
}

static inline void dma_irqn_set_channel_enabled(uint32_t irq_index, uint32_t channel, bool enabled)
{
	if (enabled) {
		hw_set_bits(&dma_hw->irq_ctrl[irq_index].inte, 1u << channel);
	} else {
		hw_clear_bits(&dma_hw->irq_ctrl[irq_index].inte, 1u << channel);
	}
}

static inline bool dma_irqn_get_channel_status(uint32_t irq_index, uint32_t channel)
{
	return dma_hw->irq_ctrl[irq_index].ints & (1u << channel);
}

/* Writing 1 to the channel's bit of INTS0 or INTS1 clears its INTR. */
static inline void dma_irqn_acknowledge_channel(uint32_t irq_index, uint32_t channel)
{
	dma_hw->irq_ctrl[irq_index].ints = 1u << channel;
}
#endif /* _HARDWARE_DMA_H */

#endif /* COMMON_DRIVERS_I2C_DW_DMA_RPI_PICO_H_ */
