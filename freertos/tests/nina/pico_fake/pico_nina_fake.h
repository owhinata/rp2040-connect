/* SPDX-License-Identifier: MIT */
/* The part of the Pico SDK that common/drivers/nina/port/freertos/nina_bus_rp2040.c with
 * its DMA part (nina_bus_dma_rp2040.c there) and the NINA core's FreeRTOS
 * binding (common/drivers/nina/port/freertos/nina_os.c: the GPIO interrupt
 * functions for READY's and the IRQ line's edges, the TIMER) use, for their
 * host test (freertos/tests/nina/main.c, which implements the functions:
 * they record the calls and play the module's side of the lines and of
 * SPI1, and the DMA's).  hardware/dma.h, hardware/gpio.h, hardware/irq.h,
 * hardware/resets.h, hardware/spi.h and hardware/timer.h next to this one
 * include it.  Names and values as in the SDK (hardware_dma, hardware_gpio,
 * hardware_irq, hardware_resets, hardware_spi, hardware_timer,
 * hardware/regs/intctrl.h, hardware/regs/dreq.h, hardware/regs/dma.h,
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
#define DMA_IRQ_0    11
#define DMA_IRQ_1    12
#define IO_IRQ_BANK0 13

typedef void (*irq_handler_t)(void);

#define PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY 0x80

void gpio_add_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler);
void irq_add_shared_handler(uint num, irq_handler_t handler, uint8_t order_priority);
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

#define SPI_SSPCR1_SSE_BITS      0x00000002u
#define SPI_SSPSR_BSY_BITS       0x00000010u
#define SPI_SSPSR_RNE_BITS       0x00000004u
#define SPI_SSPRIS_RORRIS_BITS   0x00000001u
#define SPI_SSPICR_RORIC_BITS    0x00000001u
#define SPI_SSPDMACR_TXDMAE_BITS 0x00000002u
#define SPI_SSPDMACR_RXDMAE_BITS 0x00000001u

/* hardware/regs/dreq.h */
#define DREQ_SPI1_TX 18
#define DREQ_SPI1_RX 19

spi_hw_t *spi_get_hw(spi_inst_t *spi);
uint spi_set_baudrate(spi_inst_t *spi, uint baudrate);
void spi_set_format(spi_inst_t *spi, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha,
		    spi_order_t order);
int spi_write_read_blocking(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len);
int spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len);

static inline uint spi_get_dreq(spi_inst_t *spi, bool is_tx)
{
	(void)spi;
	return is_tx ? DREQ_SPI1_TX : DREQ_SPI1_RX;
}

/* hardware_timer */
uint64_t time_us_64(void);

/* hardware/address_mapped.h: the atomic set alias (main.c: SSE in SPI1's
 * CR1, and the write 1 to clear error flags of a DMA channel's AL1_CTRL).
 */
typedef volatile uint32_t io_rw_32;

void hw_set_bits(io_rw_32 *addr, uint32_t mask);

/* hardware/regs/dma.h */
#define DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS   0x80000000u
#define DMA_CH0_CTRL_TRIG_READ_ERROR_BITS  0x40000000u
#define DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS 0x20000000u
#define DMA_CH0_CTRL_TRIG_BUSY_BITS        0x01000000u
#define DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS    0x001f8000u
#define DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB     15u
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS    0x00007800u
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB     11u
#define DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS  0x00000020u
#define DMA_CH0_CTRL_TRIG_INCR_READ_BITS   0x00000010u
#define DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS   0x0000000cu
#define DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB    2u
#define DMA_CH0_CTRL_TRIG_EN_BITS          0x00000001u

/* hardware/structs/dma.h: the channels' control registers and CHAN_ABORT
 * as memory; the addresses and counts of dma_channel_configure(), the raw
 * flags (INTR) and the lines' enables are main.c's (the functions below).
 */
#define NUM_DMA_CHANNELS 12u

typedef struct {
	io_rw_32 al1_ctrl;
} dma_channel_hw_t;

typedef struct {
	dma_channel_hw_t ch[NUM_DMA_CHANNELS];
	io_rw_32 abort;
} dma_hw_t;

extern dma_hw_t fake_dma_hw;
#define dma_hw (&fake_dma_hw)

/* hardware/dma.h: the configuration's setters and the default are the
 * SDK's; the others are main.c's.  dma_channel_is_busy() also ends the
 * aborts written to CHAN_ABORT (the fake cannot see the write itself).
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
bool dma_channel_is_busy(uint channel);
bool dma_irqn_get_channel_status(uint irq_index, uint channel);
void dma_irqn_acknowledge_channel(uint irq_index, uint channel);
void dma_irqn_set_channel_enabled(uint irq_index, uint channel, bool enabled);

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

static inline void channel_config_set_transfer_data_size(dma_channel_config_t *c,
							 dma_channel_transfer_size_t size)
{
	c->ctrl = (c->ctrl & ~DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) |
		  (((uint32_t)size) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);
}

/* The SDK's default: read increment, no write increment, DREQ_FORCE (0x3f),
 * chained to itself, 32-bit transfers, enabled (the other bits 0).
 */
static inline dma_channel_config_t dma_channel_get_default_config(uint channel)
{
	dma_channel_config_t c = {0};

	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, 0x3f);
	c.ctrl = (c.ctrl & ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) |
		 (channel << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	c.ctrl |= DMA_CH0_CTRL_TRIG_EN_BITS;
	return c;
}

static inline uint32_t dma_encode_transfer_count(uint transfer_count)
{
	return transfer_count;
}

#endif /* FREERTOS_TESTS_NINA_PICO_FAKE_PICO_NINA_FAKE_H */
