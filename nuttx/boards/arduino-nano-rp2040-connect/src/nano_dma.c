/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_dma.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com>
 *
 ****************************************************************************/

/* The RP2040 DMA for the I2C0 DMA reads (nano_i2c.c, CONFIG_NANO_I2C0_DMA):
 * a port of the Zephyr build's DMA driver, zephyr/drivers/dma/
 * dma_rpi_pico.c, with the channel allocation of Zephyr's DMA API
 * (dma_request_channel() and dma_release_channel() in
 * zephyr/include/zephyr/drivers/dma.h) and the pico-sdk calls it makes
 * (hardware/dma.h), written out on the registers.  The same register
 * values in the same order:
 *
 * - dma_config() writes nothing: it keeps the CTRL value of the pico-sdk's
 *   dma_channel_get_default_config() (EN, CHAIN_TO = the channel itself,
 *   no ring, no byte swap, IRQ_QUIET off, sniff off) with INCR_READ,
 *   INCR_WRITE, DATA_SIZE, TREQ_SEL (the DREQ of the slot) and
 *   HIGH_PRIORITY (channel_priority != 0) set from the configuration, and
 *   the addresses and the block size.
 * - dma_start(): nothing if the channel is BUSY; else acknowledge the
 *   channel on its interrupt line, enable it there (INTE set alias), then
 *   READ_ADDR, WRITE_ADDR, TRANS_COUNT (block size / data size) and
 *   CTRL_TRIG (dma_channel_configure(..., true): starts the channel).
 * - dma_stop(): disable the channel on its line (INTE clear alias), write
 *   AL1_CTRL back with the error bits cleared
 *   (rpi_pico_dma_channel_clear_error_flags()), CHAN_ABORT = the channel's
 *   bit.  It does not wait for the abort (the caller does).
 * - The interrupt line of a channel: DMA_IRQ_0 for the channels listed in
 *   the devicetree's irq0-channels, DMA_IRQ_1 for the others.  The Zephyr
 *   build has no irq0-channels property, so the list is the default {0}:
 *   channel 0 on DMA_IRQ_0, channels 1-11 on DMA_IRQ_1.  Both lines run
 *   the same handler, which goes through all channels: for each one whose
 *   status is set on its own line, the status of the bus error bits, the
 *   acknowledge, disable on the line, the error bits written back, then
 *   the callback with 0 or -EIO.
 * - dma_request_channel(dma, NULL): the lowest channel not taken (the RP2040
 *   driver's filter accepts every channel without a parameter), or -EINVAL;
 *   it never waits.  dma_release_channel(): the RP2040 driver has no
 *   chan_release, so only the channel's bit is cleared.
 * - Init (dma_rpi_pico_init()): reset the DMA block (reset_line_toggle_dt())
 *   and attach and enable both interrupts.
 *
 * NuttX's own DMA layer (arch/arm/src/rp2040/rp2040_dmac.c,
 * CONFIG_RP2040_DMAC) must be off: it takes DMA_IRQ_0 for all channels,
 * allocates channels with a wait, and stops a channel with an unbounded
 * wait.  Nothing else in this port uses the DMA (the NINA's SPI runs
 * without DMA, as in the Zephyr build), as in the Zephyr build, where i2c0
 * is the DMA's only user.
 *
 * Left out (not used by i2c_dw.c): reload, get_status, block chains,
 * memory-to-memory, more than one DMA instance.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/rp2040_dma.h"
#include "hardware/rp2040_memorymap.h"
#include "hardware/rp2040_resets.h"

/* CONFIG_DMA_LOG_LEVEL of the Zephyr build (3). */

#define LOG_LEVEL 3
#include "app_log.h"

#include "nano_dma.h"

LOG_MODULE_REGISTER(dma_rpi_pico);

#define DMA_INT_ERROR_FLAGS                                                    \
  (RP2040_DMA_CTRL_TRIG_AHB_ERROR | RP2040_DMA_CTRL_TRIG_READ_ERROR |          \
   RP2040_DMA_CTRL_TRIG_WRITE_ERROR)

/* pico-sdk enum dma_channel_transfer_size */

#define DMA_SIZE_8  0
#define DMA_SIZE_16 1
#define DMA_SIZE_32 2

struct dma_rpi_pico_channel
{
  nano_dma_callback_t callback;
  void *user_data;
  uint32_t direction;
  uint32_t config;              /* dma_channel_config: the CTRL value */
  uintptr_t source_address;
  uintptr_t dest_address;
  uint32_t block_size;
  uint32_t source_data_size;
};

static struct dma_rpi_pico_channel g_channels[NANO_DMA_CHANNELS];

/* dma_context.atomic: the taken channels. */

static uint32_t g_taken;

/* The devicetree's irq0-channels (default {0}). */

static const uint32_t g_irq0_channels[] =
{
  0
};

static bool g_ready;

/****************************************************************************
 * Register access functions (dma_rpi_pico.c)
 ****************************************************************************/

static inline void rpi_pico_dma_channel_clear_error_flags(uint32_t channel)
{
  /* ch[channel].al1_ctrl &= ~DMA_INT_ERROR_FLAGS */

  uintptr_t al1 = RP2040_DMA_AL1_CTRL(channel);

  putreg32(getreg32(al1) & ~DMA_INT_ERROR_FLAGS, al1);
}

static inline uint32_t rpi_pico_dma_channel_get_error_flags(uint32_t channel)
{
  return getreg32(RP2040_DMA_AL1_CTRL(channel)) & DMA_INT_ERROR_FLAGS;
}

static inline void rpi_pico_dma_channel_abort(uint32_t channel)
{
  putreg32(1u << channel, RP2040_DMA_CHAN_ABORT);
}

/****************************************************************************
 * Utility functions
 ****************************************************************************/

static inline uint32_t dma_rpi_pico_transfer_size(uint32_t width)
{
  switch (width)
    {
    case 4:
      return DMA_SIZE_32;
    case 2:
      return DMA_SIZE_16;
    default:
      return DMA_SIZE_8;
    }
}

static inline uint32_t dma_rpi_pico_channel_irq(uint32_t channel)
{
  for (size_t i = 0; i < sizeof(g_irq0_channels) / sizeof(g_irq0_channels[0]);
       i++)
    {
      if (g_irq0_channels[i] == channel)
        {
          return 0;
        }
    }

  return 1;
}

/* dma_irqn_set_channel_enabled(): hw_set_bits()/hw_clear_bits() on INTE0
 * or INTE1 (the atomic set and clear aliases).
 */

static inline void dma_irqn_set_channel_enabled(uint32_t irq_index,
                                                uint32_t channel,
                                                bool enabled)
{
  uintptr_t inte = irq_index ? RP2040_DMA_INTE1 : RP2040_DMA_INTE0;

  if (enabled)
    {
      setbits_reg32(1u << channel, inte);
    }
  else
    {
      clrbits_reg32(1u << channel, inte);
    }
}

static inline bool dma_irqn_get_channel_status(uint32_t irq_index,
                                               uint32_t channel)
{
  uintptr_t ints = irq_index ? RP2040_DMA_INTS1 : RP2040_DMA_INTS0;

  return (getreg32(ints) & (1u << channel)) != 0;
}

/* pico-sdk dma_channel_get_default_config() with the setters that
 * dma_rpi_pico_config() calls.
 */

static uint32_t dma_rpi_pico_ctrl(uint32_t channel, bool read_increment,
                                  bool write_increment, uint32_t size,
                                  uint32_t dreq, bool high_priority)
{
  uint32_t ctrl = 0;

  if (read_increment)
    {
      ctrl |= RP2040_DMA_CTRL_TRIG_INCR_READ;
    }

  if (write_increment)
    {
      ctrl |= RP2040_DMA_CTRL_TRIG_INCR_WRITE;
    }

  ctrl |= (dreq << RP2040_DMA_CTRL_TRIG_TREQ_SEL_SHIFT) &
          RP2040_DMA_CTRL_TRIG_TREQ_SEL_MASK;
  ctrl |= (channel << RP2040_DMA_CTRL_TRIG_CHAIN_TO_SHIFT) &
          RP2040_DMA_CTRL_TRIG_CHAIN_TO_MASK;
  ctrl |= (size << RP2040_DMA_CTRL_TRIG_DATA_SIZE_SHIFT) &
          RP2040_DMA_CTRL_TRIG_DATA_SIZE_MASK;
  ctrl |= RP2040_DMA_CTRL_TRIG_EN;
  if (high_priority)
    {
      ctrl |= RP2040_DMA_CTRL_TRIG_HIGH_PRIORITY;
    }

  return ctrl;
}

/* dma_channel_configure(channel, config, write_addr, read_addr, count,
 * true).
 */

static void dma_channel_configure(uint32_t channel, uint32_t ctrl,
                                  uintptr_t write_addr, uintptr_t read_addr,
                                  uint32_t count)
{
  putreg32(read_addr, RP2040_DMA_READ_ADDR(channel));
  putreg32(write_addr, RP2040_DMA_WRITE_ADDR(channel));
  putreg32(count, RP2040_DMA_TRANS_COUNT(channel));
  putreg32(ctrl, RP2040_DMA_CTRL_TRIG(channel));
}

/****************************************************************************
 * Interrupt handler
 ****************************************************************************/

static int dma_rpi_pico_isr(int irq, void *context, void *arg)
{
  (void)irq;
  (void)context;
  (void)arg;

  for (uint32_t i = 0; i < NANO_DMA_CHANNELS; i++)
    {
      int err = 0;

      if (!dma_irqn_get_channel_status(dma_rpi_pico_channel_irq(i), i))
        {
          continue;
        }

      if (rpi_pico_dma_channel_get_error_flags(i))
        {
          err = -EIO;
        }

      nano_dma_irqn_acknowledge_channel(dma_rpi_pico_channel_irq(i), i);
      dma_irqn_set_channel_enabled(dma_rpi_pico_channel_irq(i), i, false);
      rpi_pico_dma_channel_clear_error_flags(i);

      if (g_channels[i].callback)
        {
          g_channels[i].callback(g_channels[i].user_data, i, err);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nano_dma_config(uint32_t channel, const struct nano_dma_config *cfg)
{
  struct dma_rpi_pico_channel *ch;

  if (channel >= NANO_DMA_CHANNELS)
    {
      LOG_ERR("channel must be < %u (%u)", NANO_DMA_CHANNELS,
              (unsigned int)channel);
      return -EINVAL;
    }

  if (cfg->block_count != 1)
    {
      LOG_ERR("chained block transfer not supported.");
      return -ENOTSUP;
    }

  if (cfg->channel_priority > 3)
    {
      LOG_ERR("channel_priority must be < 4 (%u)",
              (unsigned int)cfg->channel_priority);
      return -EINVAL;
    }

  if (cfg->head_block->source_addr_adj == NANO_DMA_ADDR_ADJ_DECREMENT)
    {
      LOG_ERR("source_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
      return -ENOTSUP;
    }

  if (cfg->head_block->dest_addr_adj == NANO_DMA_ADDR_ADJ_DECREMENT)
    {
      LOG_ERR("dest_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
      return -ENOTSUP;
    }

  if (cfg->head_block->source_addr_adj != NANO_DMA_ADDR_ADJ_INCREMENT &&
      cfg->head_block->source_addr_adj != NANO_DMA_ADDR_ADJ_NO_CHANGE)
    {
      LOG_ERR("invalid source_addr_adj %u",
              (unsigned int)cfg->head_block->source_addr_adj);
      return -ENOTSUP;
    }

  if (cfg->head_block->dest_addr_adj != NANO_DMA_ADDR_ADJ_INCREMENT &&
      cfg->head_block->dest_addr_adj != NANO_DMA_ADDR_ADJ_NO_CHANGE)
    {
      LOG_ERR("invalid dest_addr_adj %u",
              (unsigned int)cfg->head_block->dest_addr_adj);
      return -ENOTSUP;
    }

  if (cfg->source_data_size != 1 && cfg->source_data_size != 2 &&
      cfg->source_data_size != 4)
    {
      LOG_ERR("source_data_size must be 1, 2, or 4 (%u)",
              (unsigned int)cfg->source_data_size);
      return -EINVAL;
    }

  if (cfg->source_data_size != cfg->dest_data_size)
    {
      return -EINVAL;
    }

  if (cfg->dest_data_size != 1 && cfg->dest_data_size != 2 &&
      cfg->dest_data_size != 4)
    {
      LOG_ERR("dest_data_size must be 1, 2, or 4 (%u)",
              (unsigned int)cfg->dest_data_size);
      return -EINVAL;
    }

  if (cfg->channel_direction > NANO_DMA_PERIPHERAL_TO_MEMORY)
    {
      LOG_ERR("channel_direction must be MEMORY_TO_MEMORY, "
              "MEMORY_TO_PERIPHERAL or PERIPHERAL_TO_MEMORY (%u)",
              (unsigned int)cfg->channel_direction);
      return -ENOTSUP;
    }

  ch = &g_channels[channel];
  ch->config = dma_rpi_pico_ctrl(
      channel,
      cfg->head_block->source_addr_adj == NANO_DMA_ADDR_ADJ_INCREMENT,
      cfg->head_block->dest_addr_adj == NANO_DMA_ADDR_ADJ_INCREMENT,
      dma_rpi_pico_transfer_size(cfg->source_data_size),
      NANO_DMA_SLOT_TO_DREQ(cfg->dma_slot),
      cfg->channel_priority != 0);
  ch->source_address = cfg->head_block->source_address;
  ch->dest_address = cfg->head_block->dest_address;
  ch->block_size = cfg->head_block->block_size;
  ch->callback = cfg->dma_callback;
  ch->user_data = cfg->user_data;
  ch->source_data_size = cfg->source_data_size;
  ch->direction = cfg->channel_direction;

  return 0;
}

int nano_dma_start(uint32_t channel)
{
  struct dma_rpi_pico_channel *ch;

  if (channel >= NANO_DMA_CHANNELS)
    {
      LOG_ERR("start channel must be < %u (%u)", NANO_DMA_CHANNELS,
              (unsigned int)channel);
      return -EINVAL;
    }

  if (nano_dma_channel_is_busy(channel))
    {
      return 0;
    }

  ch = &g_channels[channel];
  nano_dma_irqn_acknowledge_channel(dma_rpi_pico_channel_irq(channel),
                                    channel);
  dma_irqn_set_channel_enabled(dma_rpi_pico_channel_irq(channel), channel,
                               true);

  dma_channel_configure(channel, ch->config, ch->dest_address,
                        ch->source_address,
                        ch->block_size / ch->source_data_size);

  return 0;
}

int nano_dma_stop(uint32_t channel)
{
  if (channel >= NANO_DMA_CHANNELS)
    {
      LOG_ERR("stop channel must be < %u (%u)", NANO_DMA_CHANNELS,
              (unsigned int)channel);
      return -EINVAL;
    }

  dma_irqn_set_channel_enabled(dma_rpi_pico_channel_irq(channel), channel,
                               false);
  rpi_pico_dma_channel_clear_error_flags(channel);

  /* Considering the possibility of being called in an interrupt context,
   * it does not wait until the abort bit becomes clear.
   */

  rpi_pico_dma_channel_abort(channel);

  return 0;
}

int nano_dma_request_channel(void)
{
  int channel = -EINVAL;
  irqstate_t flags;

  if (!g_ready)
    {
      return channel;
    }

  /* atomic_test_and_set_bit() over the channels, lowest first. */

  flags = up_irq_save();
  for (int i = 0; i < NANO_DMA_CHANNELS; i++)
    {
      if ((g_taken & (1u << i)) == 0)
        {
          g_taken |= 1u << i;
          channel = i;
          break;
        }
    }

  up_irq_restore(flags);
  return channel;
}

void nano_dma_release_channel(uint32_t channel)
{
  irqstate_t flags;

  if (!g_ready)
    {
      return;
    }

  if (channel < NANO_DMA_CHANNELS)
    {
      flags = up_irq_save();
      g_taken &= ~(1u << channel);
      up_irq_restore(flags);
    }
}

bool nano_dma_is_ready(void)
{
  return g_ready;
}

int nano_dma_initialize(void)
{
  if (g_ready)
    {
      return 0;
    }

  /* reset_line_toggle_dt(): reset_block_num(), then
   * unreset_block_num_wait_blocking().
   */

  setbits_reg32(RP2040_RESETS_RESET_DMA, RP2040_RESETS_RESET);
  clrbits_reg32(RP2040_RESETS_RESET_DMA, RP2040_RESETS_RESET);
  while ((getreg32(RP2040_RESETS_RESET_DONE) & RP2040_RESETS_RESET_DMA) == 0)
    {
    }

  /* irq_configure(): both interrupts of the DMA node, same handler. */

  irq_attach(RP2040_DMA_IRQ_0, dma_rpi_pico_isr, NULL);
  up_enable_irq(RP2040_DMA_IRQ_0);
  irq_attach(RP2040_DMA_IRQ_1, dma_rpi_pico_isr, NULL);
  up_enable_irq(RP2040_DMA_IRQ_1);

  g_ready = true;
  return 0;
}
