/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_dma.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/* The RP2040 DMA as the Zephyr build drives it for the I2C0 DMA reads
 * (nano_i2c.c): the parts of Zephyr's DMA API (zephyr/include/zephyr/
 * drivers/dma.h: struct dma_config, struct dma_block_config,
 * dma_request_channel(), dma_release_channel(), dma_config(), dma_start(),
 * dma_stop()) and of its RP2040 driver (zephyr/drivers/dma/dma_rpi_pico.c)
 * that i2c_dw.c uses, and the pico-sdk helpers it calls directly
 * (hardware/dma.h: dma_channel_is_busy(), dma_irqn_acknowledge_channel(),
 * dma_hw->abort).  See nano_dma.c.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_DMA_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_DMA_H

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"
#include "hardware/rp2040_dma.h"

/* zephyr/include/zephyr/drivers/dma.h: enum dma_channel_direction,
 * enum dma_addr_adj.
 */

#define NANO_DMA_MEMORY_TO_MEMORY     0x0
#define NANO_DMA_MEMORY_TO_PERIPHERAL 0x1
#define NANO_DMA_PERIPHERAL_TO_MEMORY 0x2

#define NANO_DMA_ADDR_ADJ_INCREMENT 0
#define NANO_DMA_ADDR_ADJ_DECREMENT 1
#define NANO_DMA_ADDR_ADJ_NO_CHANGE 2

/* zephyr/include/zephyr/dt-bindings/dma/rpi-pico-dma-common.h and
 * rpi-pico-dma-rp2040.h: a devicetree "slot" is the inverted DREQ number.
 */

#define NANO_DMA_SLOT_TO_DREQ(s)  (~(s) & 0x3F)
#define NANO_DMA_DREQ_TO_SLOT     NANO_DMA_SLOT_TO_DREQ
#define NANO_DMA_SLOT_I2C0_TX     NANO_DMA_DREQ_TO_SLOT(0x20)
#define NANO_DMA_SLOT_I2C0_RX     NANO_DMA_DREQ_TO_SLOT(0x21)

/* The RP2040 has 12 channels (dma-channels = <12> in rp2040.dtsi). */

#define NANO_DMA_CHANNELS 12

/* dma_callback_t: status 0 when the block completed, -EIO with a bus
 * error (dma_rpi_pico_isr()).  Called from the DMA interrupt.
 */

typedef void (*nano_dma_callback_t)(void *user_data, uint32_t channel,
                                    int status);

struct nano_dma_block_config
{
  uintptr_t source_address;
  uintptr_t dest_address;
  uint32_t block_size;          /* bytes */
  uint16_t source_addr_adj;
  uint16_t dest_addr_adj;
};

struct nano_dma_config
{
  uint32_t dma_slot;
  uint32_t channel_direction;
  uint32_t channel_priority;
  uint32_t source_data_size;    /* 1, 2 or 4 bytes */
  uint32_t dest_data_size;
  uint32_t block_count;
  struct nano_dma_block_config *head_block;
  nano_dma_callback_t dma_callback;
  void *user_data;
};

/* dma_rpi_pico_init(): reset the DMA block and attach DMA_IRQ_0 and
 * DMA_IRQ_1.  From the board bringup, before I2C0.
 */

int nano_dma_initialize(void);

/* device_is_ready(dma) */

bool nano_dma_is_ready(void);

/* dma_request_channel(dma, NULL): the lowest free channel, or -EINVAL. */

int nano_dma_request_channel(void);

/* dma_release_channel(): marks the channel free (the RP2040 driver has
 * no chan_release; the hardware is left as it is).
 */

void nano_dma_release_channel(uint32_t channel);

/* dma_config(), dma_start(), dma_stop() of dma_rpi_pico.c. */

int nano_dma_config(uint32_t channel, const struct nano_dma_config *cfg);
int nano_dma_start(uint32_t channel);
int nano_dma_stop(uint32_t channel);

/* pico-sdk hardware/dma.h */

static inline bool nano_dma_channel_is_busy(uint32_t channel)
{
  /* dma_channel_is_busy(): BUSY of the channel's AL1_CTRL. */

  return (getreg32(RP2040_DMA_CH(channel) + RP2040_DMA_AL1_CTRL_OFFSET) &
          RP2040_DMA_CTRL_TRIG_BUSY) != 0;
}

static inline void nano_dma_irqn_acknowledge_channel(uint32_t irq_index,
                                                     uint32_t channel)
{
  /* dma_irqn_acknowledge_channel(): write 1 to the channel's bit of
   * INTS0 or INTS1 (which clears INTR).
   */

  putreg32(1u << channel, irq_index ? RP2040_DMA_INTS1 : RP2040_DMA_INTS0);
}

static inline uint32_t nano_dma_abort_pending(void)
{
  /* dma_hw->abort: the channels whose abort has not finished. */

  return getreg32(RP2040_DMA_CHAN_ABORT);
}

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_DMA_H */
