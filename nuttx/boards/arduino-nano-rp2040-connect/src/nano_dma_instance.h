/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_dma_instance.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com>
 *
 ****************************************************************************/

/* The NuttX part of the DMA core (common/drivers/i2c_dw/nano_dma.c, which
 * includes this file at its end): the reset of the DMA block (RESETS) and
 * the two interrupt lines, attached with irq_attach() to one handler and
 * enabled.  NuttX's own DMA layer (arch/arm/src/rp2040/rp2040_dmac.c,
 * CONFIG_RP2040_DMAC) must be off: it takes DMA_IRQ_0 for all channels,
 * allocates channels with a wait, and stops a channel with an unbounded
 * wait.  Nothing else in this port uses the DMA (the NINA's SPI runs
 * without DMA, as in the Zephyr build).
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_DMA_INSTANCE_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_DMA_INSTANCE_H

static int dma_rpi_pico_isr(int irq, void *context, void *arg)
{
  (void)irq;
  (void)context;
  (void)arg;

  dma_rpi_pico_isr_handler();
  return 0;
}

static void nano_dma_os_reset(void)
{
  setbits_reg32(RP2040_RESETS_RESET_DMA, RP2040_RESETS_RESET);
  clrbits_reg32(RP2040_RESETS_RESET_DMA, RP2040_RESETS_RESET);
  while ((getreg32(RP2040_RESETS_RESET_DONE) & RP2040_RESETS_RESET_DMA) == 0)
    {
    }
}

static void nano_dma_os_irq_enable(void)
{
  irq_attach(RP2040_DMA_IRQ_0, dma_rpi_pico_isr, NULL);
  up_enable_irq(RP2040_DMA_IRQ_0);
  irq_attach(RP2040_DMA_IRQ_1, dma_rpi_pico_isr, NULL);
  up_enable_irq(RP2040_DMA_IRQ_1);
}

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_DMA_INSTANCE_H */
