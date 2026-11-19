/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the DMA core (common/drivers/i2c_dw/dma_rpi_pico.c,
 * which includes this file at its end), on the Pico SDK, as the Zephyr
 * build's dma_rpi_pico_init() does it:
 *
 *   Zephyr                                here
 *   reset_line_toggle_dt() (Zephyr's      reset_block_num(RESET_DMA), then
 *   reset_rpi_pico.c, on the same SDK     unreset_block_num_wait_blocking()
 *   calls)
 *   irq_configure(): IRQ_CONNECT() and    irq_set_exclusive_handler() and
 *   irq_enable() of DMA_IRQ_0 (11) and    irq_set_enabled() of DMA_IRQ_0 and
 *   DMA_IRQ_1 (12), one handler           DMA_IRQ_1, one handler
 *
 * - The DMA is the core's alone (dma_rpi_pico.c allocates its channels without
 *   knowing other users).  The Pico SDK has its own record of the channels
 *   (dma_claim_unused_channel()); nothing in this build uses it (the USB
 *   stdio, TinyUSB, the flash and the FreeRTOS port use no DMA), and the
 *   reset here claims all 12 channels in it, so that a later user of the
 *   SDK's allocation fails at once (dma_claim_unused_channel() returns -1,
 *   or panics when required; dma_claim_mask() itself panics if a channel was
 *   already claimed) rather than sharing a channel.  irq_set_exclusive_handler()
 *   likewise panics if a handler is already installed on either line
 *   (hard_assert(), on in this Release build).
 * - The priority of both lines is left at the Pico SDK's default for every
 *   peripheral interrupt, PICO_DEFAULT_IRQ_PRIORITY (0x80, set by the SDK's
 *   start-up code): the same as I2C0's and the USB's, above the FreeRTOS
 *   SysTick and PendSV (0xff, the lowest).  The Zephyr build runs these
 *   lines, I2C0 and its other devices at one priority (3 of 0-3) too; the
 *   NuttX build all at 0x80.  The handlers may call the FromISR functions
 *   at any priority: the RP2040 port of FreeRTOS masks every interrupt in a
 *   critical section (PRIMASK; the Cortex-M0+ has no BASEPRI, and
 *   configMAX_SYSCALL_INTERRUPT_PRIORITY does not apply).
 * - irq_set_enabled() clears a pending interrupt of the line before it
 *   enables it (Zephyr's irq_enable() does not); both lines are level
 *   interrupts of the DMA, which the reset has just cleared, so nothing is
 *   lost.
 */

#ifndef FREERTOS_DRIVERS_I2C_DW_DMA_RPI_PICO_INSTANCE_H
#define FREERTOS_DRIVERS_I2C_DW_DMA_RPI_PICO_INSTANCE_H

#include "hardware/irq.h"
#include "hardware/resets.h"

/* dma_claim_mask() of all the channels (dma_rpi_pico_os.c: the SDK's
 * hardware/dma.h cannot follow dma_rpi_pico.h, which writes out its types
 * and functions).
 */
void dma_rpi_pico_os_claim(void);

/* The handler of DMA_IRQ_0 and DMA_IRQ_1 (Zephyr's dma_rpi_pico_isr()). */
static void dma_rpi_pico_isr(void)
{
	dma_rpi_pico_isr_handler();
}

static void dma_rpi_pico_os_reset(void)
{
	dma_rpi_pico_os_claim();
	reset_block_num(RESET_DMA);
	unreset_block_num_wait_blocking(RESET_DMA);
}

static void dma_rpi_pico_os_irq_enable(void)
{
	irq_set_exclusive_handler(DMA_IRQ_0, dma_rpi_pico_isr);
	irq_set_enabled(DMA_IRQ_0, true);
	irq_set_exclusive_handler(DMA_IRQ_1, dma_rpi_pico_isr);
	irq_set_enabled(DMA_IRQ_1, true);
}

#endif /* FREERTOS_DRIVERS_I2C_DW_DMA_RPI_PICO_INSTANCE_H */
