/* SPDX-License-Identifier: MIT */
/* <nuttx/irq.h> for the host test of rp2040_dmac.c: the handler it attaches
 * is kept by the test.
 */
#ifndef TEST_RP2040_DMAC_NUTTX_IRQ_H
#define TEST_RP2040_DMAC_NUTTX_IRQ_H
#define RP2040_DMA_IRQ_0 27
typedef int (*xcpt_t)(int irq, void *context, void *arg);
int irq_attach(int irq, xcpt_t isr, void *arg);
void up_enable_irq(int irq);
#endif
