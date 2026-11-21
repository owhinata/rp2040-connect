/* SPDX-License-Identifier: MIT */
/* <nuttx/irq.h> for the host test of i2c_dw_dma_os_rp2040.c
 * (tests/i2c_dw_dma_os.c): one thread; the test checks that the interrupts
 * are back on after each call.
 */
#ifndef TEST_I2C_DW_DMA_NUTTX_IRQ_H
#define TEST_I2C_DW_DMA_NUTTX_IRQ_H
typedef unsigned int irqstate_t;
extern int fake_irq_saved;
static inline irqstate_t up_irq_save(void)
{
	return (irqstate_t)fake_irq_saved++;
}
static inline void up_irq_restore(irqstate_t flags)
{
	fake_irq_saved = (int)flags;
}
#endif
