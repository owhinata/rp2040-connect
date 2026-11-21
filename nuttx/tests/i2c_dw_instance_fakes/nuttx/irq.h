/* SPDX-License-Identifier: MIT */
/* <nuttx/irq.h> (and the RP2040's interrupt numbers of <arch/irq.h>) for the
 * host test of the I2C core's NuttX binding (tests/i2c_dw_instance.c, which
 * implements irq_attach() and up_enable_irq(): they record the calls).  The
 * interrupt lock is a mutex of the test, so that the test's "hardware" thread
 * and the transfers' threads exclude each other as the one core does.
 */
#ifndef TEST_I2C_DW_NUTTX_IRQ_H
#define TEST_I2C_DW_NUTTX_IRQ_H
#define RP2040_IRQ_EXTINT 16
#define RP2040_I2C0_IRQ   (RP2040_IRQ_EXTINT + 23)
#define RP2040_I2C1_IRQ   (RP2040_IRQ_EXTINT + 24)
typedef unsigned int irqstate_t;
typedef int (*xcpt_t)(int irq, void *context, void *arg);
irqstate_t up_irq_save(void);
void up_irq_restore(irqstate_t flags);
int irq_attach(int irq, xcpt_t isr, void *arg);
void up_enable_irq(int irq);
#endif
