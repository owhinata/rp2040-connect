/* SPDX-License-Identifier: MIT */
/* The part of NuttX's <nuttx/irq.h> that nina_os.c uses, for the host test
 * tests/nina_os.c (which defines the functions).
 */

#ifndef TEST_NINA_OS_NUTTX_IRQ_H
#define TEST_NINA_OS_NUTTX_IRQ_H

typedef unsigned int irqstate_t;

irqstate_t enter_critical_section(void);
void leave_critical_section(irqstate_t flags);

#endif /* TEST_NINA_OS_NUTTX_IRQ_H */
