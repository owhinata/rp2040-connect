/* SPDX-License-Identifier: Apache-2.0 */
/* <nuttx/irq.h> (and sched_lock() of <sched.h>) for the host test of
 * zephyr_mutex.c: one thread, no interrupts; the test sees the nesting. */
#ifndef TEST_ZMUTEX_NUTTX_IRQ_H
#define TEST_ZMUTEX_NUTTX_IRQ_H
typedef unsigned int irqstate_t;
extern int fake_irq_saved, fake_sched_locked;
static inline irqstate_t up_irq_save(void)
{
	return (irqstate_t)fake_irq_saved++;
}
static inline void up_irq_restore(irqstate_t flags)
{
	fake_irq_saved = (int)flags;
}
static inline int sched_lock(void)
{
	fake_sched_locked++;
	return 0;
}
static inline int sched_unlock(void)
{
	fake_sched_locked--;
	return 0;
}
#endif
