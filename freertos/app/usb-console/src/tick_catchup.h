/* SPDX-License-Identifier: MIT */
/* The kernel tick's catch-up: keeps the tick count up with the TIMER when
 * tick interrupts were lost (tick.c).
 *
 * The tick is SysTick's periodic interrupt (1 ms).  While interrupts stay
 * masked for longer than a tick (a flash erase or program in
 * flash_safe_execute(), about 40 to 50 ms per erase), SysTick's pending bit
 * holds one interrupt only, so every further tick of that time is lost: the
 * count falls behind the time, and every wait counted in ticks (sleeps,
 * timeouts, delayable work, time slices) lasts that much longer.  The Zephyr
 * and NuttX builds are tickless (their kernel time is a hardware counter),
 * so they do not lose time this way.
 *
 * tick_catchup_step() is called once for every tick the kernel counts (from
 * the tick hook, also for the ticks it adds itself) with the TIMER's low 32
 * bits, and says whether the count is a whole tick behind the TIMER.  The
 * hook then pends SysTick again (ICSR.PENDSTSET), so the port's handler runs
 * once more as soon as this run returns and counts the next tick the usual
 * way: xTaskIncrementTick() wakes the tasks whose time has come, pends the
 * switch to them, counts a tick pended while the scheduler is suspended, and
 * calls the tick hook, so the compatibility layer's timeouts
 * (zephyr_compat_tick()) and the time slices see every tick too.  The ticks
 * added this way run one after another before any task runs again (SysTick
 * and PendSV share the lowest priority, so they take turns ahead of thread
 * mode), as the lost interrupts would have run.
 *
 * Never ahead of the time.  Let b(n) be the time of SysTick's n-th period end
 * (b(n + 1) = b(n) + TICK_CATCHUP_PERIOD_US exactly: SysTick counts clk_sys,
 * 125 MHz, and the TIMER 1 MHz, both from the same 12 MHz crystal, and
 * main.c checks that a tick is a whole number of clk_sys cycles).  `due` is
 * the earliest TIMER read seen for the current count c, which the interrupt
 * of c made after b(c), so due > b(c) - 1 us (the TIMER reads whole
 * microseconds, rounded down): it starts as the read of the first tick, goes
 * on by one period per tick, and is moved back whenever a tick's read comes
 * earlier (a run with less latency).  A tick is added only when now >= due +
 * period + 1 us, so after b(c + 1): the (c + 1)-th period end has passed, its
 * interrupt is pending (it merged into the one being served, or came during
 * this run) and PENDSTSET merges with it; the interrupt that counts c + 1 is
 * served once.  So the count never exceeds SysTick's periods since the start,
 * and a wait of n ticks never ends sooner than in the time of n ticks.

 * Bounded behind.  After a run of masked interrupts, the first tick
 * interrupt adds every lost tick at once; a lost interrupt whose end the
 * read just missed is added at the next tick.  So the count is at most one
 * tick behind the TIMER (counted from the first tick), apart from the time
 * interrupts are masked.  A read made late (latency) can only add a tick
 * sooner, and `due` follows the earliest reads, so it does not drift later.
 *
 * The cost is one TIMER read, an add and two compares per tick (no
 * division).  The time is 32 bits of microseconds and compared as signed
 * differences: right while the hook runs at least every 35 minutes.
 */

#ifndef TICK_CATCHUP_H
#define TICK_CATCHUP_H

#include <stdbool.h>
#include <stdint.h>

/* configTICK_RATE_HZ: include FreeRTOS.h first. */
#define TICK_CATCHUP_PERIOD_US (1000000u / configTICK_RATE_HZ)

#if TICK_CATCHUP_PERIOD_US * configTICK_RATE_HZ != 1000000u
#error "the tick is not a whole number of microseconds"
#endif

struct tick_catchup {
	uint32_t due; /* the earliest TIMER time seen for the current count */
	bool started; /* due is set (from the first tick) */
};

/* For each tick the kernel counts, with now the TIMER's low 32 bits read in
 * the tick interrupt: true if the count is a whole tick behind, so one more
 * tick is to be counted now (and step is called again for that one).
 */
static inline bool tick_catchup_step(struct tick_catchup *s, uint32_t now)
{
	if (!s->started) {
		s->due = now;
		s->started = true;
		return false;
	}
	s->due += TICK_CATCHUP_PERIOD_US;
	if ((int32_t)(now - s->due) < 0) {
		/* This tick's read came earlier: less latency than before. */
		s->due = now;
	}
	/* Past the next period end: more than a period and the rounding. */
	return (int32_t)(now - s->due) > (int32_t)TICK_CATCHUP_PERIOD_US;
}

#endif /* TICK_CATCHUP_H */
