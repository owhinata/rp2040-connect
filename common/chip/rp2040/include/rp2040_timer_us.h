/* SPDX-License-Identifier: Apache-2.0 */
/* The RP2040's TIMER block (RP2040 datasheet 4.6.5): its registers and the
 * read of its 64-bit counter, rp2040_timer_us(), for every build (Zephyr,
 * NuttX, FreeRTOS).  The registers have no side effect when read and no OS
 * owns the counter, so this header needs nothing from an OS or SDK.
 *
 * The offsets and bits are those of the Pico SDK's
 * src/rp2040/hardware_regs/include/hardware/regs/timer.h.  They differ from
 * the RP2350's, which has LOCKED and SOURCE at 0x34/0x38 and the interrupt
 * registers 8 bytes further on.  NuttX's hardware/rp2040_memorymap.h
 * defines RP2040_TIMER_BASE with the same tokens, so either may come first.
 *
 * The counter is 64 bits, counts microseconds (the watchdog's tick, 1 MHz
 * from the 12 MHz crystal) and starts at 0 when the block leaves reset at
 * boot (every boot, including a restart through the watchdog).  Each ALARMn
 * matches only the low 32 bits.
 *
 * Reading: TIMERAWH and TIMERAWL have no side effect, so they may be read
 * from any context.  TIMELR and TIMEHR latch the high word when the low word
 * is read; the latch is one for all readers, so they are not used where two
 * readers can interleave (a thread and an interrupt handler, the two cores).
 */

#ifndef COMMON_CHIP_RP2040_INCLUDE_RP2040_TIMER_US_H
#define COMMON_CHIP_RP2040_INCLUDE_RP2040_TIMER_US_H

#define RP2040_TIMER_BASE 0x40054000

#define RP2040_TIMER_TIMEHW_OFFSET   0x00 /* High word, after TIMELW */
#define RP2040_TIMER_TIMELW_OFFSET   0x04 /* Low word */
#define RP2040_TIMER_TIMEHR_OFFSET   0x08 /* High word latched by TIMELR */
#define RP2040_TIMER_TIMELR_OFFSET   0x0c /* Low word (latches TIMEHR) */
#define RP2040_TIMER_ALARM0_OFFSET   0x10 /* Writing arms the alarm */
#define RP2040_TIMER_ALARM1_OFFSET   0x14
#define RP2040_TIMER_ALARM2_OFFSET   0x18
#define RP2040_TIMER_ALARM3_OFFSET   0x1c
#define RP2040_TIMER_ARMED_OFFSET    0x20 /* Armed alarms; 1 disarms */
#define RP2040_TIMER_TIMERAWH_OFFSET 0x24 /* High word, no side effect */
#define RP2040_TIMER_TIMERAWL_OFFSET 0x28 /* Low word, no side effect */
#define RP2040_TIMER_DBGPAUSE_OFFSET 0x2c
#define RP2040_TIMER_PAUSE_OFFSET    0x30
#define RP2040_TIMER_INTR_OFFSET     0x34 /* Raw interrupts; 1 clears */
#define RP2040_TIMER_INTE_OFFSET     0x38 /* Interrupt enable */
#define RP2040_TIMER_INTF_OFFSET     0x3c /* Interrupt force */
#define RP2040_TIMER_INTS_OFFSET     0x40 /* (INTR | INTF) & INTE */

/* Alarm n is bit n in ARMED, INTR, INTE, INTF and INTS. */
#define RP2040_TIMER_ALARM(n) (1u << (n))

#ifndef __ASSEMBLY__

#include <stdint.h>

/* The read of a register at an offset.  A host test defines it before this
 * header to supply the registers (common/tests/rp2040_timer_us.c).
 */
#ifndef RP2040_TIMER_READ
#define RP2040_TIMER_READ(offset) (*(volatile uint32_t *)(RP2040_TIMER_BASE + (offset)))
#endif

/* Microseconds since the TIMER left reset at boot, from the raw registers:
 * the high word, then the low word and the high word again until the two
 * high words agree, so that a carry from the low word between the reads is
 * never mixed in (the low word was read while the high word held the value
 * returned with it).  No side effect, no lock and no state: safe from any
 * context, threads and interrupt handlers, with interrupts enabled or not,
 * and from either core.  Always inline (GCC at -Os would otherwise keep a
 * copy per file and call it): it is on the paths of the OS clocks, the
 * log's time stamps, the idle loop's CPU load and the time stamps of the IMU
 * batches.  The Pico SDK's timer_time_us_64() reads the same way.
 */
static inline __attribute__((always_inline)) uint64_t rp2040_timer_us(void)
{
	uint32_t high;
	uint32_t low;
	uint32_t again = RP2040_TIMER_READ(RP2040_TIMER_TIMERAWH_OFFSET);

	do {
		high = again;
		low = RP2040_TIMER_READ(RP2040_TIMER_TIMERAWL_OFFSET);
		again = RP2040_TIMER_READ(RP2040_TIMER_TIMERAWH_OFFSET);
	} while (again != high);

	return ((uint64_t)high << 32) | low;
}

#endif /* __ASSEMBLY__ */

#endif /* COMMON_CHIP_RP2040_INCLUDE_RP2040_TIMER_US_H */
