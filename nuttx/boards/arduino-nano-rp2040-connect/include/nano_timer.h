/****************************************************************************
 * boards/arduino-nano-rp2040-connect/include/nano_timer.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Registers of the RP2040 TIMER block (RP2040 datasheet 4.6.5).  NuttX
 * 13.1.0 has no hardware/rp2040_timer.h; the base address is
 * RP2040_TIMER_BASE (hardware/rp2040_memorymap.h) and the interrupts are
 * RP2040_TIMER_IRQ_0..3 (arch/rp2040/irq.h).  The offsets and bits are those
 * of pico-sdk src/rp2040/hardware_regs/include/hardware/regs/timer.h.  They
 * differ from the RP2350 (rp23xx/hardware/rp23xx_timer.h), which has LOCKED
 * and SOURCE at 0x34/0x38 and the interrupt registers 8 bytes further on.
 *
 * The counter is 64 bits, counts microseconds (the watchdog tick, set to
 * 1 MHz from the 12 MHz crystal in rp2040_clock.c) and starts at 0 when the
 * block leaves reset at boot.  Each ALARMn matches only the low 32 bits.
 *
 * Reading: TIMERAWH/TIMERAWL have no side effect, so they may be read from
 * any context (read high, low, high again until the two highs agree).
 * TIMELR/TIMEHR latch the high word when the low word is read and must not
 * be used where two readers can interleave.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_TIMER_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_TIMER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NANO_TIMER_BASE             0x40054000  /* = RP2040_TIMER_BASE */

#define NANO_TIMER_TIMEHW_OFFSET    0x00  /* Write high word (after TIMELW) */
#define NANO_TIMER_TIMELW_OFFSET    0x04  /* Write low word */
#define NANO_TIMER_TIMEHR_OFFSET    0x08  /* High word latched by TIMELR */
#define NANO_TIMER_TIMELR_OFFSET    0x0c  /* Low word (latches TIMEHR) */
#define NANO_TIMER_ALARM0_OFFSET    0x10  /* Writing arms the alarm */
#define NANO_TIMER_ALARM1_OFFSET    0x14
#define NANO_TIMER_ALARM2_OFFSET    0x18
#define NANO_TIMER_ALARM3_OFFSET    0x1c
#define NANO_TIMER_ARMED_OFFSET     0x20  /* Armed alarms; write 1 to disarm */
#define NANO_TIMER_TIMERAWH_OFFSET  0x24  /* High word, no side effect */
#define NANO_TIMER_TIMERAWL_OFFSET  0x28  /* Low word, no side effect */
#define NANO_TIMER_DBGPAUSE_OFFSET  0x2c
#define NANO_TIMER_PAUSE_OFFSET     0x30
#define NANO_TIMER_INTR_OFFSET      0x34  /* Raw interrupts; write 1 to clear */
#define NANO_TIMER_INTE_OFFSET      0x38  /* Interrupt enable */
#define NANO_TIMER_INTF_OFFSET      0x3c  /* Interrupt force */
#define NANO_TIMER_INTS_OFFSET      0x40  /* (INTR | INTF) & INTE */

/* Alarm n is bit n in ARMED, INTR, INTE, INTF and INTS. */

#define NANO_TIMER_ALARM(n)         (1u << (n))

#define NANO_TIMER_TIMERAWH  (NANO_TIMER_BASE + NANO_TIMER_TIMERAWH_OFFSET)
#define NANO_TIMER_TIMERAWL  (NANO_TIMER_BASE + NANO_TIMER_TIMERAWL_OFFSET)

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_INCLUDE_NANO_TIMER_H */
