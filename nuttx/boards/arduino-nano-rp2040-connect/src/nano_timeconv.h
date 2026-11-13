/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_timeconv.h
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

/* Conversions between the RP2040 TIMER count (1 MHz, 64 bits) and the OS
 * clock (100 us ticks, timespec), used by the board's replacements of the
 * weak functions of drivers/timers/arch_alarm.c (nano_oneshot.c).
 *
 * The Cortex-M0+ has neither a divide instruction nor a 32x32->64 multiply,
 * so the generic ONESHOT_COUNT conversions (include/nuttx/timers/oneshot.h)
 * end up in libgcc's 64-bit division (__aeabi_uldivmod), several
 * microseconds per call.  Here the division by 100 is done one byte at a
 * time: each step divides a number below 100 * 256 by 100 with one 32-bit
 * multiply and a shift, which is exact for every such number (see
 * nano_div100_digit()).  Division by 10^6 is three divisions by 100.
 *
 * The functions give the same results as the generic code for every 64-bit
 * count and tick (nuttx/tests/nano_timeconv.c checks this on the host).
 * They depend on nothing but <stdint.h>.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_TIMECONV_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_TIMECONV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* One OS tick is 100 TIMER counts (CONFIG_USEC_PER_TICK, which
 * nano_oneshot.c requires to be 100).
 */

/* floor(n / 100) == (n * NANO_DIV100_MUL) >> NANO_DIV100_SHIFT for every
 * n < 43690 (2^22 / 96, 96 being 100 * 41944 - 2^22; the first n that
 * fails is 43699).  The steps below use n < 100 * 256 = 25600, and
 * 25599 * 41944 < 2^32.
 */

#define NANO_DIV100_MUL      41944u
#define NANO_DIV100_SHIFT    22

/* GCC at -Os would otherwise call the per-byte step out of line. */

#ifdef __GNUC__
#  define NANO_TIMECONV_INLINE  static inline __attribute__((always_inline))
#else
#  define NANO_TIMECONV_INLINE  static inline
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_div100_digit
 *
 * Description:
 *   One step of long division by 100 in base 256: append the byte to the
 *   running remainder *n (< 100), divide, and leave the new remainder in
 *   *n.  Returns the quotient digit (< 256, since the number is below
 *   100 * 256).
 *
 ****************************************************************************/

NANO_TIMECONV_INLINE uint32_t nano_div100_digit(uint32_t *n, uint32_t byte)
{
  uint32_t d;

  *n  = (*n << 8) | byte;
  d   = (*n * NANO_DIV100_MUL) >> NANO_DIV100_SHIFT;
  *n -= d * 100u;
  return d;
}

/****************************************************************************
 * Name: nano_div100_step
 *
 * Description:
 *   Divide (*rem * 2^32 + x) by 100, given *rem < 100.  Returns the
 *   quotient (it fits 32 bits since *rem < 100) and leaves the remainder
 *   in *rem.  Written out byte by byte so that GCC does not keep a loop
 *   with a variable shift.
 *
 ****************************************************************************/

NANO_TIMECONV_INLINE uint32_t nano_div100_step(uint32_t x, uint32_t *rem)
{
  uint32_t n = *rem;
  uint32_t q;

  q  = nano_div100_digit(&n, x >> 24) << 24;
  q |= nano_div100_digit(&n, (x >> 16) & 0xffu) << 16;
  q |= nano_div100_digit(&n, (x >> 8) & 0xffu) << 8;
  q |= nano_div100_digit(&n, x & 0xffu);
  *rem = n;
  return q;
}

/****************************************************************************
 * Name: nano_div100
 *
 * Description:
 *   floor(x / 100) for any 64-bit x; the remainder goes to *rem.  The high
 *   word of a TIMER count stays below 256 for the first 12 days, so it
 *   usually takes one step.
 *
 ****************************************************************************/

static inline uint64_t nano_div100(uint64_t x, uint32_t *rem)
{
  uint32_t hi = (uint32_t)(x >> 32);
  uint32_t r  = 0;
  uint32_t qh = 0;
  uint32_t ql;

  if (hi >= 256)
    {
      qh = nano_div100_step(hi, &r);
    }
  else if (hi != 0)
    {
      qh = nano_div100_digit(&r, hi);
    }

  ql   = nano_div100_step((uint32_t)x, &r);
  *rem = r;
  return ((uint64_t)qh << 32) | ql;
}

/****************************************************************************
 * Name: nano_count_to_tick
 *
 * Description:
 *   The OS tick that contains the TIMER count: floor(count / 100).  The
 *   generic oneshot_tick_current() gives the same value (whole seconds
 *   times 10^4 plus the rest of the count, as nanoseconds, over 10^5).
 *
 ****************************************************************************/

static inline uint64_t nano_count_to_tick(uint64_t count)
{
  uint32_t rem;

  return nano_div100(count, &rem);
}

/****************************************************************************
 * Name: nano_tick_to_count
 *
 * Description:
 *   The TIMER count at which the tick starts: tick * 100, modulo 2^64 like
 *   the generic clkcnt_tick2cnt().
 *
 ****************************************************************************/

static inline uint64_t nano_tick_to_count(uint64_t tick)
{
  /* tick * 100 with shifts: libgcc's 64-bit multiply is a call. */

  return (tick << 6) + (tick << 5) + (tick << 2);
}

/****************************************************************************
 * Name: nano_count_to_time
 *
 * Description:
 *   Split the TIMER count into whole seconds and nanoseconds, as the
 *   generic oneshot_current(): *sec = count / 10^6 and
 *   *nsec = (count % 10^6) * 1000.
 *
 ****************************************************************************/

static inline void nano_count_to_time(uint64_t count, uint64_t *sec,
                                      uint32_t *nsec)
{
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint64_t q;

  /* count = ((q * 100 + r2) * 100 + r1) * 100 + r0 */

  q    = nano_div100(count, &r0);
  q    = nano_div100(q, &r1);
  q    = nano_div100(q, &r2);
  *sec  = q;
  *nsec = ((r2 * 100u + r1) * 100u + r0) * 1000u;
}

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_TIMECONV_H */
