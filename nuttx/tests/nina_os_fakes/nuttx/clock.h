/* SPDX-License-Identifier: MIT */
/* The parts of NuttX's <nuttx/clock.h> that common/drivers/nina/nuttx/nina_os.c uses,
 * for tests/nina_os.c: the tick of the board (CONFIG_USEC_PER_TICK=100)
 * with NuttX's rounding (TICK2MSEC() down; MSEC2TICK() and USEC2TICK() up)
 * and the test's clock.
 */

#ifndef TEST_NINA_OS_NUTTX_CLOCK_H
#define TEST_NINA_OS_NUTTX_CLOCK_H

#include <stdint.h>

#define TICK2MSEC(t)    ((t) / 10)
#define USEC2TICK(us)   (((us) + 99) / 100)
#define MSEC2TICK(ms)   USEC2TICK((ms) * 1000)

uint64_t clock_systime_ticks(void);

#endif /* TEST_NINA_OS_NUTTX_CLOCK_H */
