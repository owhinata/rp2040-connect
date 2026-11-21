/* SPDX-License-Identifier: MIT */
/* <nuttx/debug.h> for the host test of rp2040_dmac.c: no log, the
 * assertions on.
 */
#ifndef TEST_RP2040_DMAC_NUTTX_DEBUG_H
#define TEST_RP2040_DMAC_NUTTX_DEBUG_H
#include <assert.h>
#define dmainfo(...)   ((void)0)
#define dmaerr(...)    ((void)0)
#define DEBUGASSERT(x) assert(x)
#endif
