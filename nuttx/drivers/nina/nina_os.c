/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services of nina_spi.c on NuttX (see nina_os.h).
 *
 * Zephyr build (drivers/nina/nina_wifi.c):
 * - K_MUTEX_DEFINE(nina_lock): recursive, priority inheritance.  Here an
 *   rmutex_t, whose mutex inherits priority with
 *   CONFIG_PRIORITY_INHERITANCE.  The wait is repeated when a signal ends
 *   it (-EINTR), so that it waits for ever as K_FOREVER does.
 * - k_sleep(K_USEC(100)), k_sleep(K_MSEC(10)), k_sleep(K_MSEC(750)): Zephyr
 *   rounds a relative timeout up to ticks and adds one tick
 *   (kernel/timeout.c, z_add_timeout()).  nxsched_usleep() and
 *   nxsched_msleep() round up (USEC2TICK, MSEC2TICK) and wake at
 *   clock_systime_ticks() + ticks + 1 (clock_delay2abstick()): the same
 *   with the same 100 us tick.  They are not ended by signals; neither is
 *   k_sleep() (only k_wakeup(), which this driver does not use).
 * - K_MUTEX_DEFINE(nina_dns_lock) of nina_dns.c: a mutex_t, which also
 *   inherits priority.
 * - k_uptime_get(): ticks converted to milliseconds, rounded down
 *   (k_ticks_to_ms_floor64()).  TICK2MSEC() rounds down too.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>

#include "nina_os.h"

static rmutex_t g_nina_lock = NXRMUTEX_INITIALIZER;
static mutex_t g_dns_lock = NXMUTEX_INITIALIZER;

int nina_os_lock(void)
{
  int ret;

  do
    {
      ret = nxrmutex_lock(&g_nina_lock);
    }
  while (ret == -EINTR);

  return ret;
}

void nina_os_unlock(void)
{
  nxrmutex_unlock(&g_nina_lock);
}

void nina_os_dns_lock(void)
{
  while (nxmutex_lock(&g_dns_lock) == -EINTR)
    {
    }
}

void nina_os_dns_unlock(void)
{
  nxmutex_unlock(&g_dns_lock);
}

void nina_os_sleep_us(uint32_t us)
{
  nxsched_usleep(us);
}

void nina_os_sleep_ms(uint32_t ms)
{
  nxsched_msleep(ms);
}

int64_t nina_os_uptime_ms(void)
{
  return (int64_t)TICK2MSEC(clock_systime_ticks());
}
