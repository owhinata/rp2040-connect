/* SPDX-License-Identifier: Apache-2.0 */
/* The thread that turns the NINA sockets' state into usrsock events.
 *
 * The Zephyr build waits in the calling thread: connect() polls the module
 * every NINA_POLL_INTERVAL_MS until NINA_CONNECT_WAIT_MS
 * (drivers/nina/nina_socket.c do_connect()), and poll() polls each socket
 * every NINA_POLL_INTERVAL_MS (do_poll()).  NuttX's usrsock instead waits
 * for events from the driver.  This thread makes the same polls, every
 * NINA_POLL_INTERVAL_MS while a socket has something to report:
 *
 * - a connection in progress: one pass of the Zephyr build's connect loop
 *   (nina_sock_connect_check()), then the answer to the connect request;
 * - otherwise the module's poll state, turned into RECVFROM_AVAIL (a
 *   listening socket: a connection to accept), SENDTO_READY and
 *   REMOTE_CLOSED (nina_sock_events()), for what usrsock does not hold
 *   yet.  A socket whose events are all held is not polled; with nothing
 *   to poll the thread sleeps until a request wakes it.
 *
 * Locking: the usrsock lock, then the NINA lock (inside nina_request()),
 * as every request (nina_sock.h).  The lock is taken for one socket at a
 * time, so a request waits for at most one NINA command of this thread, as
 * a Zephyr socket call waits for at most one command of another thread's
 * poll at nina_lock.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/kthread.h>
#include <nuttx/net/usrsock.h>
#include <nuttx/semaphore.h>

#include "nina_os.h"
#include "nina_sock.h"
#include "nina_usrsock.h"
#include "priorities.h"

static sem_t g_wake = SEM_INITIALIZER(0);
static bool g_started;

/* One socket: returns true if it was polled. */

static bool poll_socket(int usockid)
{
  struct nina_sock *s = nina_sock_get(usockid);
  uint8_t want;
  uint8_t state = 0;
  int ret;

  if (s == NULL || (want = nina_sock_watch(s)) == 0)
    {
      return false;
    }

  if (want == NINA_SOCK_WATCH_CONNECT)
    {
      ret = nina_sock_connect_check(usockid, nina_os_uptime_ms());
      if (ret <= 0)
        {
          nina_usrsock_connected(usockid, ret);
        }

      return true;
    }

  ret = nina_sock_poll_state(s, &state);
  want = nina_sock_events(s, ret, state);
  if (want != 0)
    {
      nina_usrsock_report(usockid, want);
    }

  return true;
}

static int nina_poller(int argc, FAR char *argv[])
{
  for (; ; )
    {
      bool polled = false;
      int i;

      for (i = 0; i < NINA_SOCK_MAX; i++)
        {
          usrsock_lock();
          polled |= poll_socket(i);
          usrsock_unlock();
        }

      /* k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS)) of the Zephyr build's loops:
       * the interval in ticks, plus one (wd_start()).  A request that gives
       * the thread something new to look at ends the wait early.
       */

      if (polled)
        {
          nxsem_tickwait_uninterruptible(&g_wake,
                                         MSEC2TICK(NINA_POLL_INTERVAL_MS));
        }
      else
        {
          nxsem_wait_uninterruptible(&g_wake);
        }
    }

  return 0;
}

int nina_poller_start(void)
{
  int ret;

  if (g_started)
    {
      return 0;
    }

  ret = kthread_create("nina_event", NANO_PRIO_NINA_EVENT,
                       NANO_STACK_NINA_EVENT, nina_poller, NULL);
  if (ret < 0)
    {
      return ret;
    }

  g_started = true;
  return 0;
}

void nina_poller_wake(void)
{
  int value;

  if (nxsem_get_value(&g_wake, &value) == 0 && value <= 0)
    {
      nxsem_post(&g_wake);
    }
}
