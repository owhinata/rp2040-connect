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
 * A socket marked NINA_SO_CALLER_POLLS (<nina_sockopt.h>: the Zephyr
 * compatibility layer's zsock_*() sockets, MQTT and NTP) is polled here
 * only while it connects: its callers ask the module themselves, as the
 * Zephyr build's recv() and poll() do (nina_sock_watch()).
 *
 * It also times the passes of a TCP send that waits for room: the Zephyr
 * build's do_send() sleeps CONFIG_NINA_SEND_RETRY_MS between its tries.
 * Here a pass that sent nothing ends the request with -EAGAIN; after
 * NINA_SEND_RETRY_MS this thread reports SENDTO_READY, and usrsock sends
 * the request again in the caller's thread (nina_usrsock.c).  These timers
 * do not make the thread poll the other sockets: the polls keep their
 * own NINA_POLL_INTERVAL_MS.
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

/* Under the usrsock lock: the next poll of every socket (when
 * g_poll_scheduled), a poll now (g_poll_now), and the send retry timers.
 */

static bool g_poll_now;
static bool g_poll_scheduled;
static clock_t g_next_poll;
static bool g_retry_set[NINA_SOCK_MAX];
static clock_t g_retry[NINA_SOCK_MAX];

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

/* The send retry timers that are due: the next pass of each send. */

static void run_retries(void)
{
  clock_t now = clock_systime_ticks();
  struct nina_sock *s;
  int i;

  for (i = 0; i < NINA_SOCK_MAX; i++)
    {
      if (g_retry_set[i] && (clock_t)(now - g_retry[i]) >= 0)
        {
          g_retry_set[i] = false;
          s = nina_sock_get(i);
          if (s != NULL && s->send_waiting)
            {
              nina_usrsock_report(i, NINA_SOCK_EV_WRITE);
            }
        }
    }
}

/* Ticks until the next poll or retry, 0 for now, -1 for nothing. */

static clock_t next_wait(void)
{
  clock_t now = clock_systime_ticks();
  clock_t wait = -1;
  clock_t d;
  int i;

  if (g_poll_now)
    {
      return 0;
    }

  if (g_poll_scheduled)
    {
      d = (clock_t)(g_next_poll - now);
      wait = d > 0 ? d : 0;
    }

  for (i = 0; i < NINA_SOCK_MAX; i++)
    {
      if (g_retry_set[i])
        {
          d = (clock_t)(g_retry[i] - now);
          d = d > 0 ? d : 0;
          if (wait < 0 || d < wait)
            {
              wait = d;
            }
        }
    }

  return wait;
}

static int nina_poller(int argc, FAR char *argv[])
{
  for (; ; )
    {
      bool poll_all;
      clock_t wait;
      int i;

      usrsock_lock();
      poll_all = g_poll_now ||
                 (g_poll_scheduled &&
                  (clock_t)(clock_systime_ticks() - g_next_poll) >= 0);
      g_poll_now = false;
      usrsock_unlock();

      if (poll_all)
        {
          bool polled = false;

          for (i = 0; i < NINA_SOCK_MAX; i++)
            {
              usrsock_lock();
              polled |= poll_socket(i);
              usrsock_unlock();
            }

          /* k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS)) of the Zephyr build's
           * loops: the interval in ticks, plus one (the wait below ends one
           * tick after g_next_poll, as wd_start() does).
           */

          usrsock_lock();
          g_poll_scheduled = polled;
          g_next_poll = clock_systime_ticks() +
                        MSEC2TICK(NINA_POLL_INTERVAL_MS);
          usrsock_unlock();
        }

      usrsock_lock();
      run_retries();
      wait = next_wait();
      usrsock_unlock();

      if (wait < 0)
        {
          nxsem_wait_uninterruptible(&g_wake);
        }
      else if (wait > 0)
        {
          nxsem_tickwait_uninterruptible(&g_wake, (uint32_t)wait);
        }
    }

  return 0;
}

static void post(void)
{
  int value;

  if (nxsem_get_value(&g_wake, &value) == 0 && value <= 0)
    {
      nxsem_post(&g_wake);
    }
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
  g_poll_now = true;
  post();
}

void nina_poller_kick(void)
{
  if (!g_poll_scheduled)
    {
      g_poll_scheduled = true;
      g_next_poll = clock_systime_ticks() +
                    MSEC2TICK(NINA_POLL_INTERVAL_MS);
    }

  post();
}

void nina_poller_retry(int usockid)
{
  if (usockid >= 0 && usockid < NINA_SOCK_MAX)
    {
      g_retry[usockid] = clock_systime_ticks() +
                         MSEC2TICK(NINA_SEND_RETRY_MS);
      g_retry_set[usockid] = true;
      post();
    }
}

void nina_poller_cancel(int usockid)
{
  if (usockid >= 0 && usockid < NINA_SOCK_MAX)
    {
      g_retry_set[usockid] = false;
    }
}
