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
 * With the module's socket events (nina_wait_events(): the firmware has
 * them and nina_os.c waits for the IRQ line's edges, common/drivers/nina/
 * nina_wait.h), the waits end at the line's next edge instead, at most
 * NINA_EVENT_TIMEOUT_MS after the pass: the next poll of the sockets comes
 * after an edge since the edge count noted before the last pass (or before
 * the receive request that kicked it), and the next pass of a waiting send
 * after an edge since the count noted before its own pass, when that pass
 * only polled (the core's send_polled; after a pass that sent, the
 * NINA_SEND_RETRY_MS timer as before).  The interrupt wakes this thread
 * (nina_poller_edge()).  Every edge is any socket's, so a pass that an edge
 * started and that reports nothing makes the next one wait a little first
 * (nina_wait_backoff_ms(): up to NINA_POLL_INTERVAL_MS), for the polls and
 * for each send.  The poll's timer is not later than the deadline of a
 * connection in progress.
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
#include "nina_wait.h"
#include "priorities.h"

/* What poll_socket() did. */

#define POLL_NONE     0   /* nothing to ask */
#define POLL_ASKED    1   /* asked, nothing to report */
#define POLL_REPORTED 2   /* reported something, or answered a connect */

static sem_t g_wake = SEM_INITIALIZER(0);
static bool g_started;

/* Under the usrsock lock: the next poll of every socket (when
 * g_poll_scheduled), a poll now (g_poll_now), and the send retry timers.
 * With the events: the edge count before the last pass (g_poll_mark), the
 * passes in a row that an edge started and that reported nothing
 * (g_poll_woken), and the time before which an edge does not start a pass
 * (g_poll_edge_from); for each send the same (g_retry_*).
 */

static bool g_poll_now;
static bool g_poll_scheduled;
static clock_t g_next_poll;
static uint32_t g_poll_mark;
static uint8_t g_poll_woken;
static clock_t g_poll_edge_from;
static bool g_retry_set[NINA_SOCK_MAX];
static clock_t g_retry[NINA_SOCK_MAX];
static bool g_retry_edge[NINA_SOCK_MAX];       /* an edge may end the wait */
static bool g_retry_by_edge[NINA_SOCK_MAX];    /* the last one ended so */
static uint32_t g_retry_mark[NINA_SOCK_MAX];
static uint8_t g_retry_woken[NINA_SOCK_MAX];
static clock_t g_retry_edge_from[NINA_SOCK_MAX];

/* The edge count, and whether it has moved since MARK. */

static uint32_t event_seq(void)
{
#ifdef NINA_OS_EVENTS
  return nina_wait_events() ? nina_os_event_seq() : 0;
#else
  return 0;
#endif
}

static bool edge_since(uint32_t mark)
{
  return nina_wait_events() && event_seq() != mark;
}

static bool due(clock_t when, clock_t now)
{
  return (clock_t)(now - when) >= 0;
}

/* The ticks of MS milliseconds, at most until the time DEADLINE
 * (nina_os_uptime_ms()) of a call, 0 if it has passed.
 */

static clock_t wait_ticks(uint32_t ms, int64_t deadline)
{
  int64_t left = deadline - nina_os_uptime_ms();

  if (left <= 0)
    {
      return 0;
    }

  return MSEC2TICK(left < (int64_t)ms ? (uint32_t)left : ms);
}

/* One socket: what it did (POLL_*).  *DEADLINE gets the deadline of a
 * connection in progress, if earlier.
 */

static int poll_socket(int usockid, int64_t *deadline)
{
  struct nina_sock *s = nina_sock_get(usockid);
  uint8_t want;
  uint8_t state = 0;
  int ret;

  if (s == NULL || (want = nina_sock_watch(s)) == 0)
    {
      return POLL_NONE;
    }

  if (want == NINA_SOCK_WATCH_CONNECT)
    {
      ret = nina_sock_connect_check(usockid, nina_os_uptime_ms());
      if (ret <= 0)
        {
          nina_usrsock_connected(usockid, ret);
          return POLL_REPORTED;
        }

      if (s->connect_deadline < *deadline)
        {
          *deadline = s->connect_deadline;
        }

      return POLL_ASKED;
    }

  ret = nina_sock_poll_state(s, &state);
  want = nina_sock_events(s, ret, state);
  if (want != 0)
    {
      nina_usrsock_report(usockid, want);
      return POLL_REPORTED;
    }

  return POLL_ASKED;
}

/* The send retry timers that are due (or whose edge came): the next pass
 * of each send.
 */

static void run_retries(void)
{
  clock_t now = clock_systime_ticks();
  struct nina_sock *s;
  bool timer;
  bool edge;
  int i;

  for (i = 0; i < NINA_SOCK_MAX; i++)
    {
      if (!g_retry_set[i])
        {
          continue;
        }

      timer = due(g_retry[i], now);
      edge = g_retry_edge[i] && edge_since(g_retry_mark[i]) &&
             due(g_retry_edge_from[i], now);
      if (timer || edge)
        {
          g_retry_set[i] = false;
          g_retry_by_edge[i] = !timer;
          s = nina_sock_get(i);
          if (s != NULL && s->send_waiting)
            {
              nina_usrsock_report(i, NINA_SOCK_EV_WRITE);
            }
        }
    }
}

/* The ticks until T, 0 if due; the earlier of that and *WAIT (-1: none). */

static void earliest(clock_t *wait, clock_t t, clock_t now)
{
  clock_t d = (clock_t)(t - now);

  d = d > 0 ? d : 0;
  if (*wait < 0 || d < *wait)
    {
      *wait = d;
    }
}

/* Ticks until the next poll or retry, 0 for now, -1 for nothing. */

static clock_t next_wait(void)
{
  clock_t now = clock_systime_ticks();
  clock_t wait = -1;
  int i;

  if (g_poll_now)
    {
      return 0;
    }

  if (g_poll_scheduled)
    {
      earliest(&wait, g_next_poll, now);
      if (edge_since(g_poll_mark))
        {
          earliest(&wait, g_poll_edge_from, now);
        }
    }

  for (i = 0; i < NINA_SOCK_MAX; i++)
    {
      if (g_retry_set[i])
        {
          earliest(&wait, g_retry[i], now);
          if (g_retry_edge[i] && edge_since(g_retry_mark[i]))
            {
              earliest(&wait, g_retry_edge_from[i], now);
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
      bool by_edge;
      uint32_t mark;
      clock_t wait;
      clock_t now;
      int i;

      usrsock_lock();
      now = clock_systime_ticks();
      by_edge = g_poll_scheduled && !due(g_next_poll, now) &&
                edge_since(g_poll_mark) && due(g_poll_edge_from, now);
      poll_all = g_poll_now || by_edge ||
                 (g_poll_scheduled && due(g_next_poll, now));
      by_edge = by_edge && !g_poll_now;
      g_poll_now = false;
      mark = event_seq();
      usrsock_unlock();

      if (poll_all)
        {
          int64_t deadline = INT64_MAX;
          int did = POLL_NONE;
          int ret;

          for (i = 0; i < NINA_SOCK_MAX; i++)
            {
              usrsock_lock();
              ret = poll_socket(i, &deadline);
              did = ret > did ? ret : did;
              usrsock_unlock();
            }

          /* k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS)) of the Zephyr build's
           * loops: the interval in ticks, plus one (the wait below ends one
           * tick after g_next_poll, as wd_start() does).  With the events,
           * NINA_EVENT_TIMEOUT_MS or an edge.
           */

          usrsock_lock();
          now = clock_systime_ticks();
          g_poll_scheduled = did != POLL_NONE;
          if (nina_wait_events())
            {
              g_poll_woken = by_edge && did != POLL_REPORTED ?
                             (g_poll_woken < UINT8_MAX ?
                              g_poll_woken + 1 : g_poll_woken) : 0;
              g_poll_mark = mark;
              g_poll_edge_from = now +
                                 MSEC2TICK(nina_wait_backoff_ms(g_poll_woken));
              g_next_poll = now + wait_ticks(NINA_EVENT_TIMEOUT_MS, deadline);
            }
          else
            {
              g_next_poll = now + MSEC2TICK(NINA_POLL_INTERVAL_MS);
            }

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

void nina_poller_kick(uint32_t mark)
{
  if (!g_poll_scheduled)
    {
      g_poll_scheduled = true;
      if (nina_wait_events())
        {
          /* The request's count: an edge since its poll ends the wait. */

          g_poll_mark = mark;
          g_poll_woken = 0;
          g_poll_edge_from = clock_systime_ticks();
          g_next_poll = clock_systime_ticks() +
                        MSEC2TICK(NINA_EVENT_TIMEOUT_MS);
        }
      else
        {
          g_next_poll = clock_systime_ticks() +
                        MSEC2TICK(NINA_POLL_INTERVAL_MS);
        }
    }

  post();
}

void nina_poller_retry(int usockid, uint32_t mark)
{
  struct nina_sock *s = nina_sock_get(usockid);
  clock_t now = clock_systime_ticks();

  if (usockid < 0 || usockid >= NINA_SOCK_MAX)
    {
      return;
    }

  if (nina_wait_events() && s != NULL && s->send_polled)
    {
      /* The pass polled only: the module armed the socket's writable
       * event.  An edge since MARK, or NINA_EVENT_TIMEOUT_MS (not past the
       * send's deadline); after edges that did not help, a little later.
       */

      g_retry_woken[usockid] = g_retry_by_edge[usockid] ?
                               (g_retry_woken[usockid] < UINT8_MAX ?
                                g_retry_woken[usockid] + 1 :
                                g_retry_woken[usockid]) : 0;
      g_retry_edge[usockid] = true;
      g_retry_mark[usockid] = mark;
      g_retry_edge_from[usockid] = now +
        MSEC2TICK(nina_wait_backoff_ms(g_retry_woken[usockid]));
      g_retry[usockid] = now + wait_ticks(NINA_EVENT_TIMEOUT_MS,
                                          s->send_deadline);
    }
  else
    {
      g_retry_edge[usockid] = false;
      g_retry_woken[usockid] = 0;
      g_retry[usockid] = now + MSEC2TICK(NINA_SEND_RETRY_MS);
    }

  g_retry_by_edge[usockid] = false;
  g_retry_set[usockid] = true;
  post();
}

void nina_poller_cancel(int usockid)
{
  if (usockid >= 0 && usockid < NINA_SOCK_MAX)
    {
      g_retry_set[usockid] = false;
      g_retry_by_edge[usockid] = false;
    }
}

void nina_poller_edge(void)
{
  if (g_started)
    {
      post();
    }
}
