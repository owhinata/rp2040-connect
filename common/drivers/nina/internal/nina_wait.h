/* SPDX-License-Identifier: MIT */
/* The waits of the NINA socket loops (nina_wait.c): between two passes of
 * a loop that found nothing (a connect in progress, a receive or poll with
 * nothing to report, a send without room), either the loop's poll interval
 * (the 3.0.2 firmware, or nina_os_event_start() failed), or, when the
 * module reports socket events on its IRQ line (NINA_CAP_SOCKET_EVENTS,
 * nina_proto.h), the next rising edge of the line, at most
 * NINA_EVENT_TIMEOUT_MS.  The loops: Zephyr's and FreeRTOS's nina_socket.c,
 * and the NuttX build's event thread (nina_poller.c, which uses
 * nina_wait_events(), nina_os_event_seq() and nina_wait_backoff_ms()
 * itself, from the edge counts that the usrsock requests of nina_usrsock.c
 * note before their commands).
 *
 * The rule of the firmware (firmware/nina-fw/README.md): note the edge
 * count before the command that may arm an event; if the answer is "not
 * ready", wait until the count changes or a timeout passes, then ask again.
 * So a loop does, for each pass:
 *
 *   struct nina_wait w = NINA_WAIT_INIT;     (once, at the call's start)
 *   for (;;) {
 *       nina_wait_mark(&w);                 (before the pass's commands)
 *       the pass (its commands);
 *       done? break;
 *       nina_wait(&w, poll interval, the call's deadline);
 *   }
 *
 * An edge from before the pass's commands only costs one more pass.  An
 * edge that is lost (the module's select() failed, an interrupt missed)
 * costs at most NINA_EVENT_TIMEOUT_MS: the passes still ask the module.
 *
 * One line serves every socket, so an edge wakes every waiting loop; one
 * whose pass then finds nothing again backs off (nina_wait_backoff_ms():
 * 0, 0, then 1, 2, 4, 8, 16 ms, then NINA_POLL_INTERVAL_MS before it waits
 * for the next edge), so that edges of other sockets, or a socket whose
 * event comes back at once without its pass finding anything, cost no more
 * passes than the poll interval did.  A wait that ends without an edge
 * starts the count again.
 */

#ifndef NINA_WAIT_H_
#define NINA_WAIT_H_

#include <stdbool.h>
#include <stdint.h>

/* The longest wait for an edge (the README's "about 100 ms"). */
#define NINA_EVENT_TIMEOUT_MS 100

/* The Zephyr build's NINA_POLL_INTERVAL_MS (nina_socket.c:26): how often a
 * waiting poll(), receive or connect asks the module again without the
 * events.
 */
#define NINA_POLL_INTERVAL_MS 20

struct nina_wait {
	uint32_t mark;    /* the edge count before the pass */
	uint8_t woken;    /* waits in a row ended by an edge */
};

#define NINA_WAIT_INIT {0, 0}

/* Called once by nina_probe_capabilities() (nina_proto.c) when the module
 * reports socket events: nina_os_event_start(), and the waits use the
 * events from then on.  0, or nina_os_event_start()'s negative errno: the
 * waits keep the poll intervals.
 */
int nina_wait_start_events(void);

/* Whether the waits use the events. */
bool nina_wait_events(void);

/* Before a pass: the edge count (nothing without the events). */
void nina_wait_mark(struct nina_wait *w);

/* After a pass that found nothing: without the events, POLL_MS
 * (nina_os_sleep_ms(), the loops' sleep before); with them, the back-off,
 * then until an edge after the mark, at most NINA_EVENT_TIMEOUT_MS and not
 * past DEADLINE (nina_os_uptime_ms(); INT64_MAX for none).  The caller
 * checks its deadline after the next pass, as before.
 */
void nina_wait(struct nina_wait *w, uint32_t poll_ms, int64_t deadline);

/* The back-off before the wait after WOKEN waits in a row that an edge
 * ended (see above).
 */
uint32_t nina_wait_backoff_ms(unsigned int woken);

#endif /* NINA_WAIT_H_ */
