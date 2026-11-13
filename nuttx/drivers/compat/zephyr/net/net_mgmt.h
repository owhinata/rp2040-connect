/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's <zephyr/net/net_mgmt.h> (Zephyr v4.5.0) that the
 * NINA network device and the application use: requests (net_mgmt()) and
 * event callbacks, with Zephyr's names and behaviour:
 *
 *   net_mgmt(request, iface, data, len) calls the handler
 *   net_mgmt_<request>() in the caller's thread, as Zephyr's macro does
 *   (subsys/net/ip/net_mgmt.c).  The handlers of the Wi-Fi requests are in
 *   zephyr_wifi_mgmt.c (<zephyr/net/wifi_mgmt.h>).
 *
 *   net_mgmt_event_notify_with_info() puts the event and a copy of its
 *   information in a queue of CONFIG_NET_MGMT_EVENT_QUEUE_SIZE (5) entries,
 *   waiting up to CONFIG_NET_MGMT_EVENT_QUEUE_TIMEOUT (10 ms) for room and
 *   dropping the event after that (the Zephyr build's NET_WARN() there is
 *   compiled out, CONFIG_NET_MGMT_EVENT_LOG_LEVEL=0).  The "net_mgmt"
 *   thread (CONFIG_NET_MGMT_EVENT_THREAD: priority -1, 2048 bytes; here
 *   NANO_PRIO_NET_MGMT 119) takes them in order and calls each callback
 *   whose mask has the event, with cb->info and cb->info_length set, under
 *   the callback list's lock.  Started by net_mgmt_event_init() before the
 *   interface is initialized (Zephyr's net_init()).
 *
 * The event codes are bits of this port, not Zephyr's 64-bit layer/code
 * values; callbacks compare them with the same names.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_MGMT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_MGMT_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/net/net_if.h>

/* Requests: net_mgmt(NET_REQUEST_X, iface, data, len) calls
 * net_mgmt_NET_REQUEST_X(NET_REQUEST_X, iface, data, len).
 */

#define net_mgmt(_mgmt_request, _iface, _data, _len) \
  net_mgmt_##_mgmt_request(_mgmt_request, _iface, _data, _len)

typedef int (*net_mgmt_request_handler_t)(uint64_t mgmt_request,
                                          struct net_if *iface,
                                          void *data, size_t len);

#define NET_MGMT_DEFINE_REQUEST_HANDLER(_mgmt_request) \
  extern int net_mgmt_##_mgmt_request(uint64_t mgmt_request, \
                                      struct net_if *iface, \
                                      void *data, size_t len)

/* Event callbacks */

struct net_mgmt_event_callback;

typedef void (*net_mgmt_event_handler_t)(struct net_mgmt_event_callback *cb,
                                         uint64_t mgmt_event,
                                         struct net_if *iface);

struct net_mgmt_event_callback
{
  struct net_mgmt_event_callback *next;  /* in the list while added */
  net_mgmt_event_handler_t handler;
  const void *info;                      /* valid during the handler */
  size_t info_length;
  uint64_t event_mask;
};

static inline void
net_mgmt_init_event_callback(struct net_mgmt_event_callback *cb,
                             net_mgmt_event_handler_t handler,
                             uint64_t mgmt_event_mask)
{
  cb->next = NULL;
  cb->handler = handler;
  cb->info = NULL;
  cb->info_length = 0;
  cb->event_mask = mgmt_event_mask;
}

void net_mgmt_add_event_callback(struct net_mgmt_event_callback *cb);
void net_mgmt_del_event_callback(struct net_mgmt_event_callback *cb);

/* Any thread (not an interrupt handler: it may wait 10 ms for room). */

void net_mgmt_event_notify_with_info(uint64_t mgmt_event,
                                     struct net_if *iface,
                                     const void *info, size_t length);

static inline void net_mgmt_event_notify(uint64_t mgmt_event,
                                         struct net_if *iface)
{
  net_mgmt_event_notify_with_info(mgmt_event, iface, NULL, 0);
}

/* Starts the "net_mgmt" thread; 0 or a negative errno. */

int net_mgmt_event_init(void);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_MGMT_H */
