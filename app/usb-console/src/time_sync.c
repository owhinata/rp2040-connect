/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/sntp.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/clock.h>
#include "time_sync.h"

#define NTP_QUERY_TIMEOUT_MS 5000
#define NTP_RETRY_MS         60000
#define NTP_REFRESH_MS       (6 * 60 * 60 * 1000)
#define NTP_MAX_AGE_MS       (24 * 60 * 60 * 1000)
#define NTP_MIN_UNIX_TIME    1577836800ULL

static struct net_mgmt_event_callback wifi_events;
static atomic_t wifi_connected;
static atomic_t wifi_generation;
static atomic_t synchronized;
static atomic_t synchronizing;
static atomic_t last_error;
static int64_t last_sync_uptime;
K_SEM_DEFINE(sync_request, 0, 1);
K_MUTEX_DEFINE(sync_lock);

static int query_time(struct sntp_time *time)
{
	const struct zsock_addrinfo hints = {
		.ai_family = NET_AF_INET,
		.ai_socktype = NET_SOCK_DGRAM,
		.ai_protocol = NET_IPPROTO_UDP,
	};
	struct zsock_addrinfo *addr = NULL;
	struct sntp_ctx ctx;
	char port[6];
	int ret;

	snprintk(port, sizeof(port), "%u", CONFIG_NINA_NTP_PORT);
	ret = zsock_getaddrinfo(CONFIG_NINA_NTP_SERVER, port, &hints, &addr);
	if (ret) {
		return -EHOSTUNREACH;
	}
	if (!addr || !addr->ai_addr) {
		if (addr) {
			zsock_freeaddrinfo(addr);
		}
		return -EHOSTUNREACH;
	}
	ret = sntp_init(&ctx, addr->ai_addr, addr->ai_addrlen);
	zsock_freeaddrinfo(addr);
	if (ret) {
		return ret;
	}
	/* One query per attempt: do not retry rapidly against public servers.
	 */
	ret = sntp_query(&ctx, NTP_QUERY_TIMEOUT_MS, time);
	sntp_close(&ctx);
	return ret;
}

static int set_clock(const struct sntp_time *time)
{
	struct timespec ts;
	uint64_t nanoseconds;

	if (time->seconds < NTP_MIN_UNIX_TIME) {
		return -ERANGE;
	}
	ts.tv_sec = (time_t)time->seconds;
	if (ts.tv_sec < 0 || (uint64_t)ts.tv_sec != time->seconds) {
		return -ERANGE;
	}
	nanoseconds = ((uint64_t)time->fraction * NSEC_PER_SEC) >> 32;
	nanoseconds += (uint64_t)time->rsp_delay_us * NSEC_PER_USEC;
	ts.tv_sec += nanoseconds / NSEC_PER_SEC;
	ts.tv_nsec = nanoseconds % NSEC_PER_SEC;
	return sys_clock_settime(SYS_CLOCK_REALTIME, &ts);
}

static void sync_thread(void *a, void *b, void *c)
{
	int64_t next_attempt = 0;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		struct sntp_time time;
		atomic_val_t generation;
		int64_t wait;
		int ret;

		k_sem_take(&sync_request, K_FOREVER);
		while (atomic_get(&wifi_connected)) {
			wait = next_attempt - k_uptime_get();
			if (wait > 0) {
				(void)k_sem_take(&sync_request, K_MSEC(wait));
				continue;
			}
			generation = atomic_get(&wifi_generation);
			atomic_set(&synchronizing, 1);
			ret = query_time(&time);
			if (!ret &&
			    (!atomic_get(&wifi_connected) ||
			     generation != atomic_get(&wifi_generation))) {
				ret = -ECANCELED;
			}
			if (!ret) {
				ret = set_clock(&time);
			}
			atomic_set(&last_error, ret);
			if (!ret) {
				k_mutex_lock(&sync_lock, K_FOREVER);
				last_sync_uptime = k_uptime_get();
				atomic_set(&synchronized, 1);
				k_mutex_unlock(&sync_lock);
			}
			atomic_clear(&synchronizing);
			next_attempt = k_uptime_get() +
				       (ret ? NTP_RETRY_MS : NTP_REFRESH_MS);
		}
	}
}
K_THREAD_DEFINE(time_sync_tid, 2048, sync_thread, NULL, NULL, NULL, 9, 0, 0);

static void wifi_event(struct net_mgmt_event_callback *cb, uint64_t event,
		       struct net_if *iface)
{
	ARG_UNUSED(iface);
	if (event == NET_EVENT_WIFI_CONNECT_RESULT) {
		const struct wifi_status *status = cb->info;

		if (!status || status->status) {
			return;
		}
		atomic_inc(&wifi_generation);
		atomic_set(&wifi_connected, 1);
		k_sem_give(&sync_request);
	} else if (event == NET_EVENT_WIFI_DISCONNECT_RESULT) {
		atomic_inc(&wifi_generation);
		atomic_clear(&wifi_connected);
		k_sem_give(&sync_request);
	}
}

void time_sync_init(void)
{
	net_mgmt_init_event_callback(&wifi_events, wifi_event,
				     NET_EVENT_WIFI_CONNECT_RESULT |
					     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_events);
}

bool time_sync_is_ready(void)
{
	bool ready;

	k_mutex_lock(&sync_lock, K_FOREVER);
	ready = atomic_get(&synchronized) &&
		k_uptime_get() - last_sync_uptime <= NTP_MAX_AGE_MS;
	k_mutex_unlock(&sync_lock);
	return ready;
}

static int cmd_ntp_status(const struct shell *sh, size_t argc, char **argv)
{
	struct timespec now;
	int64_t age;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "NTP server: %s:%u", CONFIG_NINA_NTP_SERVER,
		    CONFIG_NINA_NTP_PORT);
	if (!time_sync_is_ready()) {
		shell_print(sh, "Time: not synchronized%s (last error: %d)",
			    atomic_get(&synchronizing) ? ", in progress" : "",
			    (int)atomic_get(&last_error));
		return 0;
	}
	if (sys_clock_gettime(SYS_CLOCK_REALTIME, &now)) {
		return -EIO;
	}
	k_mutex_lock(&sync_lock, K_FOREVER);
	age = (k_uptime_get() - last_sync_uptime) / 1000;
	k_mutex_unlock(&sync_lock);
	shell_print(sh,
		    "Unix time: %" PRId64 " (synchronized %" PRId64
		    " seconds ago)",
		    (int64_t)now.tv_sec, age);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ntp_commands,
			       SHELL_CMD(status, NULL,
					 "Show time synchronization status",
					 cmd_ntp_status),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(ntp, &ntp_commands, "Network time synchronization", NULL);
