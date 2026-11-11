/* SPDX-License-Identifier: Apache-2.0 */
/* Keeps an estimate of the NTP server's time, normally the PC that provides
 * the Wi-Fi network (the default gateway). The server is asked every few
 * seconds and the exchanges with the shortest round trip are used; see
 * time_offset.h. The board's realtime clock follows the estimate, but code
 * that needs the best time should convert from the monotonic clock with
 * time_sync_unix_us(). */
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/clock.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include "time_offset.h"
#include "time_sync.h"

#define NTP_REPLY_WAIT_MS   250
#define NTP_RETRY_MS        5000
#define NTP_MAX_AGE_MS      60000
#define NTP_MIN_UNIX_TIME   1577836800LL
#define NTP_UNIX_OFFSET     2208988800LL
/* Step the realtime clock when it is this far from the estimate. */
#define NTP_CLOCK_STEP_US   20000
#define NTP_PACKET_SIZE     48
#define NTP_LOG_SIZE        128

struct sync_log {
	uint32_t mono_ms, delay_us;
	int32_t residual_us;
};

static struct net_mgmt_event_callback wifi_events;
static atomic_t wifi_connected, wifi_generation, last_error;
static atomic_t queries, timeouts, rejected, clock_steps;
K_SEM_DEFINE(sync_request, 0, 1);
K_MUTEX_DEFINE(sync_lock);
/* Protected by sync_lock. */
static struct time_offset estimate;
static struct time_slew slewed;
static int64_t last_sample_uptime;
static struct sync_log sync_log[NTP_LOG_SIZE];
static unsigned int log_count, log_next;
static struct sockaddr_in server;

/* The RP2040's 1 MHz hardware timer, read through its raw registers (no
 * latching, so any context may read it). The kernel clock is not used: it is
 * driven by SysTick and was measured to lose about 480 ppm while streaming
 * at 6667 Hz, although it kept time when idle. */
#define TIMER_BASE   DT_REG_ADDR(DT_NODELABEL(timer))
#define TIMER_RAWH   (TIMER_BASE + 0x24)
#define TIMER_RAWL   (TIMER_BASE + 0x28)

int64_t time_sync_mono_us(void)
{
	uint32_t high, low, again = sys_read32(TIMER_RAWH);

	do {
		high = again;
		low = sys_read32(TIMER_RAWL);
		again = sys_read32(TIMER_RAWH);
	} while (again != high);
	return (int64_t)(((uint64_t)high << 32) | low);
}

static bool estimate_recent(void)
{
	return estimate.samples && k_uptime_get() - last_sample_uptime <= NTP_MAX_AGE_MS;
}

bool time_sync_unix_us(int64_t mono_us, int64_t *unix_us, uint32_t *delay_us)
{
	int64_t raw;
	bool ok;

	k_mutex_lock(&sync_lock, K_FOREVER);
	ok = estimate_recent() && time_offset_get(&estimate, mono_us, &raw, delay_us) &&
	     time_slew_get(&slewed, mono_us, unix_us);
	k_mutex_unlock(&sync_lock);
	return ok;
}

bool time_sync_estimate_us(int64_t mono_us, int64_t *unix_us, uint32_t *delay_us)
{
	bool ok;

	k_mutex_lock(&sync_lock, K_FOREVER);
	ok = estimate_recent() && time_offset_get(&estimate, mono_us, unix_us, delay_us);
	k_mutex_unlock(&sync_lock);
	return ok;
}

bool time_sync_is_ready(void)
{
	bool ready;

	k_mutex_lock(&sync_lock, K_FOREVER);
	ready = estimate_recent();
	k_mutex_unlock(&sync_lock);
	return ready;
}

static int resolve_server(void)
{
	memset(&server, 0, sizeof(server));
	server.sin_family = AF_INET;
	server.sin_port = htons(CONFIG_NINA_NTP_PORT);
	if (sizeof(CONFIG_NINA_NTP_SERVER) == 1) {
		struct net_if *iface = net_if_get_default();

		if (!iface) {
			return -ENETDOWN;
		}
		server.sin_addr = net_if_ipv4_get_gw(iface);
		return server.sin_addr.s_addr ? 0 : -EHOSTUNREACH;
	} else {
		const struct zsock_addrinfo hints = {
			.ai_family = AF_INET, .ai_socktype = SOCK_DGRAM, .ai_protocol = IPPROTO_UDP,
		};
		struct zsock_addrinfo *addr = NULL;

		if (zsock_getaddrinfo(CONFIG_NINA_NTP_SERVER, NULL, &hints, &addr) || !addr ||
		    !addr->ai_addr) {
			if (addr) {
				zsock_freeaddrinfo(addr);
			}
			return -EHOSTUNREACH;
		}
		server.sin_addr = ((struct sockaddr_in *)addr->ai_addr)->sin_addr;
		zsock_freeaddrinfo(addr);
		return 0;
	}
}

static uint32_t be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* NTP timestamp to Unix microseconds. Seconds below the 1970 offset belong
 * to the era that starts in 2036. */
static int64_t ntp_to_unix_us(const uint8_t *p)
{
	int64_t seconds = be32(p);

	if (seconds < NTP_UNIX_OFFSET) {
		seconds += 1LL << 32;
	}
	return (seconds - NTP_UNIX_OFFSET) * USEC_PER_SEC +
	       (int64_t)(((uint64_t)be32(p + 4) * USEC_PER_SEC) >> 32);
}

/* One request and its reply. The reply is polled for at 1 ms intervals:
 * every millisecond between its arrival and this thread noticing it looks
 * like path delay in one direction only. */
static int exchange(int fd, struct time_offset_sample *sample)
{
	uint8_t packet[NTP_PACKET_SIZE] = {0x23}; /* version 4, client */
	uint8_t reply[NTP_PACKET_SIZE];
	int64_t t1, t2, t3, t4, deadline;

	t1 = time_sync_mono_us();
	/* The server returns this value; it only has to identify the request. */
	packet[40] = (uint8_t)(t1 >> 56); packet[41] = (uint8_t)(t1 >> 48);
	packet[42] = (uint8_t)(t1 >> 40); packet[43] = (uint8_t)(t1 >> 32);
	packet[44] = (uint8_t)(t1 >> 24); packet[45] = (uint8_t)(t1 >> 16);
	packet[46] = (uint8_t)(t1 >> 8); packet[47] = (uint8_t)t1;
	t1 = time_sync_mono_us();
	if (zsock_sendto(fd, packet, sizeof(packet), 0, (struct sockaddr *)&server,
			 sizeof(server)) != sizeof(packet)) {
		return -errno;
	}
	deadline = k_uptime_get() + NTP_REPLY_WAIT_MS;
	for (;;) {
		ssize_t n = zsock_recv(fd, reply, sizeof(reply), ZSOCK_MSG_DONTWAIT);

		t4 = time_sync_mono_us();
		if (n < 0 && errno != EAGAIN) {
			return -errno;
		}
		if (n == sizeof(reply) && !memcmp(reply + 24, packet + 40, 8)) {
			break;
		}
		/* Anything else is a late reply to an earlier request. */
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		if (n < 0) {
			k_sleep(K_MSEC(1));
		}
	}
	/* Server mode, synchronized, stratum 1..15, transmit time present. */
	if ((reply[0] & 7) != 4 || (reply[0] >> 6) == 3 || reply[1] == 0 || reply[1] > 15 ||
	    (!be32(reply + 40) && !be32(reply + 44))) {
		return -EPROTO;
	}
	t2 = ntp_to_unix_us(reply + 32);
	t3 = ntp_to_unix_us(reply + 40);
	if (t3 < t2 || t2 < NTP_MIN_UNIX_TIME * USEC_PER_SEC || (t4 - t1) < (t3 - t2)) {
		return -EPROTO;
	}
	sample->mono_us = (t1 + t4) / 2;
	sample->offset_us = ((t2 - t1) + (t3 - t4)) / 2;
	sample->delay_us = (uint32_t)MIN((t4 - t1) - (t3 - t2), (int64_t)UINT32_MAX);
	return 0;
}

/* The kernel's realtime clock is only pushed forward. It falls behind under
 * load (see time_sync_mono_us), so this is the usual direction; being ahead
 * is left alone rather than stepping it back. It is good for dates and logs,
 * not for precise time. */
static void follow_realtime_clock(void)
{
	struct timespec now, target;
	int64_t unix_us, current_us;

	if (sys_clock_gettime(SYS_CLOCK_REALTIME, &now) ||
	    !time_slew_get(&slewed, time_sync_mono_us(), &unix_us)) {
		return;
	}
	current_us = (int64_t)now.tv_sec * USEC_PER_SEC + now.tv_nsec / NSEC_PER_USEC;
	if (unix_us - current_us < NTP_CLOCK_STEP_US) {
		return;
	}
	target.tv_sec = unix_us / USEC_PER_SEC;
	target.tv_nsec = (unix_us % USEC_PER_SEC) * NSEC_PER_USEC;
	if (!sys_clock_settime(SYS_CLOCK_REALTIME, &target)) {
		atomic_inc(&clock_steps);
	}
}

static void record(const struct time_offset_sample *sample)
{
	k_mutex_lock(&sync_lock, K_FOREVER);
	int64_t now = time_sync_mono_us(), target;

	time_offset_add(&estimate, sample);
	if (time_offset_get(&estimate, now, &target, NULL)) {
		time_slew_update(&slewed, now, target, estimate.rate_valid ? estimate.rate_ppb : 0);
	}
	last_sample_uptime = k_uptime_get();
	sync_log[log_next] = (struct sync_log){
		.mono_ms = (uint32_t)(sample->mono_us / 1000), .delay_us = sample->delay_us,
		.residual_us = (int32_t)CLAMP(estimate.last_residual_us, INT32_MIN, INT32_MAX),
	};
	log_next = (log_next + 1) % NTP_LOG_SIZE;
	log_count = MIN(log_count + 1, NTP_LOG_SIZE);
	follow_realtime_clock();
	k_mutex_unlock(&sync_lock);
}

static void sync_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&sync_request, K_FOREVER);
		while (atomic_get(&wifi_connected)) {
			atomic_val_t generation = atomic_get(&wifi_generation);
			int fd = -1, ret = resolve_server();

			if (!ret) {
				fd = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
				ret = fd < 0 ? -errno : 0;
			}
			/* A new connection may be a different network and server. */
			k_mutex_lock(&sync_lock, K_FOREVER);
			time_offset_init(&estimate);
			k_mutex_unlock(&sync_lock);
			while (!ret && atomic_get(&wifi_connected) &&
			       generation == atomic_get(&wifi_generation)) {
				struct time_offset_sample sample;

				atomic_inc(&queries);
				ret = exchange(fd, &sample);
				if (!ret) {
					record(&sample);
				} else if (ret == -ETIMEDOUT) {
					atomic_inc(&timeouts);
					ret = 0;
				} else if (ret == -EPROTO) {
					atomic_inc(&rejected);
					ret = 0;
				}
				atomic_set(&last_error, ret);
				if (!ret) {
					(void)k_sem_take(&sync_request,
							 K_MSEC(CONFIG_NINA_NTP_INTERVAL_MS));
				}
			}
			if (fd >= 0) {
				(void)zsock_close(fd);
			}
			if (ret) {
				atomic_set(&last_error, ret);
				(void)k_sem_take(&sync_request, K_MSEC(NTP_RETRY_MS));
			}
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
	time_offset_init(&estimate);
	time_slew_init(&slewed);
	net_mgmt_init_event_callback(&wifi_events, wifi_event,
				     NET_EVENT_WIFI_CONNECT_RESULT |
					     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_events);
}

static int cmd_ntp_status(const struct shell *sh, size_t argc, char **argv)
{
	struct time_offset snapshot;
	struct time_slew slew_snapshot;
	char address[INET_ADDRSTRLEN] = "?";
	int64_t unix_us = 0, slewed_us = 0, age, mono;
	uint32_t delay = 0;
	bool ready;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_mutex_lock(&sync_lock, K_FOREVER);
	snapshot = estimate;
	slew_snapshot = slewed;
	age = k_uptime_get() - last_sample_uptime;
	mono = time_sync_mono_us();
	ready = estimate_recent() && time_offset_get(&estimate, mono, &unix_us, &delay) &&
		time_slew_get(&slewed, mono, &slewed_us);
	(void)zsock_inet_ntop(AF_INET, &server.sin_addr, address, sizeof(address));
	k_mutex_unlock(&sync_lock);
	shell_print(sh, "NTP server: %s (%s:%u), every %d ms",
		    sizeof(CONFIG_NINA_NTP_SERVER) == 1 ? "default gateway" : CONFIG_NINA_NTP_SERVER,
		    address, CONFIG_NINA_NTP_PORT, CONFIG_NINA_NTP_INTERVAL_MS);
	shell_print(sh, "queries=%d samples=%" PRIu32 " timeouts=%d rejected=%d clock_steps=%d"
		    " last_error=%d", (int)atomic_get(&queries), snapshot.samples,
		    (int)atomic_get(&timeouts), (int)atomic_get(&rejected),
		    (int)atomic_get(&clock_steps), (int)atomic_get(&last_error));
	if (!ready) {
		shell_print(sh, "Time: not synchronized");
		return 0;
	}
	shell_print(sh, "delay_us last=%" PRIu32 " min=%" PRIu32 " used=%" PRIu32
		    " last_residual_us=%" PRId64, snapshot.last_delay_us, snapshot.min_delay_us,
		    delay, snapshot.last_residual_us);
	shell_print(sh, "rate_ppb=%" PRId64 " rate_valid=%d slew_ppb=%" PRId64
		    " slew_steps=%" PRIu32 " slewed_minus_estimate_us=%" PRId64,
		    snapshot.rate_ppb, snapshot.rate_valid, slew_snapshot.now.slew_ppb,
		    slew_snapshot.steps, slewed_us - unix_us);
	shell_print(sh, "Unix time: %" PRId64 ".%06" PRId64 " (last exchange %" PRId64 " ms ago)",
		    slewed_us / USEC_PER_SEC, slewed_us % USEC_PER_SEC, age);
	return 0;
}

/* For evaluating the path: one line per exchange, oldest first. */
static int cmd_ntp_samples(const struct shell *sh, size_t argc, char **argv)
{
	static struct sync_log copy[NTP_LOG_SIZE];
	unsigned int count, first;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_mutex_lock(&sync_lock, K_FOREVER);
	count = log_count;
	first = (log_next + NTP_LOG_SIZE - log_count) % NTP_LOG_SIZE;
	for (unsigned int i = 0; i < count; i++) {
		copy[i] = sync_log[(first + i) % NTP_LOG_SIZE];
	}
	k_mutex_unlock(&sync_lock);
	for (unsigned int i = 0; i < count; i++) {
		shell_print(sh, "S %" PRIu32 " %" PRIu32 " %" PRId32, copy[i].mono_ms,
			    copy[i].delay_us, copy[i].residual_us);
	}
	return 0;
}

/* For checking the estimate over a path other than Wi-Fi: the host notes its
 * own time before sending this command and after reading the answer. */
static int cmd_ntp_now(const struct shell *sh, size_t argc, char **argv)
{
	int64_t unix_us;
	uint32_t delay;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!time_sync_unix_us(time_sync_mono_us(), &unix_us, &delay)) {
		shell_print(sh, "N none");
		return 0;
	}
	shell_print(sh, "N %" PRId64 " %" PRIu32, unix_us, delay);
	return 0;
}

/* Low-latency version of "ntp now" for bounding the error of the estimate
 * over USB. While it runs, the shell hands every received byte straight to
 * this callback: each byte is answered with the estimated time taken on
 * entry, and 'q' or the time limit ends it. The host's send time is then
 * before the board's time and its receive time after, each by one USB
 * transfer plus this thread's scheduling delay. */
static int64_t probe_deadline;

static void probe_bypass(const struct shell *sh, uint8_t *data, size_t len, void *user_data)
{
	int64_t mono = time_sync_mono_us(), unix_us = 0;
	bool ok = time_sync_unix_us(mono, &unix_us, NULL);

	ARG_UNUSED(user_data);
	for (size_t i = 0; i < len; i++) {
		if (data[i] == 'q' || k_uptime_get() >= probe_deadline) {
			shell_set_bypass(sh, NULL, NULL);
			shell_print(sh, "probe done");
			return;
		}
	}
	shell_fprintf(sh, SHELL_NORMAL, "P%" PRId64 "\n", ok ? unix_us : 0);
}

static int cmd_ntp_probe(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long seconds = strtoul(argv[1], NULL, 10);

	ARG_UNUSED(argc);
	if (!seconds || seconds > 7200) {
		shell_error(sh, "Use ntp probe <1..7200 seconds>");
		return -EINVAL;
	}
	probe_deadline = k_uptime_get() + (int64_t)seconds * 1000;
	shell_print(sh, "probe: each byte is answered with P<unix us>; q ends");
	shell_set_bypass(sh, probe_bypass, NULL);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ntp_commands,
			       SHELL_CMD(status, NULL,
					 "Show time synchronization status",
					 cmd_ntp_status),
			       SHELL_CMD(samples, NULL,
					 "List recent exchanges: S <mono ms> <delay us> <residual us>",
					 cmd_ntp_samples),
			       SHELL_CMD(now, NULL,
					 "Print the estimated server time: N <unix us> <delay us>",
					 cmd_ntp_now),
			       SHELL_CMD_ARG(probe, NULL,
					     "Answer each received byte with the estimated time: <seconds>",
					     cmd_ntp_probe, 2, 0),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(ntp, &ntp_commands, "Network time synchronization", NULL);
