/*
 * Copyright (c) 2017-2018 Linaro Limited
 * Copyright (c) 2021 Nordic Semiconductor
 * Copyright (c) 2023 Arm Limited (or its affiliates). All rights reserved.
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Zephyr's lib/os/zvfs/zvfs_poll.c (Zephyr v4.5.0): zvfs_poll(), which
 * zsock_poll() is, on the layer's descriptor table (zvfs_fdtable.c here).
 * zvfs_poll_internal() is Zephyr's text: each descriptor's POLL_PREPARE
 * under its lock; an offloaded socket answers -EXDEV and the whole call goes
 * to its implementation's POLL_OFFLOAD (the NINA's sockets,
 * common/drivers/nina/port/zephyr/nina_socket.c); otherwise the events that
 * POLL_PREPARE gave wait in k_poll() (the layer's: semaphores) and each
 * descriptor's POLL_UPDATE gives its revents.  A descriptor that is not
 * open is POLLNVAL; without an offloaded one, the call first waits for the
 * events or the timeout as Zephyr's does (with no event, the whole timeout).
 *
 * Left out: TLS (net_socket_is_tls() is false: no TLS sockets) and the user
 * mode.  The timepoints are the few lines of Zephyr's <zephyr/sys/clock.h>
 * that the function needs, on the layer's tick (k_uptime_ticks() below);
 * CONFIG_ZVFS_POLL_MAX is the Zephyr build's (16).
 * Part of the network part of the Zephyr compatibility layer for an OS
 * without a network stack of its own (common/compat/net_offload), which the
 * FreeRTOS build uses.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/fdtable.h>

#ifndef CONFIG_ZVFS_POLL_MAX
#define CONFIG_ZVFS_POLL_MAX 16
#endif

#define net_socket_is_tls(obj) false

/* <zephyr/sys/clock.h>: the timepoints of a timeout, in ticks. */
#define SYS_FOREVER_MS (-1)

typedef struct {
	int64_t tick;
} k_timepoint_t;

/* The tick count since boot, as the layer's timeouts count it (the TIMER in
 * microseconds over the tick, as k_uptime_get()).
 */
static int64_t k_uptime_ticks(void)
{
	return (int64_t)(k_cycle_get_64() / Z_USEC_PER_TICK);
}

static k_timepoint_t sys_timepoint_calc(k_timeout_t timeout)
{
	k_timepoint_t timepoint;

	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		timepoint.tick = INT64_MAX;
	} else if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		timepoint.tick = 0;
	} else {
		timepoint.tick = k_uptime_ticks() + MAX(timeout.ticks, 0);
	}
	return timepoint;
}

static k_timeout_t sys_timepoint_timeout(k_timepoint_t timepoint)
{
	int64_t remaining;

	if (timepoint.tick == INT64_MAX) {
		return K_FOREVER;
	}
	if (timepoint.tick == 0) {
		return K_NO_WAIT;
	}
	remaining = timepoint.tick - k_uptime_ticks();
	return remaining > 0 ? K_TICKS(remaining) : K_NO_WAIT;
}

static int32_t k_ticks_to_ms_floor32(int64_t ticks)
{
	return (int32_t)(ticks * Z_USEC_PER_TICK / 1000);
}

int zvfs_poll_internal(struct zvfs_pollfd *fds, int nfds, k_timeout_t timeout)
{
	bool retry;
	int ret = 0;
	int i;
	struct zvfs_pollfd *pfd;
	struct k_poll_event poll_events[CONFIG_ZVFS_POLL_MAX];
	struct k_poll_event *pev;
	struct k_poll_event *pev_end = poll_events + ARRAY_SIZE(poll_events);
	const struct fd_op_vtable *vtable;
	struct k_mutex *lock;
	k_timepoint_t end;
	bool offload = false;
	const struct fd_op_vtable *offl_vtable = NULL;
	void *offl_ctx = NULL;

	end = sys_timepoint_calc(timeout);

	pev = poll_events;
	for (pfd = fds, i = nfds; i--; pfd++) {
		void *ctx;
		int result;

		/* Per POSIX, negative fd's are just ignored */
		if (pfd->fd < 0) {
			continue;
		}

		ctx = zvfs_get_fd_obj_and_vtable(pfd->fd, &vtable, &lock);
		if (ctx == NULL) {
			/* Will set POLLNVAL in return loop */
			continue;
		}

		(void)k_mutex_lock(lock, K_FOREVER);

		result = zvfs_fdtable_call_ioctl(vtable, ctx, ZFD_IOCTL_POLL_PREPARE, pfd, &pev,
						 pev_end);
		if (result == -EALREADY) {
			/* If POLL_PREPARE returned with EALREADY, it means
			 * it already detected that some socket is ready. In
			 * this case, we still perform a k_poll to pick up
			 * as many events as possible, but without any wait.
			 */
			timeout = K_NO_WAIT;
			end = sys_timepoint_calc(timeout);
			result = 0;
		} else if (result == -EXDEV) {
			/* If POLL_PREPARE returned EXDEV, it means
			 * it detected an offloaded socket.
			 * If offloaded socket is used with native TLS, the TLS
			 * wrapper for the offloaded poll will be used.
			 * In case the fds array contains a mixup of offloaded
			 * and non-offloaded sockets, the offloaded poll handler
			 * shall return an error.
			 */
			offload = true;
			if (offl_vtable == NULL || net_socket_is_tls(ctx)) {
				offl_vtable = vtable;
				offl_ctx = ctx;
			}

			result = 0;
		}

		k_mutex_unlock(lock);

		if (result < 0) {
			errno = -result;
			return -1;
		}
	}

	if (offload) {
		int poll_timeout;

		if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
			poll_timeout = SYS_FOREVER_MS;
		} else {
			poll_timeout = k_ticks_to_ms_floor32(timeout.ticks);
		}

		return zvfs_fdtable_call_ioctl(offl_vtable, offl_ctx, ZFD_IOCTL_POLL_OFFLOAD, fds,
					       nfds, poll_timeout);
	}

	timeout = sys_timepoint_timeout(end);

	do {
		ret = k_poll(poll_events, pev - poll_events, timeout);
		/* EAGAIN when timeout expired, EINTR when cancelled (i.e. EOF) */
		if (ret != 0 && ret != -EAGAIN && ret != -EINTR) {
			errno = -ret;
			return -1;
		}

		retry = false;
		ret = 0;

		pev = poll_events;
		for (pfd = fds, i = nfds; i--; pfd++) {
			void *ctx;
			int result;

			pfd->revents = 0;

			if (pfd->fd < 0) {
				continue;
			}

			ctx = zvfs_get_fd_obj_and_vtable(pfd->fd, &vtable, &lock);
			if (ctx == NULL) {
				pfd->revents = ZVFS_POLLNVAL;
				ret++;
				continue;
			}

			(void)k_mutex_lock(lock, K_FOREVER);

			result = zvfs_fdtable_call_ioctl(vtable, ctx, ZFD_IOCTL_POLL_UPDATE, pfd,
							 &pev);
			k_mutex_unlock(lock);

			if (result == -EAGAIN) {
				retry = true;
				continue;
			} else if (result != 0) {
				errno = -result;
				return -1;
			}

			if (pfd->revents != 0) {
				ret++;
			}
		}

		if (retry) {
			if (ret > 0) {
				break;
			}

			timeout = sys_timepoint_timeout(end);

			if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
				break;
			}
		}
	} while (retry);

	return ret;
}

int zvfs_poll(struct zvfs_pollfd *fds, int nfds, int poll_timeout)
{
	k_timeout_t timeout;

	if (poll_timeout < 0) {
		timeout = K_FOREVER;
	} else {
		timeout = K_MSEC(poll_timeout);
	}

	return zvfs_poll_internal(fds, nfds, timeout);
}
