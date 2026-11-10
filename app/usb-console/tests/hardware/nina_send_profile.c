/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include "../../drivers/nina/nina_send_profile.h"
#include "../../drivers/nina/nina_proto.h"

struct totals {
	uint64_t lock_us, pre_us, tx_us, ready_us, reply_us, retry_sleep_us;
	uint64_t requested, accepted;
	uint32_t calls, zero, partial, errors, retry_sleeps;
	uint32_t errors_by_phase[6];
	int last_error;
};
static struct totals stats;
static struct k_spinlock stats_lock;

void nina_send_profile_record(const struct nina_send_sample *s)
{
	k_spinlock_key_t key = k_spin_lock(&stats_lock);

	stats.calls++;
	stats.lock_us += k_cyc_to_us_floor64(s->lock_cycles);
	stats.pre_us += k_cyc_to_us_floor64(s->pre_cycles);
	stats.tx_us += k_cyc_to_us_floor64(s->tx_cycles);
	stats.ready_us += k_cyc_to_us_floor64(s->ready_cycles);
	stats.reply_us += k_cyc_to_us_floor64(s->reply_cycles);
	stats.requested += s->requested;
	stats.accepted += s->accepted;
	if (s->error) {
		stats.errors++;
		if (s->phase >= 0 && (size_t)s->phase < ARRAY_SIZE(stats.errors_by_phase)) {
			stats.errors_by_phase[s->phase]++;
		}
		stats.last_error = s->error;
	} else if (!s->accepted) {
		stats.zero++;
	} else if (s->accepted < s->requested) {
		stats.partial++;
	}
	k_spin_unlock(&stats_lock, key);
}

void nina_send_profile_retry(uint32_t cycles)
{
	k_spinlock_key_t key = k_spin_lock(&stats_lock);

	stats.retry_sleeps++;
	stats.retry_sleep_us += k_cyc_to_us_floor64(cycles);
	k_spin_unlock(&stats_lock, key);
}

static int cmd_send_profile(const struct shell *sh, size_t argc, char **argv)
{
	struct totals copy;
	k_spinlock_key_t key = k_spin_lock(&stats_lock);

	if (argc > 1 && !strcmp(argv[1], "reset")) {
		memset(&stats, 0, sizeof(stats));
	}
	copy = stats;
	k_spin_unlock(&stats_lock, key);
	shell_print(sh, "calls=%u zero=%u partial=%u errors=%u retry_sleeps=%u",
		    copy.calls, copy.zero, copy.partial, copy.errors, copy.retry_sleeps);
	shell_print(sh, "requested=%" PRIu64 " accepted=%" PRIu64,
		    copy.requested, copy.accepted);
	shell_print(sh, "error_phase pre=%u tx=%u ready=%u reply_ready=%u reply_spi=%u last=%d",
		    copy.errors_by_phase[1], copy.errors_by_phase[2],
		    copy.errors_by_phase[3], copy.errors_by_phase[4],
		    copy.errors_by_phase[5], copy.last_error);
	shell_print(sh, "us lock=%" PRIu64 " pre=%" PRIu64 " tx=%" PRIu64
		    " ready=%" PRIu64 " reply=%" PRIu64 " retry_sleep=%" PRIu64,
		    copy.lock_us, copy.pre_us, copy.tx_us, copy.ready_us,
		    copy.reply_us, copy.retry_sleep_us);
	return 0;
}
SHELL_CMD_REGISTER(nina_send_profile, NULL, "Show/reset TCP send timing", cmd_send_profile);

extern const k_tid_t nina_monitor_tid;
static int cmd_nina_monitor(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) {
		shell_error(sh, "Use nina_benchmark_monitor on|off");
		return -EINVAL;
	}
	if (!strcmp(argv[1], "off")) {
		k_thread_suspend(nina_monitor_tid);
	} else {
		k_thread_resume(nina_monitor_tid);
	}
	shell_print(sh, "NINA monitor %s", argv[1]);
	return 0;
}
SHELL_CMD_REGISTER(nina_benchmark_monitor, NULL, "Pause/resume NINA status polling",
		   cmd_nina_monitor);

static int cmd_nina_retry(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	uint8_t value, reply;
	size_t len;
	unsigned long n;
	int ret;

	if (argc != 2) {
		return -EINVAL;
	}
	n = strtoul(argv[1], &end, 10);
	if (*end || n > 10) {
		return -EINVAL;
	}
	value = n;
	struct nina_param p = {&value, 1};
	ret = nina_request(0x42, &p, 1, false, false, &reply, 1, &len);
	if (ret || len != 1 || reply != value) {
		return ret ? ret : -EIO;
	}
	shell_print(sh, "NINA retries=%u", value);
	return 0;
}
SHELL_CMD_REGISTER(nina_benchmark_retry, NULL, "Set experimental NINA retry count",
		   cmd_nina_retry);
