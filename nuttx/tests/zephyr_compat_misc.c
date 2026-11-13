/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the parts of the Zephyr compatibility layer
 * (nuttx/drivers/compat) that the IMU service (nuttx/apps/imu/imu_stream.c)
 * added, against Zephyr's definitions:
 *
 * - K_MUTEX_DEFINE(): a usable, unlocked mutex.
 * - K_THREAD_DEFINE(): the thread's name is the token ("imu_stream_tid"),
 *   the priority K_PRIO_PREEMPT(9) = 109, the stack 2048 bytes, the entry
 *   and its arguments as given; K_THREAD_DEFINE_START() starts it through
 *   z_thread_start_<name>() and sets the k_tid_t.
 * - Z_UPTIME_MS() (k_uptime_get()): Zephyr's k_ticks_to_ms_floor64() of
 *   the 100 us tick count, i.e. floor(floor(us / 100) * 100 / 1000), for
 *   values around every carry and across the 32-bit tick wrap.
 * - k_sleep(): K_MSEC(1) waits 10 + 1 ticks (fake kernel, documents the
 *   rounding the NuttX version gets from nxsched_ticksleep()).
 * - sys_put_be32()/sys_get_be32(): big-endian bytes at any alignment.
 * - snprintk(): "%02x" as in the session ID print.
 * - hwinfo_get_device_id(): the board's chip ID bytes in order, at most 8,
 *   fewer when asked, the error of nano_chip_id() passed on.
 * - shell_print()/shell_error(): stdout and stderr with "\n".
 * - zsock_inet_pton()/zsock_inet_ntop(): 1 and the dotted text.
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>

#include "zephyr_compat.h"
#include "zephyr_kernel_host.h"

/* The board's chip ID (nano_chipid.c) for zephyr_hwinfo.c. */
static const uint8_t chip_id[8] = {0x50, 0x35, 0x33, 0x74, 0x8a, 0x8c, 0x92, 0x18};
static int chip_id_result;

int nano_chip_id(uint8_t id[8])
{
	if (chip_id_result < 0) {
		return chip_id_result;
	}
	memcpy(id, chip_id, sizeof(chip_id));
	return 0;
}

K_MUTEX_DEFINE(test_lock);

static int entry_calls;
static void *entry_args[3];

static void test_entry(void *a, void *b, void *c)
{
	entry_calls++;
	entry_args[0] = a;
	entry_args[1] = b;
	entry_args[2] = c;
}

static int marker_a, marker_b;

K_THREAD_DEFINE(imu_stream_tid, 2048, test_entry, &marker_a, NULL, &marker_b, 9, 0, 0);

static void test_mutex(void)
{
	assert(test_lock.depth == 0);
	assert(k_mutex_lock(&test_lock, K_FOREVER) == 0);
	assert(k_mutex_unlock(&test_lock) == 0);
}

static void test_thread_define(void)
{
	const struct z_thread_static *t;

	assert(test_thread(0) == NULL);
	assert(K_THREAD_DEFINE_START(imu_stream_tid) == 0);
	t = test_thread(0);
	assert(t != NULL);
	assert(strcmp(t->name, "imu_stream_tid") == 0);
	assert(t->prio == 109);
	assert(t->stack_size == 2048);
	assert(t->stack != NULL);
	assert(t->tid == &imu_stream_tid && imu_stream_tid == 200);
	t->entry(t->p1, t->p2, t->p3);
	assert(entry_calls == 1);
	assert(entry_args[0] == &marker_a && entry_args[1] == NULL && entry_args[2] == &marker_b);
}

/* k_ticks_to_ms_floor64() for 10000 ticks per second. */
static int64_t ticks_to_ms_floor(uint64_t ticks)
{
	return (int64_t)(ticks * 100U / 1000U);
}

static void test_uptime(void)
{
	static const uint64_t bases[] = {
		0, 99, 100, 999, 1000, 1099, 123456789, (uint64_t)UINT32_MAX * 100U,
		(uint64_t)UINT32_MAX * 100U + 1000U, 1ULL << 40, (1ULL << 52) + 12345U,
	};

	for (size_t i = 0; i < sizeof(bases) / sizeof(bases[0]); i++) {
		for (uint64_t d = 0; d < 3000; d++) {
			uint64_t us = bases[i] + d;

			assert(Z_UPTIME_MS(us) == ticks_to_ms_floor(us / 100U));
		}
	}
	test_time_set_us(4321999);
	assert(k_uptime_get() == 4321);
	test_time_set_us(4322000);
	assert(k_uptime_get() == 4322);
}

static void test_sleep(void)
{
	assert(K_MSEC(1).ticks == 10);
	assert(K_MSEC(20).ticks == 200);
	test_time_set_us(1000000);
	assert(k_sleep(K_MSEC(1)) == 0);
	assert(test_slept_ticks() == 11);
	assert(k_uptime_get() == 1001);
}

static void test_byteorder(void)
{
	uint8_t buf[6] = {0};

	sys_put_be32(0x0000004fU, buf + 1);
	assert(buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0 && buf[4] == 0x4f &&
	       buf[5] == 0);
	assert(sys_get_be32(buf + 1) == 0x4fU);
	sys_put_be32(0x12345678U, buf + 1);
	assert(buf[1] == 0x12 && buf[2] == 0x34 && buf[3] == 0x56 && buf[4] == 0x78);
	assert(sys_get_be32(buf + 1) == 0x12345678U);
	sys_put_be32(0xffffffffU, buf);
	assert(sys_get_be32(buf) == 0xffffffffU);
}

static void test_snprintk(void)
{
	char text[33];
	uint8_t session[16] = {0x50, 0x35, 0x33, 0x74, 0x8a, 0x8c, 0x92, 0x18, 0, 0, 0, 0x4f};

	for (size_t i = 0; i < 16; i++) {
		snprintk(text + i * 2, 3, "%02x", session[i]);
	}
	assert(strcmp(text, "503533748a8c92180000004f00000000") == 0);
}

static void test_hwinfo(void)
{
	uint8_t id[16];

	memset(id, 0xee, sizeof(id));
	chip_id_result = 0;
	assert(hwinfo_get_device_id(id, sizeof(id)) == 8);
	assert(memcmp(id, chip_id, 8) == 0 && id[8] == 0xee);
	memset(id, 0xee, sizeof(id));
	assert(hwinfo_get_device_id(id, 3) == 3);
	assert(memcmp(id, chip_id, 3) == 0 && id[3] == 0xee);
	chip_id_result = -EAGAIN;
	assert(hwinfo_get_device_id(id, 8) == -EAGAIN);
}

/* Runs fn with stdout and stderr captured; returns them. */
static void capture(void (*fn)(void), char *out, size_t out_size, char *err, size_t err_size)
{
	FILE *o = tmpfile(), *e = tmpfile();
	int saved_out = dup(1), saved_err = dup(2);
	size_t n;

	assert(o && e && saved_out >= 0 && saved_err >= 0);
	fflush(stdout);
	fflush(stderr);
	dup2(fileno(o), 1);
	dup2(fileno(e), 2);
	fn();
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	rewind(o);
	rewind(e);
	n = fread(out, 1, out_size - 1, o);
	out[n] = '\0';
	n = fread(err, 1, err_size - 1, e);
	err[n] = '\0';
	fclose(o);
	fclose(e);
}

static void print_some(void)
{
	const struct shell *sh = NULL;

	shell_print(sh, "Not running");
	shell_print(sh, "  %-23s %d", "stream:", -5);
	shell_error(sh, "Sensor setup failed (%d)", -19);
}

static void test_shell(void)
{
	char out[128], err[128];

	capture(print_some, out, sizeof(out), err, sizeof(err));
	assert(strcmp(out, "Not running\n  stream:                 -5\n") == 0);
	assert(strcmp(err, "Sensor setup failed (-19)\n") == 0);
}

static void test_inet(void)
{
	struct in_addr address = {0};
	char text[32];

	assert(zsock_inet_pton(AF_INET, "10.42.0.1", &address) == 1);
	assert(address.s_addr == htonl(0x0a2a0001U));
	assert(zsock_inet_pton(AF_INET, "10.42.0", &address) != 1);
	address.s_addr = htonl(0xc0a80102U);
	assert(zsock_inet_ntop(AF_INET, &address, text, sizeof(text)) == text);
	assert(strcmp(text, "192.168.1.2") == 0);
}

int main(void)
{
	test_mutex();
	test_thread_define();
	test_uptime();
	test_sleep();
	test_byteorder();
	test_snprintk();
	test_hwinfo();
	test_shell();
	test_inet();
	puts("zephyr_compat_misc: ok");
	return 0;
}
