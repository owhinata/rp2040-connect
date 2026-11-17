/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's parts of the IMU service on the
 * FreeRTOS POSIX port, as freertos/drivers/imu/imu.cmake builds them:
 *
 * - inet_pton() and inet_ntop() (freertos/drivers/libc/inet.c), Zephyr's
 *   zsock_inet_pton() and zsock_inet_ntop() of the layer's
 *   <zephyr/net/socket.h>: dotted decimal IPv4 addresses only, POSIX's
 *   return values and errno.
 * - hwinfo_get_device_id() (freertos/drivers/compat/rp2040/zephyr_hwinfo.c)
 *   on a fake of pico_get_unique_board_id(): the 8 bytes unchanged, cut to
 *   the caller's length.
 *
 * The time synchronization (common/imu/time_sync.c) and the MQTT transport
 * (common/imu/imu_raw_mqtt.c) have their own tests, freertos/tests/time_sync
 * and freertos/tests/mqtt.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/net/socket.h>

#include "pico/unique_id.h"

#include "priorities.h"
#include "zephyr_os_port.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 16384

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* pico_unique_id's copy of the flash's unique ID. */
static pico_unique_board_id_t board_id;

void pico_get_unique_board_id(pico_unique_board_id_t *id_out)
{
	*id_out = board_id;
}

/* inet_pton(), inet_ntop() -------------------------------------------------*/

static void pton_ok(const char *text, uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	struct in_addr addr = {.s_addr = 0xdeadbeefu};
	const uint8_t *bytes = (const uint8_t *)&addr;

	assert(zsock_inet_pton(AF_INET, text, &addr) == 1);
	assert(bytes[0] == a && bytes[1] == b && bytes[2] == c && bytes[3] == d);
}

static void pton_bad(const char *text)
{
	struct in_addr addr = {.s_addr = 0xdeadbeefu};

	assert(zsock_inet_pton(AF_INET, text, &addr) == 0);
	assert(addr.s_addr == 0xdeadbeefu);
}

static void test_inet(void)
{
	static const char *const bad[] = {
		"",          "1.2.3",      "1.2.3.4.",   "1.2.3.4.5", "1.2.3.256",
		"256.1.1.1", "1.2.3.4 ",   " 1.2.3.4",   "a.b.c.d",   "1..2.3",
		".1.2.3",    "0001.2.3.4", "-1.2.3.4",   "+1.2.3.4",  "1.2.3.4x",
		"1.2.3.",    "1,2,3,4",    "1.2.3.1000", "localhost",
	};
	struct in_addr addr;
	char text[INET_ADDRSTRLEN];

	pton_ok("10.42.0.1", 10, 42, 0, 1);
	pton_ok("0.0.0.0", 0, 0, 0, 0);
	pton_ok("255.255.255.255", 255, 255, 255, 255);
	pton_ok("192.168.001.010", 192, 168, 1, 10);
	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		pton_bad(bad[i]);
	}
	errno = 0;
	assert(zsock_inet_pton(AF_UNSPEC, "1.2.3.4", &addr) == -1 && errno == EAFNOSUPPORT);
	assert(zsock_inet_pton(10 /* AF_INET6 elsewhere */, "::1", &addr) == -1);

	/* Network order: the first byte is the first number. */
	addr.s_addr = htonl(0x0a2a0061u);
	assert(zsock_inet_ntop(AF_INET, &addr, text, sizeof(text)) == text);
	assert(strcmp(text, "10.42.0.97") == 0);
	addr.s_addr = 0xffffffffu;
	assert(zsock_inet_ntop(AF_INET, &addr, text, 16) == text);
	assert(strcmp(text, "255.255.255.255") == 0);
	memset(text, 'x', sizeof(text));
	errno = 0;
	assert(zsock_inet_ntop(AF_INET, &addr, text, 15) == NULL && errno == ENOSPC);
	assert(text[0] == 'x');
	addr.s_addr = 0;
	assert(zsock_inet_ntop(AF_INET, &addr, text, 8) == text && strcmp(text, "0.0.0.0") == 0);
	assert(zsock_inet_ntop(AF_INET, &addr, text, 7) == NULL);
	errno = 0;
	assert(zsock_inet_ntop(AF_UNSPEC, &addr, text, sizeof(text)) == NULL &&
	       errno == EAFNOSUPPORT);

	assert(htons(0x1234) == 0x3412 && ntohs(0x3412) == 0x1234);
	assert(htonl(0x12345678u) == 0x78563412u && ntohl(0x78563412u) == 0x12345678u);
	printf("inet: ok\n");
}

/* hwinfo_get_device_id() --------------------------------------------------*/

static void test_hwinfo(void)
{
	/* The board's ID (the USB serial number 503533748A8C9218). */
	static const uint8_t id[8] = {0x50, 0x35, 0x33, 0x74, 0x8a, 0x8c, 0x92, 0x18};
	uint8_t buf[16];

	memcpy(board_id.id, id, sizeof(id));
	memset(buf, 0xa5, sizeof(buf));
	assert(hwinfo_get_device_id(buf, sizeof(buf)) == 8);
	assert(memcmp(buf, id, 8) == 0);
	for (size_t i = 8; i < sizeof(buf); i++) {
		assert(buf[i] == 0xa5);
	}
	memset(buf, 0xa5, sizeof(buf));
	assert(hwinfo_get_device_id(buf, 8) == 8 && memcmp(buf, id, 8) == 0 && buf[8] == 0xa5);
	memset(buf, 0xa5, sizeof(buf));
	assert(hwinfo_get_device_id(buf, 3) == 3 && memcmp(buf, id, 3) == 0 && buf[3] == 0xa5);
	assert(hwinfo_get_device_id(buf, 0) == 0 && buf[0] == 0x50);
	printf("hwinfo: ok\n");
}

static void runner(void *arg)
{
	(void)arg;
	test_inet();
	test_hwinfo();
	fflush(stdout);
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	/* The shell's priority: the commands run in the console's task. */
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_SHELL,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
