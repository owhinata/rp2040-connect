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
 * - The MQTT transport's stub (freertos/drivers/imu/imu_raw_mqtt_null.c,
 *   with common/imu/imu_raw_queue.c and imu_raw_packet.c): the checks of
 *   imu_raw_mqtt_start() (-EINVAL, -EBUSY until the stop), its warning in
 *   the log, the statistics of a session that never connects (the default
 *   gateway as 0.0.0.0, or the configured address), every valid batch
 *   counted as dropped with -EAGAIN and an invalid one refused with -EINVAL
 *   and not counted, counters reset by the next start.
 *
 * The time synchronization (common/imu/time_sync.c) has its own test,
 * freertos/tests/time_sync.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
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

#include "imu_raw_mqtt.h"
#include "imu_raw_packet.h"
#include "priorities.h"
#include "time_sync.h"
#include "zephyr_os_port.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 16384

volatile int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* The board clock of time_sync.h, which the transport's stub reads (the
 * build's is common/imu/time_sync.c, tested by freertos/tests/time_sync):
 * the test port's clock.
 */
int64_t time_sync_mono_us(void)
{
	return (int64_t)nano_mono_us();
}

/* The log (common/log/app_log.h) into a buffer. */
static char log_buf[1024];

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	static const char *const names[] = {"", "err", "wrn", "inf", "dbg"};
	size_t used = strlen(log_buf);

	(void)func;
	used += (size_t)snprintf(log_buf + used, sizeof(log_buf) - used, "<%s> %s: ", names[level],
				 module);
	assert(used < sizeof(log_buf));
	used += (size_t)vsnprintf(log_buf + used, sizeof(log_buf) - used, fmt, ap);
	assert(used + 1 < sizeof(log_buf));
	strcat(log_buf, "\n");
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	app_log_vprintf(level, module, func, fmt, ap);
	va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	(void)last_ms;
	(void)skipped;
	(void)rate_ms;
	*skipped_out = 0;
	return true;
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

/* The MQTT transport's stub ------------------------------------------------*/

static size_t make_batch(uint8_t *payload, size_t capacity)
{
	struct imu_raw_metadata m = {.accel_first_index = 10, .gyro_first_index = 20};
	const struct lsm6dso_fifo_sample samples[] = {
		{LSM6DSO_FIFO_ACCEL, {1, 2, 3}},
		{LSM6DSO_FIFO_GYRO, {4, 5, 6}},
		{LSM6DSO_FIFO_ACCEL, {7, 8, 9}},
	};
	size_t length;

	assert(imu_raw_encode(payload, capacity, &length, &m, samples, 3) == 0);
	return length;
}

static void config_init(struct imu_raw_mqtt_config *cfg, const char *device, uint16_t port)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->port = port;
	strcpy(cfg->device_id, device);
	cfg->session_id[0] = 0x50;
}

static void test_transport(void)
{
	static uint8_t payload[IMU_RAW_MAX_SIZE];
	struct imu_raw_mqtt_config cfg;
	struct imu_raw_mqtt_stats s;
	struct imu_raw_status status = {.state = IMU_RAW_STATE_MEASURING};
	struct imu_raw_info info = {.epoch_id = 1};
	char long_id[IMU_RAW_MQTT_DEVICE_ID_MAX + 1];
	size_t length = make_batch(payload, sizeof(payload));

	/* Refused: no checks pass, nothing logged. */
	config_init(&cfg, "rp2040", 0);
	assert(imu_raw_mqtt_start(&cfg) == -EINVAL);
	config_init(&cfg, "", 1883);
	assert(imu_raw_mqtt_start(&cfg) == -EINVAL);
	config_init(&cfg, "imu/x", 1883);
	assert(imu_raw_mqtt_start(&cfg) == -EINVAL);
	memset(long_id, 'a', sizeof(long_id) - 1);
	long_id[sizeof(long_id) - 1] = '\0';
	config_init(&cfg, "", 1883);
	memcpy(cfg.device_id, long_id, sizeof(long_id));
	assert(imu_raw_mqtt_start(&cfg) == 0); /* 40 characters: the maximum */
	imu_raw_mqtt_stop();
	assert(strstr(log_buf, "<wrn> imu_raw_mqtt: No MQTT transport in this build (stub)") !=
	       NULL);
	log_buf[0] = '\0';

	/* A session to the default gateway (imu.host unset), as "imu start". */
	config_init(&cfg, "rp2040", 18830);
	assert(imu_raw_mqtt_start(&cfg) == 0);
	assert(strcmp(log_buf, "<wrn> imu_raw_mqtt: No MQTT transport in this build (stub): "
			       "batches are counted as dropped\n") == 0);
	assert(imu_raw_mqtt_start(&cfg) == -EBUSY); /* until the stop */
	imu_raw_mqtt_get_stats(&s);
	assert(s.running && !s.connected && s.via_gateway && s.address.s_addr == 0 &&
	       s.port == 18830);
	assert(s.published == 0 && s.failures == 0 && s.connects == 0 && s.queued == 0 &&
	       s.dropped_batches == 0 && s.lock_wait_count == 0 && s.publish_max_us == 0);

	assert(imu_raw_mqtt_submit(NULL, payload, length) == -EAGAIN);
	assert(imu_raw_mqtt_submit(NULL, payload, length) == -EAGAIN);
	assert(imu_raw_mqtt_submit(NULL, payload, length - 1) == -EINVAL);
	imu_raw_mqtt_set_info(&info);
	imu_raw_mqtt_report(&status);
	imu_raw_mqtt_get_stats(&s);
	assert(s.dropped_batches == 2 && s.dropped_accel == 4 && s.dropped_gyro == 2);
	assert(s.queued == 0 && s.queue_high == 0 && s.lock_wait_count == 3 && s.published == 0);

	imu_raw_mqtt_stop();
	imu_raw_mqtt_get_stats(&s);
	assert(!s.running && !s.connected && s.dropped_batches == 2);

	/* A fixed server (imu.host): its address is kept; counters restart. */
	config_init(&cfg, "board-2_b", 1883);
	assert(zsock_inet_pton(AF_INET, "10.42.0.1", &cfg.address) == 1);
	assert(imu_raw_mqtt_start(&cfg) == 0);
	imu_raw_mqtt_get_stats(&s);
	assert(s.running && !s.via_gateway && s.address.s_addr == cfg.address.s_addr &&
	       s.port == 1883 && s.dropped_batches == 0 && s.lock_wait_count == 0);
	imu_raw_mqtt_stop();
	printf("imu_raw_mqtt stub: ok\n");
}

static void runner(void *arg)
{
	(void)arg;
	test_inet();
	test_hwinfo();
	test_transport();
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
	/* The shell's priority: "imu start" runs in the console's task. */
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_SHELL,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
