/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's "sensor" command
 * (freertos/drivers/sensor_shell: the shared common/drivers/sensor_shell/
 * sensor_shell.c, the copy of Zephyr's sensor_shell_stream.c with its
 * processing thread, and the FreeRTOS part sensor_shell_os.c) and of the
 * calibration it shows ("imu cal": the shared common/imu/imu_cal.c), on the
 * FreeRTOS POSIX port with the build's shell (freertos/drivers/shell, two
 * sessions on two consoles that capture), the Zephyr compatibility layer
 * (its kernel part, work queues, RTIO with its work queue, devices, gpio0,
 * the default sensor iodev and decoder) and the shared LSM6DSO driver with
 * its FIFO stream on ST's lsm6dso_reg.c, as freertos/drivers/lsm6dso/
 * lsm6dso.cmake builds them.  The sensor is a fake LSM6DSOX behind a fake
 * i2c0 (register banks, auto-increment, software reset, FIFO, timestamp, the
 * same model as the NuttX build's test, nuttx/tests/lsm6dso_driver.c) with a
 * fake INT1 on the layer's GPIO boundary (nano_gpio.h).
 *
 * - "sensor get" from either session: the processing thread prints the
 *   result on the console of the session that ran it and on no other, and
 *   the command returns to the prompt after the result (the wait of
 *   sensor_shell_os.c), in the Zephyr build's format; the node name and the
 *   node label.
 * - the calibration: the conversion of the stored values (a fake key-value
 *   store with the values of imu.cal.accel.244 and imu.cal.gyro.70000)
 *   after "imu cal reload", the gyro bias taken at rest by "imu cal zero",
 *   both in the values that "sensor get" prints and in "imu cal".  "imu"
 *   here has only "cal": the test's stand-in for common/imu/imu_stream.c's
 *   command, which calls the same imu_cal_shell().
 * - "sensor stream ... on fifo_wm incl" from one session: the batches on
 *   that session's console with their trigger line; "sensor get" from the
 *   other meanwhile ("Read failed: -16", the FIFO is the stream's); the
 *   first session's terminal gone (its text dropped) while the processing
 *   thread goes on (the other session's commands still work); "off" from
 *   the other session ("Disabling existing stream"), the stream ends.
 * - a stream of another RTIO context (as "imu start" makes one,
 *   common/imu/imu_stream.c): "sensor get", "sensor stream" and "sensor
 *   attr_set" get -EBUSY; during "sensor stream" the driver reports the FIFO
 *   active, which "imu start" refuses ("The IMU FIFO is in use").
 * - "sensor info": no such subcommand (CONFIG_SENSOR_INFO is not set), the
 *   help of "sensor".
 *
 * Built and run by freertos/script/host_tests.sh (skipped without
 * freertos/stmemsc/lsm6dso_reg.c).
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/lsm6dso.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/shell/shell.h>

#include "cli.h"
#include "imu_cal.h"
#include "kv_store.h"
#include "lsm6dso.h"
#include "nano_console.h"
#include "nano_gpio.h"
#include "nano_shell.h"
#include "pico/stdio.h"
#include "priorities.h"
#include "zephyr_compat.h"

#define SESSION_STACK 16384
#define CAPTURE_SIZE  16384

_Thread_local int z_host_isr_depth;

/* The RAM log (app_log_vprintf() below), printed when a check fails. */
static char log_text[8192];

static void fail_sessions(void);

/* Straight to the host's stderr: stdio is the consoles' here (--wrap). */
static void fail(const char *fmt, ...)
{
	static char text[CAPTURE_SIZE + 512];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(text, sizeof(text) - 1, fmt, ap);
	va_end(ap);
	if (n < 0) {
		n = 0;
	} else if (n > (int)sizeof(text) - 2) {
		n = (int)sizeof(text) - 2;
	}
	text[n++] = '\n';
	(void)!write(2, text, (size_t)n);
	(void)!write(2, log_text, strlen(log_text));
	fail_sessions();
	_exit(1);
}

#define CHECK(cond)                                                                                \
	do {                                                                                       \
		if (!(cond)) {                                                                     \
			fail("%s:%d: %s", __FILE__, __LINE__, #cond);                              \
		}                                                                                  \
	} while (0)

/* tick.c's vApplicationTickHook() (time slices and zephyr_compat_tick()),
 * counted as an interrupt handler.
 */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* The allocations of zshell.c and nano_console.c, with the scheduler
 * suspended (as in freertos/tests/shell).
 */
void *test_calloc(size_t n, size_t size);
void *test_malloc(size_t size);
void test_free(void *p);

void *test_calloc(size_t n, size_t size)
{
	void *p;

	vTaskSuspendAll();
	p = calloc(n, size);
	(void)xTaskResumeAll();
	return p;
}

void *test_malloc(size_t size)
{
	void *p;

	vTaskSuspendAll();
	p = malloc(size);
	(void)xTaskResumeAll();
	return p;
}

void test_free(void *p)
{
	vTaskSuspendAll();
	free(p);
	(void)xTaskResumeAll();
}

/* The Pico SDK's stdio, as pico_stdio (as in freertos/tests/shell) -------*/

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation)
{
	struct nano_console *con = nano_console_current();

	(void)cr_translation;
	if (len < 0) {
		len = (int)strlen(s);
	}
	nano_console_write(con, s, (size_t)len);
	if (newline) {
		nano_console_write(con, "\n", 1);
	}
	return len;
}

int __wrap_vprintf(const char *fmt, va_list args);
int __wrap_printf(const char *fmt, ...);
int __wrap_puts(const char *s);
int __wrap_putchar(int c);

int __wrap_vprintf(const char *fmt, va_list args)
{
	char buf[512];
	int n = vsnprintf(buf, sizeof(buf), fmt, args);

	if (n > 0) {
		stdio_put_string(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1, false, true);
	}
	return n;
}

int __wrap_printf(const char *fmt, ...)
{
	va_list args;
	int n;

	va_start(args, fmt);
	n = __wrap_vprintf(fmt, args);
	va_end(args);
	return n;
}

int __wrap_puts(const char *s)
{
	return stdio_put_string(s, -1, true, true);
}

int __wrap_putchar(int c)
{
	char ch = (char)c;

	stdio_put_string(&ch, 1, false, true);
	return (unsigned char)ch;
}

/* The RAM log: kept, for the driver's lines ------------------------------*/

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	static const char *const names[] = {"", "err", "wrn", "inf", "dbg"};
	char line[256];
	int n;

	(void)func;
	n = snprintf(line, sizeof(line), "<%s> %s: ", names[level & 3 ? level : 0], module);
	vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, ap);
	vTaskSuspendAll();
	if (strlen(log_text) + strlen(line) + 2 < sizeof(log_text)) {
		strcat(log_text, line);
		strcat(log_text, "\n");
	}
	(void)xTaskResumeAll();
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

/* The sensor clock (freertos/drivers/compat/zephyr_sensor_clock.c on the
 * board): the test port's clock.
 */
int sensor_clock_get_cycles(uint64_t *cycles)
{
	*cycles = nano_mono_us();
	return 0;
}

uint64_t sensor_clock_cycles_to_ns(uint64_t cycles)
{
	return cycles * 1000U;
}

/* A fake key-value store: the calibration values (common/imu/kv_store.h) */

#define KV_MAX 4

static struct {
	char key[KV_STORE_KEY_MAX];
	uint8_t value[IMU_CAL_BLOB_SIZE];
	size_t len;
} kv[KV_MAX];

ssize_t kv_store_get(const char *key, void *value, size_t capacity)
{
	for (int i = 0; i < KV_MAX; i++) {
		if (strcmp(kv[i].key, key) == 0) {
			if (kv[i].len > capacity) {
				return -ENOSPC;
			}
			memcpy(value, kv[i].value, kv[i].len);
			return (ssize_t)kv[i].len;
		}
	}
	return -ENOENT;
}

static void put_le32(uint8_t *p, uint32_t v)
{
	for (int i = 0; i < 4; i++) {
		p[i] = (uint8_t)(v >> (8 * i));
	}
}

/* Stores a calibration as host/imu_host writes it (common/imu/imu_cal.h). */
static void kv_put_cal(enum lsm6dso_cal_kind kind, uint32_t gain, const struct lsm6dso_cal *cal)
{
	int i;

	for (i = 0; i < KV_MAX && kv[i].key[0] != '\0'; i++) {
	}
	CHECK(i < KV_MAX);
	CHECK(imu_cal_key(kind, gain, kv[i].key, sizeof(kv[i].key)) > 0);
	memset(kv[i].value, 0, sizeof(kv[i].value));
	kv[i].value[0] = IMU_CAL_VERSION;
	kv[i].value[1] = kind == LSM6DSO_CAL_ACCEL ? 0 : 1;
	put_le32(kv[i].value + 4, gain);
	for (int j = 0; j < 9; j++) {
		put_le32(kv[i].value + 8 + 4 * j, (uint32_t)cal->m[j / 3][j % 3]);
	}
	for (int j = 0; j < 3; j++) {
		put_le32(kv[i].value + 44 + 4 * j, (uint32_t)cal->bias[j]);
	}
	kv[i].len = IMU_CAL_BLOB_SIZE;
}

/* Fake LSM6DSOX on a fake i2c0 --------------------------------------------*/

/* The model of nuttx/tests/lsm6dso_driver.c: three register banks, the
 * FIFO, the timestamp; the writes and reads of the Zephyr I2C API
 * (one WRITE|STOP message; WRITE, then RESTART|READ|STOP).  A FreeRTOS
 * mutex keeps the tasks that touch it (RTIO WQ 0, imu_fifo_worker, the
 * sessions, this test) one at a time.
 */
enum {
	USER,
	SHUB,
	EMB
};

static uint8_t regs[3][256];
static uint8_t fifo_data[LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE];
static size_t fifo_words;
static uint32_t timestamp;
static SemaphoreHandle_t fake_lock;

static int bank(void)
{
	uint8_t access = regs[USER][LSM6DSO_FUNC_CFG_ACCESS];

	return access & 0x80 ? EMB : access & 0x40 ? SHUB : USER;
}

static void power_on_defaults(void)
{
	uint8_t keep9 = regs[USER][LSM6DSO_CTRL9_XL] & 0x02;
	uint8_t out[12];

	/* The output registers keep what the test set. */
	memcpy(out, &regs[USER][LSM6DSO_OUTX_L_G], sizeof(out));
	memset(regs[USER], 0, sizeof(regs[USER]));
	memcpy(&regs[USER][LSM6DSO_OUTX_L_G], out, sizeof(out));
	regs[USER][LSM6DSO_PIN_CTRL] = 0x3f;
	regs[USER][LSM6DSO_WHO_AM_I] = 0x6c;
	regs[USER][LSM6DSO_CTRL3_C] = 0x04;
	regs[USER][LSM6DSO_CTRL9_XL] = 0xe0 | keep9;
	regs[USER][LSM6DSO_INTERNAL_FREQ_FINE] = 0xfd;
}

static uint8_t read_byte(int b, uint8_t reg)
{
	if (reg == LSM6DSO_FUNC_CFG_ACCESS) {
		b = USER;
	}
	if (b == USER && reg >= LSM6DSO_TIMESTAMP0 && reg <= LSM6DSO_TIMESTAMP3) {
		return (uint8_t)(timestamp >> (8 * (reg - LSM6DSO_TIMESTAMP0)));
	}
	if (b == USER && reg == LSM6DSO_STATUS_REG) {
		return 0x07; /* new data of all three */
	}
	if (b == USER && reg == LSM6DSO_FIFO_STATUS1) {
		return (uint8_t)fifo_words;
	}
	if (b == USER && reg == LSM6DSO_FIFO_STATUS2) {
		uint16_t wtm = regs[USER][LSM6DSO_FIFO_CTRL1] |
			       (uint16_t)(regs[USER][LSM6DSO_FIFO_CTRL2] & 1) << 8;

		return (uint8_t)((fifo_words >> 8) & 3) | (wtm && fifo_words >= wtm ? 0x80 : 0);
	}
	return regs[b][reg];
}

static void write_byte(int b, uint8_t reg, uint8_t value)
{
	if (reg == LSM6DSO_FUNC_CFG_ACCESS) {
		b = USER;
	}
	if (b == USER && reg == LSM6DSO_CTRL3_C && (value & 1)) {
		power_on_defaults(); /* SW_RESET, which clears itself */
		return;
	}
	if (b == USER && reg == LSM6DSO_WHO_AM_I) {
		return;
	}
	regs[b][reg] = value;
}

static int transfer_regs(struct i2c_msg *msgs, uint8_t num_msgs)
{
	uint8_t reg;
	int b;

	if (num_msgs == 1) {
		CHECK(msgs[0].flags == (I2C_MSG_WRITE | I2C_MSG_STOP) && msgs[0].len >= 2);
		reg = msgs[0].buf[0];
		b = bank();
		for (uint32_t i = 1; i < msgs[0].len; i++) {
			write_byte(i == 1 ? b : bank(), reg, msgs[0].buf[i]);
			if (regs[USER][LSM6DSO_CTRL3_C] & 0x04) {
				reg++;
			}
		}
		return 0;
	}
	CHECK(num_msgs == 2 && msgs[0].flags == I2C_MSG_WRITE && msgs[0].len == 1);
	CHECK(msgs[1].flags == (I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP));
	reg = msgs[0].buf[0];
	b = bank();
	if (b == USER && reg == LSM6DSO_FIFO_DATA_OUT_TAG) {
		size_t want = msgs[1].len;

		CHECK(want % LSM6DSO_FIFO_WORD_SIZE == 0);
		CHECK(want <= fifo_words * LSM6DSO_FIFO_WORD_SIZE);
		memcpy(msgs[1].buf, fifo_data, want);
		memmove(fifo_data, fifo_data + want, fifo_words * LSM6DSO_FIFO_WORD_SIZE - want);
		fifo_words -= want / LSM6DSO_FIFO_WORD_SIZE;
		return 0;
	}
	for (uint32_t i = 0; i < msgs[1].len; i++) {
		msgs[1].buf[i] = read_byte(b, reg);
		if (regs[USER][LSM6DSO_CTRL3_C] & 0x04) {
			reg++;
		}
	}
	if (b == USER && msgs[0].buf[0] == LSM6DSO_TIMESTAMP0) {
		timestamp += 3;
	}
	return 0;
}

static void update_int1(void);

static int fake_i2c_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	int ret;

	(void)dev;
	CHECK(addr == 0x6a);
	/* The board's transfer waits for I2C0's interrupt: the calling task
	 * sleeps and lower tasks run meanwhile (a session's command returns
	 * from the submit of "sensor get" before the read is done).
	 */
	vTaskDelay(1);
	xSemaphoreTake(fake_lock, portMAX_DELAY);
	ret = transfer_regs(msgs, num_msgs);
	xSemaphoreGive(fake_lock);
	update_int1();
	return ret;
}

static int fake_i2c_configure(const struct device *dev, uint32_t config)
{
	(void)dev;
	(void)config;
	return 0;
}

static DEVICE_API(i2c, fake_i2c_api) = {
	.configure = fake_i2c_configure,
	.transfer = fake_i2c_transfer,
};

DEVICE_DT_DEFINE(i2c0, "i2c@40044000", NULL, NULL, NULL, &fake_i2c_api);

/* Fake INT1 (GPIO24) on the layer's GPIO boundary (nano_gpio.h; on the
 * board freertos/drivers/compat/rp2040/nano_gpio.c): high while the FIFO
 * threshold is routed to it and the FIFO holds the watermark; a rising edge
 * calls the layer's handler (common/compat/zephyr_gpio.c) if enabled.
 */
static bool int1_level;
static volatile uint32_t int1_events;

int nano_gpio_configure_input(unsigned int pin, bool pull_up, bool pull_down)
{
	CHECK(pin == 24 && !pull_up && !pull_down);
	return 0;
}

uint32_t nano_gpio_get_all(void)
{
	return int1_level ? 1U << 24 : 0;
}

int nano_gpio_irq_configure(unsigned int pin, uint32_t events)
{
	CHECK(pin == 24);
	int1_events = events;
	return 0;
}

static void update_int1(void)
{
	bool rise;

	xSemaphoreTake(fake_lock, portMAX_DELAY);
	uint16_t wtm = regs[USER][LSM6DSO_FIFO_CTRL1] |
		       (uint16_t)(regs[USER][LSM6DSO_FIFO_CTRL2] & 1) << 8;
	bool level = (regs[USER][LSM6DSO_INT1_CTRL] & 0x08) && wtm && fifo_words >= wtm;

	rise = level && !int1_level;
	int1_level = level;
	xSemaphoreGive(fake_lock);
	if (rise && (int1_events & NANO_GPIO_IRQ_EDGE_RISE)) {
		nano_gpio_irq_handler(24);
	}
}

static void set_raw(uint8_t reg, int16_t x, int16_t y, int16_t z)
{
	int16_t v[3] = {x, y, z};

	xSemaphoreTake(fake_lock, portMAX_DELAY);
	for (int i = 0; i < 3; i++) {
		regs[USER][reg + 2 * i] = (uint8_t)v[i];
		regs[USER][reg + 2 * i + 1] = (uint8_t)((uint16_t)v[i] >> 8);
	}
	xSemaphoreGive(fake_lock);
}

static void push_pairs(int pairs)
{
	xSemaphoreTake(fake_lock, portMAX_DELAY);
	for (int i = 0; i < pairs; i++) {
		static const uint8_t tags[2] = {0x02, 0x01}; /* accel, gyro */
		const int16_t v[2][3] = {{0, 0, 4100}, {10, -5, 2}};

		for (int k = 0; k < 2; k++) {
			uint8_t *w = fifo_data + fifo_words * LSM6DSO_FIFO_WORD_SIZE;

			CHECK(fifo_words < LSM6DSO_FIFO_MAX_WORDS);
			w[0] = (uint8_t)(tags[k] << 3);
			for (int j = 0; j < 3; j++) {
				w[1 + 2 * j] = (uint8_t)v[k][j];
				w[2 + 2 * j] = (uint8_t)((uint16_t)v[k][j] >> 8);
			}
			fifo_words++;
		}
	}
	xSemaphoreGive(fake_lock);
	update_int1();
}

static const struct device *imu(void)
{
	return DEVICE_DT_GET(DT_NODELABEL(lsm6dsox));
}

/* The command of common/imu/imu_stream.c, with "cal" only ----------------*/

static int cmd_cal(const struct shell *sh, size_t argc, char **argv)
{
	return imu_cal_shell(sh, argc, argv);
}

SHELL_STATIC_SUBCMD_SET_CREATE(imu_commands,
			       SHELL_CMD_ARG(cal, NULL,
					     "Show the conversion used by sensor get/stream: "
					     "[reload|zero]",
					     cmd_cal, 1, 1),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(imu, &imu_commands, "IMU RAW streaming", NULL);

NANO_SHELL_CMD_DECLARE(sensor);

const struct nano_shell_cmd nano_shell_cmds[] = {
	NANO_SHELL_CMD(imu),
	NANO_SHELL_CMD(sensor),
};

const size_t nano_shell_cmd_count = sizeof(nano_shell_cmds) / sizeof(nano_shell_cmds[0]);

/* Two sessions on consoles that capture -----------------------------------*/

struct session {
	const char *prompt;
	struct nano_console console;
	struct cli cli;
	QueueHandle_t lines;
	SemaphoreHandle_t done;
	/* The terminal: what it shows, and whether there is one (else the
	 * text is dropped, as the board's consoles drop it).
	 */
	char text[CAPTURE_SIZE];
	size_t len;
	volatile bool connected;
	volatile unsigned int dropped;
};

static struct session sessions[2] = {
	{.prompt = "a$ "},
	{.prompt = "b$ "},
};

/* What the terminals show, after a failed check. */
static void fail_sessions(void)
{
	for (int i = 0; i < 2; i++) {
		(void)!write(2, "--- session ", 12);
		(void)!write(2, sessions[i].prompt, strlen(sessions[i].prompt));
		(void)!write(2, "---\n", 4);
		(void)!write(2, sessions[i].text, sessions[i].len);
		(void)!write(2, "\n", 1);
	}
}

static void capture_out(struct nano_console *con, const char *buf, size_t len)
{
	struct session *s = con->ctx;

	if (!s->connected) {
		s->dropped += (unsigned int)len;
		return;
	}
	vTaskSuspendAll();
	CHECK(s->len + len < sizeof(s->text));
	memcpy(s->text + s->len, buf, len);
	s->len += len;
	s->text[s->len] = '\0';
	(void)xTaskResumeAll();
}

static void cli_write(void *ctx, const char *text, size_t len)
{
	nano_console_write(ctx, text, len);
}

static void cli_execute(void *ctx, char *line)
{
	(void)ctx;
	(void)nano_shell_execute(line);
}

static void session_main(void *arg)
{
	struct session *s = arg;
	const char *line;

	nano_console_bind(NULL, &s->console);
	cli_init(&s->cli, s->prompt, cli_write, cli_execute, &s->console);
	for (;;) {
		xQueueReceive(s->lines, &line, portMAX_DELAY);
		for (const char *p = line; *p != '\0'; p++) {
			cli_input(&s->cli, *p);
		}
		cli_input(&s->cli, '\r');
		xSemaphoreGive(s->done);
	}
}

/* What the session's terminal shows from now on. */
static void clear(struct session *s)
{
	vTaskSuspendAll();
	s->len = 0;
	s->text[0] = '\0';
	(void)xTaskResumeAll();
}

/* Types LINE in session S and waits until the command has returned and the
 * prompt is drawn again; S's terminal is cleared first.
 */
static void run(struct session *s, const char *line)
{
	clear(s);
	xQueueSend(s->lines, &line, portMAX_DELAY);
	CHECK(xSemaphoreTake(s->done, pdMS_TO_TICKS(10000)) == pdTRUE);
}

static bool contains(struct session *s, const char *what)
{
	bool found;

	vTaskSuspendAll();
	found = strstr(s->text, what) != NULL;
	(void)xTaskResumeAll();
	return found;
}

/* Waits up to MS for WHAT on S's terminal. */
static bool wait_for(struct session *s, const char *what, int ms)
{
	for (int i = 0; i < ms; i++) {
		if (contains(s, what)) {
			return true;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	return contains(s, what);
}

static bool starts(struct session *s, const char *prefix)
{
	return strncmp(s->text, prefix, strlen(prefix)) == 0;
}

static void expect(struct session *s, const char *want)
{
	if (strcmp(s->text, want) != 0) {
		fail("session %s:\n--- got ---\n%s\n--- want ---\n%s", s->prompt, s->text, want);
	}
}

/* The three values of the "channel type=T(NAME) ..." line of a 3-axis
 * channel on S's terminal ("...ns, (x, y, z)").
 */
static void values3(struct session *s, const char *name, double v[3])
{
	char head[64];
	const char *p, *q;

	snprintf(head, sizeof(head), "(%s) index=0 shift=", name);
	p = strstr(s->text, head);
	if (p == NULL) {
		fail("no %s line:\n%s", name, s->text);
	}
	q = strstr(p, "ns, (");
	CHECK(q != NULL && q < strchr(p, '\n'));
	CHECK(sscanf(q + 5, "%lf, %lf, %lf", &v[0], &v[1], &v[2]) == 3);
}

/* The physical value of RAW through CAL, as the driver converts it
 * (lsm6dso_cal.c), in its unit.
 */
static double converted(const struct lsm6dso_cal *cal, const int16_t raw[3], int axis)
{
	int32_t out[3];

	lsm6dso_cal_apply(cal, raw, out);
	return (double)lsm6dso_cal_to_micro(out[axis]) / 1e6;
}

static void check_close(double got, double want)
{
	if (got - want > 2e-6 || want - got > 2e-6) {
		fail("value %.6f, want %.6f", got, want);
	}
}

/* The tests ----------------------------------------------------------------*/

static struct session *const a = &sessions[0];
static struct session *const b = &sessions[1];

static const char sensor_help_head[] = "sensor - Sensor commands\r\n"
				       "Subcommands:\r\n"
				       "  get       : Get sensor data.\r\n";

static void test_help(void)
{
	/* No "info" (CONFIG_SENSOR_INFO is not set): the help of "sensor". */
	run(a, "sensor info");
	CHECK(starts(a, "sensor info\r\nsensor - Sensor commands\r\n"));
	CHECK(strstr(a->text, sensor_help_head) != NULL);
	CHECK(strstr(a->text, "  info") == NULL && strstr(a->text, "  stream    : ") != NULL);
	CHECK(strstr(a->text, "  trig      : ") != NULL);
	run(b, "sensor get");
	CHECK(strstr(b->text, "get: wrong parameter count\r\n") != NULL);
	CHECK(strstr(b->text, "Subcommands:\r\n  lsm6dsox@6a\r\nb$ ") != NULL);
}

/* "sensor get" from each session: the result on that session's terminal,
 * then the prompt; nothing on the other.
 */
static void test_get(void)
{
	double v[3];

	set_raw(LSM6DSO_OUTX_L_A, 0, 0, 4100);
	set_raw(LSM6DSO_OUTX_L_G, 10, -5, 2);
	clear(b);
	run(a, "sensor get lsm6dsox@6a accel_xyz");
	CHECK(starts(a, "sensor get lsm6dsox@6a accel_xyz\r\n"
			"channel type=0(accel_x) index=0 shift=4 num_samples=1 value="));
	CHECK(strstr(a->text, "ns, (0.000000, 0.000000, 9.810572)\r\na$ ") != NULL);
	values3(a, "accel_xyz", v);
	check_close(v[2], 9.810572);
	CHECK(b->len == 0);

	run(b, "sensor get lsm6dsox gyro_xyz");
	CHECK(starts(b, "sensor get lsm6dsox gyro_xyz\r\nchannel type=4(gyro_x) "));
	CHECK(strstr(b->text, "channel type=7(gyro_xyz) index=0 shift=") != NULL);
	CHECK(strlen(b->text) > 3 && strcmp(b->text + strlen(b->text) - 5, "\r\nb$ ") == 0);
	/* 10 LSB at 70 mdps/LSB = 0.7 dps = 0.012217 rad/s, the nominal conversion. */
	values3(b, "gyro_xyz", v);
	check_close(v[0], 0.012217);
	CHECK(contains(a, "gyro") == false);

	/* All channels: 4 lines for each sensor (no die_temp in a single read). */
	run(a, "sensor get lsm6dsox@6a");
	CHECK(strstr(a->text, "(accel_xyz)") != NULL && strstr(a->text, "(gyro_xyz)") != NULL);
	CHECK(strstr(a->text, "Read failed") == NULL);
}

static void test_calibration(void)
{
	static const int16_t accel_raw[3] = {0, 0, 4100};
	static const int16_t gyro_raw[3] = {10, -5, 2};
	struct lsm6dso_cal accel, gyro, gyro_zeroed;
	double v[3];

	/* No stored values: nominal. */
	run(a, "imu cal");
	expect(a, "imu cal\r\n"
		  "conversion:\r\n"
		  "  accel: nominal (244 ug/LSB, key imu.cal.accel.244)\r\n"
		  "  gyro:  nominal (70000 udps/LSB, key imu.cal.gyro.70000)\r\n"
		  "  gyro bias: 0.00 0.00 0.00 LSB (none, 0 rejected)\r\n"
		  "a$ ");

	/* The stored values, read by "imu cal reload": the accel scaled by 1.01
	 * on z with a bias of 50 LSB, the gyro with the bias of the fake.
	 */
	lsm6dso_cal_nominal(&accel, LSM6DSO_CAL_ACCEL, 244);
	accel.m[2][2] = accel.m[2][2] / 100 * 101;
	accel.bias[2] = 50 << LSM6DSO_CAL_BIAS_FRAC;
	lsm6dso_cal_nominal(&gyro, LSM6DSO_CAL_GYRO, 70000);
	gyro.bias[0] = 10 << LSM6DSO_CAL_BIAS_FRAC;
	gyro.bias[1] = -5 * (1 << LSM6DSO_CAL_BIAS_FRAC);
	gyro.bias[2] = 1 << LSM6DSO_CAL_BIAS_FRAC;
	kv_put_cal(LSM6DSO_CAL_ACCEL, 244, &accel);
	kv_put_cal(LSM6DSO_CAL_GYRO, 70000, &gyro);
	run(b, "imu cal reload");
	expect(b, "imu cal reload\r\n"
		  "conversion:\r\n"
		  "  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)\r\n"
		  "  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)\r\n"
		  "  gyro bias: 10.00 -5.00 1.00 LSB (stored, 0 rejected)\r\n"
		  "b$ ");

	/* "sensor get" gives the calibrated values. */
	run(a, "sensor get lsm6dsox@6a accel_xyz gyro_xyz");
	values3(a, "accel_xyz", v);
	CHECK(v[2] < 9.810572 - 0.01); /* not the nominal value */
	check_close(v[0], converted(&accel, accel_raw, 0));
	check_close(v[2], converted(&accel, accel_raw, 2));
	values3(a, "gyro_xyz", v);
	check_close(v[0], 0.0);
	check_close(v[1], 0.0);
	check_close(v[2], converted(&gyro, gyro_raw, 2));
	CHECK(v[2] > 0.0001);

	/* "imu cal zero": the bias at rest replaces the stored one (in RAM). */
	run(b, "imu cal zero");
	expect(b, "imu cal zero\r\n"
		  "conversion:\r\n"
		  "  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)\r\n"
		  "  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)\r\n"
		  "  gyro bias: 10.00 -5.00 2.00 LSB (tracked at rest, 1 updates, 0 rejected)\r\n"
		  "b$ ");
	gyro_zeroed = gyro;
	gyro_zeroed.bias[2] = 2 << LSM6DSO_CAL_BIAS_FRAC;
	run(a, "sensor get lsm6dsox@6a gyro_xyz");
	values3(a, "gyro_xyz", v);
	check_close(v[2], converted(&gyro_zeroed, gyro_raw, 2));
	check_close(v[2], 0.0);

	/* "imu cal reload" goes back to the stored bias. */
	run(b, "imu cal reload");
	CHECK(strstr(b->text, "  gyro bias: 10.00 -5.00 1.00 LSB (stored, 0 rejected)\r\n") !=
	      NULL);
	run(a, "sensor get lsm6dsox@6a gyro_xyz");
	values3(a, "gyro_xyz", v);
	check_close(v[2], converted(&gyro, gyro_raw, 2));
}

static bool stream_active(void)
{
	struct lsm6dso_fifo_stats stats;

	lsm6dso_fifo_get_stats(imu(), &stats);
	return stats.active;
}

static bool wait_stream_ended(void)
{
	for (int i = 0; i < 2000; i++) {
		if (!stream_active() && ((struct lsm6dso_data *)imu()->data)->stream_sqe == NULL) {
			return true;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	return false;
}

static void test_stream(void)
{
	char want[96];
	unsigned int dropped;

	run(a, "sensor attr_set lsm6dsox@6a accel_xyz sampling_frequency 6667 gyro_xyz "
	       "sampling_frequency 6667");
	CHECK(strstr(a->text, "attr=sampling_frequency set to value=6667\r\n"
			      "lsm6dsox@6a channel=gyro_xyz") != NULL);

	/* On, from session a: the batches on a's terminal only. */
	clear(b);
	run(a, "sensor stream lsm6dsox@6a on fifo_wm incl");
	expect(a, "sensor stream lsm6dsox@6a on fifo_wm incl\r\nEnabling stream...\r\na$ ");
	for (int i = 0; i < 1000 && !stream_active(); i++) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	CHECK(stream_active());
	clear(a);
	push_pairs(20); /* 40 words, the watermark is 32 */
	CHECK(wait_for(a, "(gyro_xyz) index=0 shift=", 2000));
	CHECK(starts(a, "Trigger (10 / fifo_wm) detected\r\n"
			"channel type=3(accel_xyz) index=0 shift="));
	CHECK(strstr(a->text, "num_samples=20 value=") != NULL);
	CHECK(b->len == 0);

	/* Session b meanwhile: the FIFO is the stream's. */
	run(b, "sensor get lsm6dsox@6a accel_xyz");
	snprintf(want, sizeof(want), "sensor get lsm6dsox@6a accel_xyz\r\nRead failed: %d\r\nb$ ",
		 -EBUSY);
	expect(b, want);
	run(b, "sensor attr_set lsm6dsox@6a all fifo_watermark 64");
	snprintf(want, sizeof(want), "Failed to set channel(all) attribute(fifo_watermark): %d\r\n",
		 -EBUSY);
	CHECK(strstr(b->text, want) != NULL);
	/* "imu start" would refuse: the driver reports the FIFO in use. */
	CHECK(stream_active());

	/* Session a's terminal goes away: its text is dropped, the processing
	 * thread goes on, session b still works.
	 */
	a->connected = false;
	dropped = a->dropped;
	push_pairs(20);
	for (int i = 0; i < 2000 && a->dropped == dropped; i++) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	CHECK(a->dropped > dropped);
	run(b, "sensor get lsm6dsox@6a accel_xyz");
	CHECK(strstr(b->text, "Read failed: ") != NULL);

	/* Off from session b. */
	run(b, "sensor stream lsm6dsox@6a off");
	expect(b, "sensor stream lsm6dsox@6a off\r\nDisabling existing stream\r\nb$ ");
	CHECK(wait_stream_ended());
	a->connected = true;
	clear(a);
	push_pairs(20);
	vTaskDelay(pdMS_TO_TICKS(100));
	CHECK(a->len == 0);
	xSemaphoreTake(fake_lock, portMAX_DELAY);
	fifo_words = 0;
	xSemaphoreGive(fake_lock);
	update_int1();

	/* After the stream, "sensor get" works again from session a. */
	run(a, "sensor get lsm6dsox@6a accel_xyz");
	CHECK(strstr(a->text, "(accel_xyz) index=0 shift=4 num_samples=1 value=") != NULL);
}

/* A stream of another RTIO context, as "imu start" makes one
 * (common/imu/imu_stream.c).
 */
SENSOR_DT_STREAM_IODEV(test_stream_iodev, DT_NODELABEL(lsm6dsox),
		       {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE});
RTIO_DEFINE_WITH_MEMPOOL(test_rtio, 4, 16, 128, 64, 4);

static void test_other_stream(void)
{
	rtio_sqe_handle_t h;
	char want[160];
	struct rtio_cqe *cqe;

	CHECK(sensor_stream(&test_stream_iodev, &test_rtio, NULL, &h) == 0);
	for (int i = 0; i < 1000 && !stream_active(); i++) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	CHECK(stream_active());

	run(a, "sensor get lsm6dsox@6a");
	snprintf(want, sizeof(want), "Read failed: %d\r\na$ ", -EBUSY);
	CHECK(strstr(a->text, want) != NULL);
	/* "sensor stream" (which does not wait, as in Zephyr) submits the
	 * stream, which the driver refuses at once in the command's task; the
	 * processing thread, of a higher priority, prints the refusal before
	 * the command returns.  The earlier stream's handle is still kept by
	 * the copied sensor_shell_stream.c after its "off", so it is cancelled
	 * again first ("Disabling existing stream", as in Zephyr).
	 */
	run(a, "sensor stream lsm6dsox@6a on fifo_wm incl");
	snprintf(want, sizeof(want),
		 "sensor stream lsm6dsox@6a on fifo_wm incl\r\n"
		 "Disabling existing stream\r\n"
		 "Enabling stream...\r\n"
		 "Read failed: %d\r\n"
		 "a$ ",
		 -EBUSY);
	expect(a, want);
	run(a, "sensor stream lsm6dsox@6a off");
	expect(a, "sensor stream lsm6dsox@6a off\r\nDisabling existing stream\r\na$ ");
	run(a, "imu cal zero");
	CHECK(strstr(a->text, "The sensor is streaming; the bias is tracked from the stream\r\n") !=
	      NULL);

	rtio_sqe_cancel(&test_rtio, h);
	CHECK(wait_stream_ended());
	while ((cqe = rtio_cqe_consume(&test_rtio)) != NULL) {
		uint8_t *buf = NULL;
		uint32_t len = 0;

		(void)rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &buf, &len);
		rtio_cqe_release(&test_rtio, cqe);
		rtio_release_buffer(&test_rtio, buf, len);
	}
	run(a, "sensor get lsm6dsox@6a accel_xyz");
	CHECK(strstr(a->text, "(accel_xyz) index=0 shift=4 num_samples=1 value=") != NULL);
}

static void test_main(void *arg)
{
	(void)arg;

	/* The task "main" of the board: the sensor, the processing thread,
	 * the calibration's lookup (main.c).
	 */
	CHECK(device_init(DEVICE_DT_GET(DT_NODELABEL(i2c0))) == 0);
	CHECK(device_init(imu()) == 0);
	CHECK(strstr(log_text, "Initialize device lsm6dsox@6a") != NULL);
	CHECK(K_THREAD_DEFINE_START(sensor_shell_processing_tid) == 0);
	CHECK(imu_cal_init() == 0);

	test_help();
	test_get();
	test_calibration();
	test_stream();
	test_other_stream();
	_exit(0);
}

int main(void)
{
	fake_lock = xSemaphoreCreateMutex();
	power_on_defaults();
	CHECK(zephyr_compat_initialize() == 0);
	for (int i = 0; i < 2; i++) {
		struct session *s = &sessions[i];

		s->connected = true;
		s->lines = xQueueCreate(1, sizeof(const char *));
		s->done = xSemaphoreCreateBinary();
		nano_console_init(&s->console, s->prompt, capture_out, s);
		if (xTaskCreate(session_main, "session", SESSION_STACK / sizeof(StackType_t), s,
				NANO_PRIO_SHELL, NULL) != pdPASS) {
			return 2;
		}
	}
	/* The tasks bound to no console print nowhere (none should). */
	nano_console_set_default(NULL);
	if (xTaskCreate(test_main, "main", SESSION_STACK / sizeof(StackType_t), NULL,
			NANO_PRIO_SHELL - 1, NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
