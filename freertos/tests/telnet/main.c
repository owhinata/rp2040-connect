/* SPDX-License-Identifier: MIT */
/* The host test of the Telnet shell (freertos/drivers/telnet/shell_telnet.c)
 * and of "zperf" (common/compat/zperf_iperf3.c, zephyr_zperf_shell.c) on the
 * FreeRTOS POSIX port, with the build's shell (freertos/drivers/shell, the
 * shared zshell.c) and the zsock_*() calls on the host's sockets with the
 * NINA's behaviour (host_sock.c).  This program is the board; the client
 * side is tests/telnet/client.py, which starts it, talks to it over TCP on
 * 127.0.0.1 (with Python's sockets and the host's iperf3) and reads what it
 * prints:
 *
 *   READY                       the Telnet server is up
 *   LOG <module>: <message>     a LOG_*() line (the board's RAM log)
 *   RESULT ok | RESULT <why>    the checks of this side, at "quit"
 *
 * Besides the Telnet session, a task plays the USB session: it runs
 * "lines 40" on its own console again and again and checks that its
 * console holds exactly that each time (no output of the Telnet session
 * mixed in).  A task at net_mgmt's priority wakes every 10 ms and records
 * how late it was and, after "async on", how long it took to print a line
 * to the Telnet session's backend (another task's output to a client that
 * may not read).  The commands of the test:
 *
 *   lines <n>       prints "line 1" .. "line <n>"
 *   async on|off    the 10 ms task prints "async <k>" to this session
 *   netdown         200 ms later every connection fails (a Wi-Fi drop)
 *   listendown      200 ms later the listening socket fails and the next
 *                   three binds fail (the NINA not answering)
 *   socks           the descriptors in use
 *   stats           the 10 ms task's worst lateness and print time, the
 *                   USB session's runs and failures (and resets the worst)
 *   quit            prints RESULT and ends the program
 *   help, zperf     the build's
 *
 * Usage: telnet_test <telnet port>  (the build's CONFIG_SHELL_TELNET_PORT
 * is test_telnet_port here).
 */

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "app_log.h"
#include "host_sock.h"
#include "shell_console.h"
#include "shell_engine.h"
#include "shell_telnet.h"
#include "pico/stdio.h"
#include "priorities.h"
#include "zephyr_compat.h"

#define TASK_STACK 8192

uint16_t test_telnet_port;
_Thread_local int z_host_isr_depth;

void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

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

/* The test's own output, for client.py: one write() per line. */
static void report(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void report(const char *fmt, ...)
{
	char buf[512];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
	va_end(ap);
	if (n < 0) {
		return;
	}
	if (n > (int)sizeof(buf) - 2) {
		n = (int)sizeof(buf) - 2;
	}
	buf[n++] = '\n';
	vTaskSuspendAll();
	(void)!write(STDOUT_FILENO, buf, (size_t)n);
	(void)xTaskResumeAll();
}

/* The Pico SDK's stdio, as pico_stdio (see freertos/tests/shell/main.c). */

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation)
{
	struct shell_console *con = shell_console_current();

	(void)cr_translation;
	if (len < 0) {
		len = (int)strlen(s);
	}
	shell_console_write(con, s, (size_t)len);
	if (newline) {
		shell_console_write(con, "\n", 1);
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

/* The log: to client.py. */

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	char msg[256];

	(void)level;
	(void)func;
	vsnprintf(msg, sizeof(msg), fmt, ap);
	report("LOG %s: %s", module, msg);
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

/* The USB session --------------------------------------------------------*/

#define USB_LINES 40

static struct shell_console usb;
static char usb_text[2048];
static size_t usb_len;
static bool usb_overflow;
static char usb_want[2048];
static uint32_t usb_runs;
static uint32_t usb_bad;

static void usb_out(struct shell_console *con, const char *buf, size_t len)
{
	(void)con;
	if (usb_len + len >= sizeof(usb_text)) {
		usb_overflow = true;
		return;
	}
	memcpy(usb_text + usb_len, buf, len);
	usb_len += len;
	usb_text[usb_len] = '\0';
}

static void usb_main(void *arg)
{
	size_t n = 0;

	(void)arg;
	for (int i = 1; i <= USB_LINES; i++) {
		n += (size_t)snprintf(usb_want + n, sizeof(usb_want) - n, "line %d\r\n", i);
	}
	shell_console_bind(NULL, &usb);
	for (;;) {
		char line[16] = "lines 40";

		usb_len = 0;
		usb_text[0] = '\0';
		usb_overflow = false;
		(void)shell_engine_execute(line);
		usb_runs++;
		if (usb_overflow || strcmp(usb_text, usb_want) != 0) {
			usb_bad++;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

/* The 10 ms task -------------------------------------------------------------*/

static const struct shell *volatile async_sh;
static volatile int64_t netdown_at;
static volatile int64_t listendown_at;
static volatile uint32_t late_max_us;
static volatile uint32_t async_max_us;
static uint32_t async_count;

static void monitor_main(void *arg)
{
	(void)arg;
	for (;;) {
		uint64_t before = rp2040_timer_us();
		uint64_t slept;
		const struct shell *sh;

		/* 10 ms, so 10 to 11 ticks of 1 ms: late is what is more. */
		vTaskDelay(pdMS_TO_TICKS(10));
		slept = rp2040_timer_us() - before;
		if (slept > 11000 && slept - 11000 > late_max_us) {
			late_max_us = (uint32_t)(slept - 11000);
		}
		sh = async_sh;
		if (sh != NULL) {
			uint64_t t0 = rp2040_timer_us();
			uint32_t dt;

			shell_print(sh, "async %lu", (unsigned long)++async_count);
			dt = (uint32_t)(rp2040_timer_us() - t0);
			if (dt > async_max_us) {
				async_max_us = dt;
			}
		}
		if (netdown_at != 0 && k_uptime_get() >= netdown_at) {
			netdown_at = 0;
			fake_sock_fail_connections();
		}
		if (listendown_at != 0 && k_uptime_get() >= listendown_at) {
			listendown_at = 0;
			fake_sock_fail_listeners(3);
		}
	}
}

/* The commands -----------------------------------------------------------------*/

static int cmd_lines(const struct shell *sh, size_t argc, char **argv)
{
	int n = atoi(argv[1]);

	(void)sh;
	(void)argc;
	for (int i = 1; i <= n; i++) {
		printf("line %d\n", i);
	}
	return 0;
}

SHELL_CMD_ARG_REGISTER(lines, NULL, "Print numbered lines", cmd_lines, 2, 0);

static int cmd_async(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	async_sh = strcmp(argv[1], "on") == 0 ? sh : NULL;
	return 0;
}

SHELL_CMD_ARG_REGISTER(async, NULL, "Print from another task", cmd_async, 2, 0);

static int cmd_netdown(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;
	netdown_at = k_uptime_get() + 200;
	return 0;
}

SHELL_CMD_REGISTER(netdown, NULL, "Fail the connections in 200 ms", cmd_netdown);

static int cmd_listendown(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;
	listendown_at = k_uptime_get() + 200;
	return 0;
}

SHELL_CMD_REGISTER(listendown, NULL, "Fail the listening socket and 3 binds in 200 ms",
		   cmd_listendown);

static int cmd_socks(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;
	shell_print(sh, "socks %d", fake_sock_open_count());
	return 0;
}

SHELL_CMD_REGISTER(socks, NULL, "Descriptors in use", cmd_socks);

static int cmd_stats(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;
	shell_print(sh, "stats late_max_ms=%lu async_max_ms=%lu usb_runs=%lu usb_bad=%lu",
		    (unsigned long)(late_max_us / 1000), (unsigned long)(async_max_us / 1000),
		    (unsigned long)usb_runs, (unsigned long)usb_bad);
	late_max_us = 0;
	async_max_us = 0;
	return 0;
}

SHELL_CMD_REGISTER(stats, NULL, "Worst times, USB session", cmd_stats);

static int cmd_quit(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;
	if (usb_bad != 0 || usb_runs < 10) {
		report("RESULT usb session: %lu runs, %lu bad", (unsigned long)usb_runs,
		       (unsigned long)usb_bad);
	} else {
		report("RESULT ok");
	}
	_exit(0);
}

SHELL_CMD_REGISTER(quit, NULL, "End the test", cmd_quit);

static int cmd_help(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;
	shell_engine_help();
	return 0;
}

SHELL_CMD_REGISTER(help, NULL, "Prints the help message.", cmd_help);

SHELL_ENGINE_CMD_DECLARE(zperf);

/* clang-format off */
const struct shell_engine_cmd shell_engine_cmds[] = {
	SHELL_ENGINE_CMD(async),
	SHELL_ENGINE_CMD(help),
	SHELL_ENGINE_CMD(lines),
	SHELL_ENGINE_CMD(listendown),
	SHELL_ENGINE_CMD(netdown),
	SHELL_ENGINE_CMD(quit),
	SHELL_ENGINE_CMD(socks),
	SHELL_ENGINE_CMD(stats),
	SHELL_ENGINE_CMD(zperf),
};
/* clang-format on */

const size_t shell_engine_cmd_count = sizeof(shell_engine_cmds) / sizeof(shell_engine_cmds[0]);

/* The start -------------------------------------------------------------------*/

static void init_main(void *arg)
{
	(void)arg;
	if (shell_telnet_init("t$ ") != 0) {
		report("RESULT shell_telnet_init failed");
		_exit(1);
	}
	if (K_THREAD_DEFINE_START(zperf_server_tid) != 0) {
		report("RESULT zperf_server_tid not started");
		_exit(1);
	}
	xTaskCreate(usb_main, "usb", TASK_STACK / sizeof(StackType_t), NULL, APP_PRIO_SHELL, NULL);
	xTaskCreate(monitor_main, "monitor", TASK_STACK / sizeof(StackType_t), NULL,
		    APP_PRIO_NET_MGMT, NULL);
	report("READY");
	vTaskDelete(NULL);
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		fprintf(stderr, "usage: %s <telnet port>\n", argv[0]);
		return 2;
	}
	test_telnet_port = (uint16_t)atoi(argv[1]);
	setvbuf(stdout, NULL, _IONBF, 0);

	shell_console_init(&usb, "usb", usb_out, NULL);
	shell_console_set_default(&usb);
	if (xTaskCreate(init_main, "main", TASK_STACK / sizeof(StackType_t), NULL, APP_PRIO_MAIN,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
