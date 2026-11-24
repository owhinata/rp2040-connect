/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's shell (the shell engine
 * common/compat/shell_engine, with common/compat/port/pico_sdk/stdio_wrap.c and
 * the shared common/shell/zshell.c, subcmd.c, kernel_shell.c, the copies of
 * Zephyr's fnmatch() and getopt, common/compat/log/fault_record.c) on the
 * FreeRTOS POSIX port:
 *
 * - the line editor alone (cli.c): echo, Backspace, Ctrl-C, history with
 *   Up and Down, other escape sequences ignored;
 * - two sessions at once: two consoles that capture their output, each with
 *   a task bound to it that types its lines into its own line editor and
 *   runs them (shell_engine_execute()).  The commands are registered with
 *   SHELL_CMD_ARG_REGISTER() as the shared sources do and print with
 *   printf(), fprintf(stderr), fputs() and shell_print(); the shell prints
 *   help, the argument count error and "command not found"; one command
 *   parses options with sys_getopt() and sleeps after each, so the two
 *   sessions parse at the same time.  Each console must hold exactly its own
 *   session's transcript (with "\r\n" line ends);
 * - output to a session's backend from another task (a struct shell kept by
 *   a command), and the default console of a task bound to none;
 * - the engine's "help" and the generic commands of
 *   common/shell/kernel_shell.c ("id", "uptime", "date", "sleep", "devmem",
 *   "fault", "reboot", "bootsel"), with their output, on this directory's
 *   kernel_shell_os_impl.h, a fake hwinfo_get_device_id() and the layer's
 *   realtime clock (common/compat/port/pico_sdk/zephyr_sys_clock.c);
 * - the serial session (common/compat/shell_engine/shell_serial.c) on this
 *   directory's console transport (shell_serial_os_impl.h): nothing while
 *   the line is closed, the banner and the prompt when it opens, a command
 *   typed and its output, the default console.
 *
 * The Pico SDK's stdio is played by this file as pico_stdio does it:
 * printf(), vprintf(), puts() and putchar() are wrapped (-Wl,--wrap) and,
 * like the SDK's newlib write(), end in stdio_put_string(), which writes to
 * the calling task's console (common/compat/port/pico_sdk/shell_stdio.c on the
 * board).  The shell's calloc(), malloc() and free() run with the
 * scheduler suspended (the POSIX port may switch threads inside the C
 * library's locks).
 * Built and run by freertos/script/host_tests.sh.
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

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/sys_getopt.h>

#include "app_log.h"
#include "cli.h"
#include "fault_record.h"
#include "shell_console.h"
#include "shell_engine.h"
#include "shell_serial.h"
#include "shell_serial_os.h"
#include "pico/stdio.h"
#include "priorities.h"

#define SESSION_STACK 8192
#define CAPTURE_SIZE  4096

_Thread_local int z_host_isr_depth;

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

/* The allocations of zshell.c and shell_console.c (built with
 * -Dcalloc=test_calloc and so on), with the scheduler suspended: the POSIX
 * port may switch threads inside the C library's locks.  The port's own
 * malloc() at the start of each thread is left alone.
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

/* The Pico SDK's stdio, as pico_stdio ------------------------------------*/

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

/* The log of getopt's warnings: not looked at. */

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	(void)level;
	(void)module;
	(void)func;
	(void)fmt;
	(void)ap;
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	(void)level;
	(void)module;
	(void)func;
	(void)fmt;
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

/* Consoles that capture --------------------------------------------------*/

struct capture {
	char text[CAPTURE_SIZE];
	size_t len;
};

static void capture_out(struct shell_console *con, const char *buf, size_t len)
{
	struct capture *c = con->ctx;

	assert(c->len + len < sizeof(c->text));
	memcpy(c->text + c->len, buf, len);
	c->len += len;
	c->text[c->len] = '\0';
}

static void expect(const char *what, const struct capture *c, const char *want)
{
	if (strcmp(c->text, want) != 0) {
		fprintf(stderr, "%s:\n--- got ---\n%s\n--- want ---\n%s\n", what, c->text, want);
		_exit(1);
	}
}

/* The commands ------------------------------------------------------------*/

static const struct shell *kept_sh;

static int cmd_echo(const struct shell *sh, size_t argc, char **argv)
{
	printf("argc=%zu\n", argc);
	fprintf(stderr, "err:%s\n", argv[1]);
	shell_print(sh, "sh:%s", argv[2]);
	fputs("tail\n", stdout);
	return 0;
}

SHELL_CMD_ARG_REGISTER(echo, NULL, "Print the words", cmd_echo, 3, 0);

static int cmd_sub(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	shell_print(sh, "%s ran", argv[0]);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(grp_commands, SHELL_CMD_ARG(one, NULL, "First", cmd_sub, 2, 0),
			       SHELL_CMD(two, NULL, "Second", cmd_sub), SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(grp, &grp_commands, "Group of commands", NULL);

/* Options, with a sleep after each, so that the other session parses its
 * own meanwhile.
 */
static char opt_order[8]; /* the session of each option parsed */
static int opt_count;

static int cmd_opt(const struct shell *sh, size_t argc, char **argv)
{
	int c;

	while ((c = sys_getopt((int)argc, argv, "a:b:")) != -1) {
		struct sys_getopt_state *state = sys_getopt_state_get();

		shell_print(sh, "opt %c=%s", c, c == '?' ? "?" : state->optarg);
		if (opt_count < (int)sizeof(opt_order)) {
			opt_order[opt_count++] = sh->console->name[0];
		}
		vTaskDelay(2);
	}
	return 0;
}

SHELL_CMD_REGISTER(opt, NULL, "Options", cmd_opt);

static int cmd_later(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;
	kept_sh = sh;
	return 0;
}

SHELL_CMD_REGISTER(later, NULL, "Keep the backend", cmd_later);

/* The engine's "help" and the generic commands (common/shell/kernel_shell.c,
 * on this test's kernel_shell_os_impl.h).
 */
SHELL_ENGINE_CMD_DECLARE(bootsel);
SHELL_ENGINE_CMD_DECLARE(date);
SHELL_ENGINE_CMD_DECLARE(devmem);
SHELL_ENGINE_CMD_DECLARE(fault);
SHELL_ENGINE_CMD_DECLARE(help);
SHELL_ENGINE_CMD_DECLARE(id);
SHELL_ENGINE_CMD_DECLARE(reboot);
SHELL_ENGINE_CMD_DECLARE(sleep);
SHELL_ENGINE_CMD_DECLARE(uptime);

/* clang-format off */
const struct shell_engine_cmd shell_engine_cmds[] = {
	SHELL_ENGINE_CMD(bootsel),
	SHELL_ENGINE_CMD(date),
	SHELL_ENGINE_CMD(devmem),
	SHELL_ENGINE_CMD(echo),
	SHELL_ENGINE_CMD(fault),
	SHELL_ENGINE_CMD(grp),
	SHELL_ENGINE_CMD(help),
	SHELL_ENGINE_CMD(id),
	SHELL_ENGINE_CMD(later),
	SHELL_ENGINE_CMD(opt),
	SHELL_ENGINE_CMD(reboot),
	SHELL_ENGINE_CMD(sleep),
	SHELL_ENGINE_CMD(uptime),
};
/* clang-format on */

const size_t shell_engine_cmd_count = sizeof(shell_engine_cmds) / sizeof(shell_engine_cmds[0]);

/* The generic commands' OS part (kernel_shell_os_impl.h) and the layer's
 * hwinfo -----------------------------------------------------------------*/

unsigned int test_reboots;
unsigned int test_bootsels;
uint32_t test_restart_delay_ms;
const struct fault_record *test_fault;

static const uint8_t test_id[8] = {0xe6, 0x61, 0x41, 0x04, 0x03, 0x5a, 0x9b, 0x2c};

ssize_t hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	if (length > sizeof(test_id)) {
		length = sizeof(test_id);
	}
	memcpy(buffer, test_id, length);
	return (ssize_t)length;
}

/* The line editor alone ---------------------------------------------------*/

static struct capture editor_out;
static char executed[4][CLI_LINE_MAX + 1];
static int executed_count;

static void editor_write(void *ctx, const char *text, size_t len)
{
	struct capture *c = ctx;

	assert(c->len + len < sizeof(c->text));
	memcpy(c->text + c->len, text, len);
	c->len += len;
	c->text[c->len] = '\0';
}

static void editor_execute(void *ctx, char *line)
{
	(void)ctx;
	assert(executed_count < 4);
	strcpy(executed[executed_count++], line);
}

static void type(struct cli *cli, const char *text)
{
	for (const char *p = text; *p != '\0'; p++) {
		cli_input(cli, *p);
	}
}

static void test_editor(void)
{
	static struct cli cli;

	cli_init(&cli, "> ", editor_write, editor_execute, &editor_out);
	type(&cli, "abc\r\nxy\bz\r");
	assert(executed_count == 2 && strcmp(executed[0], "abc") == 0 &&
	       strcmp(executed[1], "xz") == 0);
	type(&cli, "xz\r"); /* the same as the newest: not kept again */
	editor_out.len = 0;
	/* Up, Up, Up (no older), Down, Right (ignored), Down (empty), Up (SS3). */
	type(&cli, "\x1b[A\x1b[A\x1b[A\x1b[B\x1b[C\x1b[B\x1bOA");
	expect("editor", &editor_out,
	       "\r> xz\x1b[K\r> abc\x1b[K\r> xz\x1b[K\r> \x1b[K\r> xz\x1b[K");
	type(&cli, "!\x03q\r"); /* Ctrl-C drops "xz!" */
	assert(executed_count == 4 && strcmp(executed[3], "q") == 0);
}

/* Two sessions ------------------------------------------------------------*/

struct session {
	const char *prompt;
	const char *const *lines;
	struct shell_console console;
	struct capture out;
	struct cli cli;
};

/* Both start with "opt", so that they parse options at the same time. */
static const char *const lines_a[] = {"opt -a 1 -b 2", "echo hi there", "grp",
				      "nope",          "later",         NULL};
static const char *const lines_b[] = {"opt -b 7 -a 8", "grp one", "echo 'a b' c", "grp two x",
				      NULL};

static struct session sessions[2] = {
	{.prompt = "a$ ", .lines = lines_a},
	{.prompt = "b$ ", .lines = lines_b},
};

static SemaphoreHandle_t done;

static void cli_write(void *ctx, const char *text, size_t len)
{
	shell_console_write(ctx, text, len);
}

static void cli_execute(void *ctx, char *line)
{
	(void)ctx;
	(void)shell_engine_execute(line);
}

static void session_main(void *arg)
{
	struct session *s = arg;

	shell_console_bind(&s->console);
	cli_init(&s->cli, s->prompt, cli_write, cli_execute, &s->console);
	cli_prompt(&s->cli);
	for (const char *const *line = s->lines; *line != NULL; line++) {
		type(&s->cli, *line);
		cli_input(&s->cli, '\r');
		vTaskDelay(1);
	}
	xSemaphoreGive(done);
	vTaskSuspend(NULL);
}

/* The generic commands, run by the checker on a console of its own. */
static struct shell_console generic_console;
static struct capture generic_out;

static void run(const char *line)
{
	char buf[128];

	generic_out.len = 0;
	generic_out.text[0] = '\0';
	strcpy(buf, line);
	(void)shell_engine_execute(buf);
}

static void test_generic(void)
{
	static volatile uint32_t word = 0x12345678u;
	static struct fault_record record;
	struct timespec ts = {.tv_sec = 1795350896};
	uint64_t before;
	char line[96];

	shell_console_init(&generic_console, "generic", capture_out, &generic_out);
	shell_console_bind(&generic_console);

	run("help");
	assert(strncmp(generic_out.text, "Available commands:\r\n", 21) == 0);
	assert(strstr(generic_out.text,
		      "  bootsel  : Restart into the USB bootloader (BOOTSEL)\r\n") != NULL);
	assert(strstr(generic_out.text, "  help     : Prints the help message.\r\n") != NULL);

	run("id");
	expect("id", &generic_out, "E6614104035A9B2C\r\n");

	run("uptime");
	assert(strstr(generic_out.text, " s (TIMER), 1234 ticks of 1/1000 s\r\n") != NULL);

	assert(sys_clock_settime(SYS_CLOCK_REALTIME, &ts) == 0);
	run("date");
	expect("date", &generic_out, "Sun, Nov 22 12:34:56 2026\r\n");

	before = k_cycle_get_64();
	run("sleep 30");
	assert(generic_out.len == 0 && k_cycle_get_64() - before >= 30000u);
	run("sleep 3x");
	expect("sleep", &generic_out, "Unable to parse input (err -22)\r\n");

	snprintf(line, sizeof(line), "devmem %lx", (unsigned long)(uintptr_t)&word);
	run(line);
	expect("devmem read", &generic_out, "Using data width 32\r\nRead value 0x12345678\r\n");
	snprintf(line, sizeof(line), "devmem %lx 8 ab", (unsigned long)(uintptr_t)&word);
	run(line);
	expect("devmem write", &generic_out, "Using data width 8\r\nWriting value 0xab\r\n");
	assert(word == 0x123456abu);
	run("devmem 0 7");
	expect("devmem width", &generic_out, "Using data width 7\r\nIncorrect data width\r\n");

	run("fault");
	expect("fault none", &generic_out, "No fatal error before this start\r\n");
	record.reason = FAULT_REASON_PANIC;
	record.pc = 0x10001234u;
	record.uptime_us = 12640000u;
	strcpy(record.task, "console");
	strcpy(record.text, "test");
	fault_record_seal(&record);
	test_fault = &record;
	run("fault");
	expect("fault", &generic_out,
	       "Fatal error: panic: test\r\n"
	       "In task console, 12.640000 s after the start\r\n"
	       "pc 0x10001234 lr 0x00000000 sp 0x00000000\r\n");

	run("reboot now");
	expect("reboot", &generic_out, "Rebooting\r\n");
	assert(test_reboots == 1 && test_restart_delay_ms == 100);
	run("bootsel");
	expect("bootsel", &generic_out, "Entering BOOTSEL (the RPI-RP2 drive)\r\n");
	assert(test_bootsels == 1 && test_restart_delay_ms == 100);
}

/* The serial session (common/compat/shell_engine/shell_serial.c) on this
 * directory's shell_serial_os_impl.h ---------------------------------------*/

volatile bool test_serial_open;
unsigned int test_serial_rx_inits;

static struct k_spinlock serial_lock;
static char serial_in[64];
static size_t serial_in_len;
static struct capture serial_out;

int test_serial_take(char *buf, size_t len)
{
	k_spinlock_key_t key = k_spin_lock(&serial_lock);
	size_t n = serial_in_len < len ? serial_in_len : len;

	memcpy(buf, serial_in, n);
	memmove(serial_in, serial_in + n, serial_in_len - n);
	serial_in_len -= n;
	k_spin_unlock(&serial_lock, key);
	return (int)n;
}

void test_serial_put_out(const char *buf, size_t len)
{
	assert(serial_out.len + len < sizeof(serial_out.text));
	memcpy(serial_out.text + serial_out.len, buf, len);
	serial_out.len += len;
	serial_out.text[serial_out.len] = '\0';
}

static void serial_type(const char *text)
{
	k_spinlock_key_t key = k_spin_lock(&serial_lock);

	assert(serial_in_len + strlen(text) <= sizeof(serial_in));
	memcpy(serial_in + serial_in_len, text, strlen(text));
	serial_in_len += strlen(text);
	k_spin_unlock(&serial_lock, key);
	shell_serial_rx_ready();
}

static void serial_wait_for(const char *text)
{
	for (int i = 0; i < 200 && strstr(serial_out.text, text) == NULL; i++) {
		vTaskDelay(5);
	}
}

static void serial_banner(void)
{
	printf("banner\n");
}

static void test_serial(void)
{
	shell_serial_init("serial");
	assert(shell_serial_start("s$ ", serial_banner) == 0);
	/* No terminal yet: nothing is written. */
	vTaskDelay(100);
	assert(test_serial_rx_inits == 1 && serial_out.len == 0);

	test_serial_open = true;
	serial_wait_for("s$ ");
	expect("serial open", &serial_out, "\r\nbanner\r\ns$ ");
	serial_out.len = 0;
	serial_out.text[0] = '\0';
	serial_type("id\r");
	serial_wait_for("C\r\ns$ ");
	expect("serial id", &serial_out, "id\r\nE6614104035A9B2C\r\ns$ ");
	/* Threads bound to no console now write to the serial session. */
	shell_console_bind(NULL);
	printf("to serial\n");
	assert(strstr(serial_out.text, "to serial\r\n") != NULL);
}

static void checker(void *arg)
{
	(void)arg;
	for (int i = 0; i < 2; i++) {
		xSemaphoreTake(done, portMAX_DELAY);
	}

	/* The two sessions parsed their options at the same time. */
	assert(opt_count == 4 && opt_order[0] != opt_order[1]);

	/* From a task bound to no console: the backend that "later" kept goes
	 * to session a, stdout to the default console (session b's).
	 */
	assert(kept_sh != NULL);
	shell_print(kept_sh, "from task");
	printf("default\n");

	expect("session a", &sessions[0].out,
	       "a$ opt -a 1 -b 2\r\n"
	       "opt a=1\r\nopt b=2\r\n"
	       "a$ echo hi there\r\n"
	       "argc=3\r\nerr:hi\r\nsh:there\r\ntail\r\n"
	       "a$ grp\r\n"
	       "grp - Group of commands\r\n"
	       "Subcommands:\r\n"
	       "  one  : First\r\n"
	       "  two  : Second\r\n"
	       "a$ nope\r\n"
	       "nope: command not found\r\n"
	       "a$ later\r\n"
	       "a$ from task\r\n");
	expect("session b", &sessions[1].out,
	       "b$ opt -b 7 -a 8\r\n"
	       "opt b=7\r\nopt a=8\r\n"
	       "b$ grp one\r\n"
	       "one: wrong parameter count\r\n"
	       "one - First\r\n"
	       "b$ echo 'a b' c\r\n"
	       "argc=3\r\nerr:a b\r\nsh:c\r\ntail\r\n"
	       "b$ grp two x\r\n"
	       "two ran\r\n"
	       "b$ default\r\n");
	/* No default console: dropped. */
	shell_console_set_default(NULL);
	fprintf(stdout, "dropped\n");
	assert(strstr(sessions[1].out.text, "dropped") == NULL);
	test_generic();
	test_serial();
	_exit(0);
}

int main(void)
{
	/* Before the scheduler starts, output goes to the default console. */
	static struct shell_console host;
	static struct capture host_out;

	shell_console_init(&host, "host", capture_out, &host_out);
	shell_console_set_default(&host);
	printf("before\n");
	expect("before the start", &host_out, "before\r\n");
	test_editor();

	for (int i = 0; i < 2; i++) {
		shell_console_init(&sessions[i].console, sessions[i].prompt, capture_out,
				   &sessions[i].out);
	}
	shell_console_set_default(&sessions[1].console);
	done = xSemaphoreCreateCounting(2, 0);
	for (int i = 0; i < 2; i++) {
		if (xTaskCreate(session_main, "session", SESSION_STACK / sizeof(StackType_t),
				&sessions[i], APP_PRIO_SHELL, NULL) != pdPASS) {
			return 2;
		}
	}
	if (xTaskCreate(checker, "checker", SESSION_STACK / sizeof(StackType_t), NULL,
			APP_PRIO_SHELL + 1, NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
