/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's shell (freertos/drivers/shell and
 * the shell engine common/compat/shell_engine, with
 * freertos/drivers/libc/stdio_wrap.c and the shared common/shell/zshell.c,
 * subcmd.c, the copies of Zephyr's fnmatch() and getopt) on the FreeRTOS
 * POSIX port:
 *
 * - the line editor alone (cli.c): echo, Backspace, Ctrl-C, history with
 *   Up and Down, other escape sequences ignored;
 * - two sessions at once: two consoles that capture their output, each with
 *   a task bound to it that types its lines into its own line editor and
 *   runs them (nano_shell_execute()).  The commands are registered with
 *   SHELL_CMD_ARG_REGISTER() as the shared sources do and print with
 *   printf(), fprintf(stderr), fputs() and shell_print(); the shell prints
 *   help, the argument count error and "command not found"; one command
 *   parses options with sys_getopt() and sleeps after each, so the two
 *   sessions parse at the same time.  Each console must hold exactly its own
 *   session's transcript (with "\r\n" line ends);
 * - output to a session's backend from another task (a struct shell kept by
 *   a command), and the default console of a task bound to none.
 *
 * The Pico SDK's stdio is played by this file as pico_stdio does it:
 * printf(), vprintf(), puts() and putchar() are wrapped (-Wl,--wrap) and,
 * like the SDK's newlib write(), end in stdio_put_string(), which writes to
 * the calling task's console (freertos/drivers/shell/nano_stdio.c on the
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

#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_getopt.h>

#include "app_log.h"
#include "cli.h"
#include "nano_console.h"
#include "nano_shell.h"
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

/* The allocations of zshell.c and nano_console.c (built with
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

static void capture_out(struct nano_console *con, const char *buf, size_t len)
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

const struct nano_shell_cmd nano_shell_cmds[] = {
	NANO_SHELL_CMD(echo),
	NANO_SHELL_CMD(grp),
	NANO_SHELL_CMD(later),
	NANO_SHELL_CMD(opt),
};

const size_t nano_shell_cmd_count = sizeof(nano_shell_cmds) / sizeof(nano_shell_cmds[0]);

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
	struct nano_console console;
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

	nano_console_bind(NULL, &s->console);
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
	nano_console_set_default(NULL);
	fprintf(stdout, "dropped\n");
	assert(strstr(sessions[1].out.text, "dropped") == NULL);
	_exit(0);
}

int main(void)
{
	/* Before the scheduler starts, output goes to the default console. */
	static struct nano_console host;
	static struct capture host_out;

	nano_console_init(&host, "host", capture_out, &host_out);
	nano_console_set_default(&host);
	printf("before\n");
	expect("before the start", &host_out, "before\r\n");
	test_editor();

	for (int i = 0; i < 2; i++) {
		nano_console_init(&sessions[i].console, sessions[i].prompt, capture_out,
				  &sessions[i].out);
	}
	nano_console_set_default(&sessions[1].console);
	done = xSemaphoreCreateCounting(2, 0);
	for (int i = 0; i < 2; i++) {
		if (xTaskCreate(session_main, "session", SESSION_STACK / sizeof(StackType_t),
				&sessions[i], NANO_PRIO_SHELL, NULL) != pdPASS) {
			return 2;
		}
	}
	if (xTaskCreate(checker, "checker", SESSION_STACK / sizeof(StackType_t), NULL,
			NANO_PRIO_SHELL + 1, NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
