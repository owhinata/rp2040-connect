#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

static int cmd_hello(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	LOG_INF("hello command received");
	shell_print(sh, "Hello from Arduino Nano RP2040 Connect!");
	return 0;
}

SHELL_CMD_REGISTER(hello, NULL, "Print a greeting", cmd_hello);

/* Thread Analyzer's callback has no user context, so serialize access to it. */
K_MUTEX_DEFINE(ps_lock);
static const struct shell *ps_shell;

static void ps_print_thread(struct thread_analyzer_info *info)
{
	shell_print(ps_shell, "%-20s CPU: %3u%%  STACK: %zu / %zu bytes",
		    info->name, info->utilization, info->stack_used, info->stack_size);
}

static int cmd_ps(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_mutex_lock(&ps_lock, K_FOREVER);
	ps_shell = sh;
	thread_analyzer_run(ps_print_thread, 0);
	ps_shell = NULL;
	k_mutex_unlock(&ps_lock);
	return 0;
}

SHELL_SUBCMD_ADD((kernel), ps, NULL, "Show thread CPU and stack usage", cmd_ps, 1, 0);

int main(void)
{
	LOG_INF("USB console started");
	return 0;
}
