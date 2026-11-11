#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "wifi_profile.h"
#include "../drivers/nina/nina_proto.h"
#include "time_sync.h"
#include "kv_store.h"

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

static int cmd_nina_version(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t version[16];
	size_t len;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ret = nina_request(0x37, NULL, 0, false, false, version, sizeof(version) - 1, &len);
	if (ret) {
		shell_error(sh, "NINA version request failed: %d", ret);
		return ret;
	}
	version[len] = '\0';
	shell_print(sh, "NINA firmware: %s", version);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(nina_commands,
	SHELL_CMD(version, NULL, "Show NINA firmware version", cmd_nina_version),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(nina, &nina_commands, "NINA Wi-Fi module", NULL);

/* Thread Analyzer's callback has no user context, so serialize access to it. */
K_MUTEX_DEFINE(ps_lock);
static const struct shell *ps_shell;

static void ps_print_thread(struct thread_analyzer_info *info)
{
	shell_print(ps_shell, "%-20s CPU: %3u%%  STACK: %zu / %zu bytes",
		    info->name, info->utilization, info->stack_used,
		    info->stack_size);
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

SHELL_SUBCMD_ADD((kernel), ps, NULL, "Show thread CPU and stack usage", cmd_ps,
		 1, 0);

int main(void)
{
	int ret;

	LOG_INF("USB console started");
	ret = kv_store_init();
	if (ret) {
		LOG_ERR("Could not initialize settings: %d", ret);
	}
	time_sync_init();
	(void)wifi_profile_init();
	return 0;
}
