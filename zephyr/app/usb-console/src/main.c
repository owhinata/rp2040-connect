#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <pico/bootrom.h>
#include "nina_passthrough.h"
#include "wifi_profile.h"
#include "time_sync.h"
#include "imu_cal.h"
#include "kv_store.h"

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

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

/* Time for the shell to send the command's last line before a restart. */
#define RESTART_DELAY_MS 100

/* Restarts into the boot ROM's USB loader (the RPI-RP2 drive), as holding
 * BOOTSEL at reset does: the ROM's reset_usb_boot() (hal_rpi_pico's
 * pico_bootrom, which Zephyr's SoC code calls for its retention boot mode),
 * with both of its interfaces and no activity LED, as the FreeRTOS build's
 * "bootsel".
 */
static int cmd_bootsel(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "Entering BOOTSEL (the RPI-RP2 drive)");
	k_msleep(RESTART_DELAY_MS);
	reset_usb_boot(0, 0);
}

SHELL_CMD_REGISTER(bootsel, NULL, "Restart into the USB bootloader (BOOTSEL)", cmd_bootsel);

int main(void)
{
	int ret;

	/* "nina passthrough confirm" asked for the NINA passthrough mode
	 * (drivers/nina/nina_passthrough.h): it runs instead of all that
	 * follows (no settings, no Wi-Fi, no time synchronization), until it
	 * restarts the board into the normal firmware.
	 */
#ifdef CONFIG_NANO_NINA_PASSTHROUGH
	if (nina_passthrough_active()) {
		nina_passthrough_run();
	}
#endif

	LOG_INF("USB console started");
	ret = kv_store_init();
	if (ret) {
		LOG_ERR("Could not initialize settings: %d", ret);
	}
	ret = imu_cal_init();
	if (ret) {
		LOG_ERR("Could not set up the IMU calibration: %d", ret);
	}
	time_sync_init();
	(void)wifi_profile_init();
	return 0;
}
