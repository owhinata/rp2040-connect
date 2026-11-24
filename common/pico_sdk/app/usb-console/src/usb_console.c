/* SPDX-License-Identifier: MIT */
/* The USB console on the Pico SDK, shared by the OSes on it: a shell on the
 * USB CDC ACM port, the key-value store, the SPI link to the NINA-W102, I2C0
 * with its DMA reads (and I2C1 as a build option), the LSM6DSOX, the IMU
 * acquisition service with its MQTT transport, Wi-Fi with the stored
 * credentials, the time synchronization with the NTP server, the shell over
 * Telnet, the "zperf" speed test, the "sensor" command and the LED's blink
 * from the idle pass (idle.c).  These are the steps of its start; the OS's
 * main() calls them in order (usb_console.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "app.h"
#include "app_log.h"
#include "app_main.h"
#include "fault.h"
#include "i2c_dw_controllers_rp2040.h"
#include "log_ram.h"
#include "flash_kv_os.h"
#include "lsm6dso_board.h"
#include "nina_passthrough.h"
#include "shell_serial.h"
#include "shell_telnet.h"
#include "usb_console.h"
#include "zephyr_compat.h"
#include "zephyr_net_init.h"

LOG_MODULE_DECLARE(usb_console, LOG_LEVEL_INF);

/* The "wifi" command's SYS_INIT(APPLICATION) in the Zephyr build
 * (common/compat/zephyr_wifi_shell.c).
 */
int wifi_shell_init(void);

/* The record of the fatal error that ended the last run, once, at the first
 * connection of the terminal after the start ("fault" shows it again, and
 * the RAM log has it, "dmesg").
 */
static void console_banner(void)
{
	static bool shown;

	if (shown || fault_last() == NULL) {
		return;
	}
	shown = true;
	printf("*** The last run ended with a fatal error (\"fault\" shows it again):\n");
	fault_record_print(fault_last());
}

void usb_console_early_init(void)
{
	bool passthrough;

	/* First: the paint for the interrupt stack's high-water mark, the RAM
	 * log kept across the restart, the request of the NINA passthrough
	 * mode (in the watchdog's scratch registers 0..3, which fault_boot()
	 * then clears) and the record the last run may have left (fault_boot()
	 * also puts it into the RAM log).
	 */
	irq_stack_paint();
	log_ram_initialize();
	passthrough = nina_passthrough_check();
	fault_boot();

	/* "nina passthrough confirm" asked for the NINA passthrough mode
	 * (common/drivers/nina/include/nina_passthrough.h): it runs instead of all
	 * that follows (no kernel, no thread, no SPI1, no Wi-Fi), until it
	 * restarts the board into the normal firmware.
	 */
	if (passthrough) {
		nina_passthrough_run();
	}
}

void usb_console_init(void)
{
	int ret;

	/* The USB CDC ACM port, its console "usb" (the default console of the
	 * threads that have none) and the stdio of the threads on the consoles
	 * (common/compat/shell_engine/shell_serial.h on the console transport
	 * common/pico_sdk/drivers/shell/shell_serial_os_impl.h).  No wait for
	 * the host (PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 0): the enumeration
	 * runs once the scheduler does.
	 */
	stdio_init_all();
	shell_serial_init("usb");

	/* What Zephyr starts before main(): the system work queue ("sysworkq"),
	 * the RTIO work queue ("RTIO WQ 0") and the gpio0 device of the
	 * compatibility layer (common/compat and the OS's part).
	 */
	if (zephyr_compat_initialize() != 0) {
		panic("zephyr_compat_initialize");
	}

	/* I2C0 (the DMA interrupt's handler for its DMA reads, then reset, pins,
	 * controller registers for 100 kHz, interrupt; the device i2c0; no
	 * transfer), as the Zephyr build initializes its DMA device
	 * (POST_KERNEL 40) before i2c0 (POST_KERNEL 50) and the NuttX build in
	 * its bringup, after the NINA's SPI bus (common/pico_sdk/drivers/i2c_dw).
	 * A failure leaves i2c0 not ready (its calls return -ENODEV) and does
	 * not stop the start.
	 */
#ifdef CONFIG_I2C_DW_I2C0
	ret = i2c_dw_i2c0_initialize();
	if (ret != 0) {
		LOG_ERR("Could not set up I2C0: %d", ret);
	}
#endif

#ifdef CONFIG_I2C_DW_I2C1
	/* I2C1 (the device i2c1), after i2c0 as Zephyr's devicetree order has
	 * them; a failure leaves i2c1 not ready and nothing else changes.
	 */
	ret = i2c_dw_i2c1_initialize();
	if (ret != 0) {
		LOG_ERR("Could not set up I2C1: %d", ret);
	}
#endif

	/* The bounds of the key-value area from the linker script; until they
	 * are found right, the store refuses to erase or program the flash
	 * (common/pico_sdk/drivers/kv).  No flash access.  After I2C0, where
	 * the NuttX build's bringup checks its flash; the store itself is
	 * mounted by the thread "main".
	 */
	(void)flash_kv_os_initialize();
}

/* What the Zephyr build's main thread runs after the kernel's start, as the
 * thread "main" with its priority (APP_PRIO_MAIN: Zephyr's 0, above the
 * console) and stack.  It runs before the console's thread, which has a
 * lower priority, as the Zephyr build's main() runs before its shell
 * thread.  The devices that need a thread come first, then the
 * application's start in its shared order (common/imu/app_main.h) with this
 * build's SYS_INIT(APPLICATION) parts.  First the LSM6DSOX (lsm6dsox@6a),
 * which Zephyr initializes in its main thread before main() (POST_KERNEL,
 * CONFIG_SENSOR_INIT_PRIORITY 90) and the NuttX build in its bringup, after
 * I2C0: it needs a thread (the driver sleeps and waits for I2C0's
 * interrupt), so not usb_console_init(), which runs before the scheduler.
 * Its lines ("Initialize device lsm6dsox@6a", "chip id 0x6c") come first in
 * "dmesg", as in the Zephyr and NuttX builds.  A failure leaves the device
 * not ready (the driver logs why; without i2c0 nothing is tried) and the
 * start goes on (common/drivers/lsm6dso/port/pico_sdk/lsm6dso_board.h).
 * While the driver waits, the console's thread may run; a command that uses
 * the sensor before it is ready gets the error of a device that is not.
 * Data in the key-value area that is neither Zephyr's NVS nor erased is not
 * touched: the settings' check logs it and every call of the store fails
 * with -EILSEQ until "kv wipe confirm" (common/compat/zephyr_flash_kv.c),
 * and the Wi-Fi part starts without stored networks.
 */
void usb_console_main_thread(const char *prompt)
{
	int ret;

	(void)lsm6dso_board_initialize();

	/* The NINA-W102's network interface, which Zephyr initializes at boot
	 * (POST_KERNEL, after the sensor) and the NuttX build in its bringup
	 * after the sensor: the event thread "net_mgmt", then the interface,
	 * dormant, with the MAC address read from the module (its first
	 * command, which resets it: about 760 ms; a module that does not
	 * answer only fails that request), as Zephyr's net_init() does it
	 * (the layer's zephyr_net_initialize(), zephyr_net_init.h).
	 */
	ret = zephyr_net_initialize();
	if (ret) {
		LOG_ERR("Could not set up the NINA's network interface: %d", ret);
	}

	/* The Wi-Fi credentials (the settings and NVS, the "fs_nvs" lines of
	 * "dmesg"), then the "wifi" command's SYS_INIT(APPLICATION): its
	 * event callback.
	 */
	app_init_post_kernel();
	(void)wifi_shell_init();

	/* The Telnet shell backend's SYS_INIT(APPLICATION, 95), after the
	 * "wifi" command's: the listening socket on port 23 (whether or not
	 * Wi-Fi is connected; the NINA keeps it across disconnections) and the
	 * threads "shell_telnet" and "telnet_service"
	 * (common/compat/shell_engine/shell_telnet.h).  A failure is logged and
	 * leaves no Telnet.
	 */
	(void)shell_telnet_init(prompt);

	app_threads_start();
	app_main();
}

void usb_console_shell_start(const char *prompt)
{
	/* The USB session, the thread "console" (shell_serial.h). */
	if (shell_serial_start(prompt, console_banner) != 0) {
		panic("shell_serial_start");
	}
	/* The LED, which the idle pass blinks once the scheduler runs. */
	idle_led_init();
}
