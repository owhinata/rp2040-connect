/* SPDX-License-Identifier: MIT */
/* The USB console on the Pico SDK and the FreeRTOS kernel: a shell on the
 * USB CDC ACM port, the key-value store, the SPI link to the NINA-W102, I2C0
 * with its DMA reads, the LSM6DSOX, the IMU acquisition service with its
 * MQTT transport, Wi-Fi with the stored credentials, the time
 * synchronization with the NTP server, the shell over Telnet, the "zperf"
 * speed test, the "sensor" command and a task that blinks the LED.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "app_log.h"
#include "cli.h"
#include "dma_rpi_pico.h"
#include "fault.h"
#include "i2c_dw_i2c0.h"
#include "imu_cal.h"
#include "kv_store.h"
#include "log_ram.h"
#include "nano_console.h"
#include "nano_flash_kv_os.h"
#include "nano_lsm6dso.h"
#include "nano_nina.h"
#include "nina_net.h"
#include "nina_passthrough.h"
#include "nano_shell.h"
#include "nano_stdio.h"
#include "nano_telnet.h"
#include "priorities.h"
#include "time_sync.h"
#include "wifi_profile.h"
#include <zephyr/net/wifi_credentials.h>
#include "zephyr_compat.h"

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

/* The "wifi" command's SYS_INIT(APPLICATION) in the Zephyr build
 * (common/compat/zephyr_wifi_shell.c).
 */
int wifi_shell_init(void);

/* Priorities and stacks (bytes): src/priorities_os.h and
 * common/compat/priorities.h.  The console is the Zephyr build's shell
 * thread (NANO_PRIO_SHELL, 14 -> 6, 3072 bytes): the commands run in it.
 */
#define CONSOLE_PRIORITY     NANO_PRIO_SHELL
#define CONSOLE_STACK_SIZE   (NANO_STACK_SHELL / sizeof(StackType_t))
#define MAIN_PRIORITY        NANO_PRIO_MAIN
#define MAIN_STACK_SIZE      (NANO_STACK_MAIN / sizeof(StackType_t))
#define HEARTBEAT_PRIORITY   NANO_PRIO_HEARTBEAT
#define HEARTBEAT_STACK_SIZE (NANO_STACK_HEARTBEAT / sizeof(StackType_t))

#define HEARTBEAT_PERIOD_MS 500
/* The console task wakes at least this often, to notice the terminal opening
 * the port.
 */
#define CONSOLE_POLL_MS     50
#define CONSOLE_PROMPT      "freertos:~$ "
/* Characters taken from the USB code at a time (its receive buffer has 64). */
#define CONSOLE_READ_SIZE   64

/* The USB console: the terminal of the USB session and the default console
 * of the tasks that have none (freertos/drivers/shell/nano_console.h).
 */
static struct nano_console usb_console;
static struct cli usb_cli;

static StackType_t console_stack[CONSOLE_STACK_SIZE];
static StaticTask_t console_tcb;
static TaskHandle_t console_task;

static StackType_t main_stack[MAIN_STACK_SIZE];
static StaticTask_t main_tcb;

static StackType_t heartbeat_stack[HEARTBEAT_STACK_SIZE];
static StaticTask_t heartbeat_tcb;

/* The run time of the tasks ("ps"), named in FreeRTOSConfig.h.  The other
 * hooks are in fault.c (fatal errors), idle.c and tick.c.
 */
uint64_t app_run_time_us(void)
{
	return time_us_64();
}

/* The USB console's terminal: the Pico SDK's USB CDC ACM driver, which
 * drops the text while no terminal has the port open (DTR) and, when the
 * terminal stops reading, waits up to 500 ms for room in its 64-byte
 * buffer and then drops the rest until room comes again
 * (PICO_STDIO_USB_STDOUT_TIMEOUT_US).
 */
static void usb_out(struct nano_console *con, const char *buf, size_t len)
{
	(void)con;
	stdio_usb.out_chars(buf, (int)len);
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

/* Called by the Pico SDK's USB code when characters have arrived: from its
 * interrupt, or from a task that is writing to the console.
 */
static void console_chars_available(void *param)
{
	(void)param;

	if (xPortIsInsideInterrupt()) {
		BaseType_t woken = pdFALSE;

		vTaskNotifyGiveFromISR(console_task, &woken);
		portYIELD_FROM_ISR(woken);
	} else {
		xTaskNotifyGive(console_task);
	}
}

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
	fault_print(fault_last());
}

static void console_main(void *arg)
{
	char buf[CONSOLE_READ_SIZE];
	bool connected = false;
	int n;

	(void)arg;

	nano_console_bind(NULL, &usb_console);
	cli_init(&usb_cli, CONSOLE_PROMPT, cli_write, cli_execute, &usb_console);
	stdio_usb.set_chars_available_callback(console_chars_available, NULL);

	for (;;) {
		/* The terminal has opened the port (DTR): show the prompt. */
		if (stdio_usb_connected() != connected) {
			connected = !connected;
			if (connected) {
				cli_write(&usb_console, "\r\n", 2);
				console_banner();
				cli_prompt(&usb_cli);
			}
		}
		while ((n = stdio_usb.in_chars(buf, sizeof(buf))) > 0) {
			for (int i = 0; i < n; i++) {
				cli_input(&usb_cli, buf[i]);
			}
		}
		/* Until characters arrive; the timeout is for noticing the terminal. */
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONSOLE_POLL_MS));
	}
}

static void start_thread(int ret, const char *name)
{
	if (ret) {
		LOG_ERR("Could not start %s: %d", name, ret);
	}
}

/* The Zephyr build's main() (zephyr/app/usb-console/src/main.c), as the task
 * "main" with its priority (NANO_PRIO_MAIN: Zephyr's 0, above the console) and
 * stack, ending when done as Zephyr's main thread does.  It runs before the
 * console's task, which has a lower priority, as the Zephyr build's main()
 * runs before its shell thread.
 * First the LSM6DSOX (lsm6dsox@6a), which Zephyr initializes in its main
 * thread before main() (POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY 90) and the
 * NuttX build in its bringup, after I2C0: it needs a task (the driver sleeps
 * and waits for I2C0's interrupt), so not main() below, which runs before
 * the scheduler.  Its lines ("Initialize device lsm6dsox@6a", "chip id
 * 0x6c") come first in "dmesg", as in the Zephyr and NuttX builds.  A failure
 * leaves the device not ready (the driver logs why; without i2c0 nothing is
 * tried) and the start goes on (freertos/drivers/lsm6dso/nano_lsm6dso.h).
 * While the driver waits, the console's task may run; a command that uses
 * the sensor before it is ready gets the error of a device that is not.
 * The settings are first initialized by wifi_credentials_init(), as the
 * Wi-Fi credentials' SYS_INIT() does it in the Zephyr build before main():
 * settings_subsys_init() checks what the key-value area holds, mounts NVS
 * on it (the "fs_nvs" lines of "dmesg") and reads the settings' handlers;
 * kv_store_init() at main()'s point calls it again, which does nothing more.
 * Data that is neither Zephyr's NVS nor erased is not touched: the check
 * logs it and every call of the store fails with -EILSEQ until "kv wipe
 * confirm" (common/compat/zephyr_flash_kv.c), and the Wi-Fi part starts
 * without stored networks.
 */
static void main_main(void *arg)
{
	int ret;

	(void)arg;

	(void)nano_lsm6dso_initialize();

	/* The NINA-W102's network interface, which Zephyr initializes at boot
	 * (POST_KERNEL, after the sensor) and the NuttX build in its bringup
	 * after the sensor: the event thread "net_mgmt", then the interface,
	 * dormant, with the MAC address read from the module (its first
	 * command, which resets it: about 760 ms; a module that does not
	 * answer only fails that request, freertos/drivers/nina/nina_net.h).
	 */
	ret = nina_net_initialize();
	if (ret) {
		LOG_ERR("Could not set up the NINA's network interface: %d", ret);
	}

	/* The Wi-Fi credentials library's SYS_INIT(POST_KERNEL), which Zephyr
	 * runs before the threads below start: initializes the settings (mounts
	 * NVS on the key-value area; data that is not Zephyr's NVS is left
	 * alone, and the call fails with -EILSEQ) and reads the stored SSIDs.
	 * Then the "wifi" command's SYS_INIT(APPLICATION): its event callback.
	 */
	(void)wifi_credentials_init();
	(void)wifi_shell_init();

	/* The Telnet shell backend's SYS_INIT(APPLICATION, 95), after the
	 * "wifi" command's: the listening socket on port 23 (whether or not
	 * Wi-Fi is connected; the NINA keeps it across disconnections) and the
	 * tasks "telnet_service" and "shell_telnet"
	 * (freertos/drivers/telnet/nano_telnet.h).  A failure is logged and
	 * leaves no Telnet.
	 */
	(void)nano_telnet_init(CONSOLE_PROMPT);

	/* The threads of K_THREAD_DEFINE(), which Zephyr starts before main()
	 * (z_init_static_threads(), after the devices) and the NuttX build in
	 * its init task at this point, in the NuttX build's order: the NINA
	 * driver's (common/drivers/nina/nina_net.c: scan, connect, link
	 * monitor), the IMU service's (common/imu/imu_stream.c), its MQTT
	 * transport's (common/imu/imu_raw_mqtt.c), the time synchronization's
	 * (common/imu/time_sync.c), the Wi-Fi profile's reconnection
	 * (common/imu/wifi_profile.c) and the sensor shell's processing thread
	 * (common/compat/sensor_shell_stream.c).  Each waits for its work at
	 * once (the monitor polls every 2 s while connected; the transport
	 * waits for "imu start"; the time synchronization waits for the Wi-Fi
	 * events of time_sync_init(); the processing thread waits for the
	 * completions of "sensor get" and "sensor stream").  They have lower
	 * priorities than this task, or its own (the processing thread, Zephyr's
	 * 0), so they first run when this task waits.
	 */
	start_thread(K_THREAD_DEFINE_START(nina_scan_tid), "nina_scan_tid");
	start_thread(K_THREAD_DEFINE_START(nina_connect_tid), "nina_connect_tid");
	start_thread(K_THREAD_DEFINE_START(nina_monitor_tid), "nina_monitor_tid");
	start_thread(K_THREAD_DEFINE_START(imu_stream_tid), "imu_stream_tid");
	start_thread(K_THREAD_DEFINE_START(imu_raw_mqtt_tid), "imu_raw_mqtt_tid");
	start_thread(K_THREAD_DEFINE_START(time_sync_tid), "time_sync_tid");
	start_thread(K_THREAD_DEFINE_START(wifi_reconnect_tid), "wifi_reconnect_tid");
	start_thread(K_THREAD_DEFINE_START(sensor_shell_processing_tid),
		     "sensor_shell_processing_tid");
	/* The iperf3 server of "zperf tcp download" (common/compat/
	 * zperf_iperf3.h), Zephyr's net_socket_service thread: waits until
	 * the server is started.
	 */
	start_thread(K_THREAD_DEFINE_START(zperf_server_tid), "zperf_server_tid");

	LOG_INF("USB console started");
	ret = kv_store_init();
	if (ret) {
		LOG_ERR("Could not initialize settings: %d", ret);
	}
	ret = imu_cal_init();
	if (ret) {
		LOG_ERR("Could not set up the IMU calibration: %d", ret);
	}
	/* The time synchronization's Wi-Fi events (common/imu/time_sync.c):
	 * from a connection on, time_sync_tid asks the NTP server (the default
	 * gateway, port 123) every 2 s; a disconnection stops it.  Before
	 * wifi_profile_init(), so that the automatic connection's event is
	 * seen, as in the Zephyr and NuttX builds.
	 */
	time_sync_init();
	/* The automatic connection: the event callback, the migration of an
	 * old profile to the stored credentials, and a request to the
	 * reconnection thread, which connects to a stored network 4 s later
	 * and retries every 32 s while there is no address (nothing when no
	 * network is stored).
	 */
	(void)wifi_profile_init();
	vTaskDelete(NULL);
}

static void heartbeat_main(void *arg)
{
	TickType_t next = xTaskGetTickCount();

	(void)arg;

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	for (;;) {
		gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
		xTaskDelayUntil(&next, pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
	}
}

int main(void)
{
	bool passthrough;
	int ret;

	/* First: the paint for the interrupt stack's high-water mark, the RAM
	 * log kept across the restart, the request of the NINA passthrough
	 * mode (in the watchdog's scratch registers 0..3, which fault_boot()
	 * then clears) and the record the last run may have left (fault_boot()
	 * also puts it into the RAM log).
	 */
	irq_stack_paint();
	log_ram_initialize();
	passthrough = nina_passthrough_requested();
	fault_boot();

	/* "nina passthrough confirm" asked for the NINA passthrough mode
	 * (freertos/drivers/nina/nina_passthrough.h): it runs instead of all
	 * that follows (no task, no SPI1, no Wi-Fi), until it restarts the
	 * board into the normal firmware.
	 */
	if (passthrough) {
		nina_passthrough_run();
	}

	/* The USB CDC ACM port, and the stdio of the tasks on the consoles
	 * (freertos/drivers/shell/nano_stdio.h).
	 */
	stdio_init_all();
	nano_console_init(&usb_console, "usb", usb_out, NULL);
	nano_console_set_default(&usb_console);
	nano_stdio_init(&stdio_usb);

	/* heap_4 sets up its heap at the first allocation and reports no free
	 * space until then ("free"). Nothing else here allocates from it.
	 */
	vPortFree(pvPortMalloc(1));

	/* What Zephyr starts before main(): the system work queue ("sysworkq"),
	 * the RTIO work queue ("RTIO WQ 0") and the gpio0 device of the
	 * compatibility layer (freertos/drivers/compat).
	 */
	if (zephyr_compat_initialize() != 0) {
		panic("zephyr_compat_initialize");
	}

	/* SPI1 to the NINA-W102 and its chip select, which the Zephyr build sets
	 * up at boot (its spi1 device, before main()) and the NuttX build in its
	 * bringup (freertos/drivers/nina/nano_nina.c).  Nothing is sent to the
	 * module here: the first request resets it (about 760 ms) and a module
	 * that does not answer only makes that request fail
	 * (common/drivers/nina/nina_spi.c), so the start never waits for it.
	 */
	ret = nano_nina_initialize();
	if (ret != 0) {
		LOG_ERR("Could not set up the NINA's SPI bus: %d", ret);
	}

	/* The DMA for the I2C0 reads (reset, both interrupts) and then I2C0
	 * (reset, pins, controller registers for 100 kHz, interrupt; the device
	 * i2c0), as the Zephyr build initializes its DMA device (POST_KERNEL 40)
	 * before i2c0 (POST_KERNEL 50) and the NuttX build in its bringup, after
	 * the NINA's SPI bus (freertos/drivers/i2c_dw).  A failure leaves i2c0
	 * not ready (its calls return -ENODEV) and does not stop the start.
	 */
	ret = dma_rpi_pico_initialize();
	if (ret != 0) {
		LOG_ERR("Could not set up the DMA: %d", ret);
	}
	ret = i2c_dw_i2c0_initialize();
	if (ret != 0) {
		LOG_ERR("Could not set up I2C0: %d", ret);
	}

	/* The bounds of the key-value area from the linker script; until they
	 * are found right, the store refuses to erase or program the flash
	 * (freertos/drivers/kv).  After I2C0, where the NuttX build's bringup
	 * checks its flash; the store itself is mounted by the task "main".
	 */
	(void)nano_flash_kv_initialize();

	xTaskCreateStatic(main_main, "main", MAIN_STACK_SIZE, NULL, MAIN_PRIORITY, main_stack,
			  &main_tcb);
	console_task = xTaskCreateStatic(console_main, "console", CONSOLE_STACK_SIZE, NULL,
					 CONSOLE_PRIORITY, console_stack, &console_tcb);
	xTaskCreateStatic(heartbeat_main, "heartbeat", HEARTBEAT_STACK_SIZE, NULL,
			  HEARTBEAT_PRIORITY, heartbeat_stack, &heartbeat_tcb);

	/* The tick hook's catch-up (tick.c, tick_catchup.h) takes SysTick's
	 * period (clock_get_hz(clk_sys) / configTICK_RATE_HZ cycles, the RP2040
	 * port's vPortSetupTimerInterrupt()) for exactly one tick of the TIMER:
	 * a shorter one would let the count run ahead of the time.
	 */
	if (clock_get_hz(clk_sys) % configTICK_RATE_HZ != 0) {
		panic("clk_sys is not a whole number of ticks");
	}

	vTaskStartScheduler();

	for (;;) {
	}
}
