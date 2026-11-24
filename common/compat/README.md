# Zephyr API compatibility layer (`compat`)

English | [日本語](README_ja.md)

The firmware of this repository is the same firmware built on four OSes, Zephyr, NuttX, Pico SDK + FreeRTOS and
Pico SDK + ThreadX, called the Zephyr, NuttX, FreeRTOS and ThreadX builds. The shared sources (`common/`) are
written on Zephyr's API. The compatibility layer provides, under Zephyr's names and no more than needed, the
part of Zephyr's API that those sources (the copied drivers and their callers: the acquisition service, the
calibration, the sensor command and others) use; the NuttX, FreeRTOS and ThreadX builds build the shared sources
on it. The Zephyr build does not use it (it has Zephyr itself). Zephyr's sensor subsystem, RTIO and kernel as a
whole are not ported.

The kernel part is made in one of two ways. The NuttX build implements Zephyr's functions (`k_sem_*()`,
`k_mutex_*()`, `k_poll()` and the others) directly on NuttX. The FreeRTOS and ThreadX builds implement only the
primitives (`port/zephyr_kernel_os.h`: 10 types and constants and 26 functions) that the generic implementation
(`kernel_generic/`) uses. The list of what a new OS writes is in
[`design/os-boundary.md`](../../design/os-boundary.md) (not copied here). The details are in the comment at the
top of each file.

## Directories

The layer has one set of the three layers that the other parts of `common/` have (`include/`, `internal/`,
`port/`). The kernel part, the log, the shell engine, the network part and the POSIX network headers are
subdirectories that separate the sources; they do not have three layers each.

| Place | Contents |
|---|---|
| Top | The OS-independent sources: work items (`zephyr_work.c`), the work queues' threads and the start (`zephyr_init.c`), RTIO (`zephyr_rtio.c`, `zephyr_rtio_workq.c`), the block pool (`zephyr_mem_blocks.c`), `k_heap` (`zephyr_heap.c`), the device list (`zephyr_device.c`), `gpio0` (`zephyr_gpio.c`), the sensor clock (`zephyr_sensor_clock.c`), the sensors' default iodev (`sensor_default_rtio.c`) and the sensor shell's processing thread (`sensor_shell_stream.c`), `net_mgmt` and the Wi-Fi requests (`zephyr_net_mgmt.c`, `zephyr_wifi_mgmt.c`, `zephyr_wifi_utils.c`), the Wi-Fi credentials (`wifi_credentials.c`, `wifi_credentials_backend_settings.c`), the key-value store's flash (`zephyr_flash_kv.c`, `kv_store_wipe.c`), and what only the builds on the Pico SDK compile (`zephyr_wifi_shell.c`, `zephyr_zperf_shell.c`, `zperf_iperf3.c`) |
| `include/` | Headers other parts read: the headers of Zephyr's API (`include/zephyr/...`) and the layer's own (`zephyr_compat.h`, `priorities.h`, `app_log.h`, `flash_kv.h` and others) |
| `internal/` | The layer's inner headers (`zephyr_os_internal.h`, `log_ring.h`, `cli.h`, `shell_argv.h`, `wifi_credentials_internal.h`, `zperf_iperf3.h`) |
| `port/` | The boundary headers an OS implements: the primitives of the generic kernel part (`zephyr_kernel_os.h`), `cpu_load.h`, `gpio_os.h`, `flash_kv_os.h`, `log_ram_os.h`, `shell_serial_os.h` |
| `port/nuttx/` | The NuttX implementation ("NuttX" below) |
| `port/freertos/`, `port/threadx/` | The primitives of FreeRTOS and ThreadX (`zephyr_kernel_os_impl.h`), the priority mapping (`priorities_os.h`), the port header (`rp2040/zephyr_os_port.h`); for ThreadX also the lock's variables (`zephyr_kernel_os.c`) |
| `port/pico_sdk/` | What the builds on the Pico SDK (the FreeRTOS and ThreadX builds) share ("Pico SDK" below) |
| `port/generic/` | The parts for an OS without the model of its own (the FreeRTOS and ThreadX builds; "Pico SDK" below) |
| `kernel_generic/` | The kernel part for an OS that offers only basic primitives, OS-independent: `k_sem`, `k_poll()`, the timeouts, the sleeps and the start of a thread (`zephyr_kernel.c`), `k_mutex` (`zephyr_mutex.c`) |
| `log/` | The RAM log (`log_ram.c`, on its ring `log_ring.c`) and the format of the fatal error record (`fault_record.c`). The NuttX, FreeRTOS and ThreadX builds share the RAM log; the FreeRTOS and ThreadX builds use the fatal error record |
| `shell_engine/` | The shell for an OS without one of its own ("Shell engine" below) |
| `net_offload/` | The network part for an OS without a network stack of its own ("Socket layer (`net_offload`)" below), with its headers in its own `include/` |
| `libc_net/` | The socket headers newlib lacks (`include/`, types and constants only), `inet_pton` and `inet_ntop` (`inet.c`) |
| `subsys/`, `lib/` | Copies of Zephyr's files: NVS, settings, CRC-8, the hex conversions ("Key-value store" below), `fnmatch()`, getopt, the `wifi cred` command (`wifi_credentials_shell.c`) |

Each header of `include/`, `internal/`, `port/` and `port/generic/` belongs to the layer as a whole or to one of
the parts (`kernel_generic`, the part of `port/generic` for an OS without the model of its own, `log`,
`shell_engine`). Where each belongs is listed in `common/tests/run.sh`, and a header that is in none, or in two, fails.
The same place checks that a part's sources read no header of the parts they must not need ("Tests" below).

## What it provides

### Headers

| Header (in `common/compat/include/` unless marked; (NuttX): in `common/compat/port/nuttx/`) | Contents |
|---|---|
| `zephyr/device.h` | `struct device` (name, config, API, state, data, init function), `device_is_ready()`, `device_init()` (the meaning of Zephyr's `do_device_init()`), `DEVICE_DT_GET()`, `DEVICE_DT_DEFINE()`, `DEVICE_API()`, `device_get_binding()` (node name), `device_get_by_dt_nodelabel()` (node label), `DEVICE_API_IS(sensor, dev)` (below) |
| `zephyr/devicetree.h` | `DT_NODELABEL()`, `DT_BUS()`; the nodes are `gpio0`, `i2c0` and `lsm6dsox` only. `DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` (Zephyr's definition): 1 for `st_lsm6dso` on `i2c`, 0 for any other compatible or bus (`lsm6dso.h`). `DT_HAS_CHOSEN()` is always 0 (the Zephyr build chooses no `zephyr,settings-partition` either) |
| `zephyr/kernel.h` | `k_timeout_t` (`K_NO_WAIT`, `K_FOREVER`, `K_MSEC()`, `K_SECONDS()`; in the OS's ticks), `k_mutex`, `k_sem` (`K_SEM_DEFINE`), `k_poll()` on semaphores (`K_POLL_TYPE_SEM_AVAILABLE`, `K_POLL_MODE_NOTIFY_ONLY`), work (`k_work`, `k_work_delayable`, `k_work_q`, `k_work_submit()`, `k_work_cancel_sync()`, `k_work_schedule_for_queue()`, `k_work_reschedule_for_queue()`, `k_work_queue_start()`), `K_THREAD_STACK_DEFINE()`, `K_PRIO_PREEMPT()`, `k_thread_name_set()`, the system work queue `k_sys_work_q`, `K_MUTEX_DEFINE()`, `K_THREAD_DEFINE()` (below), `k_current_get()`, `k_uptime_get()`, `k_sleep()`, `k_msleep()`, `k_usleep()`, `k_busy_wait()`, `k_cycle_get_32()`, `k_cycle_get_64()`, `k_cyc_to_us_floor64()`, `k_cyc_to_us_ceil64()`, `k_cyc_to_ns_floor64()`, `sys_clock_hw_cycles_per_sec()`, `k_heap` (`zephyr_heap.h`, with either kernel part). The full list is in the comment at the top of this header |
| `zephyr/spinlock.h`, `zephyr/sys/atomic.h` | `k_spin_lock()`, `atomic_t` and `atomic_get/set/clear/cas/or/inc/add()` |
| `zephyr/rtio/rtio.h`, `zephyr/rtio/work.h` | RTIO (below) and the RTIO work queue (`rtio_work_req_alloc()`, `_submit()`) |
| `zephyr/drivers/gpio.h` | `struct gpio_dt_spec`, `gpio_is_ready_dt()`, `gpio_pin_configure_dt()` (inputs only), `gpio_pin_interrupt_configure_dt()`, `gpio_pin_get_dt()`, callbacks (`gpio_init_callback()`, `gpio_add_callback()`, `gpio_remove_callback()`) |
| `zephyr/drivers/sensor.h` | `struct sensor_value`, the channel, attribute and trigger enumerations (Zephyr's numbers), `struct sensor_driver_api`, `sensor_attr_set()`/`_get()`, `sensor_sample_fetch()`/`_chan()`, `sensor_channel_get()`, `sensor_trigger_set()`, the unit conversions. The async API: `struct sensor_chan_spec`, the decoders (`struct sensor_decoder_api`, `sensor_three_axis_data` and others), `SENSOR_DT_READ_IODEV()`, `SENSOR_DT_STREAM_IODEV()`, `sensor_get_decoder()`, `sensor_stream()`, `sensor_read()`, `sensor_read_async_mempool()`, `sensor_processing_with_callback()`. The print formats of the decoded data, `PRIsensor_*_data` (Zephyr's `sensor_data_types.h`; the Q value format is in `dsp/print_format.h`) |
| `zephyr/drivers/sensor_clock.h` | `sensor_clock_get_cycles()`, `sensor_clock_cycles_to_ns()` ("Sensor clock" below) |
| `zephyr/drivers/i2c.h` (NuttX and `port/generic/`) | `struct i2c_dt_spec`, `i2c_burst_read_dt()`, `i2c_write_dt()`, `i2c_write_read_dt()`, `i2c_configure()`, `i2c_get_config()`, the speed classes (`I2C_SPEED_*`, `I2C_SPEED_SET()`); for the `i2c` command `struct i2c_msg` and the message flags (`I2C_MSG_*`), `i2c_transfer()`, `i2c_write()`, `i2c_read()`, `i2c_write_read()`, `i2c_recover_bus()`, `i2c_target_driver_register()` and `_unregister()` (`-ENOSYS`). The NuttX part is in "NuttX" below; the one in `port/generic/` is Zephyr's I2C API on the driver's device ([I2C driver](../drivers/i2c_dw/README.md)) |
| `zephyr/sys/util.h`, `sys/mpsc_lockfree.h`, `sys/mem_blocks.h`, `dsp/types.h` | `BIT()`, `ARRAY_SIZE()`, `MIN()`/`MAX()`, `IS_ENABLED()`, `CONTAINER_OF()`, `GENMASK()`/`GENMASK64()`/`FIELD_GET()`, `STRINGIFY()`, `KB()`, `ZTESTABLE_STATIC`, `hex2bin()`/`bin2hex()` (implemented by the copy of `lib/utils/hex.c`) and others (it also includes `<string.h>`, as Zephyr's does), RTIO's queues and block pool, `q31_t` |
| `zephyr/sys/__assert.h` (NuttX and `port/generic/`) | `__ASSERT()`, `__ASSERT_NO_MSG()` (NuttX's `DEBUGASSERT()` in the NuttX build; `assert()` in the host tests) |
| `zephyr/dsp/print_format.h` | A copy of Zephyr's header of that name (with only a leading comment and the include of `<zephyr/sys/util.h>` added): the Q value print format `PRIq()`, `PRIq_arg()` |
| `zephyr/types.h` | `<stddef.h>` and `<stdint.h>` (what `lsm6dso_sensor.h` reads it for) |
| `zephyr/init.h` | An empty header (`lsm6dso_sensor.c` includes it; `SYS_INIT()` and the init levels are used only by the Zephyr version's instance macros) |
| `zephyr/sys/iterable_sections.h` | `STRUCT_SECTION_FOREACH()` only, over an always empty section (the port has no Zephyr linker sections). The sensor shell includes it (and uses it only with `CONFIG_SENSOR_INFO`, which the Zephyr build does not set); the settings walk the section of static handlers (none in the port) |
| `zephyr/sys/printk.h`, `zephyr/sys/byteorder.h`, `zephyr/sys/cpu_load.h` | `snprintk()` (`snprintf()`, as in the Zephyr version's configuration with `CONFIG_PICOLIBC=y`), `sys_put_be16/32()`/`sys_get_be16/32()`, `cpu_load_get()` (`port/cpu_load.h`; the board implements it) |
| `zephyr/drivers/hwinfo.h` | `hwinfo_get_device_id()` (the NuttX build's in "NuttX" below; the builds on the Pico SDK: `port/pico_sdk/zephyr_hwinfo.c`) |
| `zephyr/shell/shell.h` | `shell_print()`, `shell_info()`, `shell_warn()`, `shell_error()`, `shell_fprintf()`, `struct shell` (a shell backend; its members are the OS part `zephyr_os_shell.h`), the command table macros (`SHELL_CMD_REGISTER()`, `SHELL_STATIC_SUBCMD_SET_CREATE()`, `SHELL_DYNAMIC_CMD_CREATE()`, `SHELL_CMD_ARG()`, `SHELL_CMD()`, `SHELL_COND_CMD()`, `SHELL_EXPR_CMD_ARG()`, `SHELL_HELP()`, `SHELL_SUBCMD_SET_END`), `shell_device_get_binding()`, `shell_device_filter()` ("Shell command handling" below) |
| `zephyr/net/socket.h` (NuttX and `net_offload/include/`) | The socket API (`zsock_*()`): NuttX's functions in the NuttX build ("NuttX" below), the copy of Zephyr's socket layer in the FreeRTOS and ThreadX builds ("Socket layer (`net_offload`)" below) |
| `zephyr/sys/clock.h` (NuttX and `port/generic/`) | `sys_clock_gettime()`, `sys_clock_settime()` (`SYS_CLOCK_REALTIME`; 0 or `-EINVAL`), `NSEC_PER_USEC`, `USEC_PER_SEC`. NuttX's `clock_gettime()`, `clock_settime()` with `CLOCK_REALTIME` in the NuttX build; the wall clock of the builds on the Pico SDK is `port/pico_sdk/zephyr_sys_clock.c` |
| `zephyr/net/net_if.h` (NuttX and `net_offload/include/`) | `struct net_if`, `net_if_get_default()`, `net_if_get_wifi_sta()`, the IPv4 address and gateway as the applications read them (`net_if_ipv4_get_global_addr()`, `net_if_ipv4_get_gw()`), `net_if_is_admin_up()` |
| `zephyr/net/net_mgmt.h` | `net_mgmt()` (requests), `struct net_mgmt_event_callback`, `net_mgmt_init_event_callback()`, `net_mgmt_add_event_callback()`, `_del_event_callback()`, `net_mgmt_event_notify_with_info()` (below) |
| `zephyr/net/wifi_mgmt.h` | The Wi-Fi types (security, band, MFP, state, mode; Zephyr's values), `struct wifi_scan_params`, `wifi_scan_result`, `wifi_connect_req_params`, `wifi_iface_status`, `wifi_status`, `struct wifi_mgmt_ops`, the offloaded interface API and `NET_DEVICE_DT_OFFLOAD_DEFINE()`, the requests `NET_REQUEST_WIFI_SCAN`, `_CONNECT`, `_DISCONNECT`, `_IFACE_STATUS`, `_CONNECT_STORED` (with `CONFIG_WIFI_CREDENTIALS`), the events `NET_EVENT_WIFI_SCAN_RESULT`, `_SCAN_DONE`, `_CONNECT_RESULT`, `_DISCONNECT_RESULT`, `wifi_mgmt_raise_connect_result_event()`, `_disconnect_result_event()` (below), `wifi_security_txt()`, `wifi_band_txt()` |
| `zephyr/net/wifi_credentials.h` | A copy of Zephyr's header of that name: `struct wifi_credentials_header`, `_personal`, `WIFI_CREDENTIALS_FLAG_*`, `wifi_credentials_set_personal_struct()`, `_get_by_ssid_personal_struct()`, `_delete_by_ssid()`, `_for_each_ssid()`, `_is_empty()` and the rest. The configuration values (`CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES` and so on, Zephyr's names) are the OS's where it sets them (in the NuttX build, `nuttx/apps/wifi/Kconfig`), else the Zephyr build's. The port's own `wifi_credentials_init()` (called by init; Zephyr's `SYS_INIT()`). The functions are in `wifi_credentials.c` (the copy of Zephyr's library) and `wifi_credentials_backend_settings.c`, which stores them in the settings |
| `zephyr/net/wifi_utils.h` | `wifi_utils_validate_chan()` (`_2g`, `_5g`, `_6g`), `net_bytes_from_str()` (in Zephyr's `<zephyr/net/net_ip.h>`) |
| `zephyr/settings/settings.h`, `zephyr/kvss/nvs.h`, `zephyr/sys/slist.h`, `zephyr/sys/list_gen.h` | Unchanged copies of Zephyr's headers of the same names ("Key-value store" below): the settings API (`settings_subsys_init()`, `settings_save_one()`, `settings_load_one()`, `settings_delete()`, `settings_load_subtree_direct()` and others), NVS's API and `struct nvs_fs`, singly linked lists |
| `zephyr/drivers/flash.h`, `zephyr/storage/flash_map.h` | The flash API that NVS and the settings use (`flash_read()`, `flash_write()`, `flash_erase()`, `flash_flatten()`, `flash_get_parameters()`, `flash_get_write_block_size()`, `flash_get_page_info_by_offs()`, `struct flash_parameters`) and the flash map (`flash_area_open()`, `flash_area_close()`, `flash_area_get_sectors()`, `PARTITION_ID(storage_partition)`). The only device is the key-value area's flash (`zephyr_flash_kv.c`, below) |
| `zephyr/sys/crc.h`, `zephyr/toolchain.h` | Declarations of `crc8_ccitt()` and others (implemented by the copy of `subsys/crc/crc8_sw.c`), `__packed`, `__weak` |
| `flash_kv.h`, `port/flash_kv_os.h` | The key-value area's constants (8 KiB from `0xffe000`, 256-byte pages, 4 KiB sectors), the OS boundary (`flash_kv_os.h`, "Key-value store" below), the range checks, the split into pages and the check of what the area holds (Zephyr's NVS, the earlier NuttX build's format, erased, other) |
| `zephyr_kv_config.h` | The Zephyr build's Kconfig values read before the copied NVS and settings (`CONFIG_SETTINGS_NVS_SECTOR_COUNT` and others; "Key-value store" below) |
| `kv_store_os.h` | The OS part of `common/imu/kv_store.c`: the `kv wipe` subcommand (the Zephyr build's `zephyr/app/usb-console/src/kv_store_os.h` is empty) |
| `zephyr/logging/log.h` | `LOG_MODULE_REGISTER()`, `LOG_ERR()` and the other logging macros. It only includes `app_log.h` (the RAM log of `dmesg`) |
| `zephyr/net/net_ip.h` | The IPv4 address type `struct in_addr` (`imu_raw_mqtt.h`). It only includes NuttX's `<netinet/in.h>` (the same member `s_addr`, network byte order). In the FreeRTOS and ThreadX builds the header of the same name in `net_offload/include/` takes its place ("Socket layer (`net_offload`)" below) |
| `zephyr_compat.h` | `zephyr_compat_initialize()` (called by the start-up, "Start-up" below), `K_THREAD_DEFINE_START()` (called by the application's table of threads, `common/imu/app_init.c`) |
| `sensor_shell.h` | A copy of Zephyr's `drivers/sensor/sensor_shell.h` (the sensor shell's shared types; the Zephyr build reads Zephyr's) |
| `zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h`, `zephyr_os_shell.h` (NuttX and `port/generic/`) | The OS headers ("Boundaries an OS implements" below) |
| `common_prelude.h` (NuttX and `port/generic/`) | The prelude (the NuttX build's in "NuttX" below) |

### Sources

| Source (in `common/compat/`) | Contents |
|---|---|
| `zephyr_work.c` | Work items and queues: the used part of Zephyr's `kernel/work.c` (OS-independent) |
| `zephyr_init.c` | The work queues' threads (`k_work_queue_start()`, the loop of `kernel/work.c` `work_queue_main()`), the system work queue `k_sys_work_q`, `zephyr_compat_initialize()`, `k_uptime_get()` (the TIMER in ms), on `z_thread_static_start()` (OS-independent) |
| `zephyr_rtio.c`, `zephyr_rtio_workq.c`, `zephyr_mem_blocks.c` | The RTIO executor (`subsys/rtio/rtio_executor.c`), the RTIO work queue (`rtio_workq.c`; its thread `RTIO WQ 0` is started by `zephyr_compat_initialize()` with `z_thread_static_start()`), the block pool (`lib/mem_blocks`) |
| `zephyr_heap.c` | `k_heap` (`zephyr_heap.h`, which `<zephyr/kernel.h>` includes), with either kernel part |
| `zephyr_gpio.c` | The `gpio0` device (Zephyr's `gpio.h` logic and `gpio_rpi_pico.c`) on the OS's `port/gpio_os.h` |
| `zephyr_sensor_clock.c` | The sensor clock on the layer's cycle counter `k_cycle_get_64()` (the TIMER, `rp2040_timer_us()`, 1 MHz) |
| `sensor_default_rtio.c` | A copy of Zephyr's `drivers/sensor/default_rtio_sensor.c` (only the log replaced): the sensors' iodev API, the fallback for single reads, the default decoder |
| `zephyr_device.c` | Device initialization and the list of devices (`gpio0`, `i2c0`, `lsm6dsox` in the Zephyr build's order of initialization, with their node labels and whether the API is a sensor API; `gpio0` with the OS's configuration `CONFIG_LSM6DSO_STREAM`) |
| `zephyr_net_mgmt.c`, `zephyr_wifi_mgmt.c` | The `net_mgmt` event queue and thread (started with `z_thread_static_start()`; its priority and stack size are `APP_PRIO_NET_MGMT`, `APP_STACK_NET_MGMT` of `priorities.h`), the Wi-Fi requests (a copy of the checks of Zephyr's `subsys/net/l2/wifi/wifi_mgmt.c`) and events, `NET_REQUEST_WIFI_CONNECT_STORED` (a copy of `connect_stored_command()` of the same file; with `CONFIG_WIFI_CREDENTIALS`). In the NuttX build only with `CONFIG_NINA_NET` |
| `zephyr_wifi_utils.c` | Copies of `wifi_security_txt()` and `wifi_band_txt()` (Zephyr's `wifi_mgmt.c`), the channel checks (`wifi_utils.c`) and `net_bytes_from_str()` (`subsys/net/ip/utils.c`), for `wifi cred`. OS-independent. In the NuttX build only with `CONFIG_NINA_NET` |
| `sensor_shell_stream.c` | A copy of Zephyr's `drivers/sensor/sensor_shell_stream.c` (the sensor shell's processing thread). It includes the OS's `sensor_shell_os.h` first, and the processing thread's callback is `sensor_shell_os_processing_callback()` (in the NuttX build, `common/shell/port/nuttx/sensor_shell_nuttx.c`; in the builds on the Pico SDK, `common/shell/port/generic/sensor_shell_os.c`) |
| `wifi_credentials.c`, `wifi_credentials_backend_settings.c` | Copies of Zephyr's `subsys/net/lib/wifi_credentials/` (the stored Wi-Fi credentials). Two changes in the backend: the read buffer is aligned to 8 bytes (it is read as a struct; the Cortex-M0+ faults on an unaligned access) and the index formats are `%zu` |
| `subsys/kvss/nvs/`, `subsys/settings/`, `subsys/crc/crc8_sw.c`, `lib/utils/hex.c` | Unchanged copies of Zephyr v4.5.0's NVS, settings, CRC-8 and hex conversions ("Key-value store" below) |
| `zephyr_flash_kv.c` | The key-value area's flash device and flash map ("Key-value store" below) |
| `kv_store_wipe.c` | `kv wipe confirm` (erases the area and initializes the settings again) |
| `subsys/portability/posix/c_lib_ext/fnmatch.c`, `lib/utils/getopt/` | Unchanged copies of Zephyr's `fnmatch()` and getopt (newlib has no `fnmatch()`; the shell engine keeps a getopt state per session) |
| `subsys/net/lib/wifi_credentials/wifi_credentials_shell.c` | `wifi cred`: an unchanged copy of Zephyr's `wifi_credentials_shell.c` |
| `zephyr_wifi_shell.c` | The `wifi` command: `connect`, `disconnect`, `scan` and `status` taken from Zephyr's `wifi_shell.c`, keeping only the options this board can use (the builds on the Pico SDK only) |
| `zephyr_zperf_shell.c`, `zperf_iperf3.c`, `internal/zperf_iperf3.h` | `zperf` (the builds on the Pico SDK only; built in by `port/pico_sdk/zperf.cmake`) |

### Work

The flags (RUNNING, CANCELING, QUEUED, DELAYED) and return values are those of `kernel/work.c`. `submit` does
nothing if already queued (0) and queues a running item again on its queue (2). `schedule` does nothing while
delayed or queued (a running item can be scheduled); `reschedule` stops the delay and schedules again
(`K_NO_WAIT` queues at once). `k_work_cancel_sync()` removes a queued item and waits for a running one (one
waiting thread only; Zephyr allows several). After an item, the queue's thread calls `k_yield()` if more are
pending. A delay ends at the current tick + ticks + 1, as in Zephyr. Not ported: flush, drain, plug, stopping a
queue, the work timeout, `k_work_cancel()`, `k_work_cancel_delayable()`.

### Semaphores and `k_poll()`

`k_sem_give()` stops at the limit and wakes the threads in `k_poll()` on that semaphore (they are told, the count
is not taken: Zephyr's `K_POLL_MODE_NOTIFY_ONLY`). `k_sem_take()` gives 0, `-EBUSY` (`K_NO_WAIT`) or `-EAGAIN`
(timed out). `k_poll()` handles semaphores only.

### RTIO

- A copy of Zephyr's in the build's configuration (`CONFIG_RTIO_SUBMIT_SEM`, `_CONSUME_SEM`, `_SYS_MEM_BLOCKS`;
  no callbacks, no user mode). The sizes come from the caller's
  `RTIO_DEFINE_WITH_MEMPOOL(name, submissions, completions, blocks, block size, alignment)` (`imu_stream.c`: 4,
  16, 128, 64, 4). An iodev's submit runs in the context of the `rtio_submit()` caller. A successful multishot
  goes back to its iodev before its completion is posted (its pool buffer unbound); an error or a cancellation
  frees its buffer and the submission. A completion that finds no free entry is dropped and counted in `xcqcnt`
  (its buffer stays allocated, nobody can release it; as in Zephyr). `rtio_cqe_consume()` does not wait and
  takes `consume_sem`. A handle is the submission's pool index and generation; every free advances the
  generation, so an old handle neither resolves nor cancels. `rtio_sqe_cancel()` marks the submission and calls
  the iodev's cancel function (if any). Pool allocation takes the first run of free blocks (first fit), trying
  from `max` down to `min` one block at a time. Differences from Zephyr: the queues (`struct mpsc`) mask
  interrupts instead of being lock-free, and the pools' free lists are filled at their first allocation instead
  of at boot (`rtio_init()`), in the same order. Not ported: chains and transactions, the executor's own
  operations (callback, await, delay), writes and the other operations, `rtio_sqe_copy_in()`,
  `rtio_cqe_copy_out()`, `rtio_pool`, completion callbacks, user mode.
- **RTIO work queue**: as Zephyr's `rtio_workq.c`, a pool of 4 requests and one thread (`RTIO WQ 0`, priority 0,
  stack 1024), run in order. The single reads' fallback (`common/drivers/lsm6dso/port/zephyr/lsm6dso_rtio.c`)
  uses it.

### GPIO

One port, `gpio0` (GPIO0..29), inputs only (outputs are `-ENOTSUP`). Pin set-up, interrupts (edges and levels,
logical or physical), levels and callbacks are copies of Zephyr's `gpio.h` and `gpio_rpi_pico.c`
(`zephyr_gpio.c`); the register writes are the OS's implementation of `port/gpio_os.h` (the NuttX build's
`port/nuttx/rp2040_gpio_os.c`, the Pico SDK builds' `port/pico_sdk/gpio_os_rp2040.c`).

### Sensor clock and cycle counter

The Zephyr version's `k_cycle_get_*()` counts the 125 MHz SysTick (which falls behind under load); on the layer
it is the board's TIMER (`rp2040_timer_us()`, 1 MHz from the crystal). The sensor clock is the same, so stream
batch and single read times are whole microseconds. A cycle is a microsecond, so the conversions to
microseconds (`k_cyc_to_us_floor64()`, `_ceil64()`) return the value as it is. `k_cycle_get_32()` is the low 32
bits and wraps after about 71.6 minutes (the Zephyr version's after about 34 s), and `k_cyc_to_ns_floor64()`
multiplies by 1000. `imu_raw_mqtt.c` times its queue lock waits and its publishes with it, the LSM6DSO driver
its FIFO reads.

### Start-up and static threads

- **Start-up**: the start-up calls `zephyr_compat_initialize()` before I2C0 (with its DMA channels) and the
  LSM6DSOX; it starts the system work queue (`sysworkq`), the RTIO work queue and `gpio0` (Zephyr starts them at
  `POST_KERNEL` before the sensor too). The LSM6DSOX's initialization (`lsm6dso_stream_init()`) creates
  `imu_fifo_worker`.
- **Static threads** (`K_THREAD_DEFINE()`): Zephyr's arguments define a thread with a static stack, the priority
  `K_PRIO_PREEMPT(prio)` and the token itself as its name (`imu_stream_tid`). Zephyr starts such threads by
  itself before `main()`; the layer has no list of them, so each one is started with
  `K_THREAD_DEFINE_START(name)` (`zephyr_compat.h`) from the application's table shared by the NuttX, FreeRTOS
  and ThreadX builds (`app_threads_start()`, `common/imu/app_init.c`, `common/imu/app_main.h`), before the part
  that does what `main()` does (`app_main()`). Every thread of the layer (the work queues', `RTIO WQ 0`,
  `net_mgmt`) starts the same way, with `z_thread_static_start()`. A thread ends when its entry function
  returns. Only `options` 0 and `delay` 0 are accepted (anything else fails to build).

### Shell command handling

- `shell_print()`, `shell_info()`, `shell_warn()` and `shell_error()` add a newline (Zephyr prints information in
  green, warnings in yellow and errors in red; no colors here). `sh` says where the text goes (NULL: the calling
  task's stdout and stderr; a backend: its session; how each OS does it is in "NuttX" and "Shell engine" below).
- The command table macros have the form of Zephyr's `shell.h`. Matching, help, wildcards and the argument
  count check are done by the shared `common/shell/zshell.c`: `z_shell_cmd_main()` follows Zephyr's `shell.c`
  `execute()`, `exec_cmd()` and `cmd_precheck()`, `shell_wildcard.c` and `shell_help.c`: it matches the split
  words (calling the dynamic subcommand functions in Zephyr's order), expands wildcards, checks the argument
  count, prints the help (the `SHELL_HELP()` description and `Usage:` line, wrapped at 80 columns) and calls the
  handler, whose `argv` is NULL after the last word as in Zephyr. Differences from Zephyr: a subcommand set says
  whether it is dynamic in its `struct shell_cmd_entry` (Zephyr tells from the linker section its
  `union shell_cmd_entry` is in); no select mode, aliases, remote commands or raw arguments; spaces where Zephyr
  moves the cursor with VT100 sequences.
- `zshell.c` is OS-independent. The OS part (listed in `common/shell/port/zshell_os.h`) is `struct shell`, the
  output to a backend `z_shell_backend_vfprintf()`, a function that returns the backend of the calling task's
  session, and two marks around the handler's call (the NuttX build's `common/shell/port/nuttx/zshell_os_impl.h`;
  the Pico SDK builds' `common/shell/port/generic/zshell_os_impl.h`, on the shell engine). The help and the
  errors are printed with `printf()` and `fprintf(stderr)`, so stdout and stderr of the task that runs a command
  must be its session's terminal.
- `shell_device_get_binding()` and `shell_device_filter()` are Zephyr's (`shell_utils.c`), on the device list
  (`zephyr_device.c`; `DEVICE_API_IS(sensor, dev)` is a mark in that list; Zephyr checks that the API table is in
  the class's linker section).

### `net_mgmt` and the Wi-Fi requests

- **`net_mgmt`**: a request `net_mgmt(NET_REQUEST_X, iface, data, len)` calls `net_mgmt_NET_REQUEST_X()` in the
  calling thread, as Zephyr's macro does. Events follow the Zephyr version's configuration
  (`CONFIG_NET_MGMT_EVENT_THREAD`, `_QUEUE`, `_INFO`, a queue of 5 entries, 10 ms of waiting for room): the
  information is copied into the queue, and the thread `net_mgmt` (priority −1, stack 2048) takes the events in
  order and calls each callback whose mask has the event (with `cb->info` and `info_length` set, under the
  callback list's lock). An event that finds no room within 10 ms is dropped (the Zephyr version's warning is
  compiled out at log level 0). Event codes are bits of this port (not Zephyr's 64-bit layer and command values)
  compared by the same names. The thread starts before the interface (Zephyr's `net_init()` order).
- **Wi-Fi requests** (`zephyr_wifi_mgmt.c`): as in Zephyr's `wifi_mgmt.c`, `-ENOTSUP` if the interface's API
  lacks the driver's operation, `-ENETDOWN` without `IFF_UP`, and the same checks of the connection parameters
  (channel, SSID length 1 to 32, passphrase length 8 to 64 (5/10/13/26 for WEP), required fields per security
  type) before the driver is called (the Zephyr version's configuration has no wpa_supplicant). Scan results
  become one `NET_EVENT_WIFI_SCAN_RESULT` each, the end `NET_EVENT_WIFI_SCAN_DONE` (a `struct wifi_status`).
  `NET_REQUEST_WIFI_CONNECT_STORED` reads the stored networks in index order, makes connection parameters from
  each (channel 0 is "any", timeout 0 is 30 seconds, a type out of range is no encryption, band and MFP from the
  flags), issues `NET_REQUEST_WIFI_CONNECT` and stops at the first one accepted. It always returns 0 (as in
  Zephyr; the Zephyr version's configuration compiles this file's log messages out, so it logs nothing either).
- The interface and the Wi-Fi driver are provided by the NINA's binding ([NINA-W102 driver](../drivers/nina/README.md)).

### Key-value store: copies of Zephyr's NVS and settings

To write the same bytes to the flash as the Zephyr build, the NuttX, FreeRTOS and ThreadX builds use unchanged
copies of the parts of Zephyr v4.5.0's NVS and settings that the Zephyr build uses, with the Zephyr build's
configuration. When the Zephyr tree (`zephyr/zephyr`) is there, a host test (`zephyr_copies` in
`common/tests/run.sh`) checks with `cmp` that the copies are the originals.

| Copy (`common/compat/`) | Original (Zephyr v4.5.0) | Changes |
|---|---|---|
| `subsys/kvss/nvs/nvs.c`, `nvs_priv.h` | `subsys/kvss/nvs/` | none |
| `subsys/settings/src/settings.c`, `settings_store.c`, `settings_init.c`, `settings_nvs.c`, `settings_priv.h` | `subsys/settings/src/` | none |
| `subsys/settings/include/settings/settings_nvs.h`, `settings_file.h` | `subsys/settings/include/settings/` | none |
| `subsys/crc/crc8_sw.c` | `subsys/crc/crc8_sw.c` | none |
| `lib/utils/hex.c` | `lib/utils/hex.c` | none |
| `zephyr/kvss/nvs.h`, `zephyr/settings/settings.h`, `zephyr/sys/slist.h`, `zephyr/sys/list_gen.h` | `include/zephyr/...` | none |
| `wifi_credentials_backend_settings.c` | `subsys/net/lib/wifi_credentials/` | the two in the table above |

- Configuration: the copied sources are built with `zephyr_kv_config.h` in front (the Zephyr build's `.config`
  values: `CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT` 1, `CONFIG_SETTINGS_NVS_SECTOR_COUNT` 8,
  `CONFIG_SETTINGS_DYNAMIC_HANDLERS`, `CONFIG_MULTITHREADING`, log level 3; `CONFIG_NVS_LOOKUP_CACHE`,
  `CONFIG_NVS_DATA_CRC`, `CONFIG_NVS_INIT_BAD_MEMORY_REGION` and `CONFIG_SETTINGS_NVS_NAME_CACHE` off, as in the
  Zephyr build). They test the options that are off with `#if`, so they are built without `-Wundef`; `nvs.c`
  prints `uint32_t` values with `%d` (Zephyr's `uint32_t` is `unsigned int`, newlib's and NuttX's
  `unsigned long`, both 32 bits), so it is built without `-Wformat`.
- The layout on the flash and the values that decide it (where the Zephyr build has them):

  | Value | Here | Zephyr build |
  |---|---|---|
  | Area | `0x2000` from `0xffe000` (`flash_kv.h`, `<zephyr/storage/flash_map.h>`) | `storage_partition` of `app.overlay` |
  | Sectors | 4096 bytes × 2 | `erase-block-size` of `rp2040.dtsi` × `SECTOR_SIZE_MULT` 1, as many as fit in 8 KiB (`settings_nvs.c`) |
  | Write block, erase value | 1, 0xFF | `flash_rpi_parameters` of `flash_rpi_pico.c`, `write-block-size` of the DT |
  | Allocation table entry | 8 bytes, `crc8_ccitt(0xff, ...)` | `nvs_priv.h` (unchanged) |
  | Name and value IDs | 0x8000, 0x8001 on, +0x4000 | `settings_nvs.h` (unchanged) |
  | Values | calibration 56 bytes, text, `imu.session` 4 bytes, a credential 224 bytes | `imu_cal_codec.c`, `imu_stream.c` (shared), `struct wifi_credentials_personal` (its layout fixed to the Zephyr build's ELF by `BUILD_ASSERT`s; arm-none-eabi's 1-byte enums) |

- Assumptions kept: the settings' lock (`K_MUTEX_DEFINE(settings_lock)`, a mutex that starts all zero) and NVS's
  lock are the layer's `k_mutex` (recursive: `kv wipe` initializes the settings again while it holds the
  settings' lock). Initialization is, as in Zephyr, the first `settings_subsys_init()` (tried again by the next
  call after a failure).
- **The flash boundary** (`port/flash_kv_os.h`; each OS implements it; on the RP2040 the NuttX, FreeRTOS and
  ThreadX builds share `common/chip/rp2040/rp2040_flash_kv.c`, below):

  ```c
  int flash_kv_os_initialize(void);
  bool flash_kv_os_ready(void);
  int flash_kv_os_read(uint32_t offset, void *buf, uint32_t len);
  int flash_kv_os_program(uint32_t page_offset, const uint8_t page[256]);
  int flash_kv_os_erase(uint32_t sector_offset);
  ```

  The build's start-up calls `initialize` once (it checks the linker script's bounds; nothing is erased).
  `offset` is from the start of the flash (XIP 0x10000000). While `ready` is false, the flash map treats the
  device as missing (`-ENODEV`). `read` returns what the flash holds now (also right after a program or an
  erase). `program` writes the page at a multiple of 256 (0xFF where nothing is written; programming only
  clears bits), `erase` the sector at a multiple of 4096. Each returns 0 or a negative errno and is called from
  threads only, one at a time (the layer holds its lock). The caller (`zephyr_flash_kv.c`) passes only aligned
  requests inside the area, but the OS checks the range again and returns `-EINVAL` without touching the flash
  otherwise. Interrupts are off, and no code runs from the flash, while a page is programmed or a sector erased
  (the NuttX build: for one page or one sector at a time).
- **The RP2040's part** (`common/chip/rp2040/rp2040_flash_kv.c`, shared by the NuttX, FreeRTOS and ThreadX
  builds): `initialize` compares the linker script's `__kv_start`, `__kv_end` and the image's end with the
  constants, then readies the build's flash operations; `read` copies from the uncached XIP alias (0x13000000);
  `program` copies the page into RAM; `program` and `erase` check the request against the constants and the
  bounds and hand it to the build's flash operations (`common/chip/rp2040/port/rp2040_flash_kv_os.h`) with the
  bounds:

  ```c
  int rp2040_flash_kv_os_init(void);
  uintptr_t rp2040_flash_kv_os_image_end(void);
  int rp2040_flash_kv_os_program(uint32_t page_offset, const uint8_t *page, uint32_t lo, uint32_t hi);
  int rp2040_flash_kv_os_erase(uint32_t sector_offset, uint32_t lo, uint32_t hi);
  ```

  The build checks the request against `[lo, hi)` once more with interrupts off, right before the flash is
  touched. The NuttX build's (`common/chip/rp2040/port/nuttx/rp2040_flash_kv_os.c`, built into `libboard.a` by
  the board's `src/Make.defs` through `VPATH`): `init` finds the boot ROM's flash functions and copies boot2 into
  RAM; `program` and `erase` run a RAM function (`.ram_code.kv_flash_op`) with interrupts disabled that calls the
  boot ROM and the copy of boot2 only. NuttX's own flash driver for the chip (`rp2040_flash_mtd.c`) is not used:
  it is an MTD device for the SmartFS region after the image, which it erases to the end of the flash at its
  first start. The FreeRTOS and ThreadX builds' (`common/chip/rp2040/port/pico_sdk/rp2040_flash_kv_os.c`): the
  Pico SDK's `flash_range_program()` and `flash_range_erase()` through `flash_safe_execute()` (the linker script
  and what happens while interrupts are off are in [`design/freertos-port.md`](../../design/freertos-port.md)).
- `zephyr_flash_kv.c`: the flash device (`z_flash_kv_device`) and the flash map. It accepts requests inside the
  area only (the Zephyr build's driver accepts the whole flash), splits writes into 256-byte pages and erases
  into sectors for the boundary. `flash_area_open()` checks once after boot what the area holds; if it is
  neither Zephyr's NVS nor erased, it logs that and returns `-EILSEQ` (the settings are not initialized and
  nothing is written). After `kv wipe confirm` it checks again.

### Socket layer (`net_offload`)

The layer's network part for an OS without a network stack of its own (the FreeRTOS and ThreadX builds). It is
OS-independent.

- The layer has Zephyr's socket layer down to its list of socket implementations (reduced copies of Zephyr's
  `sockets.c`, `getaddrinfo.c`, `socket_offload.c`, `zvfs_fdtable.c` and `zvfs_poll.c` in
  `net_offload/subsys/net/lib/sockets/` and `net_offload/lib/os/zvfs/`; what each leaves out is in its header).
  The network interface (`zephyr_net_if.c`) and the start of the network part (`zephyr_net_init.c`,
  `zephyr_net_initialize()`, which registers the interface at the start, as Zephyr's `net_init()`) are here too,
  with their headers under `net_offload/include/` (`zephyr/net/`, `zephyr/sys/fdtable.h`, `zephyr_net_init.h`).
- An implementation registers itself with `NET_SOCKET_OFFLOAD_REGISTER()` or `NET_SOCKET_REGISTER()`; the
  linker script collects the registrations sorted by priority (`port/pico_sdk/ld/section_extra_post_text.incl`),
  and `zsock_socket()` takes the first whose family matches and whose `is_supported()` accepts. The other calls
  go through the descriptor table to the implementation's `struct socket_op_vtable` under the descriptor's
  mutex, `poll` through `ZFD_IOCTL_POLL_PREPARE` and `ZFD_IOCTL_POLL_OFFLOAD`, and `zsock_getaddrinfo()` to the
  registered resolver. The NINA's sockets are the Zephyr build's binding, compiled unchanged
  (`common/drivers/nina/port/zephyr/nina_socket.c`, priority 40): as in the Zephyr build, the calling task asks
  the NINA and waits (there is no event thread like the NuttX build's usrsock and `nina_event`).
- The table has 16 descriptors, 0 to 15 from the lowest free one (none kept for stdin, stdout and stderr, as in
  the Zephyr build); the NINA has ten sockets, so an eleventh gets `EMFILE`. The descriptors are not newlib's;
  only the `zsock_*()` calls take them. The types and constants newlib lacks are in `libc_net/include/`, with
  Linux's values (`EAI_*` are Zephyr's negative values); `<zephyr/net/net_ip.h>` of `net_offload/include/` gives
  them Zephyr's `net_*` names (it is ahead of `include/` on the include path and takes the place of
  `include/zephyr/net/net_ip.h`, which the NuttX build uses).
- What the sockets do is the Zephyr build's: a poll of the module that fails is `POLLERR` on that socket (the
  call does not fail); a `poll` takes each descriptor's mutex, so it waits for a call in progress on one of its
  sockets in another task (a blocking receive, a connect); one `sendmsg` runs at a time, sleeps included; name
  resolution's results come from a 2048-byte heap (`EAI_MEMORY` when about 36 results are not freed). The
  Telnet server restarts its listening socket after an error on it, and sets it up again every second while
  that fails.
- Wi-Fi: the NINA core's Wi-Fi operations with the Zephyr build's binding of them
  (`common/drivers/nina/port/zephyr/nina_net.c`: Zephyr's Wi-Fi management and interface, the thread
  `nina_wifi_tid`), the credentials and the automatic reconnection (`common/imu/wifi_profile.c`) run on the
  layer's `net_if`, `net_mgmt` and `wifi_mgmt` (`net_offload/zephyr_net_if.c`). The NINA's side is in the
  [NINA-W102 driver](../drivers/nina/README.md).
- The network part relies neither on the generic kernel part (`kernel_generic/`) nor on the POSIX headers
  (`libc_net/`) (`net_offload_independent` in "Tests" below).

### Shell engine (`shell_engine`)

The shell for an OS without one of its own (the FreeRTOS and ThreadX builds), written on the layer's API.

| File | Contents |
|---|---|
| `cli.c` | Line editing |
| `shell_engine.c` | The command table and execution with `help` |
| `shell_argv.c` | A copy of Zephyr's argument split |
| `shell_console.c` | The consoles and each thread's output |
| `getopt_state.c` | getopt per session |
| `shell_serial.c` | The serial session (the console transport an OS provides is declared in `port/shell_serial_os.h`) |
| `shell_telnet.c` | The Telnet shell |

Its headers are in `include/` (`shell_console.h`, `shell_engine.h`, `shell_serial.h`, `shell_telnet.h`) and
`internal/` (`cli.h`, `shell_argv.h`), its backend of `zshell.c` in `common/shell/port/generic/` and
`port/generic/zephyr_os_shell.h`.

- Commands are written with Zephyr's shell API (`SHELL_CMD_REGISTER()` and so on); matching, help, wildcards and
  the argument count check are done by the shared `common/shell/zshell.c` ("Shell command handling" above). The
  command table and the line editor are the shell engine's (`shell_engine.c`, `cli.c`; `shell_engine.c` also has
  `help`).
- The consoles, the sessions and Telnet are the shell engine's, written on the layer's `k_mutex`, `k_sem`,
  `z_thread_static_start()` and thread custom data (`k_thread_custom_data_get()`). A console (`shell_console.c`)
  is a session's terminal (USB or Telnet); each task is bound to one console, and its `printf()` and
  `fprintf(stderr)` go there. Tasks bound to no console write to USB.
- Telnet (`shell_telnet.c`, built in by `port/pico_sdk/telnet.cmake`): port 23, one client at a time. As Zephyr's
  `shell_telnet.c`, it sends nothing on connection, takes Telnet commands out without answering, and does not
  echo. Where it differs from Zephyr is listed at the top of the file (it looks once more whether the first
  client has closed before refusing a second, a new client starts with an empty line, and so on).
- The getopt state is per session (`getopt_state.c`): USB and Telnet can run commands at the same time.
- The console transport (USB) is the OS's ("Pico SDK" below).

## Boundaries an OS implements

The list, in the order a new OS writes it, is in [`design/os-boundary.md`](../../design/os-boundary.md) (the
kernel part in section 2, the other boundaries in section 3, the `<zephyr/...>` headers each OS supplies in 4.1).
This section shows their shape from the layer's side.

### The kernel part (two ways)

The shared headers hold no OS types; they include these headers from the OS's directory (`port/nuttx/` in the
NuttX build, `port/generic/` in the FreeRTOS and ThreadX builds, `common/tests/zephyr_fakes/` in the host
tests). The list of what each must provide is in the leading comment of the shared header that includes it.

- `zephyr_os.h` (included first by `<zephyr/kernel.h>`): the OS headers needed before the layer,
  `Z_USEC_PER_TICK`, `struct k_mutex` and `Z_MUTEX_INITIALIZER()` (an all-zero one must work as an unlocked
  mutex), `struct k_sem`, `Z_SEM_INITIALIZER()`, `K_SEM_MAX_LIMIT`, `struct _timeout`, `_timeout_func_t`,
  `k_tid_t`, `k_current_get()`, `k_yield()`, `K_PRIO_PREEMPT()` (Zephyr's priority as the OS's, keeping the
  order), `k_cycle_get_32()`, `k_cycle_get_64()` (1 MHz), `Z_OS_ASSERT()`, and optionally
  `Z_SEM_TAKE_FOREVER()` (the work queue's wait). The functions (`k_sem_*()`, `k_mutex_*()`, `k_poll()`,
  `k_sleep()` and the others, `z_add_timeout()`, `z_abort_timeout()`, `z_thread_static_start()`, which starts
  every thread of the layer) are the OS's `.c` files; the work queues' threads, `k_sys_work_q`, `k_uptime_get()`
  and `zephyr_compat_initialize()` are shared (`zephyr_init.c`).
- `zephyr_os_spinlock.h` (included by `<zephyr/spinlock.h>`): `struct k_spinlock`, `k_spinlock_key_t`,
  `k_spin_lock()`, `k_spin_unlock()`.
- `zephyr_os_atomic.h` (included by `<zephyr/sys/atomic.h>` before it defines its names): the OS headers that
  have names the layer takes over, and the `#undef` of their macros. Empty in the host tests.
- `zephyr_os_shell.h` (included by `<zephyr/shell/shell.h>`): `struct shell`.

There are two ways to write them.

| | Directly (the NuttX build) | On primitives (the FreeRTOS and ThreadX builds) |
|---|---|---|
| What the OS writes | `zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h` and the functions above (`port/nuttx/zephyr_kernel.c`, `zephyr_mutex.c`) | The primitives of `port/zephyr_kernel_os.h` (10 types and constants, 26 functions) in `port/<os>/zephyr_kernel_os_impl.h`, and a tick hook that calls `zephyr_compat_tick()` once a tick |
| `zephyr_os.h` and the others | The OS's (`port/nuttx/`) | The shared `port/generic/zephyr_os.h`, `zephyr_os_spinlock.h` and the default (empty) `zephyr_os_atomic.h` |
| The functions | The OS's | The shared `kernel_generic/zephyr_kernel.c`, `zephyr_mutex.c` |

The generic implementation makes Zephyr's `k_sem` (with Zephyr's wake rules), `k_poll()` on semaphores, the
sorted timeout list of the delayable work items, the sleeps, the start of a thread, `k_spinlock` and `k_mutex` on
the primitives. The primitives are on the paths of `k_sem_give()`, `k_sem_take()`, `k_poll()`, `k_spin_lock()`
and the tick, so the OS's part defines them as `static inline` functions (no call through a pointer, no extra
call). The OS's part reads no header of the layer. The list and what each must do are in the leading comment of
`port/zephyr_kernel_os.h` and sections 2.5 and 2.6 of [`design/os-boundary.md`](../../design/os-boundary.md). The
tick hook's entry is `include/zephyr_compat_os.h`; the catch-up of the ticks that a periodic tick loses while
interrupts are masked for long is `include/tick_catchup.h`. The smallest form that compiles is
`common/tests/kernel_generic_host/`.

### Assumptions that hold in Zephyr

The layer reproduces the assumptions that hold in Zephyr; a missing one becomes a bug that shows only on the
board. For example, Zephyr's `k_mutex` is unlocked when all zero (`lock_count` 0), Zephyr's MQTT library locks
the client's mutex in `mqtt_abort()` before `mqtt_client_init()`, and `mqtt_client_init()` zeroes the client at
every connection; the settings' lock (`K_MUTEX_DEFINE(settings_lock)`) also starts all zero. So in every OS part
an all-zero `k_mutex` works as an unlocked mutex, and the OS holds no reference to an unlocked mutex (how each OS
does it is in "Each OS's binding" below).

### Other boundaries

Boundary headers that the shared `.c` files and headers read:

| Header | Contents | Implemented by |
|---|---|---|
| `include/priorities.h` | The table of thread priorities and stacks (`APP_STACK_NET_MGMT`, `APP_PRIO_RTIO_WQ`, `APP_STACK_RTIO_WQ` and others) | `APP_PRIO_FROM_ZEPHYR()` of the OS part `priorities_os.h` (`port/<os>/`) |
| `port/gpio_os.h` | GPIO bank 0 under `zephyr_gpio.c` | `port/nuttx/rp2040_gpio_os.c`, `port/pico_sdk/gpio_os_rp2040.c` |
| `include/app_log.h` | `<zephyr/logging/log.h>`, `sensor_default_rtio.c`, the credentials' log | the OS part `app_log_os.h` (`port/nuttx/`, `port/generic/`) |
| `port/log_ram_os.h` | The OS part of the RAM log (`log/log_ram.c`) | `log_ram_os_impl.h` (`port/nuttx/`, `port/pico_sdk/`) |
| `port/cpu_load.h` | `cpu_load_get()` of `<zephyr/sys/cpu_load.h>` and its arithmetic | the board (idle) |
| `port/flash_kv_os.h` | The key-value area's flash ("Key-value store" above) | the RP2040's shared part `common/chip/rp2040/rp2040_flash_kv.c` |
| `port/shell_serial_os.h` | The shell engine's console transport | `port/pico_sdk/shell_serial_os_impl.h` |
| `sensor_shell_os.h` | The OS part of `sensor_shell_stream.c` | `common/shell/port/nuttx/`, `common/shell/port/generic/` |

- Headers that are the OS's as a whole: `zephyr/sys/clock.h`, `zephyr/sys/__assert.h`, `zephyr/drivers/i2c.h`
  (`port/nuttx/` in the NuttX build, `port/generic/` in the FreeRTOS and ThreadX builds), `zephyr/net/socket.h`,
  `zephyr/net/net_if.h` (`port/nuttx/` in the NuttX build, `net_offload/include/` in the FreeRTOS and ThreadX
  builds).
- The configuration values `CONFIG_LSM6DSO_STREAM` (`zephyr_device.c`) and `CONFIG_WIFI_CREDENTIALS`,
  `_MAX_ENTRIES`, `_SAE_PASSWORD_LENGTH`, `_CONNECT_STORED_CONNECTION_TIMEOUT` (the Wi-Fi headers and
  `zephyr_wifi_mgmt.c`) must be defined before the layer's headers (in the NuttX build by `<nuttx/config.h>`,
  which the prelude `common_prelude.h` reads for the shared `.c` files).
- **Headers of each OS** (of the shared sources): the few lines of a shared source that differ between the
  builds are in a small header of the same name, of which each OS has its own (besides `lsm6dso_instance.h`,
  the LSM6DSO device; [LSM6DSO driver](../drivers/lsm6dso/README.md)): `sensor_shell_os.h` (included first by
  `sensor_shell.c` and by the layer's `sensor_shell_stream.c`) and `wifi_profile_os.h` (the declaration of the
  reconnection thread's `wifi_reconnect_tid`: `const k_tid_t` in the Zephyr one, `zephyr/app/usb-console/src/`,
  as `K_THREAD_DEFINE()` makes it; without `const` in the NuttX one, `nuttx/apps/wifi/`, and in
  `port/generic/`, because the layer's `K_THREAD_DEFINE()` writes the value when the thread is started).

## Each OS's binding

### Zephyr

The Zephyr build does not use the layer (it builds the shared sources against Zephyr itself). It has only the
Zephyr versions of the shared sources' per-OS headers: `zephyr/app/usb-console/src/kv_store_os.h` (empty),
`zephyr/app/usb-console/src/wifi_profile_os.h`, and `drivers/sensor/sensor_shell_os.h` of its patched tree (does
nothing).

### NuttX

**Files** (`common/compat/port/nuttx/`): the kernel part (`zephyr_kernel.c`, `zephyr_mutex.c`), the OS headers
(`zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h`, `zephyr_os_shell.h`), the headers that depend on
NuttX as a whole (`zephyr/net/socket.h`, `net_if.h`, `zephyr/sys/clock.h`, `__assert.h`, `zephyr/drivers/i2c.h`),
the `.c` files of I2C, the interface and the chip ID (`zephyr_i2c.c`, `zephyr_net_if.c`, `zephyr_hwinfo.c`), the
prelude (`common_prelude.h`), the NuttX parts of the shared headers `app_log.h`, `priorities.h` and the RAM log
(`app_log_os.h`, `priorities_os.h`, `log_ram_os_impl.h`, with the syslog channel `log_ram_syslog.c`) and GPIO
bank 0 under `gpio0` (`rp2040_gpio_os.c`). `nuttx/drivers/compat/Make.defs` builds its `.c` files into
`libexternal.a` (`log_ram_syslog.c` from `nuttx/drivers/log/Make.defs`; both find them through `VPATH`, ahead of
the shared part's directories), the board's `src/Make.defs` builds `rp2040_gpio_os.c` into `libboard.a` (it
needs the chip's private headers). None of them gets the prelude.

**The include path** is the NuttX implementation (`common/compat/port/nuttx`, for the kernel side, the board and
the applications), then the shared part (`common/compat/include`, `port`, `internal`;
`common/compat/port/generic`, the implementation for an OS without its own that the Pico SDK builds use, is not
on it). The applications with shell commands add the shell's headers (`apps/subcmd`: `common/shell/include`,
`port` and `port/nuttx`; `apps/sensor`: `common/shell/include` and `port/nuttx`; `apps/nina` and `apps/wifi`:
`common/shell/include`). The kernel side (`compat/Make.defs`) and the applications (`apps/imu`, `apps/sensor`,
`apps/wifi`) find the shared `.c` files through `VPATH` and compile them with the prelude
(`sensor_shell_stream.c` is built by `apps/sensor`, the credentials by `apps/wifi`, NVS, settings, CRC-8, the hex
conversions, `zephyr_flash_kv.c` and `kv_store_wipe.c` by `apps/imu`). The host tests have a fake kernel (the OS
headers and their functions) in `common/tests/zephyr_fakes`, ahead of the NuttX implementation on the include
path.

| Source (`common/compat/port/nuttx/`) | Contents |
|---|---|
| `zephyr_mutex.c` | `k_mutex` on NuttX (`rmutex_t`). Zephyr's `k_mutex` is unlocked when all zero (`lock_count` 0), and Zephyr's MQTT library locks the client's mutex in `mqtt_abort()` before `mqtt_client_init()`; NuttX's all-zero mutex counts as held by thread 0, and the lock never returns. So `K_MUTEX_DEFINE()` and `k_mutex_init()` set a mark (`ready`), and one without it (all zero) is initialized by its first lock or unlock (once, under the layer's spin lock) |
| `zephyr_kernel.c` | The OS part on NuttX: `k_sem` (`sem_t`, the limit checked in `k_sem_give()`), `k_poll()`, the timer of delayable work (a wdog), thread names, the start of a thread (`z_thread_static_start()`: a kernel thread on its static stack; the start data handed over, the caller waits until the thread has taken it), `k_sleep()`, `k_msleep()`, `k_usleep()`, `k_busy_wait()` |
| `zephyr_i2c.c` | I2C (on a NuttX I2C master; below) |
| `zephyr_hwinfo.c` | `hwinfo_get_device_id()` on the board's chip ID (`rp2040_chip_id()`) |
| `zephyr_net_if.c` | The interface (values of the NuttX network device; below) |

- **What the OS headers hold**: NuttX's types (`rmutex_t`, `sem_t`, a wdog, `pid_t`), the tick, the priorities,
  the TIMER, `k_yield()` (`sched_yield()`), the assertion (`DEBUGASSERT()`), the work queue's wait
  (`nxsem_wait_uninterruptible()`), `sched_lock()` and `up_irq_save()`, the takeover of `<nuttx/atomic.h>`'s
  names, `struct shell` (the terminal's path and the command's task). `k_current_get()` is the calling thread's
  NuttX thread ID, comparable with the `k_tid_t` of `K_THREAD_DEFINE()`. `k_timeout_t` counts ticks of 100 µs.
- **Spin locks**: Zephyr's single-CPU `k_spin_lock()` masks interrupts, and no other thread runs until it is
  released (a thread made ready by `k_sem_give()` or the like runs at the release). NuttX switches to a thread
  that `nxsem_post()` makes ready at once, even with interrupts masked, so the NuttX build's `k_spin_lock()`
  (`zephyr_os_spinlock.h`) is `sched_lock()` plus `up_irq_save()` (`sched_lock()` does nothing in an interrupt
  handler). The `atomic_*()` functions use it too (the Cortex-M0+ has no exclusive loads and stores; Zephyr's
  ARMv6-M build masks interrupts as well).
- **Clashing NuttX names**: NuttX's `<nuttx/atomic.h>` (read by many kernel headers) has its own `atomic_t`,
  `atomic_set()` and others with other meanings. The layer's `<zephyr/sys/atomic.h>` reads it first
  (`zephyr_os_atomic.h`), then takes the names over for the file (`atomic_t` is `zephyr_atomic_t`, NuttX's macros
  are undefined). A file that uses both includes the NuttX headers first (for the files of `common/`, the prelude
  does). The board's bring-up does not include the Zephyr headers, only `zephyr_compat.h`.
- **Prelude** (`common_prelude.h`): when the NuttX build compiles the sources shared with the Zephyr build and the
  layer's shared `.c` files (the repository's `common/`), its build definitions (the `Makefile` of `apps/imu`,
  `apps/wifi` and `apps/sensor`, `compat/Make.defs`, `lsm6dso/Make.defs`, `nina/Make.defs`, the board's
  `src/Make.defs`) include it before each file with `-include` (as the Zephyr build reads its configuration into
  every file with `-imacros autoconf.h`). It holds `<nuttx/config.h>`, the NuttX headers needed before the layer
  (those that `<zephyr/kernel.h>` reads first: `<sys/types.h>`, `<nuttx/irq.h>`, `<nuttx/mutex.h>`,
  `<nuttx/semaphore.h>`, `<nuttx/wdog.h>`, `<stdio.h>`) and the values of the Zephyr Kconfig options that the
  shared sources use (`CONFIG_SENSOR_LOG_LEVEL` 3, the Zephyr build's value; `CONFIG_SENSOR_ASYNC_API` of
  `lsm6dso_sensor.c` and `lsm6dso_sensor.h`, set when `CONFIG_LSM6DSO_STREAM` is). So the shared sources have no
  lines that include NuttX headers. The socket headers are not in it (names such as `accept()` would collide
  with the sources' own functions; the layer's `<zephyr/net/socket.h>` reads them for the files that need them).
  The host tests use the same prelude (without `__NuttX__`, only the Kconfig values; the `time_sync` test and the
  tests of the LSM6DSO driver put a fake `<nuttx/config.h>` before it).
- **Headers of each OS** (of the shared sources): the NuttX build's `sensor_shell_os.h`
  (`common/shell/port/nuttx/`, built by `nuttx/apps/sensor`) reads the Zephyr build's configuration values and
  the log and makes the two lines with which `sensor get` waits for its result to be printed, and the processing
  thread's callback, call functions.
- **Work**: the queue thread's `k_yield()` is `sched_yield()`; the delay timer is a wdog.
- **RTIO work queue**: priority 0 is NuttX's 118. NuttX turns spaces in task names into `_`, so `ps` shows
  `RTIO_WQ_0`.
- **Start-up**: the board's bring-up calls `zephyr_compat_initialize()`. The system work queue has priority 119
  and a stack of 1024.
- **Static threads**: init starts them with `K_THREAD_DEFINE_START(name)`. Each is a NuttX kernel thread
  (`kthread_create_with_stack()`).
- **GPIO**: `rp2040_gpio_os.c` writes the registers as the pico-sdk's `gpio_set_pulls()`, `gpio_set_function()`,
  `gpio_set_dir()` and `gpio_set_irq_enabled()` do (the pin's events in `INTR` are cleared before they are
  enabled or disabled). The interrupt comes from NuttX's `rp2040_gpio` bank interrupt (which clears the enabled
  events, then calls the pin's handler).
- **Time**: `k_uptime_get()` has the value of Zephyr's `k_ticks_to_ms_floor64(k_uptime_ticks())` (the tick count
  in ms, rounded down). The NuttX tick is the board's TIMER over 100 µs (the tickless timer), so it reads the
  TIMER once and divides by 1000 (the same value because 1000 is a multiple of the tick; checked by a host test).
  It is 64 bits wide, without the wrap after about 5 days of the 32-bit `clock_systime_ticks()`. `k_sleep()`
  calls `sched_yield()` for `K_NO_WAIT` and otherwise sleeps with `nxsched_ticksleep()` until the current tick +
  ticks + 1 (the rounding of Zephyr's `z_add_timeout()`). It always returns 0, as there is no `k_wakeup()`.
  `k_msleep()` and `k_usleep()` are NuttX's `nxsched_msleep()` and `nxsched_usleep()`: the time rounded up to
  ticks, then the same wake-up as `k_sleep()` (Zephyr's rule); 0 or less yields, as Zephyr's does; no signal ends
  the sleep; they return 0. `k_busy_wait()` is `up_udelay()`, which polls the TIMER (the board's
  `rp2040_oneshot.c`). The LSM6DSO driver uses these three.
- **`hwinfo_get_device_id()`**: as the Zephyr version's RP2040 driver (`hwinfo_rpi_pico.c` without
  `CONFIG_HWINFO_RPI_PICO_CHIP_ID`), the external flash's 64-bit unique ID (command 0x4B) in the order the flash
  sends it (8 bytes at most). The board reads it once at boot (`rp2040_chipid.c`); this returns the copy, or
  `-EAGAIN` if the board has not read it yet (Zephyr's driver cannot fail).
- **Shell**: where `sh` sends the text:
  - `sh` is NULL: the calling task's stdout (`shell_print()`, `shell_info()`) and stderr (`shell_warn()`,
    `shell_error()`). The commands whose tables are written with `common/shell/include/subcmd.h` (`kv`, `nina`
    and others) pass NULL.
  - `sh` is a backend: what a command registered with `SHELL_CMD_REGISTER()` (`sensor`, `imu`, `ntp`) gets. The
    NuttX build's backends (`common/shell/port/nuttx/zshell_os_impl.h`; `struct shell` is in `zephyr_os_shell.h`)
    keep one per terminal that a command's stdout is (`/dev/ttyACM0` for USB, `/dev/telnet0` for Telnet;
    `F_GETPATH`, and since a Telnet session's device passes that on to its socket, a search of `/dev` for the
    inode number of `fstat()`). Like Zephyr's shell instances they stay for as long as the system runs. Text
    from the command itself (the running task) goes to its stdout and stderr; text from another task or thread
    (the sensor shell's processing thread, after the command has returned) opens the terminal each time, writes
    it with one `write()` and closes it again. While no Telnet client is connected the terminal cannot be
    opened, so the text is dropped, and it goes to the next client (as with Zephyr's Telnet backend). A Telnet
    session's device is open only while text is written, so the end of the session (when its last descriptor is
    closed) and the next connection are not held up.
  - `SHELL_CMD_REGISTER()` makes the builtin command's entry. By default the entry is `main()`, and the
    application's main source (`MAINSRC`) is the builtin command as it is (`sensor_shell.c`, `imu_stream.c`). A
    file that is not a main source is compiled with `Z_SHELL_CMD_NAMED_MAIN`, which its build definition adds
    for that file only, and the entry is `int z_shell_main_<name>(int argc, char *argv[])`, which the
    application's `main()` calls (`ntp` of `time_sync.c`: `apps/imu/Makefile` adds it for `time_sync.c`, and
    `main()` of `apps/ntp/ntp_main.c` calls `z_shell_main_ntp()`). nsh splits the words, and Tab completion is
    nsh's. Each command's task has its session's terminal as stdout and stderr.
- **Network**: `zsock_inet_pton()` and `zsock_inet_ntop()` are the C library's `inet_pton()` and `inet_ntop()`.
  While networking is off, `imu` selects `CONFIG_LIBC_IPv4_ADDRCONV` for the IPv4 conversions.
  `zsock_getaddrinfo()` and `zsock_freeaddrinfo()` are the NINA's resolver (`nina_dns_getaddrinfo()`; errors are
  NuttX's `EAI_*`). Sockets are the NINA's usrsock; `zsock_socket()`, `zsock_connect()`, `zsock_send()`,
  `zsock_sendmsg()`, `zsock_sendto()`, `zsock_recv()`, `zsock_poll()` and `zsock_close()` are NuttX's functions
  (`socket()`, ..., `poll()`, `close()`), and `ZSOCK_MSG_DONTWAIT`, `struct zsock_pollfd`, `ZSOCK_POLLIN` and the
  others are `MSG_DONTWAIT`, `struct pollfd`, `POLLIN`. A blocking connect or send waits with the NINA driver's
  deadlines; a receive and `poll()` wait for `nina_event`'s events (the usrsock part of the
  [NINA-W102 driver](../drivers/nina/README.md)). What `poll()` gives differs from the Zephyr version's NINA
  `poll()` (`do_poll()` in `nina_socket.c`) in two cases: a TCP socket that is not connected gives
  `ZSOCK_POLLIN`, `ZSOCK_POLLOUT` and `ZSOCK_POLLHUP` (the Zephyr version: `ZSOCK_POLLHUP` alone), and the NINA's
  error flag or a failed NINA poll of a connected TCP socket gives `ZSOCK_POLLHUP` (usrsock's `REMOTE_CLOSED`;
  the Zephyr version: `ZSOCK_POLLERR`). `imu_raw_mqtt.c`, which uses `zsock_poll()` for its input check (every
  100 ms, timeout 0) and the CONNACK wait (5000 ms), ends the connection on either.
- **Interface** (`net_if.h`): `struct net_if` is one NuttX network device. The one interface is the NINA's
  `wlan0`; `net_if_get_default()` and `net_if_get_wifi_sta()` both return it (NULL before `nina_wifi_mgmt.c`
  registers it). It has one address, which the NINA's binding sets
  (`common/drivers/nina/port/nuttx/nina_netdev.c`), always "preferred" (the Zephyr version's configuration has no
  IPv4 duplicate detection either), so `net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)` returns it while
  there is one. Values are read under the network device's lock (`netdev_lock()`, as `ifconfig`'s ioctls do).
  `net_in_addr` is NuttX's `in_addr`. Interface events (`NET_EVENT_IF_UP` and so on, address added or removed)
  are not raised (nothing in the port uses them).
- **`net_mgmt` and the Wi-Fi requests**: built only with `CONFIG_NINA_NET`; the NINA's binding
  (`common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`) serves them on the core for the applications. The
  `net_mgmt` thread's priority −1 is NuttX's 119. `nina_net_initialize()` starts the thread before the interface
  (`nina_wifi_mgmt_initialize()`). On NuttX the interface has only the connection (the scan, the disconnection
  and the status are wapi's, on the core).
- **I2C** (`zephyr_i2c.c`, `zephyr/drivers/i2c.h`):
  - An I2C bus is a device holding a NuttX I2C master and a speed (the board binds them with
    `i2c_compat_bind()`). In Zephyr the bus driver keeps the speed and `i2c_configure()` changes it for every
    later transfer. A NuttX transfer carries a frequency in each message, so the layer's transfers
    (`i2c_burst_read_dt()` and so on) put the device's speed in their messages. `i2c speed` changes this speed
    too, so it changes the speed of the LSM6DSO driver's transfers. `i2c_get_config()` returns
    `I2C_MODE_CONTROLLER` and the class of this speed (the Zephyr, FreeRTOS and ThreadX builds' driver has no
    `get_config`: `-ENOSYS`; `i2c speed` ends the same). Users of the NuttX master itself, if any, run at their
    own frequency.
  - `i2c_transfer()` adds STOP to the last message, as Zephyr's, and gives each message NuttX's flags (the
    inverse of the board driver's `i2c_dw_msg_flags()`: `I2C_MSG_READ` is `I2C_M_READ`, a message without STOP
    `I2C_M_NOSTOP`, one without RESTART after a message without STOP the continuation `I2C_M_NOSTART`). At most 4
    messages at once (`-EINVAL` above). `i2c_recover_bus()` is the master's `I2C_RESET()`, for which the board's
    driver returns `-ENOTSUP` (as the Zephyr build's driver; `i2c recover` reports the failure, with NuttX's
    errno value).
  - `i2c_configure()` takes only the classes the Zephyr version's RP2040 driver takes (100 kHz, 400 kHz, 1 MHz);
    for anything else it changes nothing and returns `-EINVAL`. It ignores the mode bits (that driver forces
    controller mode too). While I2C0 is quarantined after a DMA failure, Zephyr's `i2c_configure()` returns
    `-EIO`; the layer records the speed and returns 0, and the next transfer gives `-EIO`.
  - NuttX's `<nuttx/i2c/i2c_master.h>` has functions of its own named `i2c_write()` and `i2c_read()` and
    `I2C_SPEED_*` in Hz. In the layer, Zephyr's `i2c_write()`, `i2c_read()` and `i2c_write_read()` on a bus device
    are named `i2c_compat_write()`, `i2c_compat_read()` and `i2c_compat_write_read()` (`i2c_write_read()` and the
    `_dt` forms keep Zephyr's names). A file that does not include NuttX's header (the `i2c` command) has
    `i2c_write()` and `i2c_read()` under Zephyr's names too. In a file that includes `<zephyr/drivers/i2c.h>`,
    `I2C_SPEED_*` are Zephyr's classes (include NuttX's header first if both are needed).
  - The driver itself is in the [I2C driver](../drivers/i2c_dw/README.md).

The other parts of the NuttX port are in [`design/nuttx-porting.md`](../../design/nuttx-porting.md).

### Pico SDK (shared by FreeRTOS and ThreadX)

The FreeRTOS and ThreadX builds use the generic implementation (`kernel_generic/`) as the kernel part, the socket
layer (`net_offload/`) and `libc_net/` as the network, the shell engine (`shell_engine/`) as the shell, and run
on these shared parts.

| Place | Contents |
|---|---|
| `port/generic/` | The parts for an OS without the model of its own: `zephyr/drivers/i2c.h` (Zephyr's I2C API on the driver's device), `zephyr/sys/clock.h`, `zephyr/sys/__assert.h`, the prelude (`common_prelude.h`), `app_log_os.h` (with the RAM log's size `LOG_RAM_SIZE`), `wifi_profile_os.h`, the generic kernel part's `zephyr_os.h`, `zephyr_os_spinlock.h` and default `zephyr_os_atomic.h`, the shell engine's `struct shell` (`zephyr_os_shell.h`) |
| `port/pico_sdk/` | The part on the Pico SDK: the build of the layer, the shell and the RAM log (`compat.cmake`), of the key-value store (`kv.cmake`; the flash operations are `common/chip/rp2040/port/pico_sdk/`), of Wi-Fi (`wifi.cmake`: the NINA core's `common/drivers/nina/nina_wifi.c` with its Zephyr binding `common/drivers/nina/port/zephyr/nina_net.c`, credentials, reconnection, and `ifconfig`, `common/shell/ifconfig_shell.c`), of the Telnet shell (`telnet.cmake`) and of `zperf` (`zperf.cmake`); the linker script's part for the socket implementations (`ld/section_extra_post_text.incl`), the wall clock (`zephyr_sys_clock.c`), the port header's part shared by the OSes (`zephyr_os_port_rp2040.h`: `z_os_busy_wait_us()`, `z_os_tick_now_us32()`, `z_os_tick_pend()`), GPIO bank 0 (`gpio_os_rp2040.c`), the chip ID (`zephyr_hwinfo.c`), the console transport for the shell engine (`shell_serial_os_impl.h`: the Pico SDK's USB CDC ACM driver) and the replacement of the Pico SDK's stdio driver (`shell_stdio.c`, `shell_stdio.h`), wrappers of newlib's FILE functions (`stdio_wrap.c`), the RAM log's part (`log_ram_os_impl.h`: the ring's section, the interrupt mask, the TIMER) |

- **Time**: `k_uptime_*()`, `k_cycle_*()` and `time_sync_mono_us()` come from the RP2040's TIMER (1 MHz,
  `rp2040_timer_us()`), not from the tick.
- **Consoles**: the replacement of the Pico SDK's stdio driver (`shell_stdio.c`) and `--wrap` of newlib's FILE
  functions (`stdio_wrap.c`) route every output path through the shell engine's consoles. USB: the engine's
  serial session (`shell_serial.c`, the task `console`) on the console transport (`shell_serial_os_impl.h`,
  declared in `shell_serial_os.h`: the stdio redirection, input arrived, the line open, read, write). The prompt
  appears when the terminal opens the port, after the banner of `common/imu/port/pico_sdk/usb_console.c`. When
  the terminal stops reading, the Pico SDK's driver waits up to 500 ms for room in its 64-byte buffer and drops
  the rest.
- **Shared files only the builds on the Pico SDK compile**: `zephyr_wifi_shell.c` (`wifi`),
  `zephyr_zperf_shell.c`, `zperf_iperf3.c` with `internal/zperf_iperf3.h` (`zperf`), `log/fault_record.c` and
  `include/fault_record.h` (the format of the fatal error record), `common/shell/dmesg_shell.c` (`dmesg`),
  `common/shell/kernel_shell.c` (`id`, `uptime`, `date`, `sleep`, `devmem`, `fault`, `reboot`, `bootsel`, on the
  boundary `common/shell/port/kernel_shell_os.h`, implemented by `common/shell/port/<os>/kernel_shell_os_impl.h`),
  `common/shell/ifconfig_shell.c` (`ifconfig`). The RAM log `log/log_ram.c` (on its ring `log_ring.c`) is shared
  with the NuttX build.
- The RAM log and fatal errors, the start-up steps, the idle and the code in RAM are in
  [`design/freertos-port.md`](../../design/freertos-port.md).

### FreeRTOS

**Files** (`common/compat/port/freertos/`): the primitives of the generic kernel part
(`zephyr_kernel_os_impl.h`), the priority mapping (`priorities_os.h`) and the port header
(`rp2040/zephyr_os_port.h`: `z_os_in_isr()`, `z_os_yield_from_isr()`, `z_os_task_lock()`/`_unlock()`). The OS part
of the shell's generic commands is `common/shell/port/freertos/kernel_shell_os_impl.h` (with the tick count of
`uptime`).

The kernel part is the generic implementation (`kernel_generic/zephyr_kernel.c` and `zephyr_mutex.c`; how it
works is in their comments) on primitives that FreeRTOS provides. These assumptions of Zephyr are reproduced:

- **Mutexes** (`zephyr_mutex.c` on FreeRTOS's recursive mutex): recursive, with FreeRTOS's priority inheritance.
  An all-zero `k_mutex` is unlocked and is created by its first lock (Zephyr's MQTT library and settings rely on
  it). The return values are Zephyr's.
- **Semaphores and `k_poll`**: Zephyr's rules (a give hands the count to the highest-priority waiter,
  `k_sem_reset()` ends the waits with `-EAGAIN`) on a count of the layer's own; waits use a task notification
  (index 1; 0 is left to the Pico SDK and the application). A give from an interrupt handler asks for a switch at
  the end of the handler, so the woken task runs right after it.
- **Static threads**: the tasks of `K_THREAD_DEFINE()` are started by the start-up task (`main`) from the shared
  application's table. Every thread of the layer starts the same way (`z_thread_static_start()`: a task on its
  static stack, deleted when its function returns).
- **Thread custom data** (`k_thread_custom_data_get()`) is a FreeRTOS thread local storage pointer.

Left as approximations:

| Item | Zephyr | FreeRTOS build |
|---|---|---|
| Priorities | Lower runs first; cooperative -16..-1, preemptible 0..14 | `20 − Zephyr` (preemptible 0..14 become 20..6, cooperative -1 becomes 21). `Tmr Svc` 31, idle 0 |
| Cooperative threads | Not preempted | Approximated by a higher priority (a still higher task preempts; as on NuttX) |
| Priority inheritance | Follows a chain of holders, adjusted at each unlock | One level; dropped when the holder has released all its mutexes |
| Choice among one priority | A preempted thread goes on; 20 ms time slices, counted from the switch to the thread | FreeRTOS's: at every switch the next ready task of the priority in turn, so a preempted task goes behind the other ready tasks of its priority. 20 ms slices by the tick hook (`freertos/app/usb-console/src/tick.c`) on a fixed grid of 20 ticks, not counted from the switch |
| Tick and sleeps | 100 µs; a relative wait is "rounded up to ticks, plus one tick" | 1 ms (`configTICK_RATE_HZ` 1000, 32 bits), the same rounding: a wait shorter than 1 ms lasts 1 to 2 ms. A periodic SysTick (not tickless) |

The tick's wrap, the catch-up of the tick after interrupts were disabled (`tick.c`, decision in
`include/tick_catchup.h`) and why the Pico SDK's `sleep_ms()` is not used are in
[the FreeRTOS port's "The tick"](../../design/freertos-port.md#the-tick).

The tasks, the tick hook, the start-up and the rest of the FreeRTOS build are in
[`design/freertos-port.md`](../../design/freertos-port.md).

### ThreadX

**Files** (`common/compat/port/threadx/`): the primitives of the generic kernel part
(`zephyr_kernel_os_impl.h`), the lock's variables (`zephyr_kernel_os.c`), priorities and time slices
(`priorities_os.h`), the board's port header (`rp2040/zephyr_os_port.h`). The OS part of the shell's generic
commands is `common/shell/port/threadx/kernel_shell_os_impl.h`.

The primitives and what each must do are in `port/zephyr_kernel_os.h` and sections 2.5 and 2.6 of
[`design/os-boundary.md`](../../design/os-boundary.md). The ThreadX build makes them as follows.

- **The thread's lock** (`z_os_task_lock()`): interrupts masked (PRIMASK), and at the outermost level ThreadX's
  `_tx_thread_preempt_disable` raised. The unlock lowers it with interrupts still masked, restores the mask, then
  makes the switch the services inside the lock left with `_tx_thread_system_preempt_check()`. Masking alone is
  not enough: when a service makes a more urgent thread ready, ThreadX's Cortex-M port pends PendSV and unmasks
  interrupts for an instant to switch, which would switch in the middle of the lock. Every service checks
  `_tx_thread_preempt_disable` before it switches (ThreadX uses it around its own sequences), so "no switch until
  unlock" holds. Nothing waits inside the lock, so one depth and one saved mask serve all threads. An ISR's lock
  is PRIMASK alone.
- **The wake flag and the storage after the stack**: `K_THREAD_STACK_DEFINE()` keeps 272 bytes after each stack
  (`Z_OS_THREAD_RESERVED`) for a `struct z_tx_thread`: the `TX_THREAD` (200 bytes with the execution profile
  kit's), the wake flag's `TX_SEMAPHORE` (28), the thread's function and argument (8) and the name array (32).
  The wake flag is a binary semaphore with the ceiling 1 (`tx_semaphore_ceiling_put(…, 1)`), so a wake before the
  wait is kept. The flag is in that storage, so every thread that waits through the layer is started by
  `z_os_thread_start()` (the start-up thread "main" too); a thread made with ThreadX's API directly cannot call
  `k_sem_take()` or `k_poll()`.
- **Start of a thread**: the wake flag is made, `tx_thread_create()` is called unstarted (`TX_DONT_START`),
  `*tid` is set, then `tx_thread_resume()`. The thread cannot run before `*tid` is set (at the resume at once if
  it is more urgent than the caller). Also before the scheduler.
- **End of a thread and reuse of its stack**: `z_os_thread_exit()` `tx_thread_terminate()`s the caller. ThreadX
  keeps an ended thread in its list of created threads, `TX_TERMINATED` (`TX_COMPLETED` when its function
  returned), with the mutexes it held released. The next start on the same stack makes `z_os_thread_start()`
  delete that record and the wake flag first; on the stack of a thread that has not ended it returns `-EBUSY`.
- **Mutexes** (`k_mutex`): one `TX_MUTEX` (52 bytes) with priority inheritance (`TX_INHERIT`), recursive for its
  owner. All-zero is unlocked; the first lock creates it (inside the lock, so two threads cannot both create it).
  Right after the creation it is taken out of ThreadX's list of created mutexes (`_tx_mutex_created_ptr`,
  `_tx_mutex_created_count`) and it is never deleted: the application zeroes mutexes after use
  (`mqtt_client_init()` zeroes the client at every connection), which would break the list. ThreadX then refers
  to such a mutex only while a thread owns it (the owner's list of owned mutexes) or waits on it, as Zephyr does.
  ThreadX does not lower the owner when a waiter times out until the release, so on a timeout
  `z_os_mutex_lock()` computes the owner's priority as ThreadX's release does and sets it with
  `_tx_mutex_priority_change()` (Zephyr's rule of `kernel/mutex.c`; the acceptance test's `mutex_inherit` checks
  it). Inheritance is one level, and at each release the owner returns to the highest waiter of the mutexes it
  still holds, or its own priority.
- **Priorities** (`priorities_os.h`): the layer's value is 15 − the Zephyr priority (larger more urgent),
  ThreadX's priority 31 − that value, that is the Zephyr priority + 16. Zephyr's 32 levels (−16 to 15) map one to
  one on ThreadX's 32 (0 to 31). Zephyr's idle 15 (ThreadX 31) is not given to a thread of the layer. The kernel
  has no thread of its own (no idle thread, no timer thread).
- **Time slices**: threads of Zephyr priority 0 and below in urgency (layer value 15 and lower) get ThreadX's
  `time_slice` of 20 ticks from `z_os_thread_start()` (the Zephyr build's `CONFIG_TIMESLICE_SIZE=20`,
  `CONFIG_TIMESLICE_PRIORITY=0`); the cooperative priorities get none. ThreadX keeps the rest of a preempted
  thread's slice and starts a new one when the slice is used up (Zephyr starts a new one at every switch).
- **Clocks**: `k_cycle_*()` and `k_uptime_*()` are the RP2040 TIMER (`rp2040_timer_us()`), not the tick.
- **Assertion**: `Z_OS_ASSERT()` is `app_assert_failed()` (fault record and restart). The error checking of
  ThreadX's API stays on; a bad call returns an error code and is not a fatal error.

The list of the places that use ThreadX's internals (`_tx_thread_preempt_disable` and others), the tick, the
idle, the fatal errors and the threads are in [`design/threadx-port.md`](../../design/threadx-port.md).

## Tests

The host tests are in `common/tests/run.sh`, which each OS's `script/host_tests.sh` runs before its own tests.
They need no board.

- **The layer on the fake kernel** (`common/tests/zephyr_fakes`: the OS part, `zephyr_os*.h`, and its functions):
  RTIO (pool, completions, multishot, cancellation, work queue) in `zephyr_rtio`, work items (submit, schedule,
  reschedule, cancel) in `zephyr_work`, and the minimal MQTT client (`common/imu/mqtt_min.c`) in `mqtt_min`.
  `nuttx/script/host_tests.sh` also runs the MQTT test on Zephyr's own library when the Zephyr tree is there.
- **Tick catch-up** (`tick_catchup`): the step of `include/tick_catchup.h` alone and on a model of SysTick, its
  pending bit and masked interrupts.
- **Where the headers belong**: each header of `include/`, `internal/`, `port/` and `port/generic/` belongs to the
  layer as a whole or to one part ("Directories" above). Then `kernel_generic_independent` checks that the
  generic kernel part (`<zephyr/kernel.h>`, `<zephyr/spinlock.h>`, `zephyr_compat_os.h`, the sources of
  `kernel_generic/`, `zephyr_heap.c`, `zephyr_work.c`, `zephyr_init.c`) compiles with the smallest form of an OS's
  primitives (`common/tests/kernel_generic_host`) and `port/generic` alone and reads no header of the RAM log,
  the shell engine, `net_offload` or `libc_net`, and `net_offload_independent` that the network part compiles on
  a kernel part written directly on an OS (the fake kernel) without `kernel_generic`, `port/generic`, the RAM log,
  the shell engine or `libc_net` (`cc -M`).
- **Others**: subcommand matching with the copy of Zephyr's `fnmatch()` (`subcmd_fnmatch`), the copies of getopt
  (`sys_getopt`, with two states parsing two command lines interleaved), the RAM log's ring and the RAM log
  (`log_ring_*`, `log_ram_*`, with the ring sizes of the FreeRTOS build, 4096, and of the NuttX build, 8192), the
  CPU load arithmetic (`cpu_load`), the fatal error record (`fault_record`), the hex conversions (`hex`), the
  key-value area's helpers (`flash_kv`), the key-value store (`kv_settings`: `common/imu/kv_store.c` on the copies
  of NVS and settings, `zephyr_flash_kv.c` and `kv_store_wipe.c`, on a fake flash: the bytes on the flash, reads,
  writes, limits, garbage collection, foreign data, the device's refusals, a power loss at every flash operation
  of a sequence).
- **Copies of Zephyr's files** (`zephyr_copies`): when the Zephyr tree (`zephyr/zephyr`, v4.5.0) is there, `cmp`
  checks that the 23 files of NVS, settings, CRC-8, the hex conversions, `fnmatch()`, getopt and
  `wifi_credentials_shell.c` with their headers are the originals. `wifi_credentials.c` and
  `wifi_credentials_backend_settings.c` are copies with changes (see their headers).
- **The acceptance test of the layer's kernel** (`common/tests/zephyr_kernel/`): the OS-independent
  `zephyr_kernel_test.c`; `zephyr_kernel_test.h` lists what an OS's part and a platform's part provide, and
  `kernel_generic/zephyr_kernel_test_os.h` its items on the primitives. It runs on an OS's part: the FreeRTOS
  build's in `freertos/tests/compat/` (on FreeRTOS's POSIX port), the ThreadX build's in `threadx/tests/compat/`
  (on ThreadX's Linux port).
