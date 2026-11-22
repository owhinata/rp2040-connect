# The boundary between the shared code and an OS

English | [日本語](os-boundary-ja.md)

An index of what a build supplies to the shared sources (`common/`), as the code has it today. Each
row names the header that declares an item and the file of each build that implements it. "N" is the
NuttX build, "F" the Pico SDK + FreeRTOS build. The Zephyr build compiles `common/` against Zephyr
itself and does not use `common/compat`; it appears only where it has a file of its own.

Paths below `nuttx/boards/arduino-nano-rp2040-connect/` are shortened to `N-board/`.

## 1. What a build consists of

| Part | Where | Holds |
|---|---|---|
| Application | `common/imu` | IMU service, MQTT transport, time synchronization, Wi-Fi profile, KV store, calibration |
| Drivers | `common/drivers/<driver>/` | OS-independent core; `port/` boundary headers; `port/<os>/` bindings (section 5) |
| Shell commands | `common/shell` | `zshell.c` (Zephyr shell command handling), `subcmd.c`, `nina_shell.c`, `i2c_dma_shell.c` |
| Log | `common/log` | RAM log ring (`log_ring.c`), `LOG_*` macros (`app_log.h`), fault record text (`fault_record.c`) |
| Compatibility layer, shared part | `common/compat` | Zephyr headers (`common/compat/zephyr/`), work items, RTIO, gpio0 device, net_mgmt, Wi-Fi requests, KV flash device, copies of Zephyr's NVS, settings, getopt, Wi-Fi credentials, zperf |
| Compatibility layer, network part for an OS without a network stack | `common/compat/net_offload` | Zephyr's socket layer (reduced copies of `subsys/net/lib/sockets`, `lib/os/zvfs`), the one network interface (`zephyr_net_if.c`), `k_heap` (`zephyr_heap.c`), their headers in its own include root `include/` (`zephyr/net/net_if.h`, `net_ip.h`, `socket.h`, `socket_offload.h`, `zephyr/sys/fdtable.h`, `zephyr_heap.h`); used by F only |
| POSIX network headers for a C library without them | `common/compat/libc_net` | `include/` (`arpa/inet.h`, `netdb.h`, `netinet/in.h`, `netinet/tcp.h`, `poll.h`, `sys/socket.h`, `sys/uio.h`), `inet_pton()`/`inet_ntop()` (`inet.c`); used by F only |
| Self-test of the compatibility layer (acceptance test of an OS part) | `common/compat/selftest` | `zephyr_selftest.c` (22 tests), its boundary `zephyr_selftest.h` (section 3); used only by F's `-DNANO_SELFTEST=ON` build and F's host test |
| Shell engine for an OS without a shell of its own | `common/compat/shell_engine` | line editor (`cli.c`), command table and dispatch of the `Z_SHELL_CMD_ROOT_GLOBAL` roots (`nano_shell.c`; the application supplies `nano_shell_cmds[]`), Zephyr's word split (`shell_argv.c`); used by F only |
| Host tests | `common/tests` | Tests and fakes; `common/tests/zephyr_fakes` is a host implementation of the kernel's OS part |
| Zephyr build | `zephyr/app/usb-console` | `main()`, `log_ram.c`, `kv_store_os.h`, `wifi_profile_os.h`; upstream Zephyr with patches in `zephyr/manifest/zephyr/patches` |
| N: board | `N-board/` | Bring-up, tickless timer, idle loop and CPU load, GPIO bank 0, KV flash, USB serial, `nano_mono_us()` (`include/nano_timer.h`) |
| N: compatibility layer, OS part | `nuttx/drivers/compat` | `zephyr_os*.h`, kernel, mutexes, I2C on `struct i2c_master_s`, net_if, hwinfo, prelude, per-OS `<zephyr/...>` headers |
| N: shared OS headers | `nuttx/drivers/include` | `app_log_os.h`, `priorities_os.h` |
| N: log | `nuttx/drivers/log` | `log_ram.c` |
| N: drivers | `nuttx/drivers/<driver>/` | `Make.defs`, `Kconfig` of each driver |
| N: applications | `nuttx/apps/*` | Init task (`init`), one builtin application per command, shell OS part (`subcmd`), Wi-Fi and sensor parts |
| N: NuttX changes | `nuttx/patches` | Local patches to NuttX and nuttx-apps |
| F: application | `freertos/app/usb-console` | `main()`, `FreeRTOSConfig.h`, tick hook, idle and CPU load, `os_clock.h`, `priorities_os.h`, command table, fault handling |
| F: compatibility layer, OS part | `freertos/drivers/compat` | `zephyr_os*.h`, kernel, mutexes, realtime clock, start of the network part and its event thread, linker script part of the socket registrations, `rp2040/` (port header, GPIO bank 0, hwinfo, the self-test's board part), the self-test's OS part (`zephyr_selftest_os.h`), prelude |
| F: shell | `freertos/drivers/shell` | Consoles, stdio per console, getopt state, shell OS part |
| F: C library additions | `freertos/drivers/libc` | stdout/stderr wrappers |
| F: other OS parts | `freertos/drivers/{include,log,kv,telnet,wifi,sensor_shell,imu,zperf}` | `app_log_os.h`; `log_ram.c`; KV flash; Telnet; Wi-Fi OS header; sensor shell OS part; per-source preludes; build files |
| F: drivers | `freertos/drivers/<driver>/*.cmake` | The sources and options of each driver |

## 2. Kernel part of the compatibility layer

Declared in `common/compat/zephyr/kernel.h`, which includes `"zephyr_os.h"` first.
Implementations: N `nuttx/drivers/compat/`, F `freertos/drivers/compat/`.

### 2.1 Provided by `zephyr_os.h` (13 items)

| Item | Meaning | N | F |
|---|---|---|---|
| OS headers before the layer's | the layer's `<zephyr/sys/atomic.h>` takes over atomic names | NuttX headers, `<arch/board/nano_timer.h>` | FreeRTOS headers, `zephyr_os_port.h`, `priorities.h`, `<zephyr/sys/clock.h>` |
| `Z_USEC_PER_TICK` | tick length in µs; 1000 must be a multiple | `CONFIG_USEC_PER_TICK` (100) | `1000000 / configTICK_RATE_HZ` (1000) |
| `struct k_mutex`, `Z_MUTEX_INITIALIZER(obj)` | all-zero must work as an unlocked mutex | `rmutex_t` + `ready` flag | static recursive mutex, created on first use |
| `struct k_sem`, `Z_SEM_INITIALIZER(obj, initial, limit)` | count with a limit; poll event list | `sem_t` + limit + events | the layer's own count, waiters, events |
| `K_SEM_MAX_LIMIT` | largest limit | `SEM_VALUE_MAX` | `UINT_MAX` |
| `struct _timeout`, `_timeout_func_t` | timer of a delayable work item | `struct wdog_s` | entry in the layer's timeout list |
| `k_tid_t` | thread handle | `pid_t` | `TaskHandle_t` |
| `k_current_get()` | the calling thread | `gettid()` | `xTaskGetCurrentTaskHandle()` |
| `K_PRIO_PREEMPT(x)` | OS priority of Zephyr priority x, order kept | `NANO_PRIO_FROM_ZEPHYR` (118 − x) | `NANO_PRIO_FROM_ZEPHYR` (20 − x) |
| `k_cycle_get_32()`, `k_cycle_get_64()` | 1 MHz counter | `nano_mono_us()` | `nano_mono_us()` |

F's `zephyr_os.h` also includes `common/compat/net_offload/include/zephyr_heap.h`: `struct k_heap`, `K_HEAP_DEFINE()`, `k_heap_alloc()`, `k_heap_free()` (section 4).

### 2.2 Functions the OS implements (21 functions, 1 object)

Files: N `zephyr_kernel.c`, `zephyr_mutex.c`; F `zephyr_kernel.c`, `zephyr_mutex.c`.

| Function | Required meaning (kernel.h) |
|---|---|
| `k_mutex_init()` | initialize |
| `k_mutex_lock(m, t)` | recursive for the owner; K_FOREVER not ended by a signal; K_NO_WAIT −EBUSY; timeout −EAGAIN |
| `k_mutex_unlock(m)` | release one level |
| `k_sem_init(s, initial, limit)` | 0 or −EINVAL |
| `k_sem_take(s, t)` | 0, −EBUSY (K_NO_WAIT, none), −EAGAIN (timed out) |
| `k_sem_give(s)` | +1 up to the limit, wakes pollers; any context incl. interrupts |
| `k_sem_reset(s)` | count to 0 |
| `k_sem_count_get(s)` | the count |
| `k_poll_event_init(e, type, mode, obj)` | semaphores only, `K_POLL_MODE_NOTIFY_ONLY` |
| `k_poll(events, n, t)` | 0 when one is ready (does not take), −EAGAIN on timeout |
| `k_thread_name_set(tid, name)` | name shown by `ps`; 0 or −EINVAL |
| `z_thread_static_start(thread)` | run a `K_THREAD_DEFINE()` thread on its static stack, set `*tid`; 0 or −errno |
| `k_uptime_get()` | ms since boot, rounded down (`Z_UPTIME_MS()`) |
| `k_sleep(t)` | K_NO_WAIT yields; else wakes at tick + ticks + 1; K_FOREVER for ever; returns 0 |
| `k_msleep(ms)`, `k_usleep(us)` | rounded up to ticks, then as `k_sleep()`; ≤ 0 yields; returns 0 |
| `k_busy_wait(us)` | spin at least `us` µs |
| `k_work_queue_start(q, stack, size, prio, cfg)` | start the queue's thread; it calls `z_work_queue_init()` and loops on `z_work_queue_process()` |
| `z_add_timeout(to, fn, t)` | run `fn` after the timeout, in interrupt context (not while `k_spin_lock()` is held) |
| `z_abort_timeout(to)` | stop it |
| `zephyr_compat_initialize()` | start `k_sys_work_q`, the RTIO work queue, gpio0 (declared in `common/compat/zephyr_compat.h`) |
| object `k_sys_work_q` | the system work queue ("sysworkq", `NANO_PRIO_SYSWORKQ`) |

The work item logic itself is shared: `common/compat/zephyr_work.c`.

| Function | N | F |
|---|---|---|
| mutex | `nxrmutex_*()` | `xSemaphore*Recursive()` |
| `k_sem_*` | `nxsem_*()` | the layer's count; waiters sleep on task notification index 1 |
| threads, queue threads | `kthread_create_with_stack()` | `xTaskCreateStatic()` (`z_os_task_start()`), scheduler suspended around it |
| sleeps | `nxsched_ticksleep()`, `nxsched_msleep()`, `nxsched_usleep()` | `vTaskDelay(ticks + 1)` |
| `k_busy_wait()` | `up_udelay()` | `busy_wait_us_32()` |
| `z_add_timeout()` | `wd_start()` | sorted list run by `zephyr_compat_tick()` from the tick hook |

### 2.3 Behaviour that differs between N and F

| Point | N | F (Zephyr's rules) |
|---|---|---|
| `k_sem_give()` with a thread in `k_sem_take()` | posts the count and also signals every registered poll event | the waiter takes the count; no poll event is signaled |
| `k_sem_give()` with pollers | signals every registered event, wakes every poller | signals only the first registered event |
| `k_sem_reset()` and waiters | count drained; waiters keep waiting | waiters return −EAGAIN |
| Order of registered poll events | newest first (prepended) | by thread priority, first come within one |
| `k_poll()` after waking | re-checks every semaphore's count | counts only signaled events |
| `k_sem_take()` in an interrupt with a timeout | not checked | −EBUSY |
| `k_mutex_unlock()` errors | `nxrmutex_unlock()`'s result | −EINVAL (not held), −EPERM (other holder) |
| `k_thread_name_set()` | −ENOSYS without `CONFIG_TASK_NAME_SIZE` | NULL means the caller |
| End of a `K_THREAD_DEFINE()` thread | kernel thread returns | `vTaskDelete(NULL)` |
| Tick | 100 µs | 1 ms |
| `zephyr_compat_initialize()` gpio0 | always (the whole kernel part is built only with `CONFIG_NANO_LSM6DSO_STREAM`) | only with `CONFIG_NANO_LSM6DSO_STREAM`; −ENOMEM if sysworkq did not start |
| Caller of `zephyr_compat_initialize()` | `N-board/src/rp2040_bringup.c` | `main()` before the scheduler, `freertos/app/usb-console/src/main.c` |
| Same-priority choice | NuttX's | FreeRTOS's round robin at every switch |

### 2.4 Spinlock and atomics

| Header (shared) | Includes | OS provides | N | F |
|---|---|---|---|---|
| `common/compat/zephyr/spinlock.h` | `"zephyr_os_spinlock.h"` | `struct k_spinlock`, `k_spinlock_key_t`, `k_spin_lock()`, `k_spin_unlock()`: interrupts masked and no switch until unlock; threads and interrupts; nests | `sched_lock()` + `up_irq_save()` | `taskENTER_CRITICAL()` in a task, `taskENTER_CRITICAL_FROM_ISR()` in an interrupt |
| `common/compat/zephyr/sys/atomic.h` | `"zephyr_os_atomic.h"` | OS headers that define atomic names, read first | reads `<nuttx/atomic.h>`, undefines its `atomic_*` macros | empty |

The atomic operations themselves are shared (on `k_spin_lock()`).

### 2.5 F-only port header

`freertos/drivers/compat/rp2040/zephyr_os_port.h`, read by F's `zephyr_os.h`: `nano_mono_us()` (from
`os_clock.h`), `z_os_in_isr()`, `z_os_yield_from_isr()`, `z_os_task_lock()`, `z_os_task_unlock()`,
`z_os_busy_wait_us()`, `z_os_tick_now_us32()`, `z_os_tick_pend()`. The host tests have
`freertos/tests/compat/zephyr_os_port.h`.

## 3. Other declared boundaries

| Boundary | Declared in | Items | Called by | N | F |
|---|---|---|---|---|---|
| Shell backend | `common/shell/zshell_os.h` | `zshell_os_backend_get()`, `zshell_os_cmd_enter()`, `zshell_os_cmd_exit()`, `z_shell_backend_vfprintf()`; optional `zshell_os_active_cmd_set()`/`_get()` with `ZSHELL_OS_ACTIVE_CMD` | `common/shell/zshell.c` | `nuttx/apps/subcmd/zshell_os_impl.h` (no `ZSHELL_OS_ACTIVE_CMD`) | `freertos/drivers/shell/zshell_os_impl.h` (defines it) |
| `struct shell` | read by `common/compat/zephyr/shell/shell.h` | one backend, outlives the command | shell commands | `nuttx/drivers/compat/zephyr_os_shell.h` (terminal path, command pid) | `freertos/drivers/shell/zephyr_os_shell.h` (console, active command) |
| Log, OS part | `common/log/app_log.h` | `app_log_os.h` (OS headers first, optional `CONFIG_NANO_LOG_DEFAULT_LEVEL`, `CONFIG_NANO_LOG_MAX_LEVEL`); `app_log_printf()`, `app_log_vprintf()`, `app_log_ratelimit_pass()` | `LOG_*()` macros | `nuttx/drivers/include/app_log_os.h`, `nuttx/drivers/log/log_ram.c` | `freertos/drivers/include/app_log_os.h`, `freertos/drivers/log/log_ram.c` |
| RAM log | `common/log/log_ram.h` | `log_ram_initialize()`, `log_ram_piece()` | start-up; `dmesg` | `nuttx/drivers/log/log_ram.c`; called by `N-board/src/rp2040_boardinitialize.c`, `nuttx/apps/dmesg/dmesg_main.c` | `freertos/drivers/log/log_ram.c` (also holds `dmesg`); called by `main()` |
| KV flash | `common/compat/nano_flash_kv.h` | `nano_flash_kv_os_ready()`, `_read()`, `_program()` (256-byte page), `_erase()` (4 KiB sector); range checked again by the OS | `common/compat/zephyr_flash_kv.c` | `N-board/src/nano_flash_kv.c` | `freertos/drivers/kv/nano_flash_kv.c` |
| KV wipe command | `common/compat/kv_store_os.h` | `kv_store_wipe()`, `kv_store_cmd_wipe()`, `KV_STORE_OS_COMMANDS` | `common/imu/kv_store.c` | shared: `common/compat/kv_store_wipe.c` | shared: same |
| GPIO bank 0 | `common/compat/nano_gpio.h` | OS: `nano_gpio_configure_input()`, `nano_gpio_get_all()`, `nano_gpio_irq_configure()`; layer: `nano_gpio_irq_handler()` (called from the bank interrupt) | `common/compat/zephyr_gpio.c`; N also `common/drivers/nina/port/nuttx/nina_os.c` | `N-board/src/nano_gpio.c` | `freertos/drivers/compat/rp2040/nano_gpio.c` |
| CPU load | `common/compat/cpu_load.h`, arithmetic `common/compat/nano_cpuload.h` | `cpu_load_get(reset)` in tenths of a percent, from idle windows of the TIMER | `common/imu/imu_stream.c` | `N-board/src/rp2040_idle_cpuload.c` | `freertos/app/usb-console/src/idle.c` |
| Priorities and stacks | `common/compat/priorities.h` | OS: `NANO_PRIO_FROM_ZEPHYR(p)` and its own threads; shared: 15 `NANO_PRIO_*`/`NANO_STACK_*` pairs | the layer, bindings, start-up | `nuttx/drivers/include/priorities_os.h` (118 − p; `NINA_EVENT`, `NINA_PT`) | `freertos/app/usb-console/src/priorities_os.h` (20 − p; `TIMER_SVC`, `IDLE`, `HEARTBEAT`) |
| net_mgmt thread | included at the end of `common/compat/zephyr_net_mgmt.c` | `mgmt_event_thread()` and `net_mgmt_event_init()` (declared in `common/compat/zephyr/net/net_mgmt.h`) on the file's queue and `mgmt_stack` | N `common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`; F `freertos/drivers/compat/zephyr_net_init.c` | `nuttx/drivers/compat/zephyr_net_mgmt_os.h` | `freertos/drivers/compat/zephyr_net_mgmt_os.h` |
| RTIO work queue | `common/compat/zephyr/rtio/work.h` | `rtio_workq_initialize()` starts "RTIO WQ 0" with `z_thread_static_start()`; no OS file | each `zephyr_compat_initialize()` | shared `common/compat/zephyr_rtio_workq.c` | same |
| Wi-Fi profile | included by `common/imu/wifi_profile.c` | `extern k_tid_t wifi_reconnect_tid;` | `common/imu/wifi_profile.c` | `nuttx/apps/wifi/wifi_profile_os.h` | `freertos/drivers/wifi/wifi_profile_os.h` |
| Sensor shell | included first by `common/drivers/sensor_shell/sensor_shell.c`, `common/compat/sensor_shell_stream.c` | `CONFIG_SENSOR_SHELL_STREAM`, `_THREAD_STACK_SIZE`, `_TRIG_PRINT_TIMEOUT_MS`, `_MAX_TRIGGER_DEVICES`; `sensor_shell_os_get_arm()`, `sensor_shell_os_get_wait()`, `sensor_shell_os_processing_callback` | those two files | `nuttx/apps/sensor/sensor_shell_os.h`, `sensor_shell_nuttx.h`, `sensor_shell_nuttx.c` | `freertos/drivers/sensor_shell/sensor_shell_os.h`, `sensor_shell_os.c` |
| Start of the layer | `common/compat/zephyr_compat.h` | `zephyr_compat_initialize()`, `K_THREAD_DEFINE_START(name)` | start-up | section 2 | section 2 |
| Self-test of the layer, OS part | `common/compat/selftest/zephyr_selftest.h` (`zephyr_selftest.c` reads `"zephyr_selftest_os.h"` after `<zephyr/kernel.h>`) | type `zephyr_selftest_os_prio_t`, `ZEPHYR_SELFTEST_OS_NAME_MAX`, `zephyr_selftest_os_tick()`, `_tick_sync()`, `_thread_create()`, `_thread_exit()`, `_prio_get()`, `_prio_self()`, `_name_get()`, `_current()`, `_in_isr()`, `_sched_lock()`, `_sched_unlock()` | `common/compat/selftest/zephyr_selftest.c` | — (not built) | `freertos/drivers/compat/zephyr_selftest_os.h` (FreeRTOS tasks; the same on the board and the host) |
| Self-test of the layer, board part | same | `zephyr_selftest_port_alarm()`, `_quiet()`, `_realtime()`, `_mask_irqs()` | same | — | board `freertos/drivers/compat/rp2040/zephyr_selftest_port.c`, host `freertos/tests/compat/main.c` |
| F: tick and network start | `freertos/drivers/compat/zephyr_compat_os.h` | `zephyr_compat_tick()`, `zephyr_net_initialize()` | F tick hook, `main_main()` | — | `freertos/drivers/compat/zephyr_kernel.c`, `zephyr_net_init.c` |

The Zephyr build has its own `zephyr/app/usb-console/src/kv_store_os.h` and `wifi_profile_os.h`.

## 4. What each build supplies without a shared declaration

### 4.1 `<zephyr/...>` headers supplied per OS

The OS directory comes first on the include path; `common/compat/zephyr/` has the rest. F's network
headers are in `common/compat/net_offload/include`, which F puts after its OS directory and before
`common/compat`; N does not have it on its include path.

| Header | N | F |
|---|---|---|
| `zephyr/drivers/i2c.h` | `nuttx/drivers/compat/zephyr/drivers/i2c.h` on `struct i2c_master_s` (`zephyr_i2c.c`) | `freertos/drivers/compat/zephyr/drivers/i2c.h` (Zephyr's API table) |
| `zephyr/net/net_if.h` | on `struct net_driver_s` (`zephyr_net_if.c`) | `common/compat/net_offload`: the one NINA interface (`zephyr_net_if.c`) |
| `zephyr/net/net_ip.h` | — (uses `common/compat`'s) | `common/compat/net_offload`, replaces `common/compat`'s: `net_*` names on POSIX types |
| `zephyr/net/socket.h` | `zsock_*()` are NuttX's `socket()`, `poll()`, ... (usrsock) | `common/compat/net_offload`: Zephyr's socket layer, `subsys/net/lib/sockets/sockets.c`, `socket_offload.c`, `getaddrinfo.c` |
| `zephyr/net/socket_offload.h` | — | `common/compat/net_offload`: offloaded resolver |
| `zephyr/sys/fdtable.h` | — | `common/compat/net_offload`: descriptor table, `lib/os/zvfs/zvfs_fdtable.c`, `zvfs_poll.c` |
| `zephyr/sys/__assert.h` | `DEBUGASSERT()` | nothing (as `CONFIG_ASSERT` off) |
| `zephyr/sys/clock.h` | inline on `clock_gettime()`/`clock_settime()` | `zephyr_sys_clock.c` (TIMER + base) |

### 4.2 Functions and objects

| Item | Where declared | N | F |
|---|---|---|---|
| `nano_mono_us()`: µs since boot, TIMER, any context | per OS only | `N-board/include/nano_timer.h` | `freertos/app/usb-console/src/os_clock.h` |
| TIMER read by `common/imu/time_sync.c` | the file itself (`DT_REG_ADDR(DT_NODELABEL(timer))` = 0x40054000, `sys_read32()`) | — | `freertos/drivers/imu/time_sync_prelude.h` undefines the SDK's `TIMER_BASE` |
| `k_heap` | `common/compat/net_offload/include/zephyr_heap.h`, included by F `zephyr_os.h` | not built | `common/compat/net_offload/zephyr_heap.c`; used by `common/drivers/nina/port/zephyr/nina_socket.c` |
| `zephyr_compat_tick()` | F `zephyr_compat_os.h` | — (wdog) | called by `vApplicationTickHook()`, `freertos/app/usb-console/src/tick.c` |
| `zephyr_net_initialize()` | F `zephyr_compat_os.h` | — (`nina_net_initialize()` in `rp2040_bringup.c`, `common/drivers/nina/port/nuttx/nina_net.h`) | `freertos/drivers/compat/zephyr_net_init.c` |
| `hwinfo_get_device_id()` | `common/compat/zephyr/drivers/hwinfo.h` | `nuttx/drivers/compat/zephyr_hwinfo.c` | `freertos/drivers/compat/rp2040/zephyr_hwinfo.c` |
| `nano_flash_kv_initialize()` | per OS | `N-board/src/rp2040_pico.h`, `nano_flash_kv.c`; called by `rp2040_bringup.c` | `freertos/drivers/kv/nano_flash_kv_os.h`, `nano_flash_kv.c`; called by `main()` |

### 4.3 Devices behind `DEVICE_DT_GET()`

`DEVICE_DT_GET(DT_NODELABEL(x))` is `&nano_device_x` (`common/compat/zephyr/device.h`); `DEVICE_DT_DEFINE()` makes it.
The devices listed for the shell are in `common/compat/zephyr_device.c` (hand-written).

| Node | N | F |
|---|---|---|
| `gpio0` | `common/compat/zephyr_gpio.c` | same |
| `i2c0` | `N-board/src/rp2040_lsm6dso.c`, bound to the NuttX I2C master (`zephyr_i2c.c`) | `common/drivers/i2c_dw/port/freertos/i2c_dw_instance.h` |
| `i2c1` | no Zephyr device (`struct i2c_master_s` only) | `common/drivers/i2c_dw/port/freertos/i2c_dw_instance.h` |
| `lsm6dsox` | the driver's port, `common/drivers/lsm6dso/port/` | same |
| flash device | `common/compat/zephyr_flash_kv.c` (`z_flash_kv_device`) | same |

### 4.4 Shell commands: registration, getopt, stdio

| Point | N | F |
|---|---|---|
| Registration | each command a builtin application; `SHELL_CMD_REGISTER()` makes `main()` (or `z_shell_main_<name>()` with `Z_SHELL_CMD_NAMED_MAIN`) | `Z_SHELL_CMD_ROOT_GLOBAL`: global root entries listed in `nano_shell_cmds[]`, `freertos/app/usb-console/src/commands.c`; run by `common/compat/shell_engine/nano_shell.c` |
| `SHELL_SUBCMD_ADD()` | global entry; the parent's source lists them by hand | same |
| getopt | Zephyr's getopt not built; `nuttx/apps/wifi/wifi_credentials_shell.c` maps `sys_getopt_*` to `getopt_long()` | copies in `common/compat/lib/utils/getopt`; state per session `freertos/drivers/shell/getopt_state.c` (`sys_getopt_state_get()`, `z_getopt_global_state_update()`), reset by `sys_getopt_init()` before each command |
| stdio of `zshell.c`: `printf()`, `fprintf(stderr)`, `fflush(stdout)` must reach the command's terminal; `calloc()` of about 1.2 KB per line | each command a task with its own stdio | `freertos/drivers/shell/nano_stdio.c` (the calling task's console), `freertos/drivers/libc/stdio_wrap.c` (`--wrap` of the stdout/stderr calls) |

### 4.5 `CONFIG_*` values

| Source | N | F |
|---|---|---|
| Prelude, `-include` on shared sources | `nuttx/drivers/compat/common_prelude.h`: NuttX headers first; `CONFIG_SENSOR_LOG_LEVEL` 3; `CONFIG_SENSOR_ASYNC_API` with `CONFIG_NANO_LSM6DSO_STREAM` | `freertos/drivers/compat/common_prelude.h`: `CONFIG_SENSOR_LOG_LEVEL` 3; `CONFIG_NINA_NTP_SERVER` "", `_PORT` 123, `_INTERVAL_MS` 2000; `CONFIG_NINA_TCP_MAX_IO` 3584; `CONFIG_NINA_SEND_RETRY_MS` 1; `CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY` 40; `CONFIG_NINA_PASSTHROUGH` 1; `CONFIG_SENSOR_ASYNC_API` with the stream |
| NINA and NTP options | Kconfig: `nuttx/drivers/nina/Kconfig`, `nuttx/apps/imu/Kconfig` | the prelude (above) |
| NVS and settings copies | `common/compat/zephyr_kv_config.h`, `-include` from `nuttx/apps/imu/Makefile` | same, from `freertos/drivers/kv/kv.cmake` |
| Build definitions | Kconfig (`nuttx/drivers/*/Kconfig`, `nuttx/apps/*/Kconfig`, `N-board/Kconfig`), `N-board/configs/imu/defconfig` | `compat.cmake`: `Z_SHELL_CMD_ROOT_GLOBAL`, `CONFIG_GETOPT_LONG=1`, `PICO_STDOUT_MUTEX=0`; `i2c_dw.cmake`: `CONFIG_I2C_DW_I2C0`, `_I2C1`; `lsm6dso.cmake`: `CONFIG_NANO_LSM6DSO_STREAM`; `wifi.cmake`: `CONFIG_NANO_WIFI_CREDENTIALS`; `nina.cmake`: `NINA_OS_DNS_OFFLOAD` for the resolver |
| In shared headers | `kernel.h`: `CONFIG_THREAD_NAME` 1; `zephyr/rtio/work.h`: `CONFIG_RTIO_WORKQ_POOL_ITEMS` 4 | same |
| Sensor shell | `sensor_shell_nuttx.h` | `sensor_shell_os.h` (same four values) |

### 4.6 Threads and start-up order

`K_THREAD_DEFINE()` threads are not started automatically; the start-up code lists them.

| Thread (defined in) | N | F |
|---|---|---|
| `nina_scan_tid`, `nina_connect_tid`, `nina_monitor_tid` (`common/drivers/nina/port/zephyr/nina_net.c`) | not built; `nina_net_start()` starts the core's threads | `K_THREAD_DEFINE_START()` |
| `imu_stream_tid` (`common/imu/imu_stream.c`) | `K_THREAD_DEFINE_START()` | same |
| `imu_raw_mqtt_tid` (`common/imu/imu_raw_mqtt.c`) | same | same |
| `time_sync_tid` (`common/imu/time_sync.c`) | same | same |
| `wifi_reconnect_tid` (`common/imu/wifi_profile.c`) | same | same |
| `sensor_shell_processing_tid` (`common/compat/sensor_shell_stream.c`) | same | same |
| `zperf_server_tid` (`common/compat/zperf_iperf3.c`) | not built | same |

Start-up files: Zephyr `zephyr/app/usb-console/src/main.c`; N `N-board/src/rp2040_boardinitialize.c`,
`N-board/src/rp2040_bringup.c`, `nuttx/apps/init/rp2040_init.c`; F `freertos/app/usb-console/src/main.c`
(`main()`, then the task `main_main()`).

| Step | N | F |
|---|---|---|
| RAM log | `log_ram_initialize()` in `rp2040_boardinitialize()` | first in `main()` |
| NINA passthrough check | `nina_passthrough_active()` in the init task | `nina_passthrough_check()` in `main()` |
| Layer | `zephyr_compat_initialize()` in `rp2040_bringup()` | `zephyr_compat_initialize()` in `main()` |
| I2C0, I2C1 | `i2c_dw_i2c0_initialize()`, `i2c_register()` in `rp2040_bringup()` | `i2c_dw_i2c0_initialize()`, `_i2c1_` in `main()` |
| LSM6DSOX | after I2C0 in `rp2040_bringup()` | first in `main_main()` |
| Network | `nina_net_initialize()` in `rp2040_bringup()` | `zephyr_net_initialize()` in `main_main()` |
| KV flash bounds | `nano_flash_kv_initialize()` in `rp2040_bringup()` | `nano_flash_kv_initialize()` in `main()` |
| Wi-Fi credentials | `wifi_credentials_init()` in the init task | `wifi_credentials_init()`, `wifi_shell_init()` in `main_main()` |
| Threads | table above | table above |
| Telnet | `rp2040_telnet_start()` after the threads | `nano_telnet_init()` before the threads |
| Application (same order in all three builds) | `kv_store_init()`, `imu_cal_init()`, `time_sync_init()`, `wifi_profile_init()` | same |

### 4.7 POSIX socket headers

| N | F |
|---|---|
| NuttX's libc (`<sys/socket.h>`, `<netinet/in.h>`, `<arpa/inet.h>`, `<netdb.h>`, `<poll.h>`) | `common/compat/libc_net/include`: `arpa/inet.h`, `netdb.h`, `netinet/in.h`, `netinet/tcp.h`, `poll.h`, `sys/socket.h`, `sys/uio.h`; `common/compat/libc_net/inet.c` |

## 5. Drivers

| Driver | Boundary headers (`port/`) | N files | F files | Zephyr |
|---|---|---|---|---|
| `common/drivers/i2c_dw` | `i2c_dw_os.h`: 4 types, 5 macros, 10 inline functions, `i2c_dw_os_uptime_ms()`; read at the end: `i2c_dw_instance.h` (`i2c_dw_os_prepare()`, `i2c_dw_os_irq_enable()`, the devices). `i2c_dw_dma_os.h`: 10 functions, 1 callback type | `port/nuttx/i2c_dw_os_impl.h`, `i2c_dw_instance.h` (`struct i2c_master_s`), `i2c_dw_dma_os_rp2040.c`, `i2c_dw_controllers_rp2040.h` | `port/freertos/i2c_dw_os_impl.h`, `i2c_dw_instance.h` (Zephyr devices), `i2c_dw_dma_os_rp2040.c`, `i2c_dw_controllers_rp2040.h` | upstream driver + patch, no `port/zephyr` |
| `common/drivers/nina` | `nina_bus.h` (7 functions), `nina_os.h` (17 functions, 3 more with `NINA_OS_DNS_OFFLOAD`), `nina_passthrough_bus.h` (17), `nina_passthrough_os.h` (5, a struct, log macros); the binding also implements `nina_passthrough_run()` | `port/nuttx/nina_bus_rp2040.c`, `nina_os.c`, `nina_passthrough_bus_rp2040.c`, `nina_passthrough_os.c`; network: `nina_netdev.c`, `nina_usrsock.c`, `nina_poller.c`, `nina_wifi_mgmt.c` | `port/freertos/nina_bus_rp2040.c`, `nina_os.c`, `nina_passthrough_bus_rp2040.c`, `nina_passthrough_os.c`, `nina_board_rp2040.h`; network: `port/zephyr/nina_net.c`, `nina_socket.c` on the layer | `port/zephyr/` |
| `common/drivers/lsm6dso` | see `common/drivers/lsm6dso/port/` | | | |

`i2c_dw_i2c0_initialize()` returns `struct i2c_master_s *` on N and `int` on F (`i2c_dw_controllers_rp2040.h`).

## 6. Always per OS

| Item | N | F |
|---|---|---|
| Start-up to the scheduler | NuttX boot, `N-board/src/rp2040_boardinitialize.c`, `rp2040_bringup.c`, init task `nuttx/apps/init/rp2040_init.c` | `freertos/app/usb-console/src/main.c`, `vTaskStartScheduler()` |
| Kernel tick and timer | tickless timer `N-board/src/rp2040_oneshot.c`, `rp2040_timerisr_alarm.c` (100 µs tick) | SysTick 1 ms, tick hook and catch-up `tick.c`, `tick_catchup.h`; `FreeRTOSConfig.h` |
| Interrupt attach | `irq_attach()`, `rp2040_gpio_irq_attach()` | Pico SDK `irq_set_exclusive_handler()`, `irq_add_shared_handler()`, `gpio_add_raw_irq_handler_masked()` |
| Idle loop | `N-board/src/rp2040_idle_cpuload.c` | `freertos/app/usb-console/src/idle.c` |
| USB device stack, console transport | `cdcacm`, `N-board/src/rp2040_usbserial.c`; Telnet `nuttx/apps/init/rp2040_telnet.c` | Pico SDK `stdio_usb` (TinyUSB); `freertos/drivers/shell/nano_console.c`, `nano_stdio.c`; Telnet `freertos/drivers/telnet/nano_telnet.c` |
| Heap | NuttX's | FreeRTOS `heap_4`, 32 KiB (`configTOTAL_HEAP_SIZE`) |
| The OS's own shell | nsh (sessions in `rp2040_init.c`) | none: the shared engine `common/compat/shell_engine` (`cli.c`, `nano_shell.c`) on the consoles of `freertos/drivers/shell` |
| Fatal errors | `CONFIG_BOARD_RESET_ON_ASSERT`, report in the RAM log | `freertos/app/usb-console/src/fault.c` with `common/log/fault_record.c` |
| Build system and configuration | Make, Kconfig, `defconfig`, `nuttx/patches` | CMake, `FreeRTOSConfig.h`, `freertos/drivers/*/*.cmake` |
