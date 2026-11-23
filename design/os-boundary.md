# The boundary between the shared code and an OS

English | [日本語](os-boundary-ja.md)

The firmware of this repository is built, with the same functions, on three OSes: Zephyr (`zephyr/`),
NuttX (`nuttx/`) and Pico SDK + FreeRTOS (`freertos/`); each is called a build below (the Zephyr,
NuttX and FreeRTOS builds). Most of the sources are shared by the three and are in `common/`.

This document is an index of what each build supplies so that the shared sources work (what a fourth
OS writes), as the code has it today. Each row names the header that declares an item and the file of
each build that implements it. "N" is the NuttX build, "F" the FreeRTOS build.

Many of the shared sources are written on Zephyr's API (`<zephyr/kernel.h>` and so on). The NuttX and
FreeRTOS builds compile them on a layer that provides that API under Zephyr's names (the
"compatibility layer": its OS-independent part in `common/compat`, its OS parts in each build's
directories). The Zephyr build compiles `common/` against Zephyr itself and does not use the layer;
it appears only where it has a file of its own.

Paths below `nuttx/boards/arduino-nano-rp2040-connect/` are shortened to `N-board/`.

## What a new OS writes

In the order a porter works. The model is N for an OS whose own objects the layer is written on, F
for an OS that offers only basic primitives.

1. **The layer's kernel part** (section 2), one of:
   - directly: `zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h` and the 18 functions of 2.2
     (model: `nuttx/drivers/compat/`);
   - on primitives (with `common/compat/kernel_generic`): the 10 types and constants and 26 functions of
     `zephyr_kernel_os_impl.h` (2.5), `zephyr_os_atomic.h`, a tick hook that calls `zephyr_compat_tick()`
     once per tick (model: `freertos/drivers/compat/zephyr_kernel_os_impl.h`; the smallest form that
     compiles: `common/tests/kernel_generic_host/`).

   Either way also `priorities_os.h` (section 3) and the per-OS `<zephyr/sys/clock.h>` and
   `<zephyr/sys/__assert.h>` (4.1). The acceptance test is the host test of the layer's kernel
   (`common/tests/zephyr_kernel`, the two rows of section 3), run on the OS's part.
2. **Console transport and shell** (section 3, 4.4): an OS without a shell of its own gives the shared
   shell engine (`common/compat/shell_engine`) `shell_serial_os_impl.h` and the stdio redirection
   (models: `freertos/drivers/shell/`, `freertos/drivers/libc/`); an OS with one writes
   `zshell_os_impl.h` and `zephyr_os_shell.h` (models: `nuttx/apps/subcmd/`,
   `nuttx/drivers/compat/zephyr_os_shell.h`).
3. **Log** (section 3): `app_log_os.h`, `log_ram_os_impl.h` (model: `freertos/drivers/include/`).
4. **KV flash** (section 3): on the RP2040 the 4 functions of `rp2040_flash_kv_os.h` and the linker
   script's `__kv_start`, `__kv_end` (model: `freertos/drivers/kv/rp2040_flash_kv_os.c`).
5. **Board parts** (sections 3, 6): GPIO bank 0 (`gpio_os.h`), CPU load (`cpu_load_get()`), hwinfo
   (4.2), tick, interrupts, idle loop, fatal errors.
6. **Start** (4.6): the steps before the application's start (RAM log, layer, I2C, LSM6DSOX, network,
   KV flash bounds, shell) and the calls of the three functions of `app_main.h`; the modules'
   options (4.5).
7. **Per driver** (section 5): the N and F columns are the files a build writes itself. For the
   network, either bind to the OS's network stack (N) or use the layer's network part
   (`common/compat/net_offload`, F).

A first target can be the sensor without Wi-Fi (section 7).

## 1. What a build consists of

| Part | Where | Holds |
|---|---|---|
| Application | `common/imu` | IMU service, MQTT transport, time synchronization, Wi-Fi profile, KV store, calibration; the application's start (`app_main.h`: `app_main.c`, all three builds; `app_init.c`, N and F; section 4.6) |
| Drivers | `common/drivers/<driver>/` | OS-independent core; `port/` boundary headers; `port/<os>/` bindings (section 5) |
| Shell commands | `common/shell` | `zshell.c` (Zephyr shell command handling), `subcmd.c`, `nina_shell.c`, `i2c_dma_shell.c`; F only: `dmesg_shell.c` (N has `nuttx/apps/dmesg`), `kernel_shell.c` (`id`, `uptime`, `date`, `sleep`, `devmem`, `fault`, `reboot`, `bootsel`; on `kernel_shell_os.h`, section 3), `ifconfig_shell.c` (on the network part's interface) |
| Log | `common/compat/log` | RAM log and the functions of the `LOG_*` macros (`log_ram.c`, on its OS part `log_ram_os.h`, section 3), its ring (`log_ring.c`), `LOG_*` macros (`app_log.h`), fault record text and its printing, the HardFault's exception frame (Cortex-M0+) and the choice of the record at the next start (`fault_record.c`) |
| Compatibility layer, shared part | `common/compat` | Zephyr headers (`common/compat/zephyr/`), `k_heap` (`zephyr_heap.h`, `zephyr_heap.c`; read by `<zephyr/kernel.h>` with either kernel part, 4.2), work items, the work queues' threads and the layer's start (`zephyr_init.c`), RTIO, gpio0 device, net_mgmt, Wi-Fi requests, KV flash device, copies of Zephyr's NVS, settings, getopt, Wi-Fi credentials, zperf |
| Compatibility layer, network part for an OS without a network stack | `common/compat/net_offload` | Zephyr's socket layer (reduced copies of `subsys/net/lib/sockets`, `lib/os/zvfs`), the one network interface (`zephyr_net_if.c`) and its start (`zephyr_net_init.c`), their headers in its own include root `include/` (`zephyr/net/net_if.h`, `net_ip.h`, `socket.h`, `socket_offload.h`, `zephyr/sys/fdtable.h`, `zephyr_net_init.h`); used by F only |
| POSIX network headers for a C library without them | `common/compat/libc_net` | `include/` (`arpa/inet.h`, `netdb.h`, `netinet/in.h`, `netinet/tcp.h`, `poll.h`, `sys/socket.h`, `sys/uio.h`), `inet_pton()`/`inet_ntop()` (`inet.c`); used by F only |
| Kernel part of the compatibility layer for an OS that offers only basic primitives | `common/compat/kernel_generic` | `k_sem`, `k_poll()`, the timeout list, the sleeps and the start of a thread (`zephyr_kernel.c`), `k_mutex` (`zephyr_mutex.c`), the OS part of `<zephyr/kernel.h>` and `<zephyr/spinlock.h>` (`zephyr_os.h`, `zephyr_os_spinlock.h`), the tick's entry (`zephyr_compat_tick()`, `zephyr_compat_os.h`), all on the primitives declared in `zephyr_kernel_os.h` (section 2); used by F only |
| Host test of the compatibility layer's kernel (acceptance test of an OS part) | `common/tests/zephyr_kernel` | `zephyr_kernel_test.c` (22 tests), its boundary `zephyr_kernel_test.h` (section 3), the boundary's items on the primitives of the generic kernel part (`kernel_generic/zephyr_kernel_test_os.h`); run by F's host tests (`freertos/tests/compat`), not built into a firmware |
| Shell engine for an OS without a shell of its own | `common/compat/shell_engine` | on the layer's API: line editor (`cli.c`), command table and dispatch of the `Z_SHELL_CMD_ROOT_GLOBAL` roots and `help` (`shell_engine.c`; the application supplies `shell_engine_cmds[]`), Zephyr's word split (`shell_argv.c`), the consoles and each thread's stdout (`shell_console.c`), getopt state per session (`getopt_state.c`), the backend of `zshell.c` (`zshell_os_impl.h`, `zephyr_os_shell.h`), the serial session (`shell_serial.c`, the thread "console", on the console transport `shell_serial_os.h`, section 3), the Telnet shell (`shell_telnet.c`, the threads "shell_telnet" and "telnet_service", on `zsock_*()`); used by F only |
| RP2040 code that no OS owns | `common/chip/rp2040` | `include/rp2040_timer_us.h` (the TIMER's registers and its 64-bit read `rp2040_timer_us()`, always inline; all three builds), `include/rp2040_watchdog_reboot.h` (`rp2040_watchdog_reboot()`, the watchdog's restart on the raw registers; the Zephyr and NuttX builds); the KV flash boundary on the RP2040 for an OS without a flash driver for the area (`rp2040_flash_kv.c` on the build's flash operations `include/rp2040_flash_kv_os.h`, section 3; `include/rp2040_flash_kv.h`; N and F, built by `N-board/src/Make.defs` and `freertos/drivers/kv/kv.cmake`). Each build puts `include/` on its include path (Zephyr: the app's `CMakeLists.txt`; N: `N-board/scripts/Make.defs`, for every source; F: the app's `CMakeLists.txt`) |
| Host tests | `common/tests` | Tests and fakes; `common/tests/zephyr_fakes` is a host implementation of the kernel's OS part |
| Zephyr build | `zephyr/app/usb-console` | `main()`, `log_ram.c`, `kv_store_os.h`, `wifi_profile_os.h`; upstream Zephyr with patches in `zephyr/manifest/zephyr/patches` |
| N: board | `N-board/` | Bring-up, tickless timer, idle loop and CPU load, GPIO bank 0, KV flash, USB serial, BOOTSEL (`include/rp2040_restart.h`) |
| N: compatibility layer, OS part | `nuttx/drivers/compat` | `zephyr_os*.h`, kernel, mutexes, I2C on `struct i2c_master_s`, net_if, hwinfo, prelude, per-OS `<zephyr/...>` headers |
| N: shared OS headers | `nuttx/drivers/include` | `app_log_os.h`, `log_ram_os_impl.h`, `priorities_os.h` |
| N: log | `nuttx/drivers/log` | `log_ram_syslog.c` (the RAM log as the syslog channel), `Make.defs` |
| N: drivers | `nuttx/drivers/<driver>/` | `Make.defs`, `Kconfig` of each driver |
| N: applications | `nuttx/apps/*` | Init task (`init`), one builtin application per command, shell OS part (`subcmd`), Wi-Fi and sensor parts |
| N: NuttX changes | `nuttx/patches` | Local patches to NuttX and nuttx-apps |
| F: application | `freertos/app/usb-console` | `main()`, `FreeRTOSConfig.h`, tick hook, idle and CPU load, `os_clock.h`, `priorities_os.h`, the commands only it can answer (`version`, `free`, `ps`) and the command table, the generic commands' OS part (`kernel_shell_os_impl.h`), fault handling |
| F: compatibility layer, OS part | `freertos/drivers/compat` | the primitives of the generic kernel part (`zephyr_kernel_os_impl.h`), `zephyr_os_atomic.h`, realtime clock, linker script part of the socket registrations, `rp2040/` (port header, GPIO bank 0, hwinfo), prelude |
| F: console transport | `freertos/drivers/shell` | `shell_serial_os_impl.h` (the Pico SDK's `stdio_usb`), the SDK's stdio on the consoles (`shell_stdio.c`) |
| F: C library additions | `freertos/drivers/libc` | stdout/stderr wrappers |
| F: other OS parts | `freertos/drivers/{include,kv,telnet,wifi,sensor_shell,imu,zperf}` | `app_log_os.h`, `log_ram_os_impl.h`; KV flash operations; Wi-Fi OS header; sensor shell OS part; per-source preludes; build files (Telnet: `telnet.cmake` only) |
| F: drivers | `freertos/drivers/<driver>/*.cmake` | The sources and options of each driver |

### 1.1 What the selectable parts need

The kernel part (written directly, as N, or `kernel_generic`), the network part (the OS's network
stack or `net_offload`), the POSIX network headers (the C library's or `libc_net`) and the shell (the
OS's or `shell_engine`) are chosen independently. What each part reads (from its `#include` lines):

| Part | Needs | Does not need |
|---|---|---|
| `common/compat/kernel_generic` | the headers of `common/compat`; the OS's primitives (`zephyr_kernel_os_impl.h`; `z_os_thread_t` is a pointer type, NULL for no thread); the OS's `zephyr_os_atomic.h`, `priorities_os.h`, `<zephyr/sys/clock.h>` | `net_offload`, `libc_net` (`kernel_generic_independent` in `common/tests/run.sh`) |
| `common/compat/net_offload` | a kernel part (either); POSIX socket headers (the C library's or `libc_net`); `net_mgmt.h`, `wifi_mgmt.h` of `common/compat`; the OS's `<zephyr/sys/__assert.h>` | `kernel_generic` (`net_offload_independent`, on a fake kernel of N's form) |
| `common/compat/libc_net` | the C library's `<sys/types.h>` | everything else |
| `common/compat/shell_engine` | the kernel part's 4 optional items (2.1); `common/shell/subcmd.h`; the getopt copies (`common/compat/lib/utils/getopt`); the log (`app_log.h`); the console transport (section 3); only the Telnet shell (`shell_telnet.c`) `zsock_*()` (the OS's network stack or `net_offload`) | `kernel_generic` (either kernel part) |
| `common/tests/zephyr_kernel` | a kernel part; `zephyr_kernel_test_os.h` (with the generic kernel part, this directory's `kernel_generic/` one and the OS's `zephyr_kernel_test_os_impl.h`; an OS with a direct kernel part writes its own); the platform part (section 3) | network, shell |
| `common/compat/log` | the OS's `app_log_os.h`, `log_ram_os_impl.h` | the rest of the layer |
| `common/chip/rp2040` | only `rp2040_flash_kv.c`: `flash_kv.h`, `flash_kv_os.h` of `common/compat`, the log (`app_log.h`); the headers of `include/` read nothing | — |
| `common/shell` | `kernel_shell.c`: the log (`fault_record.h`), `<zephyr/sys/clock.h>`, hwinfo; `dmesg_shell.c`: the log; `ifconfig_shell.c`: `<zephyr/net/net_if.h>` (the network part); `nina_shell.c`: the NINA's `include/`; `zshell.c`: the shell backend (section 3) | — |

## 2. Kernel part of the compatibility layer

Declared in `common/compat/zephyr/kernel.h`, which includes `"zephyr_os.h"` first. An OS provides it in
one of two ways:

- **Directly**, as N does (`nuttx/drivers/compat/`): its own `zephyr_os.h`, `zephyr_os_spinlock.h`,
  `zephyr_os_atomic.h` and the functions of 2.2 on the OS's own objects (semaphores, watchdogs,
  kernel threads).
- **On primitives**, as F does: the generic implementation (`common/compat/kernel_generic/`: `zephyr_os.h`,
  `zephyr_os_spinlock.h`, `zephyr_kernel.c`, `zephyr_mutex.c`, `zephyr_os_internal.h`,
  `zephyr_compat_os.h`) implements 2.1, 2.2 and the
  spinlock with Zephyr's rules on the primitives declared in `common/compat/kernel_generic/zephyr_kernel_os.h` (2.5). The OS
  writes those primitives (`zephyr_kernel_os_impl.h`, static inline), `zephyr_os_atomic.h`, a tick hook
  that calls `zephyr_compat_tick()` once per tick from the tick interrupt, and for the host test of
  the layer's kernel `zephyr_kernel_test_os_impl.h` (F: `freertos/tests/compat/`).

### 2.1 Provided by `zephyr_os.h` (16 items, 3 optional)

"Generic" is the generic implementation's `zephyr_os.h`, which F uses; its items come from the primitives.

| Item | Meaning | N | Generic (F) |
|---|---|---|---|
| OS headers before the layer's | the layer's `<zephyr/sys/atomic.h>` takes over atomic names | NuttX headers, `"rp2040_timer_us.h"` (`common/chip/rp2040`) | `zephyr_kernel_os.h` (the OS's headers: F's FreeRTOS headers and `zephyr_os_port.h`), `priorities.h`, `<zephyr/sys/clock.h>` |
| `Z_USEC_PER_TICK` | tick length in µs; 1000 must be a multiple | `CONFIG_USEC_PER_TICK` (100) | `1000000 / Z_OS_TICK_HZ` (F: `configTICK_RATE_HZ`, 1000) |
| `struct k_mutex`, `Z_MUTEX_INITIALIZER(obj)` | all-zero must work as an unlocked mutex | `rmutex_t` + `ready` flag | the OS's (F: static recursive mutex, created on first use) |
| `struct k_sem`, `Z_SEM_INITIALIZER(obj, initial, limit)` | count with a limit; poll event list | `sem_t` + limit + events | the layer's own count, waiters, events |
| `K_SEM_MAX_LIMIT` | largest limit | `SEM_VALUE_MAX` | `UINT_MAX` |
| `struct _timeout`, `_timeout_func_t` | timer of a delayable work item | `struct wdog_s` | entry in the layer's timeout list |
| `k_tid_t` | thread handle | `pid_t` | `z_os_thread_t` (F: `TaskHandle_t`) |
| `K_THREAD_STACK_RESERVED` (optional, 0 by default) | bytes `K_THREAD_STACK_DEFINE()` adds after each stack for the OS | — (0) | `Z_OS_THREAD_RESERVED` (F: the task's control block, 136) |
| `k_current_get()` | the calling thread | `gettid()` | `z_os_thread_self()` |
| `k_yield()` | the next ready thread of the priority, or go on | `sched_yield()` | `z_os_yield()` |
| `K_PRIO_PREEMPT(x)` | OS priority of Zephyr priority x, order kept | `APP_PRIO_FROM_ZEPHYR` (118 − x) | `APP_PRIO_FROM_ZEPHYR` (F: 20 − x) |
| `k_cycle_get_32()`, `k_cycle_get_64()` | 1 MHz counter | `rp2040_timer_us()` | `z_os_clock_us()` (F: `rp2040_timer_us()`) |
| `Z_OS_ASSERT(test)` | the OS's assertion, for a work queue's thread that did not start | `DEBUGASSERT()` (nothing in the imu configuration) | the OS's (F: `configASSERT()`: fault record, restart) |
| `k_is_in_isr()`, `k_is_pre_kernel()`, `k_thread_custom_data_get()`, `k_thread_custom_data_set()` (optional: for the shell engine) | in an ISR; before the kernel runs threads; the calling thread's one pointer (NULL until set and before the kernel runs threads) | — (not built) | `z_os_in_isr()`, `z_os_pre_kernel()`, `z_os_thread_local_get()`, `_set()` (F: TLS pointer 0) |
| `Z_SEM_TAKE_FOREVER(sem)` (optional) | `k_sem_take(sem, K_FOREVER)` the OS's shorter way: the work queue's wait | `nxsem_wait_uninterruptible()` (what `k_sem_take()` does, without the call) | not defined: `k_sem_take()` (`kernel.h`'s default) |

`<zephyr/kernel.h>` includes `common/compat/zephyr_heap.h` at its end: `struct k_heap`, `K_HEAP_DEFINE()`, `k_heap_alloc()`, `k_heap_free()`; OS-independent (on `k_spin_lock()`), the same with either kernel part (4.2).

### 2.2 Functions the OS implements (18 functions)

Files: N `zephyr_kernel.c`, `zephyr_mutex.c`; generic `common/compat/kernel_generic/zephyr_kernel.c`, `zephyr_mutex.c`.

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
| `z_thread_static_start(thread)` | run `entry(p1, p2, p3)` in a new thread with the name, static stack (`K_THREAD_STACK_DEFINE()`, with its `K_THREAD_STACK_RESERVED` bytes for the OS) and priority of `*thread` (which stays valid while it runs), end it when `entry` returns, set `*tid`; 0 or −errno. Every thread of the layer starts with it |
| `k_sleep(t)` | K_NO_WAIT yields; else wakes at tick + ticks + 1; K_FOREVER for ever; returns 0 |
| `k_msleep(ms)`, `k_usleep(us)` | rounded up to ticks, then as `k_sleep()`; ≤ 0 yields; returns 0 |
| `k_busy_wait(us)` | spin at least `us` µs |
| `z_add_timeout(to, fn, t)` | run `fn` after the timeout, in interrupt context (not while `k_spin_lock()` is held) |
| `z_abort_timeout(to)` | stop it |

Shared, on these: the work item logic (`common/compat/zephyr_work.c`); `common/compat/zephyr_init.c`:
`k_work_queue_start()` (the queue keeps its thread's `struct z_thread_static`; the thread loops on
`z_work_queue_process()`, `k_yield()` and `Z_SEM_TAKE_FOREVER()`), the object `k_sys_work_q`
("sysworkq", `APP_PRIO_SYSWORKQ`), `zephyr_compat_initialize()` (declared in
`common/compat/zephyr_compat.h`: `k_sys_work_q`, −ENOMEM if its thread did not start, the RTIO work
queue, gpio0 with `CONFIG_LSM6DSO_STREAM`), `k_uptime_get()` (`Z_UPTIME_MS()` of
`k_cycle_get_64()`); the "net_mgmt" thread (`common/compat/zephyr_net_mgmt.c`).

| Function | N | Generic (F) |
|---|---|---|
| mutex | `nxrmutex_*()` | the timeout and Zephyr's results around `z_os_mutex_init()`, `_lock()`, `_unlock()` (F: `xSemaphore*Recursive()`, FreeRTOS's priority inheritance) |
| `k_sem_*` | `nxsem_*()` | the layer's count; waiters sleep on their wake flag (`z_os_wait()`; F: task notification index 1) |
| `z_thread_static_start()` | `kthread_create_with_stack()`; the start data handed over through a variable, the caller waits until the new thread has taken it | `z_os_thread_start()` with the data as the thread's argument (F: `xTaskCreateStatic()`, scheduler suspended around it) |
| sleeps | `nxsched_ticksleep()`, `nxsched_msleep()`, `nxsched_usleep()` | `z_os_sleep(ticks + 1)` (F: `vTaskDelay()`) |
| `k_busy_wait()` | `up_udelay()` | `z_os_busy_wait_us()` (F: `busy_wait_us_32()`) |
| `z_add_timeout()` | `wd_start()` | sorted list run by `zephyr_compat_tick()` from the OS's tick hook |

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
| End of a thread of `z_thread_static_start()` | kernel thread returns | `z_os_thread_exit()` (F: `vTaskDelete(NULL)`) |
| Start of a thread | the caller waits until the new thread has taken its start data | the caller goes on; the task runs when the scheduler picks it (at once if its priority is higher) |
| A work queue's thread that did not start (`Z_OS_ASSERT()`) | `DEBUGASSERT()`: nothing in the imu configuration, the queue's thread stays 0 | `configASSERT()`: fault record and restart |
| Tick | 100 µs | 1 ms |
| Builds `zephyr_init.c` (and so gpio0's start) | only with `CONFIG_LSM6DSO_STREAM` (the whole kernel part) | always; gpio0 only with `CONFIG_LSM6DSO_STREAM` |
| Caller of `zephyr_compat_initialize()` | `N-board/src/rp2040_bringup.c` | `main()` before the scheduler, `freertos/app/usb-console/src/main.c` |
| Same-priority choice | NuttX's | FreeRTOS's round robin at every switch |

The F column holds for any OS on the generic implementation, except the tick, the same-priority choice,
the start-up and the assertion, which are the OS's.

### 2.4 Spinlock and atomics

| Header (shared) | Includes | OS provides | N | Generic (F) |
|---|---|---|---|---|
| `common/compat/zephyr/spinlock.h` | `"zephyr_os_spinlock.h"` | `struct k_spinlock`, `k_spinlock_key_t`, `k_spin_lock()`, `k_spin_unlock()`: interrupts masked and no switch until unlock; threads and interrupts; nests | `sched_lock()` + `up_irq_save()` | `z_os_task_lock()` in a thread (F: `taskENTER_CRITICAL()`), `z_os_isr_lock()` in an interrupt (F: `taskENTER_CRITICAL_FROM_ISR()`) |
| `common/compat/zephyr/sys/atomic.h` | `"zephyr_os_atomic.h"` | OS headers that define atomic names, read first | reads `<nuttx/atomic.h>`, undefines its `atomic_*` macros | the OS's (F: empty, `freertos/drivers/compat/zephyr_os_atomic.h`) |

The atomic operations themselves are shared (on `k_spin_lock()`).

### 2.5 Primitives of the generic implementation (`common/compat/kernel_generic/zephyr_kernel_os.h`)

`zephyr_kernel_os.h` includes the OS's `"zephyr_kernel_os_impl.h"` and declares what it must define, so a
missing or different one does not compile. All are static inline: they are on the paths of
`k_sem_give()`, `k_sem_take()`, `k_poll()`, `k_spin_lock()` and the tick (no call through a pointer).
F's are `freertos/drivers/compat/zephyr_kernel_os_impl.h`, with the port-dependent ones in
`zephyr_os_port.h` (board `freertos/drivers/compat/rp2040/`, host `freertos/tests/compat/`).

Types and constants (10): `z_os_thread_t` (`TaskHandle_t`), `z_os_prio_t` (larger more urgent;
`UBaseType_t`), `z_os_ticks_t` (unsigned; `TickType_t`), `Z_OS_TICKS_FOREVER` (its largest value, no
timeout; `portMAX_DELAY`), `Z_OS_TICK_HZ` (`configTICK_RATE_HZ`), `z_os_woken_t` (0 at first;
`BaseType_t`), `Z_OS_THREAD_NAME_SIZE` (`configMAX_TASK_NAME_LEN`), `Z_OS_THREAD_RESERVED` (the bytes the OS
keeps after each thread's stack for its control block, a multiple of 8, 0 if none: `K_THREAD_STACK_DEFINE()`
adds them, as Zephyr's `K_THREAD_STACK_RESERVED`, and `K_THREAD_STACK_SIZEOF()` leaves them out; F:
`sizeof(StaticTask_t)` rounded up to 8, 136), `struct k_mutex` with `Z_MUTEX_INITIALIZER()`, `Z_OS_ASSERT()`.

Functions (26). "Locked" is inside `k_spin_lock()`; "ISR" includes the tick hook.

| Primitive | Meaning | F |
|---|---|---|
| `z_os_in_isr()` | true in an ISR; any context | IPSR ≠ 0 (host: the tick hook's depth) |
| `z_os_pre_kernel()` | true before the OS runs threads; any context | `xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED` |
| `z_os_task_lock()`, `z_os_task_unlock()` | a thread's lock: no ISR and no other thread until unlock; nests; the wakes made inside keep it, their switch happens at the outermost unlock | `taskENTER_CRITICAL()`, `taskEXIT_CRITICAL()` (host: also suspends the scheduler) |
| `z_os_isr_lock()`, `z_os_isr_unlock(state)` | an ISR's lock: saves and restores the mask; the state is never `UINT32_MAX` | `taskENTER_CRITICAL_FROM_ISR()`, `taskEXIT_CRITICAL_FROM_ISR()` |
| `z_os_thread_self()` | the calling thread; threads only | `xTaskGetCurrentTaskHandle()` |
| `z_os_thread_prio_self()` | its current priority (with inheritance); threads only | `uxTaskPriorityGet(NULL)` |
| `z_os_thread_start(fn, name, stack, size, prio, arg, tid)` | new thread on a static stack: the storage of `K_THREAD_STACK_DEFINE()` and its `K_THREAD_STACK_SIZEOF()` (a multiple of 8), the `Z_OS_THREAD_RESERVED` bytes after it the OS's for the control block, so nothing is allocated; `*tid` set before it can run; from a thread or before the scheduler; 0, −EINVAL or another negative errno of the OS | `xTaskCreateStatic()` with the control block after the stack, `vTaskSuspendAll()` around it once the scheduler runs; 0 or −EINVAL |
| `z_os_thread_exit()` | ends the calling thread | `vTaskDelete(NULL)` |
| `z_os_thread_name(tid)` | the thread's writable name array (`Z_OS_THREAD_NAME_SIZE` bytes), as the OS shows it | `pcTaskGetName()` |
| `z_os_yield()` | next ready thread of the priority, or go on; threads, unlocked | `taskYIELD()` |
| `z_os_thread_local_get()`, `z_os_thread_local_set(p)` | the calling thread's one pointer (Zephyr's custom data); NULL until set; get is NULL before the OS runs threads | `pvTaskGetThreadLocalStoragePointer()`, `vTaskSetThreadLocalStoragePointer()` (index 0) |
| `z_os_wait_prepare()` | clears the caller's wake flag; threads, locked | `xTaskNotifyStateClearIndexed()`, `ulTaskNotifyValueClearIndexed()` (index 1) |
| `z_os_wait(ticks)` | waits for the flag or until the current tick + ticks (`Z_OS_TICKS_FOREVER`: no end), clears it; a finite wait returns only for one of the two; threads, unlocked | `ulTaskNotifyTakeIndexed(1, pdTRUE, ticks)` |
| `z_os_wake(tid)` | sets tid's flag, whether it waits yet or not (latched); threads, locked; the switch waits for the unlock | `xTaskNotifyGiveIndexed()` |
| `z_os_wake_from_isr(tid, &woken)` | the same from an ISR, locked; sets `woken` when a switch is due | `vTaskNotifyGiveIndexedFromISR()` |
| `z_os_yield_from_isr(woken)` | switch at the end of the ISR if `woken`; unlocked | `portYIELD_FROM_ISR()` (host: nothing) |
| `z_os_sleep(ticks)` | until the current tick + ticks; threads, unlocked | `vTaskDelay()` |
| `z_os_suspend()` | stops the caller with no end (may return; called again) | `vTaskSuspend(NULL)` |
| `z_os_busy_wait_us(us)` | spins at least `us` µs; any context | `busy_wait_us_32()` (host: the clock) |
| `z_os_clock_us()` | µs since the start, 64 bits, monotonic; any context | `rp2040_timer_us()` (TIMER; host: `CLOCK_MONOTONIC`) |
| `z_os_mutex_init(m)`, `z_os_mutex_lock(m, ticks)`, `z_os_mutex_unlock(m)` | the OS's recursive mutex with its own priority inheritance; all-zero is unlocked; lock true when taken; unlock 0, −EINVAL (not held), −EPERM (other holder) | `xSemaphoreCreateRecursiveMutexStatic()` (also at the first use), `xSemaphoreTakeRecursive()`, `xSemaphoreGetMutexHolder()` + `xSemaphoreGiveRecursive()` |

F's port header also has the tick hook's `z_os_tick_now_us32()` and `z_os_tick_pend()` (the catch-up of
lost ticks, `freertos/app/usb-console/src/tick.c`), which are not primitives of the generic part.

## 3. Other declared boundaries

| Boundary | Declared in | Items | Called by | N | F |
|---|---|---|---|---|---|
| Generic shell commands | `common/shell/kernel_shell_os.h` (reads `"kernel_shell_os_impl.h"`; static inline) | `kernel_shell_os_tick_count()` (the OS's tick count, for `uptime`), `kernel_shell_os_reboot(delay_ms)`, `kernel_shell_os_bootsel(delay_ms)` (restart, into the USB loader for the second, `delay_ms` later; do not return), `kernel_shell_os_fault_last()` (the last run's `struct fault_record` or NULL); the rest is the layer's (`k_cycle_get_64()`, `k_sleep()`, `Z_USEC_PER_TICK`, `sys_clock_gettime()`, `hwinfo_get_device_id()`) | `common/shell/kernel_shell.c` | — (not built; nsh's commands, `nuttx/apps/bootsel`) | `freertos/app/usb-console/src/kernel_shell_os_impl.h`: `os_tick_count()`, the Pico SDK's `watchdog_reboot()` and `reset_usb_boot()`, `fault_last()` (`fault.c`); host `freertos/tests/shell/kernel_shell_os_impl.h` |
| Shell backend | `common/shell/zshell_os.h` | `zshell_os_backend_get()`, `zshell_os_cmd_enter()`, `zshell_os_cmd_exit()`, `z_shell_backend_vfprintf()`; optional `zshell_os_active_cmd_set()`/`_get()` with `ZSHELL_OS_ACTIVE_CMD` | `common/shell/zshell.c` | `nuttx/apps/subcmd/zshell_os_impl.h` (no `ZSHELL_OS_ACTIVE_CMD`) | the shell engine's `common/compat/shell_engine/zshell_os_impl.h` (defines it; on the consoles) |
| Console transport | `common/compat/shell_engine/shell_serial_os.h` (reads `"shell_serial_os_impl.h"`; static inline) | `shell_serial_os_stdio_init()` (every thread's stdout and stderr to `shell_console_write(shell_console_current(), ...)`, unbuffered), `shell_serial_os_rx_init()` (call `shell_serial_rx_ready()`, any context, when input arrives), `shell_serial_os_connected()`, `shell_serial_os_read()` (no wait), `shell_serial_os_write()` (drops what the terminal cannot take) | `common/compat/shell_engine/shell_serial.c` | — (not built) | `freertos/drivers/shell/shell_serial_os_impl.h`: `stdio_usb`, `shell_stdio_init()` (`shell_stdio.c`) with `freertos/drivers/libc/stdio_wrap.c`; host `freertos/tests/shell/shell_serial_os_impl.h` |
| `struct shell` | read by `common/compat/zephyr/shell/shell.h` | one backend, outlives the command | shell commands | `nuttx/drivers/compat/zephyr_os_shell.h` (terminal path, command pid) | the shell engine's `common/compat/shell_engine/zephyr_os_shell.h` (console, active command) |
| Log, OS part | `common/compat/log/app_log.h` | `app_log_os.h` (OS headers first, optional `CONFIG_LOG_DEFAULT_LEVEL`, `CONFIG_LOG_MAX_LEVEL`, and `LOG_RAM_SIZE`, the RAM log's ring in bytes, read by `log_ring.h` and `log_ram.h`: N 8192, F 4096; without it they do not compile); `app_log_printf()`, `app_log_vprintf()`, `app_log_ratelimit_pass()` | `LOG_*()` macros | `nuttx/drivers/include/app_log_os.h`; the functions shared: `common/compat/log/log_ram.c` | `freertos/drivers/include/app_log_os.h`; shared: same |
| RAM log | `common/compat/log/log_ram.h` | `log_ram_initialize()`, `log_ram_store()` (the OS's own log, raw bytes), `log_ram_piece()` | start-up; the OS's log; `dmesg` | shared `common/compat/log/log_ram.c`; called by `N-board/src/rp2040_boardinitialize.c`, `nuttx/drivers/log/log_ram_syslog.c`, `nuttx/apps/dmesg/dmesg_main.c` | shared: same; called by `main()`, `common/shell/dmesg_shell.c` (`log_ram_store()` unused) |
| RAM log, OS part | `common/compat/log/log_ram_os.h` (`log_ram.c` reads `"log_ram_os_impl.h"` first) | `LOG_RAM_OS_RETAINED(name)` (the ring in RAM the start-up code leaves alone), type `log_ram_os_irqstate_t`; inline `log_ram_os_irq_save()`, `_irq_restore()` (the lock: interrupts masked, any context), `_uptime_us()` (time stamp and rate limit), `_attach()` (make it the OS's own log) | `common/compat/log/log_ram.c` | `nuttx/drivers/include/log_ram_os_impl.h`: `.noinit`, `up_irq_save()`, `rp2040_timer_us()`, registers the syslog channel (`nuttx/drivers/log/log_ram_syslog.c`) | `freertos/drivers/include/log_ram_os_impl.h`: `__uninitialized_ram`, `save_and_disable_interrupts()`, `rp2040_timer_us()` of `os_clock.h`, nothing to attach; host `common/tests/log_ram_os_host/` |
| KV flash | `common/compat/flash_kv_os.h` (the area's constants and the checks in `flash_kv.h`) | `flash_kv_os_initialize()` (once, by the start-up), `flash_kv_os_ready()`, `_read()`, `_program()` (256-byte page), `_erase()` (4 KiB sector); range checked again by the OS | `common/compat/zephyr_flash_kv.c`; the start-up | shared on the RP2040: `common/chip/rp2040/rp2040_flash_kv.c` (next row); called by `rp2040_bringup.c` | same; called by `main()` |
| KV flash operations on the RP2040 | `common/chip/rp2040/include/rp2040_flash_kv_os.h`; the linker script's `__kv_start`, `__kv_end` | `rp2040_flash_kv_os_init()` (once, after the linker script's check), `_image_end()` (the XIP address where the image ends), `_program(offset, page, lo, hi)` (256-byte page in RAM), `_erase(offset, lo, hi)` (4 KiB sector): XIP off and interrupts disabled, the request checked against `[lo, hi)` once more right before the flash is touched | `common/chip/rp2040/rp2040_flash_kv.c` | `N-board/src/rp2040_flash_kv_os.c` (the boot ROM, a RAM function; NuttX's `rp2040_flash_mtd.c` is not used) | `freertos/drivers/kv/rp2040_flash_kv_os.c` (the Pico SDK's `flash_range_*()` in `flash_safe_execute()`) |
| KV wipe command | `common/compat/kv_store_os.h` | `kv_store_wipe()`, `kv_store_cmd_wipe()`, `KV_STORE_OS_COMMANDS` | `common/imu/kv_store.c` | shared: `common/compat/kv_store_wipe.c` | shared: same |
| GPIO bank 0 | `common/compat/gpio_os.h` | OS: `gpio_os_configure_input()`, `gpio_os_get_all()`, `gpio_os_irq_configure()`; layer: `gpio_os_irq_handler()` (called from the bank interrupt) | `common/compat/zephyr_gpio.c`; N also `common/drivers/nina/port/nuttx/nina_os.c` | `N-board/src/rp2040_gpio_os.c` | `freertos/drivers/compat/rp2040/gpio_os_rp2040.c` |
| CPU load | `common/compat/cpu_load.h` (with the arithmetic) | `cpu_load_get(reset)` in tenths of a percent, from idle windows of the TIMER | `common/imu/imu_stream.c` | `N-board/src/rp2040_idle_cpuload.c` | `freertos/app/usb-console/src/idle.c` |
| Priorities and stacks | `common/compat/priorities.h` | OS: `APP_PRIO_FROM_ZEPHYR(p)` and its own threads; shared: 15 `APP_PRIO_*`/`APP_STACK_*` pairs | the layer, bindings, start-up | `nuttx/drivers/include/priorities_os.h` (118 − p; `NINA_EVENT`, `NINA_PT`) | `freertos/app/usb-console/src/priorities_os.h` (20 − p; `TIMER_SVC`, `IDLE`; the layer's `SYSWORKQ`, `NET_MGMT`, `SENSOR_SHELL` and `TELNET_SERVICE` stacks at measured sizes) |
| net_mgmt thread | `common/compat/zephyr/net/net_mgmt.h` | `net_mgmt_event_init()` starts "net_mgmt" with `z_thread_static_start()`; no OS file | N `common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`; F `common/compat/net_offload/zephyr_net_init.c` | shared `common/compat/zephyr_net_mgmt.c` | same |
| RTIO work queue | `common/compat/zephyr/rtio/work.h` | `rtio_workq_initialize()` starts "RTIO WQ 0" with `z_thread_static_start()`; no OS file | `zephyr_compat_initialize()` | shared `common/compat/zephyr_rtio_workq.c` | same |
| Wi-Fi profile | included by `common/imu/wifi_profile.c` | `extern k_tid_t wifi_reconnect_tid;` | `common/imu/wifi_profile.c` | `nuttx/apps/wifi/wifi_profile_os.h` | `freertos/drivers/wifi/wifi_profile_os.h` |
| Sensor shell | included first by `common/drivers/sensor_shell/sensor_shell.c`, `common/compat/sensor_shell_stream.c` | `CONFIG_SENSOR_SHELL_STREAM`, `_THREAD_STACK_SIZE`, `_TRIG_PRINT_TIMEOUT_MS`, `_MAX_TRIGGER_DEVICES`; `sensor_shell_os_get_arm()`, `sensor_shell_os_get_wait()`, `sensor_shell_os_processing_callback` | those two files | `nuttx/apps/sensor/sensor_shell_os.h`, `sensor_shell_nuttx.h`, `sensor_shell_nuttx.c` | `freertos/drivers/sensor_shell/sensor_shell_os.h`, `sensor_shell_os.c` |
| Start of the layer | `common/compat/zephyr_compat.h` | `zephyr_compat_initialize()`, `K_THREAD_DEFINE_START(name)` | start-up; `K_THREAD_DEFINE_START()`: `common/imu/app_init.c` | shared `common/compat/zephyr_init.c` (section 2) | same |
| Host test of the layer's kernel, OS part | `common/tests/zephyr_kernel/zephyr_kernel_test.h` (`zephyr_kernel_test.c` reads `"zephyr_kernel_test_os.h"` after `<zephyr/kernel.h>`) | type `zephyr_kernel_test_os_prio_t`, `ZEPHYR_KERNEL_TEST_OS_NAME_MAX`, `zephyr_kernel_test_os_tick()`, `_tick_sync()`, `_thread_create()`, `_thread_exit()`, `_prio_get()`, `_prio_self()`, `_name_get()`, `_current()`, `_in_isr()`, `_sched_lock()`, `_sched_unlock()` | `common/tests/zephyr_kernel/zephyr_kernel_test.c` | — (not run) | on the primitives `common/tests/zephyr_kernel/kernel_generic/zephyr_kernel_test_os.h` (`_tick_sync()`, `_thread_exit()`, `_prio_self()`, `_name_get()`, `_in_isr()`, the type and the name length); the rest `freertos/tests/compat/zephyr_kernel_test_os_impl.h` (FreeRTOS tasks) |
| Host test of the layer's kernel, platform part | same | `zephyr_kernel_test_port_alarm()`, `_quiet()`, `_realtime()`, `_mask_irqs()` (a platform without an interrupt to raise, a tick that follows real time or interrupts to mask answers -ENOTSUP or false, and those checks are skipped) | same | — | `freertos/tests/compat/main.c` (the FreeRTOS POSIX port: all four skip) |
| Tick of the generic kernel part | `common/compat/kernel_generic/zephyr_compat_os.h` | `zephyr_compat_tick()`: once per tick, from the tick interrupt | F tick hook | — | `common/compat/kernel_generic/zephyr_kernel.c` |
| Start of the network part for an OS without a network stack | `common/compat/net_offload/include/zephyr_net_init.h` | `zephyr_net_initialize()` | F `main_main()` | — (`nina_net_initialize()`) | shared `common/compat/net_offload/zephyr_net_init.c` |
| Start of the application | `common/imu/app_main.h` | `app_init_post_kernel()`, `app_threads_start()`, `app_main()`, called in this order (section 4.6); the modules' options (`CONFIG_KV_STORE`, `CONFIG_APP_IMU`, `CONFIG_IMU_RAW_MQTT`, `CONFIG_TIME_SYNC`, `CONFIG_WIFI_CREDENTIALS`, `CONFIG_WIFI_PROFILE`, `CONFIG_APP_SENSOR`, `CONFIG_WIFI_NINA_LOCAL`, `CONFIG_NET_ZPERF`), each defined by a build that builds the module; `CONFIG_APP_IMU` always uses the API of the MQTT transport and of the time synchronization, so without `CONFIG_IMU_RAW_MQTT` or `CONFIG_TIME_SYNC` a build compiles `common/imu/imu_raw_mqtt_null.c` or `time_sync_null.c` in their place (section 7) | the start-up file of each build (Zephyr: `app_main()` only) | shared `common/imu/app_main.c`, `app_init.c`; the options from Kconfig; called by `nuttx/apps/init/rp2040_init.c` | shared: same; the options from the modules' `.cmake` files; called by `main_main()` |

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
| `rp2040_timer_us()`: µs since boot, TIMER, any context | `common/chip/rp2040/include/rp2040_timer_us.h` (shared; also the Zephyr build's `common/imu/time_sync.c`) | read by `nuttx/drivers/compat/zephyr_os.h`, `nuttx/drivers/include/log_ram_os_impl.h` and the board | read by `freertos/app/usb-console/src/os_clock.h` |
| `rp2040_watchdog_reboot()`: restart through the watchdog, scratch 0..3 kept | `common/chip/rp2040/include/rp2040_watchdog_reboot.h` (shared; also the Zephyr build's `common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c`) | `N-board/src/rp2040_restart.c` (fallback of `rp2040_reset_usb_boot()`), `common/drivers/nina/port/nuttx/nina_passthrough_bus_rp2040.c` | not used: the Pico SDK's `watchdog_reboot()` |
| `k_heap` | `common/compat/zephyr_heap.h`, included by `<zephyr/kernel.h>` with either kernel part | not built (no user) | `common/compat/zephyr_heap.c`; used by `common/drivers/nina/port/zephyr/nina_socket.c` |
| `zephyr_compat_tick()` | `common/compat/kernel_generic/zephyr_compat_os.h` (generic kernel part) | — (wdog) | called by `vApplicationTickHook()`, `freertos/app/usb-console/src/tick.c` |
| `zephyr_net_initialize()` | `common/compat/net_offload/include/zephyr_net_init.h` | — (`nina_net_initialize()` in `rp2040_bringup.c`, `common/drivers/nina/port/nuttx/nina_net.h`) | `common/compat/net_offload/zephyr_net_init.c` |
| `hwinfo_get_device_id()` | `common/compat/zephyr/drivers/hwinfo.h` | `nuttx/drivers/compat/zephyr_hwinfo.c` | `freertos/drivers/compat/rp2040/zephyr_hwinfo.c` |

### 4.3 Devices behind `DEVICE_DT_GET()`

`DEVICE_DT_GET(DT_NODELABEL(x))` is `&__device_x` (`common/compat/zephyr/device.h`); `DEVICE_DT_DEFINE()` makes it.
The devices listed for the shell are in `common/compat/zephyr_device.c` (hand-written).

| Node | N | F |
|---|---|---|
| `gpio0` | `common/compat/zephyr_gpio.c` | same |
| `i2c0` | `N-board/src/rp2040_lsm6dso.c`, bound to the NuttX I2C master (`zephyr_i2c.c`) | `common/drivers/i2c_dw/port/freertos/i2c_dw_instance.h` |
| `i2c1` | no Zephyr device (`struct i2c_master_s` only) | `common/drivers/i2c_dw/port/freertos/i2c_dw_instance.h` |
| `lsm6dsox` | `common/drivers/lsm6dso/port/nuttx/lsm6dso_instance.h` (read last by `port/zephyr/lsm6dso_sensor.c`) | `common/drivers/lsm6dso/port/freertos/lsm6dso_instance.h` (same) |
| flash device | `common/compat/zephyr_flash_kv.c` (`z_flash_kv_device`) | same |

### 4.4 Shell commands: registration, getopt, stdio

| Point | N | F |
|---|---|---|
| Registration | each command a builtin application; `SHELL_CMD_REGISTER()` makes `main()` (or `z_shell_main_<name>()` with `Z_SHELL_CMD_NAMED_MAIN`) | `Z_SHELL_CMD_ROOT_GLOBAL`: global root entries listed in `shell_engine_cmds[]`, `freertos/app/usb-console/src/commands.c`; run by `common/compat/shell_engine/shell_engine.c` |
| `SHELL_SUBCMD_ADD()` | global entry; the parent's source lists them by hand | same |
| getopt | Zephyr's getopt not built; `nuttx/apps/wifi/wifi_credentials_shell.c` maps `sys_getopt_*` to `getopt_long()` | copies in `common/compat/lib/utils/getopt`; state per session `common/compat/shell_engine/getopt_state.c` (`sys_getopt_state_get()`, `z_getopt_global_state_update()`), reset by `sys_getopt_init()` before each command |
| stdio of `zshell.c`: `printf()`, `fprintf(stderr)`, `fflush(stdout)` must reach the command's terminal; `calloc()` of about 1.2 KB per line | each command a task with its own stdio | the console transport's stdio redirection (section 3): `freertos/drivers/shell/shell_stdio.c` (the calling task's console), `freertos/drivers/libc/stdio_wrap.c` (`--wrap` of the stdout/stderr calls) |

### 4.5 `CONFIG_*` values

| Source | N | F |
|---|---|---|
| Prelude, `-include` on shared sources | `nuttx/drivers/compat/common_prelude.h`: NuttX headers first; `CONFIG_SENSOR_LOG_LEVEL` 3; `CONFIG_SENSOR_ASYNC_API` with `CONFIG_LSM6DSO_STREAM` | `freertos/drivers/compat/common_prelude.h`: `CONFIG_SENSOR_LOG_LEVEL` 3; `CONFIG_NINA_NTP_SERVER` "", `_PORT` 123, `_INTERVAL_MS` 2000; `CONFIG_NINA_TCP_MAX_IO` 3584; `CONFIG_NINA_SEND_RETRY_MS` 1; `CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY` 40; `CONFIG_NINA_PASSTHROUGH` 1; `CONFIG_SENSOR_ASYNC_API` with the stream |
| NINA and NTP options | Kconfig: `nuttx/drivers/nina/Kconfig`, `nuttx/apps/imu/Kconfig` | the prelude (above) |
| NVS and settings copies | `common/compat/zephyr_kv_config.h`, `-include` from `nuttx/apps/imu/Makefile` | same, from `freertos/drivers/kv/kv.cmake` |
| Build definitions | Kconfig (`nuttx/drivers/*/Kconfig`, `nuttx/apps/*/Kconfig`, `N-board/Kconfig`), `N-board/configs/imu/defconfig` | `compat.cmake`: `Z_SHELL_CMD_ROOT_GLOBAL`, `CONFIG_GETOPT_LONG=1`, `PICO_STDOUT_MUTEX=0`; `i2c_dw.cmake`: `CONFIG_I2C_DW_I2C0`, `_I2C1`; `lsm6dso.cmake`: `CONFIG_LSM6DSO_STREAM`; `wifi.cmake`: `CONFIG_WIFI_CREDENTIALS`, `CONFIG_WIFI_PROFILE`; `nina.cmake`: `NINA_OS_DNS_OFFLOAD` for the resolver, `CONFIG_WIFI_NINA_LOCAL`; the modules' options of the application's start (section 4.6): `kv.cmake` `CONFIG_KV_STORE`, `imu.cmake` `CONFIG_APP_IMU`, `CONFIG_IMU_RAW_MQTT`, `CONFIG_TIME_SYNC`, `sensor_shell.cmake` `CONFIG_APP_SENSOR`, `zperf.cmake` `CONFIG_NET_ZPERF` |
| In shared headers | `kernel.h`: `CONFIG_THREAD_NAME` 1; `zephyr/rtio/work.h`: `CONFIG_RTIO_WORKQ_POOL_ITEMS` 4 | same |
| Sensor shell | `sensor_shell_nuttx.h` | `sensor_shell_os.h` (same four values) |

### 4.6 Threads and start-up order

The application's start is one shared sequence, `common/imu/app_main.h`, in the order of the Zephyr
build's kernel and `main()`:

1. `app_init_post_kernel()` (`app_init.c`): the application's part of `SYS_INIT(POST_KERNEL)`,
   `wifi_credentials_init()` (`CONFIG_WIFI_CREDENTIALS`; mounts NVS, reads the stored SSIDs);
2. the OS's `SYS_INIT(APPLICATION)` parts of the shell (F: `wifi_shell_init()`, `shell_telnet_init()`);
3. `app_threads_start()` (`app_init.c`): `z_init_static_threads()`, the one table of the threads below,
   each started with `K_THREAD_DEFINE_START()` after the one before it, a failure logged
   (`Could not start <name>: <errno>`);
4. `app_main()` (`app_main.c`): `main()`: `USB console started`, `kv_store_init()`,
   `imu_cal_init()`, `time_sync_init()`, `wifi_profile_init()`.

The Zephyr build calls only `app_main()` from its `main()`; its kernel does 1 to 3 (`SYS_INIT()`,
`K_THREAD_DEFINE()`). N and F call 1, 3 and 4 from a thread, after their devices; the Zephyr build does
not compile `app_init.c`. Each entry is under the option of its module; `common/tests/app_threads.py`
(run by `common/tests/run.sh`) checks that the table names every `K_THREAD_DEFINE()` of the sources
once and that each build sets the option exactly when it compiles the defining source.

| Thread (defined in), in the table's order | Option | N | F |
|---|---|---|---|
| `nina_scan_tid`, `nina_connect_tid`, `nina_monitor_tid` (`common/drivers/nina/port/zephyr/nina_net.c`) | `CONFIG_WIFI_NINA_LOCAL` | not built; in their place `nina_net_start()` (`CONFIG_NINA_NET`) starts the core's threads | started |
| `imu_stream_tid` (`common/imu/imu_stream.c`) | `CONFIG_APP_IMU` | started | same |
| `imu_raw_mqtt_tid` (`common/imu/imu_raw_mqtt.c`) | `CONFIG_IMU_RAW_MQTT` | same | same |
| `time_sync_tid` (`common/imu/time_sync.c`) | `CONFIG_TIME_SYNC` | same | same |
| `wifi_reconnect_tid` (`common/imu/wifi_profile.c`) | `CONFIG_WIFI_PROFILE` | same | same |
| `sensor_shell_tid` (`common/compat/sensor_shell_stream.c`) | `CONFIG_APP_SENSOR` | same | same |
| `zperf_server_tid` (`common/compat/zperf_iperf3.c`) | `CONFIG_NET_ZPERF` | not built | same |

Start-up files: Zephyr `zephyr/app/usb-console/src/main.c` (the passthrough branch, `app_main()`); N
`N-board/src/rp2040_boardinitialize.c`, `N-board/src/rp2040_bringup.c`, `nuttx/apps/init/rp2040_init.c`;
F `freertos/app/usb-console/src/main.c` (`main()`, then the task `main_main()`). What an OS still writes
is the steps before the application's start (the table below) and the calls of 1 to 4 at its point.

| Step | N | F |
|---|---|---|
| RAM log | `log_ram_initialize()` in `rp2040_boardinitialize()` | first in `main()` |
| NINA passthrough check | `nina_passthrough_active()` in the init task | `nina_passthrough_check()` in `main()` |
| Layer | `zephyr_compat_initialize()` in `rp2040_bringup()` | `zephyr_compat_initialize()` in `main()` |
| I2C0, I2C1 | `i2c_dw_i2c0_initialize()`, `i2c_register()` in `rp2040_bringup()` | `i2c_dw_i2c0_initialize()`, `_i2c1_` in `main()` |
| LSM6DSOX | after I2C0 in `rp2040_bringup()` | first in `main_main()` |
| Network | `nina_net_initialize()` in `rp2040_bringup()` | `zephyr_net_initialize()` in `main_main()` |
| KV flash bounds | `flash_kv_os_initialize()` in `rp2040_bringup()` | `flash_kv_os_initialize()` in `main()` |
| Shell | `nsh_initialize()` in the init task, before 1 | the console's task (`shell_serial_start()`) in `main()` |
| Application's start | 1, 3, Telnet (`rp2040_telnet_start()`, after the threads), 4 in the init task | 1, 2 (`wifi_shell_init()`, `shell_telnet_init()`), 3, 4 in `main_main()` |

### 4.7 POSIX socket headers

| N | F |
|---|---|
| NuttX's libc (`<sys/socket.h>`, `<netinet/in.h>`, `<arpa/inet.h>`, `<netdb.h>`, `<poll.h>`) | `common/compat/libc_net/include`: `arpa/inet.h`, `netdb.h`, `netinet/in.h`, `netinet/tcp.h`, `poll.h`, `sys/socket.h`, `sys/uio.h`; `common/compat/libc_net/inet.c` |

## 5. Drivers

| Driver | Boundary headers (`port/`) | N files | F files | Zephyr |
|---|---|---|---|---|
| `common/drivers/i2c_dw` | `i2c_dw_os.h`: 4 types, 5 macros, 10 inline functions, `i2c_dw_os_uptime_ms()`; read at the end: `i2c_dw_instance.h` (`i2c_dw_os_prepare()`, `i2c_dw_os_irq_enable()`, the devices). `i2c_dw_dma_os.h`: 10 functions, 1 callback type | `port/nuttx/i2c_dw_os_impl.h`, `i2c_dw_instance.h` (`struct i2c_master_s`), `i2c_dw_dma_os_rp2040.c`, `i2c_dw_controllers_rp2040.h` | `port/freertos/i2c_dw_os_impl.h`, `i2c_dw_instance.h` (Zephyr devices), `i2c_dw_dma_os_rp2040.c`, `i2c_dw_controllers_rp2040.h` | upstream driver + patch, no `port/zephyr` |
| `common/drivers/nina` | `nina_bus.h` (7 functions), `nina_os.h` (17 functions, 3 more with `NINA_OS_DNS_OFFLOAD`), `nina_passthrough_bus.h` (17), `nina_passthrough_os.h` (5, a struct, log macros); the binding also implements `nina_passthrough_run()` | `port/nuttx/nina_bus_rp2040.c`, `nina_os.c`, `nina_passthrough_bus_rp2040.c`, `nina_passthrough_os.c`; network: `nina_netdev.c`, `nina_usrsock.c`, `nina_poller.c`, `nina_wifi_mgmt.c` | `port/freertos/nina_bus_rp2040.c` (its long transfers by DMA in `nina_bus_dma_rp2040.c`), `nina_os.c`, `nina_passthrough_bus_rp2040.c`, `nina_passthrough_os.c`, `nina_board_rp2040.h`; network: `port/zephyr/nina_net.c`, `nina_socket.c` on the layer | `port/zephyr/` |
| `common/drivers/lsm6dso` | `lsm6dso_os.h`: 5 types, log macros, 20 functions (the INT1 line's enable and disable ordinary functions, the rest static inline); `lsm6dso_stream_req.h` (the FIFO stream's user): 2 types, 6 inline functions. Both implemented on Zephyr's API in `port/zephyr/` (`lsm6dso_os_impl.h`, `lsm6dso_os.c`; RTIO's `lsm6dso_stream_req_impl.h`), the same for all three builds | `port/nuttx/lsm6dso_instance.h` (one device, `board.h`'s values); the rest is `port/zephyr/` on the layer (`lsm6dso_sensor.c`, `lsm6dso_rtio.c`, `lsm6dso_rtio_stream.c`, `lsm6dso_decoder.c`, `lsm6dso_os.c`); the sensor's start `N-board/src/rp2040_lsm6dso.c`; build `nuttx/drivers/lsm6dso/Make.defs`, `Kconfig` | `port/freertos/lsm6dso_instance.h`; the same `port/zephyr/`; the sensor's start `freertos/drivers/lsm6dso/lsm6dso_board.c`; build `freertos/drivers/lsm6dso/lsm6dso.cmake` | a Zephyr module `port/zephyr` (`zephyr/module.yml`, the app's `ZEPHYR_EXTRA_MODULES`), `CONFIG_LSM6DSO_LOCAL` (`port/zephyr/Kconfig`, in place of Zephyr's `CONFIG_LSM6DSO`), `port/zephyr/lsm6dso_instance.h` (the devicetree instances) |

`i2c_dw_i2c0_initialize()` returns `struct i2c_master_s *` on N and `int` on F (`i2c_dw_controllers_rp2040.h`).

The LSM6DSO's core (the `.c` files at the top of `common/drivers/lsm6dso`, `include/`, `internal/`, the
boundary headers of `port/`) reads no OS definition (`lsm6dso_core_independent` in
`common/tests/run.sh`). What a build writes itself is the device's `port/<os>/lsm6dso_instance.h`
(chosen by the include path, `port/<os>` before `port/zephyr`), the sensor's start on the board and
the build rules. ST's register driver `lsm6dso_reg.c` is fetched by each build's `setup.sh` (N
`nuttx/stmemsc`, F `freertos/stmemsc`); its glue is the shared `common/drivers/stmemsc`.

## 6. Always per OS

| Item | N | F |
|---|---|---|
| Start-up to the scheduler | NuttX boot, `N-board/src/rp2040_boardinitialize.c`, `rp2040_bringup.c`, init task `nuttx/apps/init/rp2040_init.c` | `freertos/app/usb-console/src/main.c`, `vTaskStartScheduler()` |
| Kernel tick and timer | tickless timer `N-board/src/rp2040_oneshot.c`, `rp2040_timerisr_alarm.c` (100 µs tick) | SysTick 1 ms, tick hook and catch-up `tick.c`, `tick_catchup.h`; `FreeRTOSConfig.h` |
| Interrupt attach | `irq_attach()`, `rp2040_gpio_irq_attach()` | Pico SDK `irq_set_exclusive_handler()`, `irq_add_shared_handler()`, `gpio_add_raw_irq_handler_masked()` |
| Idle loop (the window arithmetic is shared, `common/compat/cpu_load.h`) | `N-board/src/rp2040_idle_cpuload.c` | `freertos/app/usb-console/src/idle.c` |
| Entry to BOOTSEL (the ROM's `reset_usb_boot()`) | `N-board/src/rp2040_restart.c` on NuttX's `ROM_LOOKUP()` | the Pico SDK's `reset_usb_boot()` (Zephyr: hal_rpi_pico's) |
| Chip ID (the flash's unique ID, read once with XIP off) | `N-board/src/rp2040_chipid.c` on NuttX's `rp2040_flash_cmd()` | the Pico SDK's `pico_unique_id` (Zephyr: its hwinfo driver) |
| USB device stack, console transport | `cdcacm`, `N-board/src/rp2040_usbserial.c`; Telnet `nuttx/apps/init/rp2040_telnet.c` | Pico SDK `stdio_usb` (TinyUSB); the console transport `freertos/drivers/shell/shell_serial_os_impl.h`, `shell_stdio.c`, `freertos/drivers/libc/stdio_wrap.c`; Telnet is the shell engine's |
| Heap | NuttX's | none in the kernel (`configSUPPORT_DYNAMIC_ALLOCATION` 0: every kernel object is static); newlib's `malloc()` from the end of `.bss` |
| The OS's own shell | nsh (sessions in `rp2040_init.c`) | none: the shared engine `common/compat/shell_engine` (consoles, serial session, Telnet) on the console transport of `freertos/drivers/shell` |
| The OS's own log in the RAM log | `syslog()`: the syslog channel, `nuttx/drivers/log/log_ram_syslog.c` | none (the FreeRTOS kernel and the Pico SDK have no log) |
| Fatal errors | `CONFIG_BOARD_RESET_ON_ASSERT`, report in the RAM log | `freertos/app/usb-console/src/fault.c` (the entries: `panic()`, `isr_hardfault`, the FreeRTOS hooks, `_exit()`; the task's name, the record in `__uninitialized_ram`, the watchdog's scratch registers and the SDK's `watchdog_reboot()`) with `common/compat/log/fault_record.c` (the record, `fault_record_set_frame()`, `fault_record_take()`) |
| Build system and configuration | Make, Kconfig, `defconfig`, `nuttx/patches` | CMake, `FreeRTOSConfig.h`, `freertos/drivers/*/*.cmake` |

## 7. Example: the IMU alone

The smallest composition that streams the sensor's data to the shell (`imu`, `sensor`) without Wi-Fi,
MQTT or time synchronization, listed from what the NuttX build compiles without `CONFIG_NINA` (and
without NuttX's `CONFIG_NET_USRSOCK`).

| Item | Holds |
|---|---|
| Options | `CONFIG_KV_STORE`, `CONFIG_APP_IMU`, `CONFIG_LSM6DSO_STREAM` (N also `CONFIG_LSM6DSO_LOCAL`); optionally `CONFIG_APP_SENSOR` (the `sensor` command). Not defined: `CONFIG_IMU_RAW_MQTT`, `CONFIG_TIME_SYNC`, `CONFIG_WIFI_CREDENTIALS`, `CONFIG_WIFI_PROFILE`, `CONFIG_WIFI_NINA_LOCAL`, `CONFIG_NET_ZPERF` |
| `common/imu` | `app_main.c`, `app_init.c`, `imu_stream.c`, `imu_raw_batch.c`, `imu_raw_packet.c`, `imu_raw_queue.c`, `imu_raw_time.c`, `imu_tick_clock.c`, `imu_cal.c`, `imu_cal_codec.c`, `kv_store.c`; in place of the transport and the time synchronization `imu_raw_mqtt_null.c` (every batch dropped and counted, as while disconnected) and `time_sync_null.c` (the monotonic clock only, no server time) |
| `common/compat` | `zephyr_init.c`, `zephyr_work.c`, `zephyr_rtio.c`, `zephyr_rtio_workq.c`, `zephyr_mem_blocks.c`, `zephyr_device.c`, `zephyr_gpio.c`, `zephyr_sensor_clock.c`, `sensor_default_rtio.c`, `sensor_shell_stream.c` (`CONFIG_APP_SENSOR`), `zephyr_flash_kv.c`, `kv_store_wipe.c`, `lib/utils/hex.c`, the settings and NVS copies (`subsys/settings/src/settings.c`, `settings_store.c`, `settings_init.c`, `settings_nvs.c`, `subsys/kvss/nvs/nvs.c`, `subsys/crc/crc8_sw.c`); the log `log/log_ram.c`, `log_ring.c` |
| Drivers | `common/drivers/i2c_dw/i2c_dw.c` with the DMA of `port/<os>/`; the 4 core files of `common/drivers/lsm6dso` and the 5 of `port/zephyr/` (section 5); `common/drivers/stmemsc/stmemsc_i2c.c`, `stmemsc_mdelay.c`; ST's `lsm6dso_reg.c`; `common/drivers/sensor_shell/sensor_shell.c` (`CONFIG_APP_SENSOR`); `common/chip/rp2040/rp2040_flash_kv.c` |
| Shell | `common/shell/zshell.c`, `subcmd.c` (optionally `i2c_dma_shell.c`); an OS without a shell of its own: `common/compat/shell_engine` but Telnet |
| The OS's | the kernel part (section 2), log, KV flash, GPIO bank 0 (INT1), CPU load, priorities, hwinfo (session IDs), I2C's `port/<os>/`, `lsm6dso_instance.h` and the sensor's start, console, start (4.6 without the network step) |
| Not needed | `common/compat/net_offload`; `common/compat/libc_net` (when the C library has the `<arpa/inet.h>` that `imu_raw_mqtt_null.c` reads; F's newlib does not); `common/compat/zephyr_heap.c`; the NINA driver (its passthrough mode, `CONFIG_NINA_PASSTHROUGH`, is chosen separately); `common/imu/mqtt_min*.c`, `imu_raw_status.c`, `time_sync.c`, `time_offset.c`, `wifi_profile.c` |
