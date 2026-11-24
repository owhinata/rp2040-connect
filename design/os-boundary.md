# The boundary between the shared code and an OS

English | [日本語](os-boundary-ja.md)

The firmware of this repository is built, with the same functions, on four OSes: Zephyr (`zephyr/`),
NuttX (`nuttx/`), Pico SDK + FreeRTOS (`freertos/`) and Pico SDK + ThreadX (`threadx/`); each is called a
build below (the Zephyr, NuttX, FreeRTOS and ThreadX builds). Most of the sources are shared by the four
and are in `common/`.

This document is an index of what each build supplies so that the shared sources work (what a new OS
writes), as the code has it today. Each row names the header that declares an item and the file of each
build that implements it. "N" is the NuttX build, "F" the FreeRTOS build, "T" the ThreadX build. F's and
T's files under `common/pico_sdk/`, the drivers' `port/pico_sdk/` and `common/compat/port/generic/` are
neither FreeRTOS's nor ThreadX's: they are written on the Pico SDK and the layer and shared by the builds
on the Pico SDK (F and T). "Same" in a T column means the same shared file as F.

Many of the shared sources are written on Zephyr's API (`<zephyr/kernel.h>` and so on). The NuttX,
FreeRTOS and ThreadX builds compile them on a layer that provides that API under Zephyr's names (the
"compatibility layer": its OS-independent part in `common/compat`, its OS parts in each build's
directories). The Zephyr build compiles `common/` against Zephyr itself and does not use the layer;
it appears only where it has a file of its own.

Paths below `nuttx/boards/arduino-nano-rp2040-connect/` are shortened to `N-board/`.

## What a new OS writes

In the order a porter works. The model is N for an OS whose own objects the layer is written on, F and T
for an OS on the primitives (F for an OS that offers only basic primitives, T for one with semaphores and
mutexes; both use the generic kernel part).

1. **The layer's kernel part** (section 2), one of:
   - directly: `zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h` and the 18 functions of 2.2
     (model: `nuttx/drivers/compat/`);
   - on primitives (with `common/compat/kernel_generic`): the 10 types and constants and 26 functions of
     `zephyr_kernel_os_impl.h` (2.5) and a tick hook that calls `zephyr_compat_tick()` once per tick
     (models: `zephyr_kernel_os_impl.h` of `freertos/drivers/compat/` and `threadx/drivers/compat/`, with
     their port headers `rp2040/zephyr_os_port.h`; the smallest form that compiles:
     `common/tests/kernel_generic_host/`). The OS part also meets the rules of 2.6 (the wake flag, the
     all-zero mutex, the direction of priorities, time slices, a thread's end and the reuse of its stack).
     `zephyr_os_atomic.h` has an empty default in `kernel_generic`; an OS whose headers define atomic
     names writes its own (F and T take the default).

   Either way also `priorities_os.h` (section 3; with the generic kernel part `APP_PRIO_FROM_ZEPHYR()` is
   "larger = more urgent" whatever the OS's own numbering) and the per-OS `<zephyr/sys/clock.h>`,
   `<zephyr/sys/__assert.h>` and `<zephyr/drivers/i2c.h>` (4.1); an OS without a model of its own for
   them takes `common/compat/port/generic`'s (F and T). The acceptance test is the host test of the
   layer's kernel (`common/tests/zephyr_kernel`, the two rows of section 3), run on the OS's part, on the
   OS's host port where it has one (F: FreeRTOS's POSIX port; T: ThreadX's Linux port).
2. **Console transport and shell** (section 3, 4.4): an OS without a shell of its own gives the shared
   shell engine (`common/compat/shell_engine`) `shell_serial_os_impl.h` and the stdio redirection
   (models: `common/pico_sdk/drivers/shell/`, `common/pico_sdk/drivers/libc/`); an OS with one writes
   `zshell_os_impl.h` and `zephyr_os_shell.h` (models: `nuttx/apps/subcmd/`,
   `nuttx/drivers/compat/zephyr_os_shell.h`).
3. **Log** (section 3): `app_log_os.h`, `log_ram_os_impl.h` (models: `common/compat/port/generic/app_log_os.h`,
   `common/pico_sdk/drivers/include/log_ram_os_impl.h`).
4. **KV flash** (section 3): on the RP2040 the 4 functions of `rp2040_flash_kv_os.h` and the linker
   script's `__kv_start`, `__kv_end` (model: `common/pico_sdk/drivers/kv/rp2040_flash_kv_os.c`).
5. **Board parts** (sections 3, 6): GPIO bank 0 (`gpio_os.h`), CPU load (`cpu_load_get()`), hwinfo
   (4.2), tick, interrupts, idle loop, fatal errors.
6. **Start** (4.6): the steps before the application's start (RAM log, layer, I2C, LSM6DSOX, network,
   KV flash bounds, shell) and the calls of the three functions of `app_main.h`; the modules'
   options (4.5).
7. **Per driver** (section 5): the N, F and T columns are the files a build writes itself. For the
   network, either bind to the OS's network stack (N) or use the layer's network part
   (`common/compat/net_offload`, F and T).

A first target can be the sensor without Wi-Fi (section 7).

**On the Pico SDK.** An OS on the Pico SDK with the generic kernel part takes items 2 to 7 and most of 1
from what F and T share (`common/pico_sdk/`, the drivers' `port/pico_sdk/`, `common/compat/port/generic/`,
`common/compat/include/tick_catchup.h`), and writes only:

- the kernel part: `zephyr_kernel_os_impl.h` (2.5, 2.6), its port header `zephyr_os_port.h`
  (`z_os_in_isr()` and the interrupt mask; F's also `z_os_yield_from_isr()`, `z_os_task_lock()`,
  `z_os_task_unlock()`; it includes `common/pico_sdk/drivers/compat/rp2040/zephyr_os_port_rp2040.h`, which
  has `z_os_busy_wait_us()`, `z_os_tick_now_us32()` and `z_os_tick_pend()`), `priorities_os.h` (the
  mapping, the OS's own threads, the stacks of the layer's threads) and the OS's configuration (F:
  `FreeRTOSConfig.h`; T: `tx_user.h`);
- the tick: the OS's tick handler or hook with the OS's tick, the catch-up of lost ticks
  (`tick_catchup_step()`) and `zephyr_compat_tick()` (F: the hook in `freertos/app/usb-console/src/tick.c`;
  T: `isr_systick` in `threadx/app/usb-console/src/tick.c`); where the OS's port does not set up SysTick,
  that and the exception priorities (T: `src/tx_initialize_low_level.c`), and the OS's handlers on the
  vectors of the Pico SDK's crt0 (T: `--defsym` in the link);
- the entry of the start (4.6): `main()` in `main.c`, which calls the four shared functions of
  `usb_console.h` in order, with the kernel's set-up, the thread "main" and the scheduler's start between them;
- the OS parts of the application on the Pico SDK (section 3): `idle_os.h` (idle, in one of two shapes), `fault_os.c` (the running thread's name and the OS's own fatal
  entries, the stack check among them, which call the shared `fault_begin()` and `fault_finish()`), `commands_os.c` (`ps` with each thread's run time, the
  kernel's name and version; the shared `commands.c` registers the commands), `kernel_shell_os_impl.h` (generic commands);
- the tie of the Pico SDK's locks and sleeps to the OS's threads (section 3), unless the OS's port brings
  it (F: FreeRTOS's RP2040 port; T: `threadx/drivers/pico_sdk_interop/`). Without it a second thread
  takes the USB console's mutex for its own and the output and input are silently dropped;
- the drivers' waits (section 5): `common/drivers/i2c_dw/port/<os>/i2c_dw_os_impl.h` (with
  `i2c_dw_os_sem_init()`), `common/drivers/nina/port/<os>/nina_os.c` and `nina_bus_dma_os_impl.h`;
- the build: a `CMakeLists.txt` that sets the variables of
  `common/pico_sdk/app/usb-console/usb-console.cmake` and lists the sources (F:
  `freertos/app/usb-console/CMakeLists.txt`; T: `threadx/app/usb-console/CMakeLists.txt` with
  `threadx_kernel.cmake`, which adds the kernel as sources), the scripts that fetch, build, flash, format
  and run the host tests (`<os>/script/`), and the build's `--target` and USB product name in the flash
  script `script/flash_uf2.sh`;
- host tests: the OS part of the host test of the layer's kernel (`zephyr_kernel_test_os_impl.h` and the
  platform part), and the OS part of the tests of the drivers' waits (the FreeRTOS build's test bodies
  `freertos/tests/{i2c_dw,nina}/main.c` compiled with a per-kernel `*_test_os.[ch]`; T:
  `threadx/tests/i2c_dw/`, `threadx/tests/nina/`).

What T wrote (`wc -l`; in brackets the lines without comments and blank lines, for the scripts without
`#` lines and blank lines):

| Part | T's files | Lines |
|---|---|---|
| Kernel part (the 26 primitives) | `threadx/drivers/compat/zephyr_kernel_os_impl.h`, `zephyr_kernel_os.c` | 521 (296) |
| Port header | `threadx/drivers/compat/rp2040/zephyr_os_port.h` | 70 (31) |
| Kernel configuration | `threadx/app/usb-console/tx_user.h` | 82 (16) |
| Tick, SysTick and the exception priorities | `src/tick.c`, `src/tx_initialize_low_level.c` | 136 (44) |
| The start and the application's boundaries | `src/main.c` 115, `idle_os.h` 32, `fault_os.c` 71, `commands_os.c` 206, `run_time.[ch]` 100, `kernel_shell_os_impl.h` 49 | 573 (297) |
| Priorities and time slices | `src/priorities_os.h` | 76 (10) |
| The Pico SDK's locks | `threadx/drivers/pico_sdk_interop/` (`.c`, `.h`, `.cmake`) | 254 (126) |
| The drivers' waits | `common/drivers/i2c_dw/port/threadx/i2c_dw_os_impl.h` 313, `common/drivers/nina/port/threadx/nina_os.c` 510, `nina_bus_dma_os_impl.h` 106 | 929 (439) |
| Build | `threadx/app/usb-console/CMakeLists.txt`, `threadx_kernel.cmake` | 170 (80) |
| Firmware total | | 2,811 (1,339) |
| Scripts | `setup.sh`, `build.sh`, `flash.sh`, `format.sh`, `host_tests.sh` of `threadx/script/` | 587 (396) |
| Host tests | `threadx/tests/compat/`, `i2c_dw/`, `nina/` | 1,426 (948) |
| Board test firmware (optional) | `threadx/tests/board_kernel/` | 1,351 (939) |

The one addition T made to a shared file is `--target threadx` of `script/flash_uf2.sh`.

## 1. What a build consists of

| Part | Where | Holds |
|---|---|---|
| Application | `common/imu` | IMU service, MQTT transport, time synchronization, Wi-Fi profile, KV store, calibration; the application's start (`app_main.h`: `app_main.c`, all four builds; `app_init.c`, N, F and T; section 4.6) |
| Drivers | `common/drivers/<driver>/` | OS-independent core; `port/` boundary headers; `port/<os>/` bindings (section 5) |
| Shell commands | `common/shell` | `zshell.c` (Zephyr shell command handling), `subcmd.c`, `nina_shell.c`, `i2c_dma_shell.c`; F and T only: `dmesg_shell.c` (N has `nuttx/apps/dmesg`), `kernel_shell.c` (`id`, `uptime`, `date`, `sleep`, `devmem`, `fault`, `reboot`, `bootsel`; on `kernel_shell_os.h`, section 3), `ifconfig_shell.c` (on the network part's interface) |
| Log | `common/compat/log` (the sources; the headers are the layer's, `common/compat/include`, `internal`, `port`) | RAM log and the functions of the `LOG_*` macros (`log_ram.c`, on its OS part `log_ram_os.h`, section 3), its ring (`log_ring.c`), `LOG_*` macros (`app_log.h`), fault record text and its printing, the HardFault's exception frame (Cortex-M0+) and the choice of the record at the next start (`fault_record.c`) |
| Compatibility layer, shared part | `common/compat` | One set of headers for the whole layer and its parts: `include/` (the Zephyr headers, `include/zephyr/`, and every header that other code includes), `internal/` (headers that only the layer's sources include), `port/` (the boundary headers that an OS implements: `gpio_os.h`, `flash_kv_os.h`, `cpu_load.h`, `zephyr_kernel_os.h`, `log_ram_os.h`, `shell_serial_os.h`) and `port/generic/` (next row); the network part's and the POSIX headers keep their own include directories (`net_offload/include`, `libc_net/include`), whose headers replace or add to the C library's and the layer's. The shared part's sources are at the top:  `k_heap` (`zephyr_heap.h`, `zephyr_heap.c`; read by `<zephyr/kernel.h>` with either kernel part, 4.2), work items, the work queues' threads and the layer's start (`zephyr_init.c`), RTIO, gpio0 device, net_mgmt, Wi-Fi requests, KV flash device, copies of Zephyr's NVS, settings, getopt, Wi-Fi credentials, zperf |
| Compatibility layer, network part for an OS without a network stack | `common/compat/net_offload` | Zephyr's socket layer (reduced copies of `subsys/net/lib/sockets`, `lib/os/zvfs`), the one network interface (`zephyr_net_if.c`) and its start (`zephyr_net_init.c`), their headers in its own include root `include/` (`zephyr/net/net_if.h`, `net_ip.h`, `socket.h`, `socket_offload.h`, `zephyr/sys/fdtable.h`, `zephyr_net_init.h`); used by F and T |
| POSIX network headers for a C library without them | `common/compat/libc_net` | `include/` (`arpa/inet.h`, `netdb.h`, `netinet/in.h`, `netinet/tcp.h`, `poll.h`, `sys/socket.h`, `sys/uio.h`), `inet_pton()`/`inet_ntop()` (`inet.c`); used by F and T |
| Kernel part of the compatibility layer on an OS's primitives | `common/compat/kernel_generic` (the sources; `zephyr_os.h`, `zephyr_os_spinlock.h` and the default `zephyr_os_atomic.h` are in `port/generic`, `zephyr_kernel_os.h` in `port/`, `zephyr_compat_os.h` and `tick_catchup.h` in `include/`) | `k_sem`, `k_poll()`, the timeout list, the sleeps and the start of a thread (`zephyr_kernel.c`), `k_mutex` (`zephyr_mutex.c`), the OS part of `<zephyr/kernel.h>` and `<zephyr/spinlock.h>` (`zephyr_os.h`, `zephyr_os_spinlock.h`), the tick's entry (`zephyr_compat_tick()`, `zephyr_compat_os.h`), all on the primitives declared in `zephyr_kernel_os.h` (section 2); the catch-up of lost ticks for an OS's periodic tick (`tick_catchup.h`); the default `zephyr_os_atomic.h` (empty); used by F and T |
| Compatibility layer, parts for an OS without the model of its own | `common/compat/port/generic` | `zephyr/drivers/i2c.h` (Zephyr's I2C API on the driver's device, 4.1), `zephyr/sys/clock.h`, `zephyr/sys/__assert.h` (nothing, as `CONFIG_ASSERT` off), `app_log_os.h`, `wifi_profile_os.h` (section 3), the prelude of the shared sources for a build without Kconfig (`common_prelude.h`, 4.5), the generic kernel part's `zephyr_os.h`, `zephyr_os_spinlock.h` and default `zephyr_os_atomic.h`, the shell engine's `zshell_os_impl.h` and `zephyr_os_shell.h`; the sensor shell's OS part is `common/shell/port/generic` (`sensor_shell_os.h`, `sensor_shell_os.c`); used by F and T |
| Parts shared by the builds on the Pico SDK | `common/pico_sdk` | `app/usb-console/`: the shared steps of the start (`src/usb_console.[ch]`, called by the OS's `main()`), the commands `version`, `free` and the table (`src/commands.c`), the idle windows and the CPU load (`src/idle.c`), fatal errors (`src/fault.[ch]`), the interrupt stack (`src/irq_stack.c`), on the boundaries `idle_os.h`, `fault.h`, `commands_os.h` (section 3); the build (`usb-console.cmake`, `ram_code.cmake`, `ld/`). `drivers/`: the layer's part on the Pico SDK (`compat/`: `compat.cmake`, the realtime clock `zephyr_sys_clock.c`, the linker script part of the socket registrations, `rp2040/`: `zephyr_os_port_rp2040.h`, GPIO bank 0, hwinfo), the console transport (`shell/`), newlib's stdout/stderr wrappers (`libc/`), the RAM log's part (`include/log_ram_os_impl.h`), the KV flash operations and linker script parts (`kv/`), and the build files of the drivers (`<driver>/<driver>.cmake`). Used by F and T |
| Host test of the compatibility layer's kernel (acceptance test of an OS part) | `common/tests/zephyr_kernel` | `zephyr_kernel_test.c` (22 tests), its boundary `zephyr_kernel_test.h` (section 3), the boundary's items on the primitives of the generic kernel part (`kernel_generic/zephyr_kernel_test_os.h`); run by F's and T's host tests (`freertos/tests/compat`, `threadx/tests/compat`) and by T's board test firmware (`threadx/tests/board_kernel`), not built into a product firmware |
| Shell engine for an OS without a shell of its own | `common/compat/shell_engine` (the sources; the headers are in `common/compat/include`, `internal`, `port`, `port/generic`) | on the layer's API: line editor (`cli.c`), command table and dispatch of the `Z_SHELL_CMD_ROOT_GLOBAL` roots and `help` (`shell_engine.c`; the application supplies `shell_engine_cmds[]`), Zephyr's word split (`shell_argv.c`), the consoles and each thread's stdout (`shell_console.c`), getopt state per session (`getopt_state.c`), the backend of `zshell.c` (`zshell_os_impl.h`, `zephyr_os_shell.h`), the serial session (`shell_serial.c`, the thread "console", on the console transport `shell_serial_os.h`, section 3), the Telnet shell (`shell_telnet.c`, the threads "shell_telnet" and "telnet_service", on `zsock_*()`); used by F and T |
| RP2040 code that no OS owns | `common/chip/rp2040` | `include/rp2040_timer_us.h` (the TIMER's registers and its 64-bit read `rp2040_timer_us()`, always inline; all four builds), `include/rp2040_watchdog_reboot.h` (`rp2040_watchdog_reboot()`, the watchdog's restart on the raw registers; the Zephyr and NuttX builds); the KV flash boundary on the RP2040 for an OS without a flash driver for the area (`rp2040_flash_kv.c` on the build's flash operations `include/rp2040_flash_kv_os.h`, section 3; `include/rp2040_flash_kv.h`; N, F and T, built by `N-board/src/Make.defs` and `common/pico_sdk/drivers/kv/kv.cmake`). Each build puts `include/` on its include path (Zephyr: the app's `CMakeLists.txt`; N: `N-board/scripts/Make.defs`, for every source; F and T: the shared `usb-console.cmake`, T also `threadx_kernel.cmake` for ThreadX's sources) |
| Host tests | `common/tests` | Tests and fakes; `common/tests/zephyr_fakes` is a host implementation of the kernel's OS part |
| Zephyr build | `zephyr/app/usb-console` | `main()`, `log_ram.c`, `kv_store_os.h`, `wifi_profile_os.h`; upstream Zephyr with patches in `zephyr/manifest/zephyr/patches` |
| N: board | `N-board/` | Bring-up, tickless timer, idle loop and CPU load, GPIO bank 0, KV flash, USB serial, BOOTSEL (`include/rp2040_restart.h`) |
| N: compatibility layer, OS part | `nuttx/drivers/compat` | `zephyr_os*.h`, kernel, mutexes, I2C on `struct i2c_master_s`, net_if, hwinfo, prelude, per-OS `<zephyr/...>` headers |
| N: shared OS headers | `nuttx/drivers/include` | `app_log_os.h`, `log_ram_os_impl.h`, `priorities_os.h` |
| N: log | `nuttx/drivers/log` | `log_ram_syslog.c` (the RAM log as the syslog channel), `Make.defs` |
| N: drivers | `nuttx/drivers/<driver>/` | `Make.defs`, `Kconfig` of each driver |
| N: applications | `nuttx/apps/*` | Init task (`init`), one builtin application per command, shell OS part (`subcmd`), Wi-Fi and sensor parts |
| N: NuttX changes | `nuttx/patches` | Local patches to NuttX and nuttx-apps |
| F: application | `freertos/app/usb-console` | the build (`CMakeLists.txt`: the settings of `common/pico_sdk/app/usb-console/usb-console.cmake` and the sources), `FreeRTOSConfig.h`, the entry of the start (`src/main.c`: `main()`, the task "main", the scheduler), the tick hook (`src/tick.c`), `src/priorities_os.h`, the FreeRTOS parts of the application's boundaries on the Pico SDK (`src/idle_os.h`; `src/fault_os.c`: the task's name and the stack overflow hook; `src/commands_os.c`: `ps`, the clock of its run times, the kernel's version) and the generic commands' OS part (`src/kernel_shell_os_impl.h`) |
| F: compatibility layer, OS part | `freertos/drivers/compat` | the primitives of the generic kernel part (`zephyr_kernel_os_impl.h`) and its port header (`rp2040/zephyr_os_port.h`) |
| F: drivers' bindings | `common/drivers/<driver>/port/freertos` | the NINA's (`nina_os.c`, `nina_bus_dma_os_impl.h`) and I2C's (`i2c_dw_os_impl.h`) waits, locks and clocks; the rest of the bindings is `port/pico_sdk` (section 5) |
| T: application | `threadx/app/usb-console` | the build (`CMakeLists.txt`: the settings of `usb-console.cmake` and the sources; `threadx_kernel.cmake`: ThreadX's kernel, its Cortex-M0 port and the execution profile kit as sources, the board's part, the vectors, the Pico SDK's locks), the kernel configuration (`tx_user.h`), SysTick and the exception priorities (`src/tx_initialize_low_level.c`), the tick (`src/tick.c`), `src/priorities_os.h` (with the time slices), the entry of the start (`src/main.c`: `main()`, the kernel's set-up, the thread "main", the scheduler), the ThreadX parts of the application's boundaries on the Pico SDK (`src/idle_os.h`; `src/fault_os.c`: the thread's name and the stack check's handler; `src/commands_os.c`: `ps` and the kernel's version), the run times of `ps` (`src/run_time.[ch]`) and the generic commands' OS part (`src/kernel_shell_os_impl.h`) |
| T: compatibility layer, OS part | `threadx/drivers/compat` | the primitives of the generic kernel part (`zephyr_kernel_os_impl.h`, with the lock's variables in `zephyr_kernel_os.c`) and its port header (`rp2040/zephyr_os_port.h`) |
| T: the Pico SDK's locks | `threadx/drivers/pico_sdk_interop` | the hooks of the Pico SDK's `pico/lock_core.h` (the waits of its mutexes, semaphores and sleeps) on ThreadX's threads (section 3) |
| T: drivers' bindings | `common/drivers/<driver>/port/threadx` | the NINA's (`nina_os.c`, `nina_bus_dma_os_impl.h`) and I2C's (`i2c_dw_os_impl.h`) waits, locks and clocks; the rest of the bindings is `port/pico_sdk` (section 5) |
| T: tests | `threadx/tests` | host tests (`compat/`: the layer's kernel; `i2c_dw/`, `nina/`: the drivers' waits; on ThreadX's Linux port), the board test firmware (`board_kernel/`, not a product firmware) |

### 1.1 What the selectable parts need

The kernel part (written directly, as N, or `kernel_generic`), the network part (the OS's network
stack or `net_offload`), the POSIX network headers (the C library's or `libc_net`) and the shell (the
OS's or `shell_engine`) are chosen independently. What each part reads (from its `#include` lines):

| Part | Needs | Does not need |
|---|---|---|
| `common/compat/kernel_generic` | the layer's headers; the OS's primitives (`zephyr_kernel_os_impl.h`; `z_os_thread_t` is a pointer type, NULL for no thread); the OS's `priorities_os.h`; a `zephyr_os_atomic.h` (the OS's or the default in `port/generic`) and a `<zephyr/sys/clock.h>` (the OS's or `port/generic`'s) | `net_offload`, `libc_net`, the RAM log's and the shell engine's headers (`kernel_generic_independent` in `common/tests/run.sh`) |
| `common/compat/net_offload` | a kernel part (either); POSIX socket headers (the C library's or `libc_net`); `net_mgmt.h`, `wifi_mgmt.h` of the layer; a `<zephyr/sys/__assert.h>` (the OS's or `port/generic`'s) | `kernel_generic`, `port/generic`, the RAM log's and the shell engine's headers (`net_offload_independent`, on a fake kernel of N's form) |
| `common/compat/libc_net` | the C library's `<sys/types.h>` | everything else |
| `common/compat/shell_engine` | the kernel part's 4 optional items (2.1); `common/shell/subcmd.h`; the getopt copies (`common/compat/lib/utils/getopt`); the log (`app_log.h`); the console transport (section 3); only the Telnet shell (`shell_telnet.c`) `zsock_*()` (the OS's network stack or `net_offload`) | `kernel_generic` (either kernel part) |
| `common/tests/zephyr_kernel` | a kernel part; `zephyr_kernel_test_os.h` (with the generic kernel part, this directory's `kernel_generic/` one and the OS's `zephyr_kernel_test_os_impl.h`; an OS with a direct kernel part writes its own); the platform part (section 3) | network, shell |
| `common/compat/log` | the OS's `app_log_os.h`, `log_ram_os_impl.h` | the rest of the layer |
| `common/compat/port/generic` | `i2c.h`: `<zephyr/device.h>`, `<zephyr/sys/util.h>` of the layer; `common/shell/port/generic/sensor_shell_os.*`: a kernel part (`k_sem`), `priorities.h`, the sensor shell's headers; `wifi_profile_os.h`: `<zephyr/kernel.h>`; the rest nothing | — |
| `common/pico_sdk` | the Pico SDK; the generic kernel part with the OS's primitives (`gpio_os_rp2040.c`: `z_os_task_lock()`); the entry `main.c` and the OS parts of section 3 for the application (`idle_os.h`, `fault_os.c`, `commands_os.c`) | no OS's name (each build passes its directories, `PICO_SDK_OS_*`) |
| `common/chip/rp2040` | only `rp2040_flash_kv.c`: the layer's `flash_kv.h` and `flash_kv_os.h`, the log (`app_log.h`); the headers of `include/` read nothing | — |
| `common/shell` | `kernel_shell.c`: the log (`fault_record.h`), `<zephyr/sys/clock.h>`, hwinfo; `dmesg_shell.c`: the log; `ifconfig_shell.c`: `<zephyr/net/net_if.h>` (the network part); `nina_shell.c`: the NINA's `include/`; `zshell.c`: the shell backend (section 3) | — |

## 2. Kernel part of the compatibility layer

Declared in `common/compat/include/zephyr/kernel.h`, which includes `"zephyr_os.h"` first. An OS provides it in
one of two ways:

- **Directly**, as N does (`nuttx/drivers/compat/`): its own `zephyr_os.h`, `zephyr_os_spinlock.h`,
  `zephyr_os_atomic.h` and the functions of 2.2 on the OS's own objects (semaphores, watchdogs,
  kernel threads).
- **On primitives**, as F and T do: the generic implementation (`zephyr_kernel.c`, `zephyr_mutex.c` of `common/compat/kernel_generic/`,
  `zephyr_os.h`, `zephyr_os_spinlock.h` of `common/compat/port/generic/`, `internal/zephyr_os_internal.h`,
  `include/zephyr_compat_os.h`) implements 2.1, 2.2 and the
  spinlock with Zephyr's rules on the primitives declared in `common/compat/port/zephyr_kernel_os.h` (2.5). The OS
  writes those primitives (`zephyr_kernel_os_impl.h`, static inline; `zephyr_os_atomic.h` only when its
  headers define atomic names, the default of `kernel_generic` being empty), a tick hook
  that calls `zephyr_compat_tick()` once per tick from the tick interrupt, and for the host test of
  the layer's kernel `zephyr_kernel_test_os_impl.h` (F: `freertos/tests/compat/`; T: `threadx/tests/compat/`).
  The rules the OS part meets are gathered in 2.6.

### 2.1 Provided by `zephyr_os.h` (16 items, 3 optional)

"Generic" is the generic implementation's `zephyr_os.h`, which F and T use; its items come from the primitives.

| Item | Meaning | N | Generic (F, T) |
|---|---|---|---|
| OS headers before the layer's | the layer's `<zephyr/sys/atomic.h>` takes over atomic names | NuttX headers, `"rp2040_timer_us.h"` (`common/chip/rp2040`) | `zephyr_kernel_os.h` (the OS's headers: F's FreeRTOS headers and `zephyr_os_port.h`; T's ThreadX `tx_api.h`, `tx_thread.h` and the like and `zephyr_os_port.h`), `priorities.h`, `<zephyr/sys/clock.h>` |
| `Z_USEC_PER_TICK` | tick length in µs; 1000 must be a multiple | `CONFIG_USEC_PER_TICK` (100) | `1000000 / Z_OS_TICK_HZ` (F: `configTICK_RATE_HZ`; T: `TX_TIMER_TICKS_PER_SECOND`; both 1000) |
| `struct k_mutex`, `Z_MUTEX_INITIALIZER(obj)` | all-zero must work as an unlocked mutex (also one zeroed after use, 2.6) | `rmutex_t` + `ready` flag | the OS's (F: static recursive mutex; T: a ThreadX mutex; both created on first use) |
| `struct k_sem`, `Z_SEM_INITIALIZER(obj, initial, limit)` | count with a limit; poll event list | `sem_t` + limit + events | the layer's own count, waiters, events |
| `K_SEM_MAX_LIMIT` | largest limit | `SEM_VALUE_MAX` | `UINT_MAX` |
| `struct _timeout`, `_timeout_func_t` | timer of a delayable work item | `struct wdog_s` | entry in the layer's timeout list |
| `k_tid_t` | thread handle | `pid_t` | `z_os_thread_t` (F: `TaskHandle_t`; T: `TX_THREAD *`) |
| `K_THREAD_STACK_RESERVED` (optional, 0 by default) | bytes `K_THREAD_STACK_DEFINE()` adds after each stack for the OS | — (0) | `Z_OS_THREAD_RESERVED` (F: the task's control block, 136; T: the control block, the wake flag, the entry and the name, 272) |
| `k_current_get()` | the calling thread | `gettid()` | `z_os_thread_self()` |
| `k_yield()` | the next ready thread of the priority, or go on | `sched_yield()` | `z_os_yield()` |
| `K_PRIO_PREEMPT(x)` | OS priority of Zephyr priority x, order kept | `APP_PRIO_FROM_ZEPHYR` (118 − x) | `APP_PRIO_FROM_ZEPHYR` (larger more urgent; F: 20 − x; T: 15 − x) |
| `k_cycle_get_32()`, `k_cycle_get_64()` | 1 MHz counter | `rp2040_timer_us()` | `z_os_clock_us()` (F and T: `rp2040_timer_us()`) |
| `Z_OS_ASSERT(test)` | the OS's assertion, for a work queue's thread that did not start | `DEBUGASSERT()` (nothing in the imu configuration) | the OS's (F: `configASSERT()`; T: the port header's `Z_TX_ASSERT()`; both `app_assert_failed()`: fault record, restart) |
| `k_is_in_isr()`, `k_is_pre_kernel()`, `k_thread_custom_data_get()`, `k_thread_custom_data_set()` (optional: for the shell engine) | in an ISR; before the kernel runs threads; the calling thread's one pointer (NULL until set and before the kernel runs threads) | — (not built) | `z_os_in_isr()`, `z_os_pre_kernel()`, `z_os_thread_local_get()`, `_set()` (F: TLS pointer 0; T: `tx_thread_zephyr_local` of `TX_THREAD_USER_EXTENSION`) |
| `Z_SEM_TAKE_FOREVER(sem)` (optional) | `k_sem_take(sem, K_FOREVER)` the OS's shorter way: the work queue's wait | `nxsem_wait_uninterruptible()` (what `k_sem_take()` does, without the call) | not defined: `k_sem_take()` (`kernel.h`'s default) |

`<zephyr/kernel.h>` includes `common/compat/include/zephyr_heap.h` at its end: `struct k_heap`, `K_HEAP_DEFINE()`, `k_heap_alloc()`, `k_heap_free()`; OS-independent (on `k_spin_lock()`), the same with either kernel part (4.2).

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
| `z_thread_static_start(thread)` | run `entry(p1, p2, p3)` in a new thread with the name, static stack (`K_THREAD_STACK_DEFINE()`, with its `K_THREAD_STACK_RESERVED` bytes for the OS) and priority of `*thread` (which stays valid while it runs), end it when `entry` returns, set `*tid`; 0 or −errno. Every thread of the layer starts with it. It may be called again on a stack whose thread has ended (2.6) |
| `k_sleep(t)` | K_NO_WAIT yields; else wakes at tick + ticks + 1; K_FOREVER for ever; returns 0 |
| `k_msleep(ms)`, `k_usleep(us)` | rounded up to ticks, then as `k_sleep()`; ≤ 0 yields; returns 0 |
| `k_busy_wait(us)` | spin at least `us` µs |
| `z_add_timeout(to, fn, t)` | run `fn` after the timeout, in interrupt context (not while `k_spin_lock()` is held) |
| `z_abort_timeout(to)` | stop it |

Shared, on these: the work item logic (`common/compat/zephyr_work.c`); `common/compat/zephyr_init.c`:
`k_work_queue_start()` (the queue keeps its thread's `struct z_thread_static`; the thread loops on
`z_work_queue_process()`, `k_yield()` and `Z_SEM_TAKE_FOREVER()`), the object `k_sys_work_q`
("sysworkq", `APP_PRIO_SYSWORKQ`), `zephyr_compat_initialize()` (declared in
`common/compat/include/zephyr_compat.h`: `k_sys_work_q`, −ENOMEM if its thread did not start, the RTIO work
queue, gpio0 with `CONFIG_LSM6DSO_STREAM`), `k_uptime_get()` (`Z_UPTIME_MS()` of
`k_cycle_get_64()`); the "net_mgmt" thread (`common/compat/zephyr_net_mgmt.c`).

| Function | N | Generic (F, T) |
|---|---|---|
| mutex | `nxrmutex_*()` | the timeout and Zephyr's results around `z_os_mutex_init()`, `_lock()`, `_unlock()` (F: `xSemaphore*Recursive()`, FreeRTOS's priority inheritance; T: `tx_mutex_*()`, ThreadX's priority inheritance, `TX_INHERIT`) |
| `k_sem_*` | `nxsem_*()` | the layer's count; waiters sleep on their wake flag (`z_os_wait()`; F: task notification index 1; T: a binary `TX_SEMAPHORE` of each thread) |
| `z_thread_static_start()` | `kthread_create_with_stack()`; the start data handed over through a variable, the caller waits until the new thread has taken it | `z_os_thread_start()` with the data as the thread's argument (F: `xTaskCreateStatic()`, scheduler suspended around it; T: `tx_thread_create()` unstarted, then `tx_thread_resume()`) |
| sleeps | `nxsched_ticksleep()`, `nxsched_msleep()`, `nxsched_usleep()` | `z_os_sleep(ticks + 1)` (F: `vTaskDelay()`; T: `tx_thread_sleep()`) |
| `k_busy_wait()` | `up_udelay()` | `z_os_busy_wait_us()` (F and T: `busy_wait_us_32()`) |
| `z_add_timeout()` | `wd_start()` | sorted list run by `zephyr_compat_tick()` from the OS's tick hook |

### 2.3 Behaviour that differs between N and F, T

| Point | N | F (Zephyr's rules) | T |
|---|---|---|---|
| `k_sem_give()` with a thread in `k_sem_take()` | posts the count and also signals every registered poll event | the waiter takes the count; no poll event is signaled | as F |
| `k_sem_give()` with pollers | signals every registered event, wakes every poller | signals only the first registered event | as F |
| `k_sem_reset()` and waiters | count drained; waiters keep waiting | waiters return −EAGAIN | as F |
| Order of registered poll events | newest first (prepended) | by thread priority, first come within one | as F |
| `k_poll()` after waking | re-checks every semaphore's count | counts only signaled events | as F |
| `k_sem_take()` in an interrupt with a timeout | not checked | −EBUSY | as F |
| `k_mutex_unlock()` errors | `nxrmutex_unlock()`'s result | −EINVAL (not held), −EPERM (other holder) | as F |
| `k_thread_name_set()` | −ENOSYS without `CONFIG_TASK_NAME_SIZE` | NULL means the caller | as F |
| When an owner raised by priority inheritance drops back | NuttX's | when it has released all the mutexes it holds; also when a waiter times out | at each release, to the highest waiter of the mutexes it still holds; also when a waiter times out (2.6) |
| End of a thread of `z_thread_static_start()` | kernel thread returns | `z_os_thread_exit()` (F: `vTaskDelete(NULL)`) | `z_os_thread_exit()` (`tx_thread_terminate()`; the thread stays in ThreadX's list until the next start on its stack deletes it) |
| Start of a thread | the caller waits until the new thread has taken its start data | the caller goes on; the task runs when the scheduler picks it (at once if its priority is higher) | as F |
| A work queue's thread that did not start (`Z_OS_ASSERT()`) | `DEBUGASSERT()`: nothing in the imu configuration, the queue's thread stays 0 | `configASSERT()`: fault record and restart | `Z_TX_ASSERT()`: fault record and restart |
| Tick | 100 µs | 1 ms | 1 ms |
| Builds `zephyr_init.c` (and so gpio0's start) | only with `CONFIG_LSM6DSO_STREAM` (the whole kernel part) | always; gpio0 only with `CONFIG_LSM6DSO_STREAM` | as F |
| Caller of `zephyr_compat_initialize()` | `N-board/src/rp2040_bringup.c` | `usb_console_init()` before the scheduler (`common/pico_sdk/app/usb-console/src/usb_console.c`, called by `main()` in `freertos/app/usb-console/src/main.c`) | as F (with interrupts masked, 4.6) |
| Same-priority choice | NuttX's | FreeRTOS's round robin at every switch; the time slices in the tick hook | ThreadX's first come; a preempted thread stays at the head of its priority; the time slices are ThreadX's `time_slice` of each thread |

T is on the generic implementation, so the rows of Zephyr's rules are as F. What differs is what belongs
to the OS: how priority inheritance drops back, a thread's end, the assertion, the same-priority choice.

### 2.4 Spinlock and atomics

| Header (shared) | Includes | OS provides | N | Generic (F, T) |
|---|---|---|---|---|
| `common/compat/include/zephyr/spinlock.h` | `"zephyr_os_spinlock.h"` | `struct k_spinlock`, `k_spinlock_key_t`, `k_spin_lock()`, `k_spin_unlock()`: interrupts masked and no switch until unlock; threads and interrupts; nests | `sched_lock()` + `up_irq_save()` | `z_os_task_lock()` in a thread (F: `taskENTER_CRITICAL()`; T: PRIMASK and ThreadX's preemption lock `_tx_thread_preempt_disable`), `z_os_isr_lock()` in an interrupt (F: `taskENTER_CRITICAL_FROM_ISR()`; T: PRIMASK) |
| `common/compat/include/zephyr/sys/atomic.h` | `"zephyr_os_atomic.h"` | OS headers that define atomic names, read first | reads `<nuttx/atomic.h>`, undefines its `atomic_*` macros | the OS's, or the generic part's empty default (F and T: the default, `common/compat/port/generic/zephyr_os_atomic.h`) |

The atomic operations themselves are shared (on `k_spin_lock()`).

### 2.5 Primitives of the generic implementation (`common/compat/port/zephyr_kernel_os.h`)

`zephyr_kernel_os.h` includes the OS's `"zephyr_kernel_os_impl.h"` and declares what it must define, so a
missing or different one does not compile. All are static inline: they are on the paths of
`k_sem_give()`, `k_sem_take()`, `k_poll()`, `k_spin_lock()` and the tick (no call through a pointer).
F's are `freertos/drivers/compat/zephyr_kernel_os_impl.h`, T's `threadx/drivers/compat/zephyr_kernel_os_impl.h`,
with the port-dependent ones in `zephyr_os_port.h` (board `<os>/drivers/compat/rp2040/`, host
`<os>/tests/compat/`); the board's includes `common/pico_sdk/drivers/compat/rp2040/zephyr_os_port_rp2040.h`,
which has `z_os_busy_wait_us()` for every OS on the Pico SDK.

Types and constants (10): `z_os_thread_t` (F `TaskHandle_t`, T `TX_THREAD *`), `z_os_prio_t` (larger more
urgent; F `UBaseType_t`, T `UINT`, ThreadX's number being 31 − this value), `z_os_ticks_t` (unsigned;
`TickType_t`, `ULONG`), `Z_OS_TICKS_FOREVER` (its largest value, no timeout; `portMAX_DELAY`,
`TX_WAIT_FOREVER`), `Z_OS_TICK_HZ` (`configTICK_RATE_HZ`, `TX_TIMER_TICKS_PER_SECOND`), `z_os_woken_t` (0 at
first; `BaseType_t`, `UINT`), `Z_OS_THREAD_NAME_SIZE` (`configMAX_TASK_NAME_LEN`, 32), `Z_OS_THREAD_RESERVED`
(the bytes the OS keeps after each thread's stack for its control block, a multiple of 8, 0 if none:
`K_THREAD_STACK_DEFINE()` adds them, as Zephyr's `K_THREAD_STACK_RESERVED`, and `K_THREAD_STACK_SIZEOF()`
leaves them out; F: `sizeof(StaticTask_t)` rounded up to 8, 136; T: the `TX_THREAD`, the wake flag's
`TX_SEMAPHORE`, the entry and its argument and the name array, rounded up to 8, 272), `struct k_mutex`
with `Z_MUTEX_INITIALIZER()` (T: one `TX_MUTEX`), `Z_OS_ASSERT()`.

Functions (26). "Locked" is inside `k_spin_lock()`; "ISR" includes the tick hook.

| Primitive | Meaning | F | T |
|---|---|---|---|
| `z_os_in_isr()` | true in an ISR; any context | IPSR ≠ 0 (host: the tick hook's depth) | IPSR ≠ 0 (host: `_tx_thread_system_state`) |
| `z_os_pre_kernel()` | true before the OS runs threads; any context | `xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED` | `_tx_thread_system_state >= TX_INITIALIZE_IN_PROGRESS` |
| `z_os_task_lock()`, `z_os_task_unlock()` | a thread's lock: no ISR and no other thread until unlock; nests; the wakes made inside keep it, their switch happens at the outermost unlock | `taskENTER_CRITICAL()`, `taskEXIT_CRITICAL()` (host: also suspends the scheduler) | interrupts masked, `_tx_thread_preempt_disable` raised at the outermost level; the unlock lowers it, restores the mask and calls `_tx_thread_system_preempt_check()` (a ThreadX service unmasks interrupts for an instant to switch, so the mask alone is not enough) |
| `z_os_isr_lock()`, `z_os_isr_unlock(state)` | an ISR's lock: saves and restores the mask; the state is never `UINT32_MAX` | `taskENTER_CRITICAL_FROM_ISR()`, `taskEXIT_CRITICAL_FROM_ISR()` | the port's PRIMASK save and restore (`__disable_interrupts()`, `__restore_interrupts()`) |
| `z_os_thread_self()` | the calling thread; threads only | `xTaskGetCurrentTaskHandle()` | `_tx_thread_current_ptr` |
| `z_os_thread_prio_self()` | its current priority (with inheritance); threads only | `uxTaskPriorityGet(NULL)` | from `tx_thread_priority` |
| `z_os_thread_start(fn, name, stack, size, prio, arg, tid)` | new thread on a static stack: the storage of `K_THREAD_STACK_DEFINE()` and its `K_THREAD_STACK_SIZEOF()` (a multiple of 8), the `Z_OS_THREAD_RESERVED` bytes after it the OS's for the control block, so nothing is allocated; `*tid` set before it can run; from a thread or before the scheduler; may be called again on the stack of a thread that has ended (2.6); 0, −EINVAL or another negative errno of the OS | `xTaskCreateStatic()` with the control block after the stack, `vTaskSuspendAll()` around it once the scheduler runs; 0 or −EINVAL | deletes the records of an ended thread first, makes the wake flag, `tx_thread_create(TX_DONT_START)`, sets `*tid`, `tx_thread_resume()`; gives the time slice (2.6); 0, −EINVAL, −EBUSY (the stack's thread has not ended) |
| `z_os_thread_exit()` | ends the calling thread | `vTaskDelete(NULL)` | `tx_thread_terminate()` of the caller |
| `z_os_thread_name(tid)` | the thread's writable name array (`Z_OS_THREAD_NAME_SIZE` bytes), as the OS shows it | `pcTaskGetName()` | `tx_thread_name` (the array in the storage after the stack) |
| `z_os_yield()` | next ready thread of the priority, or go on; threads, unlocked | `taskYIELD()` | `tx_thread_relinquish()` |
| `z_os_thread_local_get()`, `z_os_thread_local_set(p)` | the calling thread's one pointer (Zephyr's custom data); NULL until set; get is NULL before the OS runs threads | `pvTaskGetThreadLocalStoragePointer()`, `vTaskSetThreadLocalStoragePointer()` (index 0) | `tx_thread_zephyr_local` of `TX_THREAD_USER_EXTENSION` (cleared by `tx_thread_create()`) |
| `z_os_wait_prepare()` | clears the caller's wake flag; threads, locked | `xTaskNotifyStateClearIndexed()`, `ulTaskNotifyValueClearIndexed()` (index 1) | takes the flag's semaphore without a wait |
| `z_os_wait(ticks)` | waits for the flag or until the current tick + ticks (`Z_OS_TICKS_FOREVER`: no end), clears it; a finite wait returns only for one of the two; threads, unlocked | `ulTaskNotifyTakeIndexed(1, pdTRUE, ticks)` | `tx_semaphore_get(flag, ticks)` |
| `z_os_wake(tid)` | sets tid's flag, whether it waits yet or not (latched); threads, locked; the switch waits for the unlock | `xTaskNotifyGiveIndexed()` | `tx_semaphore_ceiling_put(flag, 1)` |
| `z_os_wake_from_isr(tid, &woken)` | the same from an ISR, locked; sets `woken` when a switch is due | `vTaskNotifyGiveIndexedFromISR()` | the same put; `woken` when tid is more urgent than the interrupted thread |
| `z_os_yield_from_isr(woken)` | switch at the end of the ISR if `woken`; unlocked | `portYIELD_FROM_ISR()` (host: nothing) | nothing (ThreadX switches at the end of the interrupt by itself) |
| `z_os_sleep(ticks)` | until the current tick + ticks; threads, unlocked | `vTaskDelay()` | `tx_thread_sleep()` |
| `z_os_suspend()` | stops the caller with no end (may return; called again) | `vTaskSuspend(NULL)` | `tx_thread_suspend()` of the caller |
| `z_os_busy_wait_us(us)` | spins at least `us` µs; any context | `busy_wait_us_32()` (host: the clock) | same (the port header's shared part) |
| `z_os_clock_us()` | µs since the start, 64 bits, monotonic; any context | `rp2040_timer_us()` (TIMER; host: `CLOCK_MONOTONIC`) | `rp2040_timer_us()` |
| `z_os_mutex_init(m)`, `z_os_mutex_lock(m, ticks)`, `z_os_mutex_unlock(m)` | the OS's recursive mutex with its own priority inheritance; all-zero is unlocked (also one zeroed after use); a waiter's timeout drops the owner's inherited priority; lock true when taken; unlock 0, −EINVAL (not held), −EPERM (other holder) | `xSemaphoreCreateRecursiveMutexStatic()` (also at the first use), `xSemaphoreTakeRecursive()`, `xSemaphoreGetMutexHolder()` + `xSemaphoreGiveRecursive()` | `tx_mutex_create(TX_INHERIT)` at the first use, taken out of ThreadX's list of created mutexes at once; `tx_mutex_get()`, on a timeout `_tx_mutex_priority_change()` of the owner; `tx_mutex_put()` after the owner's check |

The port header on the RP2040 also has, from `zephyr_os_port_rp2040.h`, the tick hook's
`z_os_tick_now_us32()` and `z_os_tick_pend()` for the catch-up of lost ticks
(`common/compat/include/tick_catchup.h`, on `Z_OS_TICK_HZ`; F's hook
`freertos/app/usb-console/src/tick.c`, T's `threadx/app/usb-console/src/tick.c`), which are not primitives
of the generic part.

### 2.6 What an OS part on the primitives meets

Besides the declarations of `zephyr_kernel_os.h`, the shared code and the acceptance test rely on these.

| Rule | Why (the user) | F | T |
|---|---|---|---|
| Only threads the layer starts (`z_os_thread_start()`) need a wake flag. Every thread that waits through the layer (`k_sem_take()`, `k_poll()`, the work queues) is started by `z_os_thread_start()`, the start-up thread "main" too (`threadx/app/usb-console/src/main.c`) | the flag has to live somewhere in the OS's record of a thread | every task has task notifications | the flag is a semaphore in the storage after the stack: a thread made another way cannot wait through the layer |
| An all-zero `k_mutex` is unlocked. A created one may be zeroed while unlocked and then be `k_mutex_init()`ed or locked as it is: the OS must hold no reference to an unlocked mutex | `mqtt_client_init()` (`common/imu/mqtt_min.c`) zeroes the client with its mutex at every connection; `K_MUTEX_DEFINE()` and settings | a static mutex created on first use | taken out of ThreadX's list of created mutexes (`_tx_mutex_created_ptr`) right after its creation; never deleted |
| A waiter's timeout drops the owner's inherited priority (Zephyr's `kernel/mutex.c`) | the acceptance test's `mutex_inherit` checks it | FreeRTOS does it | ThreadX keeps it until the release, so `z_os_mutex_lock()` drops it with `_tx_mutex_priority_change()` |
| `APP_PRIO_FROM_ZEPHYR(p)` of `priorities_os.h` and the OS's own threads' priorities are in the order of `z_os_prio_t` (larger more urgent). Where the OS numbers the other way, `z_os_thread_start()` and `z_os_thread_prio_self()` convert. The value 0 stands for Zephyr's idle and is not given to a thread of the layer | the layer orders waiters by priority | 20 − p (FreeRTOS's numbers go the same way) | 15 − p; ThreadX's number is 31 − the value (= p + 16, 0 most urgent) |
| Threads of Zephyr priority 0 and below in urgency have 20 ms time slices (the Zephyr build's `CONFIG_TIMESLICE_SIZE=20`, `CONFIG_TIMESLICE_PRIORITY=0`). The generic part makes no time slices: the OS part provides them | the acceptance test's `timeslice` checks it | the tick hook (`tick.c`) | `z_os_thread_start()` gives ThreadX's `time_slice` 20 (`APP_TIMESLICE_TICKS`, `APP_TIMESLICE_PRIO_MAX` of `priorities_os.h`) |
| `z_os_thread_start()` may be called again on the static stack of a thread that has ended; an OS that keeps records of that thread removes them first. It refuses the stack of a thread that runs | the acceptance test runs twice and starts threads again on the same stacks | `vTaskDelete()` removes the records | an ended thread (`TX_COMPLETED`, `TX_TERMINATED`) and its flag are `tx_thread_delete()`d and `tx_semaphore_delete()`d first; −EBUSY if it has not ended |
| Objects and threads can be made before the start (from the kernel's set-up to the scheduler's start) | `usb_console_init()` starts the layer before the scheduler, and the OS's `main()` makes the thread "main" | made as they are | after `_tx_initialize_kernel_setup()`; `tx_kernel_enter()` skips the initialization done |
| The tick hook calls `zephyr_compat_tick()` once per tick and catches up lost ticks (`tick_catchup.h`) | the layer's timeouts | `vApplicationTickHook()` | `isr_systick` (after ThreadX's `_tx_timer_interrupt()`) |

## 3. Other declared boundaries

| Boundary | Declared in | Items | Called by | N | F | T |
|---|---|---|---|---|---|---|
| Generic shell commands | `common/shell/kernel_shell_os.h` (reads `"kernel_shell_os_impl.h"`; static inline) | `kernel_shell_os_tick_count()` (the OS's tick count, for `uptime`), `kernel_shell_os_reboot(delay_ms)`, `kernel_shell_os_bootsel(delay_ms)` (restart, into the USB loader for the second, `delay_ms` later; do not return), `kernel_shell_os_fault_last()` (the last run's `struct fault_record` or NULL); the rest is the layer's (`k_cycle_get_64()`, `k_sleep()`, `Z_USEC_PER_TICK`, `sys_clock_gettime()`, `hwinfo_get_device_id()`) | `common/shell/kernel_shell.c` | — (not built; nsh's commands, `nuttx/apps/bootsel`) | `freertos/app/usb-console/src/kernel_shell_os_impl.h`: `xTaskGetTickCount()`, the Pico SDK's `watchdog_reboot()` and `reset_usb_boot()`, `fault_last()` (`common/pico_sdk/app/usb-console/src/fault.c`); host `freertos/tests/shell/kernel_shell_os_impl.h` | `threadx/app/usb-console/src/kernel_shell_os_impl.h`: `tx_time_get()`, `tx_thread_sleep()`, the rest as F |
| Shell backend | `common/shell/zshell_os.h` | `zshell_os_backend_get()`, `zshell_os_cmd_enter()`, `zshell_os_cmd_exit()`, `z_shell_backend_vfprintf()`; optional `zshell_os_active_cmd_set()`/`_get()` with `ZSHELL_OS_ACTIVE_CMD` | `common/shell/zshell.c` | `nuttx/apps/subcmd/zshell_os_impl.h` (no `ZSHELL_OS_ACTIVE_CMD`) | the shell engine's `common/compat/port/generic/zshell_os_impl.h` (defines it; on the consoles) | same |
| Console transport | `common/compat/port/shell_serial_os.h` (reads `"shell_serial_os_impl.h"`; static inline) | `shell_serial_os_stdio_init()` (every thread's stdout and stderr to `shell_console_write(shell_console_current(), ...)`, unbuffered), `shell_serial_os_rx_init()` (call `shell_serial_rx_ready()`, any context, when input arrives), `shell_serial_os_connected()`, `shell_serial_os_read()` (no wait), `shell_serial_os_write()` (drops what the terminal cannot take) | `common/compat/shell_engine/shell_serial.c` | — (not built) | `common/pico_sdk/drivers/shell/shell_serial_os_impl.h`: `stdio_usb`, `shell_stdio_init()` (`shell_stdio.c`) with `common/pico_sdk/drivers/libc/stdio_wrap.c`; host `freertos/tests/shell/shell_serial_os_impl.h` | same (the owner of `stdio_usb`'s mutex is told apart per thread: "The Pico SDK's locks" below) |
| `struct shell` | read by `common/compat/include/zephyr/shell/shell.h` | one backend, outlives the command | shell commands | `nuttx/drivers/compat/zephyr_os_shell.h` (terminal path, command pid) | the shell engine's `common/compat/port/generic/zephyr_os_shell.h` (console, active command) | same |
| Log, OS part | `common/compat/include/app_log.h` | `app_log_os.h` (OS headers first, optional `CONFIG_LOG_DEFAULT_LEVEL`, `CONFIG_LOG_MAX_LEVEL`, and `LOG_RAM_SIZE`, the RAM log's ring in bytes, read by `log_ring.h` and `log_ram.h`: N 8192, F and T 4096; without it they do not compile); `app_log_printf()`, `app_log_vprintf()`, `app_log_ratelimit_pass()` | `LOG_*()` macros | `nuttx/drivers/include/app_log_os.h`; the functions shared: `common/compat/log/log_ram.c` | `common/compat/port/generic/app_log_os.h`; shared: same | same |
| RAM log | `common/compat/include/log_ram.h` | `log_ram_initialize()`, `log_ram_store()` (the OS's own log, raw bytes), `log_ram_piece()` | start-up; the OS's log; `dmesg` | shared `common/compat/log/log_ram.c`; called by `N-board/src/rp2040_boardinitialize.c`, `nuttx/drivers/log/log_ram_syslog.c`, `nuttx/apps/dmesg/dmesg_main.c` | shared: same; called by `main()`, `common/shell/dmesg_shell.c` (`log_ram_store()` unused) | same |
| RAM log, OS part | `common/compat/port/log_ram_os.h` (`log_ram.c` reads `"log_ram_os_impl.h"` first) | `LOG_RAM_OS_RETAINED(name)` (the ring in RAM the start-up code leaves alone), type `log_ram_os_irqstate_t`; inline `log_ram_os_irq_save()`, `_irq_restore()` (the lock: interrupts masked, any context), `_uptime_us()` (time stamp and rate limit), `_attach()` (make it the OS's own log) | `common/compat/log/log_ram.c` | `nuttx/drivers/include/log_ram_os_impl.h`: `.noinit`, `up_irq_save()`, `rp2040_timer_us()`, registers the syslog channel (`nuttx/drivers/log/log_ram_syslog.c`) | `common/pico_sdk/drivers/include/log_ram_os_impl.h`: `__uninitialized_ram`, `save_and_disable_interrupts()`, `rp2040_timer_us()`, nothing to attach; host `common/tests/log_ram_os_host/` | same |
| KV flash | `common/compat/port/flash_kv_os.h` (the area's constants and the checks in `flash_kv.h`) | `flash_kv_os_initialize()` (once, by the start-up), `flash_kv_os_ready()`, `_read()`, `_program()` (256-byte page), `_erase()` (4 KiB sector); range checked again by the OS | `common/compat/zephyr_flash_kv.c`; the start-up | shared on the RP2040: `common/chip/rp2040/rp2040_flash_kv.c` (next row); called by `rp2040_bringup.c` | same; called by `main()` | same |
| KV flash operations on the RP2040 | `common/chip/rp2040/include/rp2040_flash_kv_os.h`; the linker script's `__kv_start`, `__kv_end` | `rp2040_flash_kv_os_init()` (once, after the linker script's check), `_image_end()` (the XIP address where the image ends), `_program(offset, page, lo, hi)` (256-byte page in RAM), `_erase(offset, lo, hi)` (4 KiB sector): XIP off and interrupts disabled, the request checked against `[lo, hi)` once more right before the flash is touched | `common/chip/rp2040/rp2040_flash_kv.c` | `N-board/src/rp2040_flash_kv_os.c` (the boot ROM, a RAM function; NuttX's `rp2040_flash_mtd.c` is not used) | `common/pico_sdk/drivers/kv/rp2040_flash_kv_os.c` (the Pico SDK's `flash_range_*()` in `flash_safe_execute()`) | same |
| KV wipe command | `common/compat/include/kv_store_os.h` | `kv_store_wipe()`, `kv_store_cmd_wipe()`, `KV_STORE_OS_COMMANDS` | `common/imu/kv_store.c` | shared: `common/compat/kv_store_wipe.c` | shared: same | same |
| GPIO bank 0 | `common/compat/port/gpio_os.h` | OS: `gpio_os_configure_input()`, `gpio_os_get_all()`, `gpio_os_irq_configure()`; layer: `gpio_os_irq_handler()` (called from the bank interrupt) | `common/compat/zephyr_gpio.c`; N also `common/drivers/nina/port/nuttx/nina_os.c` | `N-board/src/rp2040_gpio_os.c` | `common/pico_sdk/drivers/compat/rp2040/gpio_os_rp2040.c` | same |
| CPU load | `common/compat/port/cpu_load.h` (with the arithmetic) | `cpu_load_get(reset)` in tenths of a percent, from idle windows of the TIMER | `common/imu/imu_stream.c` | `N-board/src/rp2040_idle_cpuload.c` | the idle task's hook (`idle_os.h`) on the shared `common/pico_sdk/app/usb-console/src/idle.c` | the hooks of the scheduler's wait (`idle_os.h`) on the same `idle.c` |
| Priorities and stacks | `common/compat/include/priorities.h` | OS: `APP_PRIO_FROM_ZEPHYR(p)` and its own threads (larger more urgent with the generic kernel part, 2.6); shared: 15 `APP_PRIO_*`/`APP_STACK_*` pairs | the layer, bindings, start-up | `nuttx/drivers/include/priorities_os.h` (118 − p; `NINA_EVENT`, `NINA_PT`) | `freertos/app/usb-console/src/priorities_os.h` (20 − p; `TIMER_SVC`, `IDLE`; the layer's `SYSWORKQ`, `NET_MGMT`, `SENSOR_SHELL` and `TELNET_SERVICE` stacks at measured sizes) | `threadx/app/usb-console/src/priorities_os.h` (15 − p; no thread of the OS's own; the time slices' `APP_TIMESLICE_TICKS`, `APP_TIMESLICE_PRIO_MAX`; the layer's thread stacks at F's values) |
| net_mgmt thread | `common/compat/include/zephyr/net/net_mgmt.h` | `net_mgmt_event_init()` starts "net_mgmt" with `z_thread_static_start()`; no OS file | N `common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`; F and T `common/compat/net_offload/zephyr_net_init.c` | shared `common/compat/zephyr_net_mgmt.c` | same | same |
| RTIO work queue | `common/compat/include/zephyr/rtio/work.h` | `rtio_workq_initialize()` starts "RTIO WQ 0" with `z_thread_static_start()`; no OS file | `zephyr_compat_initialize()` | shared `common/compat/zephyr_rtio_workq.c` | same | same |
| Wi-Fi profile | included by `common/imu/wifi_profile.c` | `extern k_tid_t wifi_reconnect_tid;` | `common/imu/wifi_profile.c` | `nuttx/apps/wifi/wifi_profile_os.h` | `common/compat/port/generic/wifi_profile_os.h` | same |
| Sensor shell | included first by `common/shell/sensor_shell.c`, `common/compat/sensor_shell_stream.c` | `CONFIG_SENSOR_SHELL_STREAM`, `_THREAD_STACK_SIZE`, `_TRIG_PRINT_TIMEOUT_MS`, `_MAX_TRIGGER_DEVICES`; `sensor_shell_os_get_arm()`, `sensor_shell_os_get_wait()`, `sensor_shell_os_processing_callback` | those two files | `nuttx/apps/sensor/sensor_shell_os.h`, `sensor_shell_nuttx.h`, `sensor_shell_nuttx.c` | `common/shell/port/generic/sensor_shell_os.h`, `sensor_shell_os.c` | same |
| Start of the layer | `common/compat/include/zephyr_compat.h` | `zephyr_compat_initialize()`, `K_THREAD_DEFINE_START(name)` | start-up; `K_THREAD_DEFINE_START()`: `common/imu/app_init.c` | shared `common/compat/zephyr_init.c` (section 2) | same | same |
| Host test of the layer's kernel, OS part | `common/tests/zephyr_kernel/zephyr_kernel_test.h` (`zephyr_kernel_test.c` reads `"zephyr_kernel_test_os.h"` after `<zephyr/kernel.h>`) | type `zephyr_kernel_test_os_prio_t`, `ZEPHYR_KERNEL_TEST_OS_NAME_MAX`, `zephyr_kernel_test_os_tick()`, `_tick_sync()`, `_thread_create()`, `_thread_exit()`, `_prio_get()`, `_prio_self()`, `_name_get()`, `_current()`, `_in_isr()`, `_sched_lock()`, `_sched_unlock()` | `common/tests/zephyr_kernel/zephyr_kernel_test.c` | — (not run) | on the primitives `common/tests/zephyr_kernel/kernel_generic/zephyr_kernel_test_os.h` (`_tick_sync()`, `_thread_exit()`, `_prio_self()`, `_name_get()`, `_in_isr()`, the type and the name length); the rest `freertos/tests/compat/zephyr_kernel_test_os_impl.h` (FreeRTOS tasks) | the same `kernel_generic/zephyr_kernel_test_os.h`; the rest `threadx/tests/compat/zephyr_kernel_test_os_impl.h` (ThreadX threads; the board test firmware reads the same file) |
| Host test of the layer's kernel, platform part | same | `zephyr_kernel_test_port_alarm()`, `_quiet()`, `_realtime()`, `_mask_irqs()` (a platform without an interrupt to raise, a tick that follows real time or interrupts to mask answers -ENOTSUP or false, and those checks are skipped) | same | — | `freertos/tests/compat/main.c` (the FreeRTOS POSIX port: the interrupt, real-time and interrupt-mask checks skip) | `threadx/tests/compat/main.c` (ThreadX's Linux port: the same three skip); on the board `threadx/tests/board_kernel/test_port.c` (a TIMER alarm's interrupt; nothing skips) |
| Start on the Pico SDK | `common/pico_sdk/app/usb-console/src/usb_console.h` | called by the OS's `main()` in this order (4.6): `usb_console_early_init()` (first; interrupts enabled, before any kernel object; in the passthrough mode it runs the mode and does not return); `usb_console_init()` (after the kernel's set-up, before the thread "main" is made: the USB stdio, the consoles, the layer, I2C, the key-value area's bounds; needs no interrupt and no time to pass); `usb_console_main_thread(prompt)` (the body of the thread "main"; returns when done and the OS ends the thread); `usb_console_shell_start(prompt)` (after the thread "main" is made, right before the scheduler: the thread "console" and the LED). The thread "main" is static, at `APP_PRIO_MAIN` with `APP_STACK_MAIN` bytes; its stack and the OS's storage after it are in SCRATCH_X, not loaded, which keeps what the last run left: an OS that keeps records there clears them before it makes the thread | the OS's `main()` | — | `freertos/app/usb-console/src/main.c`: the prompt, `xTaskCreateStatic()` (stack in SCRATCH_X) and `vTaskDelete(NULL)`, the tick's check and `vTaskStartScheduler()` | `threadx/app/usb-console/src/main.c`: the prompt, `_tx_initialize_kernel_setup()` (interrupts masked from here) and the stack check's handler, the OS's storage zeroed before `z_os_thread_start()`, `z_os_thread_exit()`, the tick's check and `tx_kernel_enter()`, an empty `tx_application_define()` |
| Idle on the Pico SDK | `common/pico_sdk/app/usb-console/src/idle.c` (reads `"idle_os.h"`) | one of two shapes. `IDLE_OS_ENTER`, `IDLE_OS_EXIT`: the names of the two halves `idle.c` defines (open the window and blink the LED; close the window), for an OS whose own idle wait masks interrupts, calls the first, waits with `WFI`, calls the second and unmasks (any mode and stack, interrupts masked). `IDLE_OS_HOOK`: the name of the whole pass (mask interrupts, open the window, `WFI`, close the window, unmask), which the OS calls again and again at the lowest priority, in thread mode with interrupts enabled, when no other thread is ready | the OS's idle | — | `freertos/app/usb-console/src/idle_os.h`: `IDLE_OS_HOOK` as `vApplicationIdleHook` | `threadx/app/usb-console/src/idle_os.h`: `IDLE_OS_ENTER`, `IDLE_OS_EXIT` as `tx_low_power_enter`, `tx_low_power_exit` (`TX_LOW_POWER`, `TX_ENABLE_WFI`; the scheduler's wait inside PendSV) |
| Fatal errors on the Pico SDK | `common/pico_sdk/app/usb-console/src/fault.h` | the OS writes `fault_os_thread_name(name, size)` (the running thread's name, only from RAM, nothing when no thread runs; interrupts off, any context) and its own fatal error entries (functions the kernel calls). The shared functions an entry calls: `fault_begin(reason, pc)` (disables interrupts and starts the record; NULL inside the exit itself), then, with the record filled, `fault_finish(r)` (the thread's name, the time, the seal, the scratch registers, the restart; does not return); both from any context, also before the scheduler. The RAM's bounds `FAULT_RAM_START`, `FAULT_RAM_END` | `fault_finish()` (the thread's name), the kernel (the entries) | — | `freertos/app/usb-console/src/fault_os.c`: the task's name from its control block, `vApplicationStackOverflowHook()` | `threadx/app/usb-console/src/fault_os.c`: the thread's name from its control block, the handler of ThreadX's stack check `fault_os_stack_error()` (registered by `main()`) |
| Commands on the Pico SDK | `common/pico_sdk/app/usb-console/src/commands_os.h` | `commands_os_kernel_name()`, `commands_os_kernel_version()` (`version`: "<name> <version>, Pico SDK <x>, GCC <y>"); `commands_os_ps(sh, argc, argv)`, the handler of `ps` (one row per thread, `(interrupts)`, `cpu_active` since the previous `ps`); all from a shell's thread (two at once too). `commands.c` registers the commands and holds the table | `common/pico_sdk/app/usb-console/src/commands.c` | — | `freertos/app/usb-console/src/commands_os.c`: `tskKERNEL_VERSION_NUMBER`, `uxTaskGetSystemState()`, the run time clock `app_run_time_us()` (declared by `FreeRTOSConfig.h`) | `threadx/app/usb-console/src/commands_os.c`: `THREADX_*_VERSION` of `tx_api.h`, ThreadX's list of created threads and the execution profile kit's run times (`src/run_time.[ch]`) |
| The Pico SDK's locks and sleeps | the Pico SDK's `pico/lock_core.h` (the OS's header given to every source as `PICO_CONFIG_RTOS_ADAPTER_HEADER`) | `lock_owner_id_t`, `lock_get_caller_owner_id()` (the owner of a mutex is the thread; the SDK's default is the core number), `lock_internal_spin_unlock_with_wait()`, `_with_notify()`, `_with_best_effort_wait_or_timeout()` (a thread waits in the OS, lower priorities run) | the Pico SDK's `mutex_*`, `sem_*`, `sleep_ms()`, `stdio_usb`, the `malloc()` mutex | — | FreeRTOS's RP2040 port (`configSUPPORT_PICO_SYNC_INTEROP`) | `threadx/drivers/pico_sdk_interop/` (the same design; one `TX_EVENT_FLAGS_GROUP`, the bit of a lock its spin lock's number) |
| The NINA's SPI DMA waits on the Pico SDK | `common/drivers/nina/port/pico_sdk/nina_bus_dma_os.h` (reads `"nina_bus_dma_os_impl.h"`; static inline) | types `nina_bus_dma_os_thread_t` (NULL none), `nina_bus_dma_os_woken_t` (`NINA_BUS_DMA_OS_NOT_WOKEN`), `nina_bus_dma_os_ticks_t`; threads with `nina_os_lock()` held: `nina_bus_dma_os_running()`, `_self()`, `_wake_clear()`, `_ticks_from_ms(ms)` (rounded up, plus one), `_tick_count()`, `_wake_wait(ticks)`; the DMA interrupt: `_wake_from_isr(thread, &woken)`, `_isr_end(woken)`. The wake-up may be the calling thread's own (shared with `nina_os.h`'s waits) or an object of the binding's (one transfer at a time, so one serves) | `common/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c` | — (N's binding has its own DMA) | `common/drivers/nina/port/freertos/nina_bus_dma_os_impl.h`: the task notification index 2 (shared with `nina_os.h`'s waits; always inline) | `common/drivers/nina/port/threadx/nina_bus_dma_os_impl.h`: one binary `TX_SEMAPHORE` of the transfers (always inline) |
| Tick catch-up for a periodic tick | `common/compat/include/tick_catchup.h` (on `Z_OS_TICK_HZ`) | `struct tick_catchup`, `tick_catchup_step(s, now)` (once per tick counted, with the TIMER's low 32 bits; true: pend one more tick, `z_os_tick_pend()`) | the OS's tick hook (with `z_os_tick_now_us32()`) | — (tickless) | `freertos/app/usb-console/src/tick.c` | `threadx/app/usb-console/src/tick.c` (SysTick is above PendSV, so the added ticks run one after another before the switch) |
| Tick of the generic kernel part | `common/compat/include/zephyr_compat_os.h` | `zephyr_compat_tick()`: once per tick, from the tick interrupt | F's and T's tick hooks | — | `common/compat/kernel_generic/zephyr_kernel.c` | same |
| Start of the network part for an OS without a network stack | `common/compat/net_offload/include/zephyr_net_init.h` | `zephyr_net_initialize()` | F's and T's `usb_console_main_thread()` | — (`nina_net_initialize()`) | shared `common/compat/net_offload/zephyr_net_init.c` | same |
| Start of the application | `common/imu/app_main.h` | `app_init_post_kernel()`, `app_threads_start()`, `app_main()`, called in this order (section 4.6); the modules' options (`CONFIG_KV_STORE`, `CONFIG_APP_IMU`, `CONFIG_IMU_RAW_MQTT`, `CONFIG_TIME_SYNC`, `CONFIG_WIFI_CREDENTIALS`, `CONFIG_WIFI_PROFILE`, `CONFIG_APP_SENSOR`, `CONFIG_WIFI_NINA_LOCAL`, `CONFIG_NET_ZPERF`), each defined by a build that builds the module; `CONFIG_APP_IMU` always uses the API of the MQTT transport and of the time synchronization, so without `CONFIG_IMU_RAW_MQTT` or `CONFIG_TIME_SYNC` a build compiles `common/imu/imu_raw_mqtt_null.c` or `time_sync_null.c` in their place (section 7) | the start-up file of each build (Zephyr: `app_main()` only) | shared `common/imu/app_main.c`, `app_init.c`; the options from Kconfig; called by `nuttx/apps/init/rp2040_init.c` | shared: same; the options from the modules' `.cmake` files; called by `usb_console_main_thread()` | as F |

The Zephyr build has its own `zephyr/app/usb-console/src/kv_store_os.h` and `wifi_profile_os.h`.

## 4. What each build supplies without a shared declaration

### 4.1 `<zephyr/...>` headers supplied per OS

The OS directory comes first on the include path (for F and T, `common/compat/port/generic` next); `common/compat/include/zephyr/` has the rest. F's and T's
network headers are in `common/compat/net_offload/include`, which F and T put after the OS directory and
before `common/compat/include`; N does not have it on its include path. T has no `<zephyr/...>` header of its
own and uses all of the F column's.

| Header | N | F, T |
|---|---|---|
| `zephyr/drivers/i2c.h` | `nuttx/drivers/compat/zephyr/drivers/i2c.h` on `struct i2c_master_s` (`zephyr_i2c.c`) | `common/compat/port/generic/zephyr/drivers/i2c.h` (Zephyr's API table) |
| `zephyr/net/net_if.h` | on `struct net_driver_s` (`zephyr_net_if.c`) | `common/compat/net_offload`: the one NINA interface (`zephyr_net_if.c`) |
| `zephyr/net/net_ip.h` | — (uses `common/compat/include`'s) | `common/compat/net_offload/include`, replaces `common/compat/include`'s: `net_*` names on POSIX types |
| `zephyr/net/socket.h` | `zsock_*()` are NuttX's `socket()`, `poll()`, ... (usrsock) | `common/compat/net_offload`: Zephyr's socket layer, `subsys/net/lib/sockets/sockets.c`, `socket_offload.c`, `getaddrinfo.c` |
| `zephyr/net/socket_offload.h` | — | `common/compat/net_offload`: offloaded resolver |
| `zephyr/sys/fdtable.h` | — | `common/compat/net_offload`: descriptor table, `lib/os/zvfs/zvfs_fdtable.c`, `zvfs_poll.c` |
| `zephyr/sys/__assert.h` | `DEBUGASSERT()` | `common/compat/port/generic/zephyr/sys/__assert.h`: nothing (as `CONFIG_ASSERT` off) |
| `zephyr/sys/clock.h` | inline on `clock_gettime()`/`clock_settime()` | `common/compat/port/generic/zephyr/sys/clock.h`, implemented by `common/pico_sdk/drivers/compat/zephyr_sys_clock.c` (TIMER + base) |

### 4.2 Functions and objects

| Item | Where declared | N | F | T |
|---|---|---|---|---|
| `rp2040_timer_us()`: µs since boot, TIMER, any context | `common/chip/rp2040/include/rp2040_timer_us.h` (shared; also the Zephyr build's `common/imu/time_sync.c`) | read by `nuttx/drivers/compat/zephyr_os.h`, `nuttx/drivers/include/log_ram_os_impl.h` and the board | read by the port header `freertos/drivers/compat/rp2040/zephyr_os_port.h`, `common/pico_sdk/drivers/include/log_ram_os_impl.h`, `zephyr_sys_clock.c`, `idle.c`, `fault.c` | the same shared files, and the port header `threadx/drivers/compat/rp2040/zephyr_os_port.h`, `tx_user.h` (the execution profile kit's clock), `src/run_time.h` |
| `rp2040_watchdog_reboot()`: restart through the watchdog, scratch 0..3 kept | `common/chip/rp2040/include/rp2040_watchdog_reboot.h` (shared; also the Zephyr build's `common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c`) | `N-board/src/rp2040_restart.c` (fallback of `rp2040_reset_usb_boot()`), `common/drivers/nina/port/nuttx/nina_passthrough_bus_rp2040.c` | not used: the Pico SDK's `watchdog_reboot()` | same |
| `k_heap` | `common/compat/include/zephyr_heap.h`, included by `<zephyr/kernel.h>` with either kernel part | not built (no user) | `common/compat/zephyr_heap.c`; used by `common/drivers/nina/port/zephyr/nina_socket.c` | same |
| `zephyr_compat_tick()` | `common/compat/include/zephyr_compat_os.h` (generic kernel part) | — (wdog) | called by `vApplicationTickHook()`, `freertos/app/usb-console/src/tick.c` | called by `isr_systick()`, `threadx/app/usb-console/src/tick.c` |
| `zephyr_net_initialize()` | `common/compat/net_offload/include/zephyr_net_init.h` | — (`nina_net_initialize()` in `rp2040_bringup.c`, `common/drivers/nina/port/nuttx/nina_net.h`) | `common/compat/net_offload/zephyr_net_init.c` | same |
| `hwinfo_get_device_id()` | `common/compat/include/zephyr/drivers/hwinfo.h` | `nuttx/drivers/compat/zephyr_hwinfo.c` | `common/pico_sdk/drivers/compat/rp2040/zephyr_hwinfo.c` | same |

### 4.3 Devices behind `DEVICE_DT_GET()`

`DEVICE_DT_GET(DT_NODELABEL(x))` is `&__device_x` (`common/compat/include/zephyr/device.h`); `DEVICE_DT_DEFINE()` makes it.
The devices listed for the shell are in `common/compat/zephyr_device.c` (hand-written). T uses the same
shared files as F.

| Node | N | F, T |
|---|---|---|
| `gpio0` | `common/compat/zephyr_gpio.c` | same |
| `i2c0` | `N-board/src/rp2040_lsm6dso.c`, bound to the NuttX I2C master (`zephyr_i2c.c`) | `common/drivers/i2c_dw/port/pico_sdk/i2c_dw_instance.h` |
| `i2c1` | no Zephyr device (`struct i2c_master_s` only) | `common/drivers/i2c_dw/port/pico_sdk/i2c_dw_instance.h` |
| `lsm6dsox` | `common/drivers/lsm6dso/port/nuttx/lsm6dso_instance.h` (read last by `port/zephyr/lsm6dso_sensor.c`) | `common/drivers/lsm6dso/port/pico_sdk/lsm6dso_instance.h` (same) |
| flash device | `common/compat/zephyr_flash_kv.c` (`z_flash_kv_device`) | same |

### 4.4 Shell commands: registration, getopt, stdio

T uses the same shared files as F.

| Point | N | F, T |
|---|---|---|
| Registration | each command a builtin application; `SHELL_CMD_REGISTER()` makes `main()` (or `z_shell_main_<name>()` with `Z_SHELL_CMD_NAMED_MAIN`) | `Z_SHELL_CMD_ROOT_GLOBAL`: global root entries listed in `shell_engine_cmds[]`, `common/pico_sdk/app/usb-console/src/commands.c`; run by `common/compat/shell_engine/shell_engine.c` |
| `SHELL_SUBCMD_ADD()` | global entry; the parent's source lists them by hand | same |
| getopt | Zephyr's getopt not built; `nuttx/apps/wifi/wifi_credentials_shell.c` maps `sys_getopt_*` to `getopt_long()` | copies in `common/compat/lib/utils/getopt`; state per session `common/compat/shell_engine/getopt_state.c` (`sys_getopt_state_get()`, `z_getopt_global_state_update()`), reset by `sys_getopt_init()` before each command |
| stdio of `zshell.c`: `printf()`, `fprintf(stderr)`, `fflush(stdout)` must reach the command's terminal; `calloc()` of about 1.2 KB per line | each command a task with its own stdio | the console transport's stdio redirection (section 3): `common/pico_sdk/drivers/shell/shell_stdio.c` (the calling thread's console), `common/pico_sdk/drivers/libc/stdio_wrap.c` (`--wrap` of the stdout/stderr calls) |

### 4.5 `CONFIG_*` values

T takes them from the same shared files as F.

| Source | N | F, T |
|---|---|---|
| Prelude, `-include` on shared sources | `nuttx/drivers/compat/common_prelude.h`: NuttX headers first; `CONFIG_SENSOR_LOG_LEVEL` 3; `CONFIG_SENSOR_ASYNC_API` with `CONFIG_LSM6DSO_STREAM` | `common/compat/port/generic/common_prelude.h`: `CONFIG_SENSOR_LOG_LEVEL` 3; `CONFIG_NINA_NTP_SERVER` "", `_PORT` 123, `_INTERVAL_MS` 2000; `CONFIG_NINA_TCP_MAX_IO` 3584; `CONFIG_NINA_SEND_RETRY_MS` 1; `CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY` 40; `CONFIG_NINA_PASSTHROUGH` 1; `CONFIG_SENSOR_ASYNC_API` with the stream |
| NINA and NTP options | Kconfig: `nuttx/drivers/nina/Kconfig`, `nuttx/apps/imu/Kconfig` | the prelude (above) |
| NVS and settings copies | `common/compat/include/zephyr_kv_config.h`, `-include` from `nuttx/apps/imu/Makefile` | same, from `common/pico_sdk/drivers/kv/kv.cmake` |
| Build definitions | Kconfig (`nuttx/drivers/*/Kconfig`, `nuttx/apps/*/Kconfig`, `N-board/Kconfig`), `N-board/configs/imu/defconfig` | `compat.cmake`: `Z_SHELL_CMD_ROOT_GLOBAL`, `CONFIG_GETOPT_LONG=1`, `PICO_STDOUT_MUTEX=0`; `i2c_dw.cmake`: `CONFIG_I2C_DW_I2C0`, `_I2C1`; `lsm6dso.cmake`: `CONFIG_LSM6DSO_STREAM`; `wifi.cmake`: `CONFIG_WIFI_CREDENTIALS`, `CONFIG_WIFI_PROFILE`; `nina.cmake`: `NINA_OS_DNS_OFFLOAD` for the resolver, `CONFIG_WIFI_NINA_LOCAL`; `i2c_shell.cmake`: `CONFIG_I2C_SHELL_BUFFER_SIZE`, `CONFIG_LOG_DEFAULT_LEVEL` (for `i2c_shell.c` only; N: Kconfig); the modules' options of the application's start (section 4.6): `kv.cmake` `CONFIG_KV_STORE`, `imu.cmake` `CONFIG_APP_IMU`, `CONFIG_IMU_RAW_MQTT`, `CONFIG_TIME_SYNC`, `sensor_shell.cmake` `CONFIG_APP_SENSOR`, `zperf.cmake` `CONFIG_NET_ZPERF` |
| In shared headers | `kernel.h`: `CONFIG_THREAD_NAME` 1; `zephyr/rtio/work.h`: `CONFIG_RTIO_WORKQ_POOL_ITEMS` 4 | same |
| Sensor shell | `sensor_shell_nuttx.h` | `sensor_shell_os.h` (same four values) |

### 4.6 Threads and start-up order

The application's start is one shared sequence, `common/imu/app_main.h`, in the order of the Zephyr
build's kernel and `main()`:

1. `app_init_post_kernel()` (`app_init.c`): the application's part of `SYS_INIT(POST_KERNEL)`,
   `wifi_credentials_init()` (`CONFIG_WIFI_CREDENTIALS`; mounts NVS, reads the stored SSIDs);
2. the OS's `SYS_INIT(APPLICATION)` parts of the shell (F and T: `wifi_shell_init()`, `shell_telnet_init()`);
3. `app_threads_start()` (`app_init.c`): `z_init_static_threads()`, the one table of the threads below,
   each started with `K_THREAD_DEFINE_START()` after the one before it, a failure logged
   (`Could not start <name>: <errno>`);
4. `app_main()` (`app_main.c`): `main()`: `USB console started`, `kv_store_init()`,
   `imu_cal_init()`, `time_sync_init()`, `wifi_profile_init()`.

The Zephyr build calls only `app_main()` from its `main()`; its kernel does 1 to 3 (`SYS_INIT()`,
`K_THREAD_DEFINE()`). N, F and T call 1, 3 and 4 from a thread, after their devices; the Zephyr build does
not compile `app_init.c`. Each entry is under the option of its module; `common/tests/app_threads.py`
(run by `common/tests/run.sh`) checks that the table names every `K_THREAD_DEFINE()` of the sources
once and that each build sets the option exactly when it compiles the defining source.

| Thread (defined in), in the table's order | Option | N | F, T |
|---|---|---|---|
| `nina_wifi_tid` (`common/drivers/nina/port/zephyr/nina_net.c`) | `CONFIG_WIFI_NINA_LOCAL` | not built; in its place `nina_net_start()` (`CONFIG_NINA_NET`) starts the core's thread | started |
| `imu_stream_tid` (`common/imu/imu_stream.c`) | `CONFIG_APP_IMU` | started | same |
| `imu_raw_mqtt_tid` (`common/imu/imu_raw_mqtt.c`) | `CONFIG_IMU_RAW_MQTT` | same | same |
| `time_sync_tid` (`common/imu/time_sync.c`) | `CONFIG_TIME_SYNC` | same | same |
| `wifi_reconnect_tid` (`common/imu/wifi_profile.c`) | `CONFIG_WIFI_PROFILE` | same | same |
| `sensor_shell_tid` (`common/compat/sensor_shell_stream.c`) | `CONFIG_APP_SENSOR` | same | same |
| `zperf_server_tid` (`common/compat/zperf_iperf3.c`) | `CONFIG_NET_ZPERF` | not built | same |

#### The entry of each build

In all four builds the entry is in the OS's directory and calls shared functions. The numbers 1 to 4 below
are the steps of the application's start above.

**Zephyr build**: the entry is `zephyr/app/usb-console/src/main.c`.

- The kernel initializes the devices (I2C0, the LSM6DSOX, the NINA's network) and the `SYS_INIT()`s (1, 2),
  and starts the threads (3).
- `main()` in the thread "main": in the passthrough mode (the path to rewrite the NINA's firmware from the
  PC), `nina_passthrough_run()` (no return).
- Then `app_main()` (4).

**NuttX build**: the entries are `N-board/src/rp2040_boardinitialize.c`, `N-board/src/rp2040_bringup.c`,
`nuttx/apps/init/rp2040_init.c`.

- `rp2040_boardinitialize()`: `log_ram_initialize()`, `nina_passthrough_check()`.
- `rp2040_bringup()`: `zephyr_compat_initialize()`, `i2c_dw_i2c0_initialize()` and `i2c_register()`, the
  LSM6DSOX, `nina_net_initialize()`, `flash_kv_os_initialize()`.
- The init task: in the passthrough mode `nina_passthrough_run()`; `nsh_initialize()`, 1, 3, Telnet
  (`rp2040_telnet_start()`), 4.

**FreeRTOS build**: the entry is `main()` in `freertos/app/usb-console/src/main.c`.

- `usb_console_early_init()`: the interrupt stack's paint, the RAM log, the passthrough check, the last
  run's fatal error record. The passthrough mode branches off here.
- No kernel set-up is needed.
- `usb_console_init()`: the USB stdio, the console "usb", the layer, I2C0 (and I2C1), the key-value area's
  bounds.
- `xTaskCreateStatic()`: the thread "main", whose body is `usb_console_main_thread()`.
- `usb_console_shell_start()`: the thread "console" (the shell on USB), the LED.
- The tick's check, `vTaskStartScheduler()`.
- The thread "main": `usb_console_main_thread()` runs the LSM6DSOX, `zephyr_net_initialize()`, 1, 2
  (`wifi_shell_init()`, `shell_telnet_init()`), 3, 4; then `vTaskDelete(NULL)`.

**ThreadX build**: the entry is `main()` in `threadx/app/usb-console/src/main.c`.

- `usb_console_early_init()`: as F.
- `_tx_initialize_kernel_setup()`, the stack check's handler registered. Interrupts are masked from here.
- `usb_console_init()`: as F.
- `z_os_thread_start()`: the thread "main", after the OS's storage after its stack is zeroed; its body is
  `usb_console_main_thread()`.
- `usb_console_shell_start()`: as F.
- The tick's check, `tx_kernel_enter()`. Interrupts are unmasked when it takes the first thread.
- The thread "main": as F; then `z_os_thread_exit()`.

The shared functions are declared in `common/pico_sdk/app/usb-console/src/usb_console.h` and defined in
`usb_console.c`. The order is set only inside them and by the sequence of the OS's `main()`. What a new OS
on the Pico SDK writes is this `main()` with its kernel set-up, its thread "main" and its scheduler's start.

| Step | N | F, T |
|---|---|---|
| RAM log | `rp2040_boardinitialize()` | `usb_console_early_init()` |
| NINA passthrough check | `rp2040_boardinitialize()` and the init task | `usb_console_early_init()` (interrupts enabled; no kernel is started) |
| Kernel set-up | NuttX's boot | the OS's `main()` (F: none, T: `_tx_initialize_kernel_setup()`) |
| Layer, I2C0, I2C1, key-value area's bounds | `rp2040_bringup()` | `usb_console_init()` (no I2C transfer, no flash access) |
| Shell | `nsh_initialize()` in the init task, before 1 | `usb_console_shell_start()` |
| LSM6DSOX, network | `rp2040_bringup()` | first in `usb_console_main_thread()` |
| Application's start | 1, 3, Telnet (after the threads), 4 in the init task | 1, 2, 3, 4 in `usb_console_main_thread()` |

From the kernel set-up to the scheduler's start, nothing needs an interrupt or time to pass:

- the USB stdio's start, with no wait for the host (`PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS` 0);
- the consoles and the layer's objects and threads;
- I2C0's start (resets, pins, registers, handlers), with no transfer: the thread "main" sets up the
  LSM6DSOX;
- the key-value area's bounds, the LED's GPIO;
- log lines on the way go to the RAM log.

The OS may keep interrupts masked meanwhile, and T does. Nothing that needs an interrupt or a wait may be
added to `usb_console_init()` or `usb_console_shell_start()`.

### 4.7 POSIX socket headers

| N | F, T |
|---|---|
| NuttX's libc (`<sys/socket.h>`, `<netinet/in.h>`, `<arpa/inet.h>`, `<netdb.h>`, `<poll.h>`) | `common/compat/libc_net/include`: `arpa/inet.h`, `netdb.h`, `netinet/in.h`, `netinet/tcp.h`, `poll.h`, `sys/socket.h`, `sys/uio.h`; `common/compat/libc_net/inet.c` |

## 5. Drivers

| Driver | Boundary headers (`port/`) | N files | F files | T files | Zephyr |
|---|---|---|---|---|---|
| `common/drivers/i2c_dw` | `i2c_dw_os.h`: 4 types, 5 macros, 10 inline functions, `i2c_dw_os_uptime_ms()`; read at the end: `i2c_dw_instance.h` (`i2c_dw_os_prepare()`, `i2c_dw_os_irq_enable()`, the devices). `i2c_dw_dma_os.h`: 10 functions, 1 callback type. The shared instance on the Pico SDK (`port/pico_sdk/i2c_dw_instance.h`) also calls the OS's `i2c_dw_os_sem_init(sem, initial, limit)` (below) | `port/nuttx/i2c_dw_os_impl.h`, `i2c_dw_instance.h` (`struct i2c_master_s`), `i2c_dw_dma_os_rp2040.c`, `i2c_dw_controllers_rp2040.h` | `port/freertos/i2c_dw_os_impl.h`; on the Pico SDK (shared) `port/pico_sdk/i2c_dw_instance.h` (Zephyr devices), `i2c_dw_dma_os_rp2040.c`, `i2c_dw_controllers_rp2040.h` | `port/threadx/i2c_dw_os_impl.h` (`TX_SEMAPHORE`); the rest the same `port/pico_sdk/` as F | upstream driver + patch, no `port/zephyr` |
| `common/drivers/nina` | `nina_bus.h` (7 functions), `nina_os.h` (18 functions, 3 more with `NINA_OS_DNS_OFFLOAD`), `nina_passthrough_bus.h` (17), `nina_passthrough_os.h` (5, a struct, log macros); the binding also implements `nina_passthrough_run()` | `port/nuttx/nina_bus_rp2040.c`, `nina_os.c`, `nina_passthrough_bus_rp2040.c`, `nina_passthrough_os.c`; network: `nina_netdev.c`, `nina_usrsock.c`, `nina_poller.c`, `nina_wifi_mgmt.c` | `port/freertos/nina_os.c`, `nina_bus_dma_os_impl.h`; on the Pico SDK (shared) `port/pico_sdk/nina_bus_rp2040.c` (its long transfers by DMA in `nina_bus_dma_rp2040.c`, on the OS's waits of `nina_bus_dma_os.h`, section 3), `nina_passthrough_bus_rp2040.c`, `nina_passthrough_os.c`, `nina_board_rp2040.h`; network: `port/zephyr/nina_net.c`, `nina_socket.c` on the layer | `port/threadx/nina_os.c` (`TX_MUTEX`, `TX_SEMAPHORE`), `nina_bus_dma_os_impl.h`; the rest as F | `port/zephyr/` |
| `common/drivers/lsm6dso` | `lsm6dso_os.h`: 5 types, log macros, 20 functions (the INT1 line's enable and disable ordinary functions, the rest static inline); `lsm6dso_stream_req.h` (the FIFO stream's user): 2 types, 6 inline functions. Both implemented on Zephyr's API in `port/zephyr/` (`lsm6dso_os_impl.h`, `lsm6dso_os.c`; RTIO's `lsm6dso_stream_req_impl.h`), the same for all four builds | `port/nuttx/lsm6dso_instance.h` (one device, `board.h`'s values); the rest is `port/zephyr/` on the layer (`lsm6dso_sensor.c`, `lsm6dso_rtio.c`, `lsm6dso_rtio_stream.c`, `lsm6dso_decoder.c`, `lsm6dso_os.c`); the sensor's start `N-board/src/rp2040_lsm6dso.c`; build `nuttx/drivers/lsm6dso/Make.defs`, `Kconfig` | on the Pico SDK (shared) `port/pico_sdk/lsm6dso_instance.h`; the same `port/zephyr/`; the sensor's start `common/drivers/lsm6dso/port/pico_sdk/lsm6dso_board.c`; build `common/pico_sdk/drivers/lsm6dso/lsm6dso.cmake` | nothing (as F) | a Zephyr module `port/zephyr` (`zephyr/module.yml`, the app's `ZEPHYR_EXTRA_MODULES`), `CONFIG_LSM6DSO_LOCAL` (`port/zephyr/Kconfig`, in place of Zephyr's `CONFIG_LSM6DSO`), `port/zephyr/lsm6dso_instance.h` (the devicetree instances) |

`i2c_dw_i2c0_initialize()` returns `struct i2c_master_s *` on N and `int` on F and T (`i2c_dw_controllers_rp2040.h`).

What an OS on the Pico SDK meets in the drivers' waits:

- **`i2c_dw_os_sem_init(sem, initial, limit)`** (`k_sem_init()`) is not in the list of `i2c_dw_os.h`; the
  start of the shared instance on the Pico SDK (`i2c_dw_os_initialize()` of `port/pico_sdk/i2c_dw_instance.h`)
  calls it for `device_sync_sem`, `bus_sem` and `rp2040_dma_sem`, and the OS's `i2c_dw_os_impl.h` defines it.
  It is called before the kernel runs (`main()`), and again on the same semaphores when the controller
  did not start the first time; the second call sets the count and the limit anew (T keeps the object it
  made and writes the count and limit, as ThreadX refuses a second creation and a deletion before the
  kernel runs).
- **Calls before the kernel runs and in an ISR**: a take with a time and the bus lock take the count
  without a wait if there is one and return −EAGAIN otherwise (no thread can block; T tries once more
  without a wait when ThreadX refuses the wait with `TX_WAIT_ERROR`). The NINA's locks are taken without a
  wait before the kernel runs (no other thread can hold them). Sleeps do not wait before the kernel runs.
  Before the scheduler runs, every NINA SPI transfer is polled (`nina_bus_dma_os_running()`).
- **What the waits wait on**: an object apart from the layer's wake flag, so that a wake-up of the one
  never ends a wait of the other. F: a task notification index per driver (layer 1, NINA 2, I2C 3); T:
  `TX_SEMAPHORE`s the drivers own (ThreadX's semaphore is a count and a list of waiters, light; ThreadX has
  no wake-up of a thread's own that a driver may use; suspending and resuming a thread would lose a
  wake-up that comes before the suspension).

The LSM6DSO's core (the `.c` files at the top of `common/drivers/lsm6dso`, `include/`, `internal/`, the
boundary headers of `port/`) reads no OS definition (`lsm6dso_core_independent` in
`common/tests/run.sh`). What a build writes itself is the device's `port/<os>/lsm6dso_instance.h`
(chosen by the include path, `port/<os>` before `port/zephyr`; on the Pico SDK `port/pico_sdk`), the sensor's start on the board and
the build rules. ST's register driver `lsm6dso_reg.c` is fetched by each build's `setup.sh` (N
`nuttx/stmemsc`, F `freertos/stmemsc`, T `threadx/stmemsc`, which F's and T's `CMakeLists.txt` pass to the
shared cmake files as `PICO_SDK_STMEMSC_DIR`); its glue is the shared `common/drivers/stmemsc`.

## 6. Always per OS

| Item | N | F | T |
|---|---|---|---|
| Start-up to the scheduler | NuttX boot, `N-board/src/rp2040_boardinitialize.c`, `rp2040_bringup.c`, init task `nuttx/apps/init/rp2040_init.c` | `main()` in `freertos/app/usb-console/src/main.c` (`xTaskCreateStatic()`, `vTaskStartScheduler()`) on the shared `common/pico_sdk/app/usb-console/src/usb_console.c` | `main()` in `threadx/app/usb-console/src/main.c` (`_tx_initialize_kernel_setup()`, `z_os_thread_start()`, `tx_kernel_enter()`) on the same `usb_console.c` |
| Kernel tick and timer | tickless timer `N-board/src/rp2040_oneshot.c`, `rp2040_timerisr_alarm.c` (100 µs tick) | SysTick 1 ms, tick hook `tick.c` with the shared catch-up `common/compat/include/tick_catchup.h`; `FreeRTOSConfig.h` | SysTick 1 ms (set up by `src/tx_initialize_low_level.c`; priority 0x80, above PendSV's and SVCall's 0xC0), `isr_systick` (`src/tick.c`: ThreadX's tick, the same catch-up, `zephyr_compat_tick()`); `tx_user.h` (`TX_TIMER_PROCESS_IN_ISR`: no timer thread) |
| Interrupt attach | `irq_attach()`, `rp2040_gpio_irq_attach()` | Pico SDK `irq_set_exclusive_handler()`, `irq_add_shared_handler()`, `gpio_add_raw_irq_handler_masked()` | same; ThreadX's handlers on crt0's weak `isr_pendsv` and `isr_svcall` by `--defsym` in the link (`threadx_kernel.cmake`) |
| Idle loop (the window arithmetic is shared, `common/compat/port/cpu_load.h`) | `N-board/src/rp2040_idle_cpuload.c` | the idle task's hook (`idle_os.h`) on the shared `common/pico_sdk/app/usb-console/src/idle.c` | no idle thread: the `TX_LOW_POWER` hooks of the scheduler's wait inside PendSV (`idle_os.h`) on the two halves of the same `idle.c` |
| Entry to BOOTSEL (the ROM's `reset_usb_boot()`) | `N-board/src/rp2040_restart.c` on NuttX's `ROM_LOOKUP()` | the Pico SDK's `reset_usb_boot()` (Zephyr: hal_rpi_pico's) | same |
| Chip ID (the flash's unique ID, read once with XIP off) | `N-board/src/rp2040_chipid.c` on NuttX's `rp2040_flash_cmd()` | the Pico SDK's `pico_unique_id` (Zephyr: its hwinfo driver) | same |
| USB device stack, console transport | `cdcacm`, `N-board/src/rp2040_usbserial.c`; Telnet `nuttx/apps/init/rp2040_telnet.c` | Pico SDK `stdio_usb` (TinyUSB); the console transport `common/pico_sdk/drivers/shell/shell_serial_os_impl.h`, `shell_stdio.c`, `common/pico_sdk/drivers/libc/stdio_wrap.c`; Telnet is the shell engine's | same |
| The waits of the Pico SDK's locks and sleeps | — | FreeRTOS's RP2040 port (`configSUPPORT_PICO_SYNC_INTEROP`; the timer task wakes a waiter released in an interrupt) | `threadx/drivers/pico_sdk_interop/` (the interrupt handler wakes it itself) |
| Heap | NuttX's | none in the kernel (`configSUPPORT_DYNAMIC_ALLOCATION` 0: every kernel object is static); newlib's `malloc()` from the end of `.bss` | none in the kernel (every object's memory is the caller's; no byte or block pool is created); newlib's `malloc()` the same (serialized by a Pico SDK mutex) |
| The OS's own shell | nsh (sessions in `rp2040_init.c`) | none: the shared engine `common/compat/shell_engine` (consoles, serial session, Telnet) on the console transport of `common/pico_sdk/drivers/shell` | same |
| The OS's own log in the RAM log | `syslog()`: the syslog channel, `nuttx/drivers/log/log_ram_syslog.c` | none (the FreeRTOS kernel and the Pico SDK have no log) | none (nor has the ThreadX kernel) |
| Fatal errors | `CONFIG_BOARD_RESET_ON_ASSERT`, report in the RAM log | `common/pico_sdk/app/usb-console/src/fault.c` with `src/fault_os.c` (the entries: `panic()`, `isr_hardfault`, `app_assert_failed()` of `configASSERT()`, FreeRTOS's stack overflow hook, `_exit()`; the task's name, the record in `__uninitialized_ram`, the watchdog's scratch registers and the SDK's `watchdog_reboot()`) with `common/compat/log/fault_record.c` (the record, `fault_record_set_frame()`, `fault_record_take()`) | the same `fault.c` with `src/fault_os.c` (the entries: `panic()`, `isr_hardfault`, `app_assert_failed()` of `Z_TX_ASSERT()`, ThreadX's stack check (`TX_ENABLE_STACK_CHECKING`, `tx_thread_stack_error_notify()`), `_exit()`) |
| Build system and configuration | Make, Kconfig, `defconfig`, `nuttx/patches` | CMake (`freertos/app/usb-console/CMakeLists.txt` on the shared `common/pico_sdk/app/usb-console/usb-console.cmake` and `common/pico_sdk/drivers/*/*.cmake`), `FreeRTOSConfig.h` | CMake (`threadx/app/usb-console/CMakeLists.txt` and `threadx_kernel.cmake` on the same shared files), `tx_user.h` |

### 6.1 What differs visibly between F and T

All within the boundaries; the use (the commands' syntax and the form of their output) is the same.

| Point | F | T |
|---|---|---|
| A post to the NINA's Wi-Fi thread (`nina_os_wifi_post()`) before its first wait | not kept (the first wait returns at once whatever came, and notes the waiting task) | kept (as in the Zephyr build; the first wait returns on it) |
| When an owner raised by priority inheritance drops back | when it has released all the mutexes it holds | at each release, to the highest waiter of the mutexes it still holds |
| When the stack is checked | at every switch (`configCHECK_FOR_STACK_OVERFLOW` 2, PendSV) | when a thread suspends itself and when it is resumed (`TX_ENABLE_STACK_CHECKING`; not at a preemption by PendSV) |
| PRIO of `ps` | FreeRTOS's number (20 − Zephyr; larger more urgent) | ThreadX's number (Zephyr + 16; 0 most urgent) |
| The rows of `ps` | ended tasks are gone (FreeRTOS deletes them); rows for the kernel's tasks `Tmr Svc` and `IDLE` | ended threads ("main") left out; no kernel thread; the scheduler's wait as the row `(idle)` |
| First word of `version` | `FreeRTOS kernel` | `ThreadX kernel` |
| Prompt, USB product name | `freertos:~$ `, "FreeRTOS USB Console" | `threadx:~$ `, "ThreadX USB Console" |

## 7. Example: the IMU alone

The smallest composition that streams the sensor's data to the shell (`imu`, `sensor`) without Wi-Fi,
MQTT or time synchronization, listed from what the NuttX build compiles without `CONFIG_NINA` (and
without NuttX's `CONFIG_NET_USRSOCK`).

| Item | Holds |
|---|---|
| Options | `CONFIG_KV_STORE`, `CONFIG_APP_IMU`, `CONFIG_LSM6DSO_STREAM` (N also `CONFIG_LSM6DSO_LOCAL`); optionally `CONFIG_APP_SENSOR` (the `sensor` command). Not defined: `CONFIG_IMU_RAW_MQTT`, `CONFIG_TIME_SYNC`, `CONFIG_WIFI_CREDENTIALS`, `CONFIG_WIFI_PROFILE`, `CONFIG_WIFI_NINA_LOCAL`, `CONFIG_NET_ZPERF` |
| `common/imu` | `app_main.c`, `app_init.c`, `imu_stream.c`, `imu_raw_batch.c`, `imu_raw_packet.c`, `imu_raw_queue.c`, `imu_raw_time.c`, `imu_tick_clock.c`, `imu_cal.c`, `imu_cal_codec.c`, `kv_store.c`; in place of the transport and the time synchronization `imu_raw_mqtt_null.c` (every batch dropped and counted, as while disconnected) and `time_sync_null.c` (the monotonic clock only, no server time) |
| `common/compat` | `zephyr_init.c`, `zephyr_work.c`, `zephyr_rtio.c`, `zephyr_rtio_workq.c`, `zephyr_mem_blocks.c`, `zephyr_device.c`, `zephyr_gpio.c`, `zephyr_sensor_clock.c`, `sensor_default_rtio.c`, `sensor_shell_stream.c` (`CONFIG_APP_SENSOR`), `zephyr_flash_kv.c`, `kv_store_wipe.c`, `lib/utils/hex.c`, the settings and NVS copies (`subsys/settings/src/settings.c`, `settings_store.c`, `settings_init.c`, `settings_nvs.c`, `subsys/kvss/nvs/nvs.c`, `subsys/crc/crc8_sw.c`); the log `log/log_ram.c`, `log_ring.c` |
| Drivers | `common/drivers/i2c_dw/i2c_dw.c` with the DMA of `port/<os>/`; the 4 core files of `common/drivers/lsm6dso` and the 5 of `port/zephyr/` (section 5); `common/drivers/stmemsc/stmemsc_i2c.c`, `stmemsc_mdelay.c`; ST's `lsm6dso_reg.c`; `common/shell/sensor_shell.c` (`CONFIG_APP_SENSOR`); optionally `common/shell/i2c_shell.c` (the `i2c` command: a copy of the Zephyr build's patched file with its patch 0004, with no OS part, on the layer's `<zephyr/drivers/i2c.h>`, including `i2c_get_config()`, `i2c_recover_bus()` and the like, and `shell_hexdump()`); `common/chip/rp2040/rp2040_flash_kv.c` |
| Shell | `common/shell/zshell.c`, `subcmd.c` (optionally `i2c_dma_shell.c`); an OS without a shell of its own: `common/compat/shell_engine` but Telnet |
| The OS's | the kernel part (section 2), log, KV flash, GPIO bank 0 (INT1), CPU load, priorities, hwinfo (session IDs), I2C's `port/<os>/`, `lsm6dso_instance.h` and the sensor's start, console, start (4.6 without the network step); on the Pico SDK all but the kernel part, the priorities, the tick, the tie of the Pico SDK's locks, I2C's `i2c_dw_os_impl.h` and the application's OS parts are shared ("On the Pico SDK" above) |
| Not needed | `common/compat/net_offload`; `common/compat/libc_net` (when the C library has the `<arpa/inet.h>` that `imu_raw_mqtt_null.c` reads; F's and T's newlib does not); `common/compat/zephyr_heap.c`; the NINA driver (its passthrough mode, `CONFIG_NINA_PASSTHROUGH`, is chosen separately); `common/imu/mqtt_min*.c`, `imu_raw_status.c`, `time_sync.c`, `time_offset.c`, `wifi_profile.c` |
