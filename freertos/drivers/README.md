# Drivers and compatibility layer (FreeRTOS build)

English | [日本語](README_ja.md)

The firmware of this repository is built on four OSes, Zephyr, NuttX, the Pico SDK with FreeRTOS and the Pico SDK
with ThreadX: the Zephyr, NuttX, FreeRTOS and ThreadX builds.
The internals of the FreeRTOS build (`freertos/`). How to use it and the list of commands are in
[`freertos/README.md`](../README.md), the summary of the design in §13 of the
[design document](../../design/imu-raw-mqtt-design.md). The details are in the comment at the top of each
file; this page shows where the files are and how they relate to the Zephyr and NuttX builds.

Most of the FreeRTOS build's sources are in `common/`, which the four builds share (acquisition,
transport, time synchronization, calibration, key-value store, the LSM6DSO, NINA and I2C drivers, the
shell's command handling). The shared sources are written on Zephyr's API, so the FreeRTOS build, like the
NuttX and ThreadX builds, runs them on a compatibility layer that provides Zephyr's API under Zephyr's names. The
OS-independent part of the layer is `common/compat/`, the FreeRTOS part `freertos/drivers/compat/`. The
layer's network part for an OS without a network stack of its own (Zephyr's socket layer, the network
interface) is `common/compat/net_offload/`, and the POSIX network headers newlib lacks are
`common/compat/libc_net/`; the layer's kernel part for an OS that offers only basic primitives (Zephyr's
semaphores, `k_poll()`, timeouts, sleeps and threads on about twenty primitives of the OS) is
`common/compat/kernel_generic/`, on FreeRTOS's primitives in this build; the FreeRTOS and ThreadX builds use them.

## Directories

The FreeRTOS build's own files are in `freertos/`; what it shares with the other builds on the Pico SDK (the ThreadX build)
(the parts written on the Pico SDK and the compatibility layer, not on FreeRTOS) is in
`common/pico_sdk/`, whose layout follows `freertos/` (`app/usb-console/`, `drivers/<part>/`), in the
drivers' `port/pico_sdk/` directories, and in `common/compat/os_generic/` (the layer's parts for an OS
without the model of its own). The FreeRTOS build gives them its parts through a few boundaries (the
`*_os.h` headers below), each with a FreeRTOS implementation in `freertos/`.

| Place | Contents |
|---|---|
| `app/usb-console/` | The build (`CMakeLists.txt`: the FreeRTOS settings of `common/pico_sdk/app/usb-console/usb-console.cmake`, the list of the sources), the kernel configuration (`FreeRTOSConfig.h`, which also checks the one core), and `src/`: the entry of the start (`main.c`: `main()`, the task "main", the scheduler), the priority mapping (`priorities_os.h`), the tick hook (`tick.c`; the catch-up's decision is `common/compat/kernel_generic/tick_catchup.h`), the OS part of the generic commands (`kernel_shell_os_impl.h`, with the tick count of `uptime`), and the FreeRTOS parts of the shared application's boundaries: the idle pass (`idle_os.h`: the idle hook), the fatal errors (`fault_os_impl.h`: the running task's name, the stack overflow hook), the commands (`commands_os_impl.h`: `ps` with the clock of its run times, the kernel's version) |
| `common/pico_sdk/app/usb-console/` | Shared with the builds on the Pico SDK: the build (`usb-console.cmake`: the drivers, the include directories, the SDK's options; `ram_code.cmake`: the code that runs from RAM; `ld/`: the start-up thread's stack in SCRATCH_X) and `src/`: the shared steps of the start (`usb_console.[ch]`, called by the OS's `main()`), the commands `version` and `free` and the table of all commands (`commands.c`, on `commands_os.h`), the idle pass and the CPU load (`idle.c`; the LED's blink `idle_blink.h`), fatal errors (`fault.[ch]`, on `fault_os.h`), the free interrupt stack (`irq_stack.c`), `app.h` |
| `drivers/compat/` | The FreeRTOS part of the layer: the primitives of the generic kernel part (`zephyr_kernel_os_impl.h`) and the port header (`rp2040/zephyr_os_port.h`: `z_os_in_isr()`, `z_os_yield_from_isr()`, `z_os_task_lock()`/`_unlock()`) |
| `common/pico_sdk/drivers/compat/` | The layer's part on the Pico SDK: the build of the layer, the shell and the RAM log (`compat.cmake`), the linker script's part for the socket implementations (`ld/`), the wall clock (`zephyr_sys_clock.c`), the RP2040 part (`rp2040/`: the port header's part shared by the OSes, `zephyr_os_port_rp2040.h`; GPIO bank 0, `gpio_os_rp2040.c`; chip ID, `zephyr_hwinfo.c`) |
| `common/compat/os_generic/` | The layer's parts for an OS without the model of its own: `include/zephyr/drivers/i2c.h` (Zephyr's I2C API on the driver's device), `include/zephyr/sys/clock.h`, `include/zephyr/sys/__assert.h`, the prelude (`common_prelude.h`), `app_log_os.h`, the sensor shell's OS part (`include/sensor_shell_os.h`, `sensor_shell_os.c`), `wifi_profile_os.h` |
| `common/compat/kernel_generic/` | The layer's kernel part for an OS that offers only basic primitives, OS-independent: `k_sem`, `k_poll()`, the timeouts, the sleeps and the start of a thread (`zephyr_kernel.c`), `k_mutex` (`zephyr_mutex.c`), the OS part of `<zephyr/kernel.h>` and `<zephyr/spinlock.h>` (`zephyr_os.h`, `zephyr_os_spinlock.h`), the tick hook's entry (`zephyr_compat_os.h`) and its catch-up (`tick_catchup.h`), the default `zephyr_os_atomic.h`, and the declaration of the primitives (`zephyr_kernel_os.h`). The work queues' threads, the system work queue and `zephyr_compat_initialize()` are the shared `common/compat/zephyr_init.c` |
| `common/tests/zephyr_kernel/` | The host test of the layer's kernel, OS-independent (`zephyr_kernel_test.c`; `zephyr_kernel_test.h` lists what an OS's part and a platform's part provide; `kernel_generic/zephyr_kernel_test_os.h`, its items on the primitives). This build's parts are in `tests/compat/` |
| `common/compat/net_offload/` | The layer's network part, OS-independent: Zephyr's socket layer (reduced copies of Zephyr's files in `subsys/net/lib/sockets/` and `lib/os/zvfs/`), the network interface (`zephyr_net_if.c`) and the start of the network part (`zephyr_net_init.c`, `zephyr_net_initialize()`), their headers under `include/` (`zephyr/net/`, `zephyr/sys/fdtable.h`, `zephyr_net_init.h`). `k_heap` (`zephyr_heap.c`, `zephyr_heap.h`, which `<zephyr/kernel.h>` includes) is in `common/compat/`, with either kernel part |
| `common/compat/libc_net/` | The socket headers newlib lacks (`include/`, types and constants only), `inet_pton` and `inet_ntop` (`inet.c`) |
| `common/compat/shell_engine/` | The shell for an OS without one of its own, on the layer's API: line editing (`cli.c`), the command table and execution with `help` (`shell_engine.c`), a copy of Zephyr's argument split (`shell_argv.c`), the consoles and each thread's output (`shell_console.c`), getopt per session (`getopt_state.c`), the shell's backend for `zshell.c` (`zshell_os_impl.h`, `zephyr_os_shell.h`), the serial session (`shell_serial.c`; the console transport an OS provides is declared in `shell_serial_os.h`), the Telnet shell (`shell_telnet.c`) |
| `common/pico_sdk/drivers/shell/` | The console transport on the Pico SDK for the shell engine (`shell_serial_os_impl.h`: the Pico SDK's USB CDC ACM driver) and the replacement of the Pico SDK's stdio driver (`shell_stdio.c`) |
| `common/pico_sdk/drivers/libc/` | Wrappers of newlib's FILE functions (`stdio_wrap.c`) |
| `common/pico_sdk/drivers/include/` | The RAM log's part on the Pico SDK (`log_ram_os_impl.h`: the ring's section, the interrupt mask, the TIMER) |
| `common/pico_sdk/drivers/kv/` | The key-value store's build (`kv.cmake`), the flash operations on the Pico SDK (`rp2040_flash_kv_os.c`, under the RP2040's shared part `common/chip/rp2040/rp2040_flash_kv.c`) and the linker script override (`ld/`) |
| `common/pico_sdk/drivers/nina/` | `nina.cmake`, which adds the NINA driver to the build. The driver is in `common/drivers/nina/`; its binding is FreeRTOS's `common/drivers/nina/port/freertos/` (the `nina_os.h` boundary, `nina_os.c`; the DMA's waits, `nina_bus_dma_os_impl.h`) with the Pico SDK's `common/drivers/nina/port/pico_sdk/`: the passthrough's host side (`nina_passthrough_os.c`), SPI1 and the module's lines (`nina_bus_rp2040.c`, its long transfers by DMA in `nina_bus_dma_rp2040.c` on the waits of `nina_bus_dma_os.h`), the passthrough's module side (`nina_passthrough_bus_rp2040.c`: UART1, the module's lines, the watchdog's scratch registers, the LED), the board's lines (`nina_board_rp2040.h`); the sockets and the interface are the Zephyr binding's (`common/drivers/nina/port/zephyr/nina_socket.c`, `nina_net.c`) |
| `common/pico_sdk/drivers/i2c_dw/` | `i2c_dw.cmake`, which adds the I2C driver to the build (I2C0, and I2C1 as an option). The driver is in `common/drivers/i2c_dw/`; its binding is FreeRTOS's `common/drivers/i2c_dw/port/freertos/` (the `i2c_dw_os.h` boundary on FreeRTOS and the TIMER, `i2c_dw_os_impl.h`) with the Pico SDK's `common/drivers/i2c_dw/port/pico_sdk/`: the devices with Zephyr's I2C API, their interrupts, resets and pins (`i2c_dw_instance.h`), their start (`i2c_dw_controllers_rp2040.h`), I2C0's DMA reads on the Pico SDK's DMA (`i2c_dw_dma_os_rp2040.c`) |
| `common/pico_sdk/drivers/lsm6dso/` | The LSM6DSOX's build rules (`lsm6dso.cmake`); the device and its start are `common/drivers/lsm6dso/port/pico_sdk/` (`lsm6dso_instance.h`, `lsm6dso_board.c`) |
| `common/pico_sdk/drivers/imu/` | Builds in the acquisition service, MQTT and time synchronization (`common/imu/`; `imu.cmake`) |
| `common/pico_sdk/drivers/wifi/` | Builds in Wi-Fi (the NINA core's `common/drivers/nina/nina_wifi.c` with its Zephyr binding `common/drivers/nina/port/zephyr/nina_net.c`, credentials, reconnection) and `ifconfig` (`common/shell/ifconfig_shell.c`) |
| `common/pico_sdk/drivers/telnet/` | Builds in the Telnet shell (`telnet.cmake`; the code is `common/compat/shell_engine/shell_telnet.c`) |
| `common/pico_sdk/drivers/zperf/` | Builds in `zperf` (the code is `common/compat/zperf_iperf3.c` and `zephyr_zperf_shell.c`) |
| `common/pico_sdk/drivers/sensor_shell/` | Builds in the `sensor` command (`sensor_shell.cmake`; its OS part is `common/compat/os_generic/sensor_shell_os.c`) |
| `common/pico_sdk/drivers/i2c_shell/` | Adds the `i2c` command (`i2c_shell.cmake`; the command is `common/shell/i2c_shell.c`, the same file as `drivers/i2c/i2c_shell.c` of the Zephyr build's patched tree) |
| `tests/` | Host tests ("Host tests" below) |

Shared files only the FreeRTOS and ThreadX builds on the Pico SDK compile: `common/compat/zephyr_wifi_shell.c` (`connect`,
`disconnect`, `scan` and `status` taken from Zephyr's `wifi_shell.c`), `zephyr_zperf_shell.c` and
`zperf_iperf3.[ch]` (`zperf`), `common/compat/log/fault_record.[ch]` (the format of the fatal error record),
`common/shell/dmesg_shell.c` (`dmesg`), `common/shell/kernel_shell.c` (`id`, `uptime`, `date`, `sleep`, `devmem`,
`fault`, `reboot`, `bootsel`, on the boundary `common/shell/kernel_shell_os.h`), `common/shell/ifconfig_shell.c`
(`ifconfig`). The RAM log `common/compat/log/log_ram.c` (on its ring `log_ring.c`) is shared with
the NuttX and ThreadX builds.
`i2c_dma`'s `common/shell/i2c_dma_shell.c` and `nina`'s `common/shell/nina_shell.c` (`nina version`,
`nina passthrough`) are built by the Zephyr build too. `i2c`'s `common/shell/i2c_shell.c` is a
byte-for-byte copy of Zephyr's `drivers/i2c/i2c_shell.c` with the local patch 0004 (`zephyr_i2c_shell_patched`
of `common/tests/run.sh` compares them), built by the NuttX build too (the Zephyr build builds the file in its tree).

## The FreeRTOS part of the Zephyr compatibility layer

What the layer provides is listed at the top of `common/compat/zephyr/kernel.h`. Its kernel part is the
generic implementation (`common/compat/kernel_generic/zephyr_kernel.c` and `zephyr_mutex.c`; how it works
is in their comments) on primitives that FreeRTOS provides (`drivers/compat/zephyr_kernel_os_impl.h`; the
list and what each must do are `common/compat/kernel_generic/zephyr_kernel_os.h` and section 2.5 of
`design/os-boundary.md`). These
assumptions of Zephyr are reproduced:

- **Mutexes** (`zephyr_mutex.c` on FreeRTOS's recursive mutex): recursive, with FreeRTOS's priority
  inheritance. An all-zero `k_mutex` is
  unlocked and is created by its first lock (Zephyr's MQTT library and settings rely on it). The return
  values are Zephyr's.
- **Semaphores and `k_poll`**: Zephyr's rules (a give hands the count to the highest-priority waiter,
  `k_sem_reset()` ends the waits with `-EAGAIN`) on a count of the layer's own; waits use a task
  notification (index 1; 0 is left to the Pico SDK and the application). A give from an interrupt
  handler asks for a switch at the end of the handler, so the woken task runs right after it.
- **Static threads**: the tasks of `K_THREAD_DEFINE()` are started by the start-up task (`main`) from
  the application's table shared with the NuttX and ThreadX builds (`app_threads_start()`, `common/imu/app_init.c`). Every thread of the layer (the work queues', `RTIO WQ 0`, `net_mgmt`) starts the
  same way (`z_thread_static_start()`: a task on its static stack, deleted when its function returns).
- **Time**: `k_uptime_*()`, `k_cycle_*()` and `time_sync_mono_us()` come from the RP2040's TIMER (1 MHz),
  not from the tick (`rp2040_timer_us()`).

Left as approximations:

| Item | Zephyr | FreeRTOS build |
|---|---|---|
| Priorities | Lower runs first; cooperative -16..-1, preemptible 0..14 | `20 − Zephyr` (preemptible 0..14 become 20..6, cooperative -1 becomes 21). `Tmr Svc` 31, idle 0 |
| Cooperative threads | Not preempted | Approximated by a higher priority (a still higher task preempts; as on NuttX) |
| Priority inheritance | Follows a chain of holders, adjusted at each unlock | One level; dropped when the holder has released all its mutexes |
| Choice among one priority | A preempted thread goes on; 20 ms time slices, counted from the switch to the thread | FreeRTOS's: at every switch the next ready task of the priority in turn, so a preempted task goes behind the other ready tasks of its priority. 20 ms slices by the tick hook (`tick.c`) on a fixed grid of 20 ticks, not counted from the switch |
| Tick and sleeps | 100 µs; a relative wait is "rounded up to ticks, plus one tick" | 1 ms (`configTICK_RATE_HZ` 1000, 32 bits), the same rounding: a wait shorter than 1 ms lasts 1 to 2 ms. A periodic SysTick (not tickless) |

- The 32-bit tick wraps after about 49.7 days.
- While interrupts are disabled (a flash erase or program; about 40 to 50 ms per erase), SysTick keeps one
  interrupt pending and the other ticks are lost. The tick hook (`tick.c`, decision in `common/compat/kernel_generic/tick_catchup.h`)
  compares the count with the TIMER at every tick and, when it is a whole tick behind, pends SysTick
  again, so the port's usual handler counts the lost ticks one by one (wake-ups, switch requests, and the
  tick hook's timeouts of the layer and time slices run on each). The tick never gets ahead of the
  TIMER, so apart from the time interrupts are disabled it stays within one tick of it.
  `xTaskCatchUpTicks()` is not used (it can be called from tasks only, and the tick hook does not run for
  the ticks it adds). `main.c` checks that clk_sys is a whole number of ticks. `k_uptime_*()` is the TIMER.
- The Pico SDK's `sleep_ms()` (which spins the last 150 µs) is not used.

## Tasks

The priorities are in `common/compat/priorities.h` (the Zephyr build's values and `20 − Zephyr`) and
`app/usb-console/src/priorities_os.h` (the tasks of this build only, and the stacks of the layer's own
threads sized as measured on this build: peak use plus half, at least 256 bytes, a multiple of 64; the
shell sessions keep Zephyr's 3072). Stacks are in bytes. Interrupts
use a stack of their own (MSP, 2048 bytes; the `(interrupts)` row of `ps`).

| Task | Priority (Zephyr) | Stack | Role |
|---|---|---|---|
| `Tmr Svc` | 31 (–) | 384 | The kernel's timer task; wakes the waiters of Pico SDK locks released in interrupt handlers |
| `net_mgmt`, `sysworkq` | 21 (-1) | 2048, 384 | Network events; the system work queue (the work of the LSM6DSOX FIFO interrupt) |
| `main` | 20 (0) | 2048 | The second half of the start-up ("Start-up" below); ends when done. Its stack is in SCRATCH_X, which only core 1 (never started) would use (`common/pico_sdk/app/usb-console/ld/`, `src/main.c`) |
| `sensor_shell_tid`, `RTIO WQ 0` | 20 (0) | 2048, 1024 | Printing `sensor` results; RTIO work |
| `nina_wifi_tid`, `wifi_reconnect_tid` | 12 (8) | 2048, 2048 | Wi-Fi scan, connection and link monitor (one at a time); automatic reconnection |
| `time_sync_tid`, `imu_stream_tid` | 11 (9) | 2048, 2048 | Time synchronization, acquisition |
| `imu_fifo_worker` | 10 (10) | 2048 | FIFO reads |
| `imu_raw_mqtt_tid` | 9 (11) | 2048 | MQTT transport |
| `console`, `shell_telnet`, `telnet_service`, `zperf_server_tid` | 6 (14) | 3072, 3072, 2048, 2048 | The USB and Telnet shells (commands run in these tasks), Telnet accepts, the server of `zperf tcp download` |
| `IDLE` | 0 (–) | 256 | Idle; measures the CPU load and blinks the LED (GPIO6) every 500 ms while it runs (`common/pico_sdk/app/usb-console/src/idle.c`, the idle hook of `idle_os.h`) |

The kernel has no heap (`configSUPPORT_DYNAMIC_ALLOCATION` 0): every task, queue and semaphore is
static. Each thread of the layer keeps its control block (136 bytes) after its stack, in the array of
`K_THREAD_STACK_DEFINE()` (`Z_OS_THREAD_RESERVED`, `compat/zephyr_kernel_os_impl.h`); the sizes above are
the stacks without it.

**Start-up** (`src/main.c`): `main()` calls the shared steps (`common/pico_sdk/app/usb-console/src/usb_console.h`) in order.
`usb_console_early_init()` sets up the RAM log and reads the fatal error record. `usb_console_init()` sets up the console,
the layer, the NINA's SPI, I2C0 (first the DMA interrupt's handler for its reads) and the flash of the key-value store.
Then `main()` makes the task `main`, `usb_console_shell_start()` makes the task `console`, and the scheduler starts.
The task `main` (`usb_console_main_thread()`) starts the
LSM6DSOX and the NINA's network interface, then the application's start shared by the four builds
(`common/imu/app_main.h`, the Zephyr build's initialization order) with this build's `wifi` command and
Telnet in it: the Wi-Fi credentials, the `wifi` command, Telnet, the `K_THREAD_DEFINE()` threads, the
key-value store, the calibration, the time synchronization and the automatic connection, in this order.

**CPU load** (`common/pico_sdk/app/usb-console/src/idle.c`): the idle hook disables interrupts, opens the window, waits with `WFI`, closes the
window and enables interrupts (the measurement of the Zephyr build's `CONFIG_CPU_LOAD` and the NuttX
build's `up_idle()`). The tick interrupt every 1 ms counts as load (the Zephyr and NuttX builds have no
such interrupt). The CPU column of `ps` is the share of the
interval since the previous `ps` (from USB or Telnet; since the start for the first) in all the time of
the interval, idle included, so the rows add up to about 100 % (type it at the start and at the end of
the interval of interest); timed by the TIMER.

## Shell and consoles

- Commands are written with Zephyr's shell API (`SHELL_CMD_REGISTER()` and so on); matching, help,
  wildcards and the argument count check are done by the shared `common/shell/zshell.c`. The command table
  and the line editor are `common/compat/shell_engine/` (`shell_engine.c`, `cli.c`; `shell_engine.c` also has `help`). The
  generic commands are `common/shell/kernel_shell.c`; what they need from the OS (the tick count, the two
  restarts, the last fatal error's record) is declared in `common/shell/kernel_shell_os.h` and is
  `app/usb-console/src/kernel_shell_os_impl.h` here. The commands only the OS and the Pico SDK answer
  are `common/pico_sdk/app/usb-console/src/commands.c` (`version`, `free`, with the table of all
  commands) and this build's `app/usb-console/src/commands_os_impl.h` (`ps`, the kernel's version).
- The consoles, the sessions and Telnet are the shell engine's (`common/compat/shell_engine/`), written on
  the layer's `k_mutex`, `k_sem`, `z_thread_static_start()` and thread custom data
  (`k_thread_custom_data_get()`, on a FreeRTOS thread local storage pointer). A console (`shell_console.c`)
  is a session's terminal (USB or Telnet); each task is bound to one console, and its `printf()` and
  `fprintf(stderr)` go there. Tasks bound to no console write to USB. The replacement of the Pico SDK's
  stdio driver (`shell_stdio.c`) and `--wrap` of newlib's FILE functions (`common/pico_sdk/drivers/libc/stdio_wrap.c`)
  route every output path through it.
- USB: the engine's serial session (`shell_serial.c`, the task `console`) on this build's console transport
  (`common/pico_sdk/drivers/shell/shell_serial_os_impl.h`, declared in `shell_serial_os.h`: the stdio redirection, input
  arrived, the line open, read, write). The prompt appears when the terminal opens the port, after the
  banner of `usb_console.c`. When the terminal stops reading, the Pico SDK's driver waits up to 500 ms for room in
  its 64-byte buffer and drops the rest.
- Telnet (`common/compat/shell_engine/shell_telnet.c`, built in by `../common/pico_sdk/drivers/telnet/telnet.cmake`): port 23,
  one client at a time. As Zephyr's `shell_telnet.c`,
  it sends nothing on connection, takes Telnet commands out without answering, and does not echo. Where it
  differs from Zephyr is listed at the top of the file (it looks once more whether the first client has
  closed before refusing a second, a new client starts with an empty line, and so on).
- The getopt state is per session (`getopt_state.c`): USB and Telnet can run commands at the same time.

## RAM log and fatal errors

- The RAM log (`common/compat/log/log_ram.c`, shared with the NuttX and ThreadX builds; the part on the Pico SDK, shared with the ThreadX build, is
  `common/pico_sdk/drivers/include/log_ram_os_impl.h`; its size, `LOG_RAM_SIZE` in `common/compat/os_generic/include/app_log_os.h`) is
  4 KiB, as in the Zephyr build (a fatal error leaves a record of a few lines, written into the log at the
  next start), in RAM that the start-up code does not initialize (`__uninitialized_ram`). It survives the watchdog's restart, `reboot` and the RESET button, not a power
  cycle. `LOG_*` lines are formatted in the caller's context (interrupt handlers included), timed by the
  TIMER. `dmesg` (`common/shell/dmesg_shell.c`) prints it.
- Fatal errors (`common/pico_sdk/app/usb-console/src/fault.[ch]`, with this build's `app/usb-console/src/fault_os_impl.h`): `panic()`, HardFault, `configASSERT()` and `assert()`,
  stack overflow (`configCHECK_FOR_STACK_OVERFLOW` 2) and `_exit()` end in one exit,
  which keeps a record (reason, PC, LR, SP, exception frame, task, time) in RAM and in the watchdog's
  scratch registers 0..3 and restarts the board with the watchdog. The next start writes the record to
  `dmesg`, and `fault` and the console's banner show it. The record's format, the reading of the
  HardFault's exception frame (only where the frame lies in the RAM) and the choice at the next start
  between the record in RAM and the scratch registers' copy are `common/compat/log/fault_record.[ch]`;
  `fault.c` keeps the entries, the handler's entry (`isr_hardfault`), the task's name, the watchdog's
  registers and the restart.

## The key-value store's flash

As in the Zephyr and NuttX builds, the key-value store is in the last 8 KiB of the flash
(`0xffe000`..`0xffffff`, two 4 KiB sectors) in the format of Zephyr's NVS and settings (the copies are in
`common/compat/subsys/`, the format is described in "Key-value store: copies of Zephyr's NVS and settings"
of [`nuttx/drivers/README.md`](../../nuttx/drivers/README.md)). The FreeRTOS build implements the flash
boundary (the five functions of `common/compat/flash_kv_os.h`) with the RP2040's part shared with the NuttX and ThreadX builds
(`common/chip/rp2040/rp2040_flash_kv.c`: the checks, the reads, the page copy) on the flash operations on the Pico SDK
(`common/chip/rp2040/include/rp2040_flash_kv_os.h`: `common/pico_sdk/drivers/kv/rp2040_flash_kv_os.c`, shared with the ThreadX build, the Pico SDK's
`flash_range_program()` and `flash_range_erase()` through `flash_safe_execute()`).

- The linker script (`common/pico_sdk/drivers/kv/ld/`, `kv.cmake`) sets the FLASH length to `0xffe000`, keeping the image
  out of the area, and checks `__kv_start`, `__kv_end` and the end of the image with `ASSERT`.
- A program or erase request is checked against the area three times: in `zephyr_flash_kv.c`, in the
  RP2040's shared part, and right before the flash is touched (with interrupts disabled). Until the linker
  symbols are found equal to the constants, programs and erases are refused.
- `flash_range_program()` and `flash_range_erase()` are called through `flash_safe_execute()` with
  `PICO_FLASH_ASSUME_CORE1_SAFE=1`, which disables this core's interrupts only (core 1 is not used).
  Interrupts stay disabled during an erase; the tick catches up with the TIMER afterwards (above).
- Data that is neither Zephyr's NVS nor erased is left alone and logged to `dmesg`; the store's calls
  return `-EILSEQ` (138 with newlib). `kv wipe confirm` erases it (`common/compat/kv_store_wipe.c`).

## NINA-W102, sockets, Wi-Fi

- The SPI transport, the socket table and commands, name resolution and the Wi-Fi operations are the
  shared core (`internal/nina_proto.c`, `nina_sock.c`, `nina_dns.c` and `nina_wifi.c` of `common/drivers/nina/`). The FreeRTOS build implements the
  boundaries `nina_bus.h` (`common/drivers/nina/port/pico_sdk/nina_bus_rp2040.c`: SPI1 at 7.8125 MHz and the CS, READY, RESET
  and IRQ lines, set up in the order and with the values of Zephyr's drivers; transfers shorter than 64 bytes polled,
  longer ones by DMA while the calling task sleeps, as in the Zephyr and NuttX builds, `nina_bus_dma_rp2040.c`: see below) and `nina_os.h`
  (the core's FreeRTOS binding `common/drivers/nina/port/freertos/nina_os.c`, written on FreeRTOS and the
  Pico SDK without the compatibility layer: FreeRTOS recursive mutexes with priority inheritance for the
  NINA lock, the name resolution lock and the Wi-Fi operations' lock, the Wi-Fi thread's task notification
  2 as its wake-up (the core keeps its requests itself), the "rounded up, plus one tick" sleep, the TIMER in ms as the clock). Its objects are
  static and made at the first use of any of them.
  The boundaries mean what the "NINA-W102" tables of
  [`nuttx/drivers/README.md`](../../nuttx/drivers/README.md) say.
- The core's waits for READY sleep on a binary semaphore that READY's edge interrupt (GPIO10) gives, in
  waits of at most 10 ms, after which READY is read again. The binding has its own raw handler of
  `IO_IRQ_BANK0` for GPIO10 and GPIO2 (`gpio_add_raw_irq_handler_masked()`, beside the GPIO bank 0
  backend's for GPIO24; each handler acknowledges only its own pins). The Zephyr build does the same; the
  NuttX build waits without the 10 ms slices. Whether the interrupt is used is decided at run time: if
  `nina_os_ready_start()` fails after the first reset, READY is read every 100 µs instead (in all four
  builds; the FreeRTOS binding's does not fail).
- Sockets: the layer has Zephyr's socket layer down to its list of socket implementations (reduced
  copies of Zephyr's `sockets.c`, `getaddrinfo.c`, `socket_offload.c`, `zvfs_fdtable.c` and
  `zvfs_poll.c` in `common/compat/net_offload/`; what each leaves out is in its header). An implementation registers itself with
  `NET_SOCKET_OFFLOAD_REGISTER()` or `NET_SOCKET_REGISTER()`; the linker script collects the
  registrations sorted by priority (`common/pico_sdk/drivers/compat/ld/section_extra_post_text.incl`), and
  `zsock_socket()` takes the first whose family matches and whose `is_supported()` accepts. The other
  calls go through the descriptor table to the implementation's `struct socket_op_vtable` under the
  descriptor's mutex, `poll` through `ZFD_IOCTL_POLL_PREPARE` and `ZFD_IOCTL_POLL_OFFLOAD`, and
  `zsock_getaddrinfo()` to the registered resolver. The NINA's sockets are the Zephyr build's binding,
  compiled unchanged (`common/drivers/nina/port/zephyr/nina_socket.c`, priority 40): as in the Zephyr build,
  the calling task asks the NINA and waits (there is no event thread like the NuttX build's usrsock and
  `nina_event`). The table has 16 descriptors, 0 to 15 from the lowest free one (none kept for stdin,
  stdout and stderr, as in the Zephyr build); the NINA has ten sockets, so an eleventh gets `EMFILE`.
  The descriptors are not newlib's; only the `zsock_*()` calls take them. The types and constants newlib
  lacks are in `common/compat/libc_net/include/`, with Linux's values (`EAI_*` are Zephyr's negative values); the
  layer's `<zephyr/net/net_ip.h>` gives them Zephyr's `net_*` names.
- What the sockets do is the Zephyr build's: a poll of the module that fails is `POLLERR` on that
  socket (the call does not fail); a `poll` takes each descriptor's mutex, so it waits for a call in
  progress on one of its sockets in another task (a blocking receive, a connect); one `sendmsg` runs at a
  time, sleeps included; name resolution's results come from a 2048-byte heap (`EAI_MEMORY` when about
  36 results are not freed). The Telnet server restarts its listening socket after an error on it, and
  sets it up again every second while that fails.
- The firmware additions (patches 0003 to 0006, the bits of command 0x6A) work as in the Zephyr build
  (described in the section of the same name in
  [`zephyr/app/usb-console/drivers/README.md`](../../zephyr/app/usb-console/drivers/README.md)): a scan
  while connected that keeps the link and the asynchronous scan (`nina_wifi.c`), the waits for socket
  events (`common/drivers/nina/internal/nina_wait.c` and the wait loops of `port/zephyr/nina_socket.c`). The events are the
  rising edges of the IRQ line (GPIO2) on its interrupt; the binding counts them and wakes every waiting
  task (the waiters are on a list, each woken on its task notification 2). If `nina_os_event_start()` fails, the waits keep their
  intervals (20 ms, 1 ms for sends) whatever the firmware. Checking with a poll command that the socket
  is writable before a TCP send's data goes again does not depend on the firmware.
- Wi-Fi: the core's Wi-Fi operations with the Zephyr build's binding of them
  (`common/drivers/nina/port/zephyr/nina_net.c`: Zephyr's Wi-Fi management and interface, the thread `nina_wifi_tid`),
  the credentials (a copy of Zephyr's `wifi_credentials`) and the automatic reconnection
  (`common/imu/wifi_profile.c`) run on the layer's `net_if`, `net_mgmt` and `wifi_mgmt` (`net_offload/zephyr_net_if.c`;
  `net_offload/zephyr_net_init.c` registers the interface at the start, as Zephyr's `net_init()`); this build has no
  network layer of its own, so it uses the Zephyr binding. The `wifi` command
  is `common/compat/zephyr_wifi_shell.c`, keeping only the options this board can use; `wifi cred` is an
  unchanged copy of Zephyr's `wifi_credentials_shell.c`. `ifconfig` (`common/shell/ifconfig_shell.c`)
  prints in the NuttX build's format.

- The esptool passthrough (`nina passthrough confirm`): the NINA driver's core shared with the Zephyr,
  NuttX and ThreadX builds (`common/drivers/nina/nina_passthrough.c`), with its host side on TinyUSB and the Pico SDK
  (`common/drivers/nina/port/pico_sdk/nina_passthrough_os.c`) and its module side (`common/drivers/nina/port/pico_sdk/nina_passthrough_bus_rp2040.c`)
  (wiring and lines in "The esptool passthrough" of
  [`zephyr/app/usb-console/drivers/README.md`](../../zephyr/app/usb-console/drivers/README.md)). The command
  leaves a mark in the watchdog's scratch registers 0 to 3 and restarts; `main()` sees it before anything
  else and, with no task, scheduler, SPI1, NINA, I2C or KV store, relays between TinyUSB (`tud_task()`) and
  UART1 in one loop. From the module to the PC, UART1's receive interrupt moves the bytes into a 16 KiB
  buffer (from `malloc()`); that interrupt and the core's part of it run from RAM. The LED blinks at
  5 Hz. The core's host test is `nina_passthrough` of `common/tests/run.sh`.

## Divider, `memcpy` and `memset`, boot2

The Pico SDK's defaults. Integer division is `pico_divider` on the hardware divider, and the port
(`portUSE_DIVIDER_SAVE_RESTORE`) saves and restores the divider's four words at every switch (the Zephyr
and NuttX builds, with local patches, save it only in the middle of a division, DIRTY). A division by
zero gives the Pico SDK routines' results, the same in the four builds. `memcpy` and `memset` are
`pico_mem_ops`, the boot ROM's routines; boot2 is the Pico SDK board definition's, for the W25Q080
(divider 2, 62.5 MHz).

## Code in RAM

The flash is executed through the RP2040's 16 KiB XIP cache, and the code on the 6667 Hz path of the
sensor and on the send path does not fit in it. The code (`.text*`; their read-only data stays in the flash)
of the objects listed in `common/pico_sdk/app/usb-console/ram_code.cmake` runs from RAM:

- the layer's kernel part, work queue, RTIO, memory blocks, sensor clock, wall clock and `gpio0`, with the
  Pico SDK's `hardware_gpio`;
- I2C0 (`i2c_dw.c`, also I2C1's when it is built) with its DMA part and the Pico SDK's `hardware_dma`;
- the LSM6DSOX driver and the stmemsc glue (not ST's `lsm6dso_reg.c`, which only configures);
- the IMU service's stream, batches, queue and time stamps (not the NTP exchange nor the calibration);
- MQTT, the layer's sockets, the NINA's sockets, protocol, waits and FreeRTOS binding with its SPI and DMA,
  and the Pico SDK's `hardware_spi`.

`ram_code.cmake` replaces the Pico SDK's `default_text_excludes.incl` with a copy in the build directory that
leaves these objects out of the flash's `.text`; the SDK's `.data` takes their code, and crt0 copies it to
RAM before anything else runs. While sending, `cpu_active` was 19.7% with this placement and 33.1% without
it, and the cache missed about 5,900 times a second instead of about 342,000 (2026-11-23). To check a
placement, clear the cache's counters while sending (`devmem 0x1400000c 32 0`, `devmem 0x14000010 32 0`;
they saturate) and read them about 15 s later (the hits and the accesses).

The code costs about 49 KiB of RAM (`.data` is about 57 KiB instead of 8 KiB, with the long-branch stubs
between RAM and the flash). It is taken from what newlib's `malloc()` can grow into, between the end of
`.bss` and the stacks (`free`). About 87 KiB is left above the Pico SDK's 2 KiB minimum. The
shell takes from it (about 1.2 KiB per command line while it runs, and a line of output longer than 127
characters while it is printed), `zperf` its buffers (about 2.1 KiB for an upload, 2.5 KiB while the server
of `zperf tcp download` runs), and the NINA passthrough mode its 16 KiB buffer. When it has no room,
`malloc()` returns NULL (`PICO_MALLOC_PANIC=0`; the Pico SDK's default is a fatal error) and the caller
fails: a command prints `out of memory`, `zperf` fails with -12 (`ENOMEM`). A check after the link (in
`ram_code.cmake`, on the map) stops the build when a listed object's code is not in RAM (a source no longer
built or renamed; a listed source that does not exist stops the configuration) or when less than 16 KiB is
left.

Also in RAM, as in the Pico SDK's defaults: its `__not_in_flash_func()` functions (the flash's write and
erase, the mutexes, the blocking SPI calls, parts of the USB driver), the NINA passthrough's UART interrupt
and libgcc. The fatal error path (`src/fault.c`, `common/compat/log/fault_record.c`) and the key-value
store's flash part stay in the flash; while XIP is off, only the Pico SDK's own RAM functions run, with
interrupts disabled.

## I2C0 and DMA, LSM6DSOX, GPIO interrupt

- **The driver**: I2C0 and I2C1 are the I2C core shared with the NuttX and ThreadX builds (`common/drivers/i2c_dw/`;
  the core itself and its relation to the Zephyr build's driver are in section 12 of the
  [design](../../design/imu-raw-mqtt-design.md)). The FreeRTOS build implements the core's
  `port/i2c_dw_os.h` boundary in its FreeRTOS binding, `common/drivers/i2c_dw/port/freertos/`:
  `i2c_dw_os_impl.h` (semaphores, clock, sleeps, interrupt masking) and `i2c_dw_instance.h` (the layer's
  devices with Zephyr's I2C API, the interrupt registration, reset, pins). The boundary is listed in the
  "I2C0" table of [`nuttx/drivers/README.md`](../../nuttx/drivers/README.md).
- **The binding's kernel services** are FreeRTOS's own, not the compatibility layer's. Each semaphore
  (`device_sync_sem`, `bus_sem`, the DMA reads' `rp2040_dma_sem`) is a count with its limit and a list of
  waiting tasks in the driver's own memory (no FreeRTOS queue, no heap), set up by the instance's start
  (`i2c_dw_i2c0_initialize()`); `bus_sem` has the limit 1 (no priority inheritance, as Zephyr's `k_sem`;
  its waiters by priority). A give hands the count to the first waiter, as Zephyr's, and wakes it with its
  task notification of the driver's own index 3 (`vTaskNotifyGiveIndexedFromISR()` and the switch at the end
  of an interrupt handler, `xTaskNotifyGiveIndexed()` from a task; `configTASK_NOTIFICATION_ARRAY_ENTRIES` is
  4). A waiter decides by its done flag, not by the wake-up, so a give between its look at the count and its
  block is not lost and a notification left from before only makes it wait again for the rest of its time.
  A timed wait blocks for the time rounded up to ticks plus one tick, as Zephyr's `k_sem_take(K_MSEC())`
  (the transfer's 1000 ms are 1001 ticks of 1 ms, the abort's 1 ms 2), the sleeps the same with
  `vTaskDelay()`, and the clock is the TIMER (`time_us_64()`) in ms, not the tick. The only things of
  Zephyr's left are the API it is used with: the layer's devices and `<zephyr/drivers/i2c.h>`.
- **Refused requests**: a transfer with a message of no bytes returns `-ENOTSUP` before anything is sent
  (the core, as the Zephyr build's driver: the controller cannot transfer no bytes; `i2c scan` then probes
  with one-byte reads, as in the Zephyr build). A request for a bus recovery (`i2c_recover_bus()`, `i2c recover`)
  is `-ENOTSUP`, as the Zephyr build's driver without a recover-bus callback (the board has no bus
  recovery). As in the Zephyr build's driver there is no `get_config`: `i2c_get_config()` returns `-ENOSYS`,
  so `i2c speed` calls `i2c_configure()` with `I2C_MODE_CONTROLLER` and the new speed class (the later
  transfers run at it).
- **The instances** are chosen as in the NuttX build's Kconfig: `../common/pico_sdk/drivers/i2c_dw/i2c_dw.cmake` defines
  `CONFIG_I2C_DW_I2C0` always (the LSM6DSOX is on it; `../common/pico_sdk/drivers/lsm6dso/lsm6dso.cmake` stops the
  configuration without it) and `CONFIG_I2C_DW_I2C1` with the CMake option `I2C_DW_I2C1` (off by
  default). Each instance has its own device, interrupt handler, speed class and start-up
  (`i2c_dw_i2c0_initialize()`, `i2c_dw_i2c1_initialize()` in `i2c_dw_controllers_rp2040.h`) under its symbol;
  the API, the pins' setup, the reset and the interrupt's connection are shared.
- **I2C1** (nothing is connected to it on the board) is the device `i2c1`
  (`DEVICE_DT_GET(DT_NODELABEL(i2c1))`, name `i2c@40048000`), with the same Zephyr I2C API as `i2c0`, on
  GPIO18 (SDA, Arduino D6) and GPIO19 (SCL, D7) with the internal pull-ups (`I2C1_SDA_GPIO`,
  `I2C1_SCL_GPIO` in `i2c_dw_instance.h`), always on the interrupts (the DMA reads and `i2c_dma` are
  I2C0's). It starts after I2C0 in `usb_console_init()`; a failure is logged and leaves `i2c1` not ready. It is not
  in the layer's device list (`device_get_binding()`), so the `i2c` command does not reach it. I2C1 has been built and host-tested but never used with a real device.
- **The DMA reads of I2C0** use the Pico SDK's own DMA (`hardware_dma`, `hardware_irq`) through the core's
  second boundary, `port/i2c_dw_dma_os.h`, implemented in `port/pico_sdk/i2c_dw_dma_os_rp2040.c`. Per read, two
  channels come from `dma_claim_unused_channel(false)` (none free: the read takes the interrupts and counts a
  fallback); they are set up with the SDK's channel configuration (TX: 16-bit command words to IC_DATA_CMD,
  DREQ 32; RX: bytes from it, DREQ 33), started with `dma_channel_start()` and given back with
  `dma_channel_unclaim()`. Nothing is reset at start-up.
- **Their interrupt and stop**: the channels' interrupts go to DMA_IRQ_1, on a handler added with
  `irq_add_shared_handler()` (the default order priority) that looks only at the channels the read holds.
  A stop writes the channel's bit to CHAN_ABORT and the core waits for it with a bound (100 ms, then the
  controller is quarantined); the SDK's `dma_channel_abort()` is not used, as it would wait without one.
- **Other code that uses DMA**: other code of this build may use the DMA the usual way: claim channels with
  the SDK's allocation, and share the interrupt lines with `irq_add_shared_handler()`. It must not: touch the
  two channels a read holds (claimed in the SDK's record during the read); install an exclusive handler on
  DMA_IRQ_1 (`irq_set_exclusive_handler()` panics when a shared one is there) or acknowledge other channels'
  flags in its handler of a shared line; reset the DMA block or write the whole of INTE0/INTE1, INTS0/INTS1 or
  CHAN_ABORT (only its own channels' bits); claim all twelve channels for good if the reads are to keep the
  DMA (they then fall back to the interrupts).
- **The NINA's SPI by DMA** (`common/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c`): a transfer of at least
  64 bytes (the SOCKET_SEND bodies, the long reads) goes through two channels claimed with
  `dma_claim_unused_channel(false)` at the bus's first use (the normal mode only; without two free channels every
  transfer stays polled) and kept: RX first (SSPDR to the buffer, or to one sink byte, DREQ 19), then SSPDMACR's
  TXDMAE and RXDMAE, then TX (the buffer to SSPDR, DREQ 18), 8-bit transfers. The calling task sleeps on the
  NINA's task notification (index 2) until the RX channel's interrupt on DMA_IRQ_1 (a second handler added with
  `irq_add_shared_handler()`, which looks only at its own channel), then SSPDMACR is cleared and SPI1 is left as
  the polled calls leave it (RX FIFO empty, overrun flag cleared). A transfer that does not end within 2 ms +
  LEN / 256 ms has both channels aborted and gives `-EIO`; a lost byte (the overrun flag) and a bus error of a
  channel also give `-EIO`. The core passes the error on as the request's (a socket call fails with `EIO`).
  Each of the three is logged (`dmesg`: `<err> nina_bus: SPI1 DMA did not end in ...`, `SPI1 RX overrun by DMA
  ...`, `SPI1 DMA bus error ...`), at most once per 5 s each with the number held back. Channels that do not
  stop after an abort, and a start without two free channels, are logged once and leave every transfer
  polled. Shorter transfers and those before the scheduler runs are polled.
- **Shared interrupt handlers**: the Pico SDK keeps them in a fixed number of slots for all lines
  (`PICO_MAX_SHARED_IRQ_HANDLERS`, 6 by default with stdio over USB), and adding one more than that ends in
  `hard_assert()`. Six are in use: USBCTRL_IRQ's two (TinyUSB, stdio_usb), IO_IRQ_BANK0's two raw handlers
  (`gpio_os_rp2040.c`, the NINA's `nina_os.c`) and DMA_IRQ_1's two (I2C0's reads, the NINA's SPI). The
  application sets 8 (`common/pico_sdk/app/usb-console/usb-console.cmake`; 12 bytes of RAM per slot).
- Interrupt priorities stay at the Pico SDK's default 0x80, so all device interrupts are at one level (as
  on NuttX; the Zephyr build uses 0xC0).
- The LSM6DSOX runs on the shared driver (`common/drivers/lsm6dso/`, the Zephyr build's driver: its core
  `lsm6dso.c`, `lsm6dso_stream.c` (the FIFO stream), `lsm6dso_cal.c` and `lsm6dso_fifo.c`, and its Zephyr
  binding `port/zephyr/`, with the core's OS part `port/zephyr/lsm6dso_os_impl.h` and `lsm6dso_os.c` (the
  stream's work queue `imu_fifo_worker` and INT1's interrupt) and the stream's user part
  `lsm6dso_stream_req_impl.h` (RTIO), on the compatibility layer; its extended API
  `<zephyr/drivers/sensor/lsm6dso.h>` is `include/lsm6dso.h`, through `common/compat`), ST's
  `lsm6dso_reg.c` (fetched to `freertos/stmemsc/` by `setup.sh`) and the stmemsc glue
  (`common/drivers/stmemsc/`); one device is made with the board's values
  (`common/drivers/lsm6dso/port/pico_sdk/lsm6dso_instance.h`, before `port/zephyr/` on the include path).
- GPIO (`common/pico_sdk/drivers/compat/rp2040/gpio_os_rp2040.c`): GPIO bank 0 under the layer's `gpio0`, driven with the same
  Pico SDK calls as Zephyr's `gpio_rpi_pico.c`. The interrupt handler is added with
  `gpio_add_raw_irq_handler_masked()`; only the LSM6DSOX's INT1 (GPIO24) can interrupt through `gpio0`
  (other pins get `-ENOTSUP`; the NINA's READY and IRQ line are on the NINA binding's handler).

## Acquisition, time synchronization, MQTT

The acquisition service (`imu_stream.c`), the time synchronization (`time_sync.c`, `time_offset.c`), the
transport (`imu_raw_mqtt.c`, `imu_raw_status.c`) and the calibration (`imu_cal.c`) are the Zephyr build's
files in `common/imu/`, built in by `../common/pico_sdk/drivers/imu/imu.cmake`. The MQTT client is `common/imu/mqtt_min.c` and
`mqtt_min_sock.c`, shared with the NuttX and ThreadX builds.

- The wall clock (`common/pico_sdk/drivers/compat/zephyr_sys_clock.c`) is the TIMER plus a base (built as in the NuttX
  build); newlib's `gettimeofday()` reads the same clock. `date` shows it in UTC.
- `time_sync.c` reads the TIMER with `rp2040_timer_us()` of `common/chip/rp2040`, as the layer's clocks do.
- The chip ID (the first 8 bytes of the session ID) is `pico_get_unique_board_id()`, the same flash ID as
  the Zephyr build's `hwinfo_get_device_id()` (`common/pico_sdk/drivers/compat/rp2040/zephyr_hwinfo.c`).

## Build options

Given as `freertos/script/build.sh -DNAME=VALUE` (kept until `--clean`).

| Option | Default | Meaning |
|---|---|---|
| `I2C_DW_I2C1` | OFF | Builds in I2C1, the device `i2c1` (above) |

## Host tests

`freertos/script/host_tests.sh [-v]` runs, after the tests of the shared sources
(`common/tests/run.sh`), these on FreeRTOS's POSIX port. No board is needed.

| Test | Subject |
|---|---|
| `compat` | The layer's kernel (`common/tests/zephyr_kernel/zephyr_kernel_test.c`, 22 items: mutexes, semaphores, sleep, clock, `k_poll`, work, spinlock, atomic, static threads, time slices; wake-ups from the tick interrupt; on this port the checks of real time, of a hardware interrupt and of the tick's catch-up are skipped) |
| `shell` | Two shell sessions, line editing, output routing, `help` and the generic commands with their output |
| `kv` | The flash boundary and the key-value store (on fakes of the Pico SDK's flash functions) |
| `nina` | The NINA's SPI and line boundaries (with the long transfers by DMA on a fake DMA: the threshold, the end state of SPI1, the shared DMA_IRQ_1, the timeout and its abort, a lost byte, a bus error, a transfer after each, channels that do not stop, no free channels, the messages logged) and the FreeRTOS binding of `nina_os.h` (no compatibility layer; built twice, with a 1 ms and a 4 ms tick): the wait for READY on its interrupt, the waits for the IRQ line's edges (two tasks woken by one edge, the timeout), the shared bank interrupt, locks, the Wi-Fi thread's wake-up, sleeps' ticks, the clock across the tick's wrap |
| `i2c_dw` | I2C0's boundary and device, Zephyr's I2C API, the `i2c` command on it (the devices of its arguments, `speed` and the speed of the transfers after it, `read_byte`, `write_byte`, `read` in hex, an unknown device, `scan` refused, `recover`) (built twice: i2c0 only, and with `CONFIG_I2C_DW_I2C1`, which adds i2c1's start, pins, transfers, an unanswered address, long reads that never take the DMA, i2c0 unaffected, and transfers on both buses from two tasks); the binding's semaphores, waits' ticks, sleeps and clock; the DMA part (`i2c_dw_dma_os_rp2040.c`) on fakes of the Pico SDK's DMA: claims, setup, start, the shared handler (end, bus error, other users' channels), stop, release; the core's DMA reads on it: the read, the 32-byte limit, fallbacks, bus errors on TX and RX, timeout, the reads off, a stop that does not end (quarantine), the counters; the core's DMA command words |
| `gpio` | GPIO bank 0's boundary, `gpio0`, the FIFO interrupt |
| `imu` | Address conversions, `hwinfo` |
| `mqtt` | The MQTT transport (fake broker); then a run against the host's `imu-host check` (skipped without `host/.venv`) |
| `time_sync` | Wall clock, TIMER read, NTP exchanges, `ntp status` |
| `sockets` | The layer's socket part on fake implementations: the descriptor table, the registrations' order, the dispatch of every call, fcntl, poll (offloaded, other kinds, mixed), the resolver, `k_heap` |
| `socket` | The NINA's sockets on the layer: descriptors, wait loops, options, poll, the descriptor's lock, name resolution |
| `wifi` | Interface, events, credentials, reconnection, `wifi` and `ifconfig`, the scans by the firmware's bits (link kept, asynchronous, failures, one that never ends, the link lost during it, 3.0.2) |
| `telnet` | The Telnet protocol, one client, reconnections, `zperf` (with the host's python3 and iperf3; that part is skipped without iperf3) |
| `sensor_shell` | The `sensor` command, two sessions, calibration (skipped without ST's `lsm6dso_reg.c`) |

## Differences from the Zephyr and NuttX builds

The custom commands (`imu`, `kv`, `ntp`, `nina`, `dmesg`, `i2c_dma`, `sensor`, `wifi cred`), `i2c` and `zperf`
have the Zephyr build's syntax and output.

| Item | Zephyr build | NuttX build | FreeRTOS build | ThreadX build |
|---|---|---|---|---|
| Prompt | `uart:~$ ` on USB, `~$ ` on Telnet | `nsh$ ` | `freertos:~$ ` | `threadx:~$ ` |
| Version, restart, task list, time since start | `kernel version`, `kernel reboot`, `kernel ps`, `kernel uptime` | `uname -a`, `reboot`, `ps`, `uptime` | `version`, `reboot`, `ps`, `uptime` | As FreeRTOS |
| IP address, date | `net iface`, `date get` | `ifconfig`, `date` | `ifconfig`, `date` (UTC, show only) | As FreeRTOS |
| `wifi` | All of Zephyr's | `wifi cred` only (`wapi` connects) | `connect` (`-s`, `-p`, `-k 0/1`), `disconnect`, `scan`, `status`, `cred` | As FreeRTOS |
| Speed test | `zperf` (iperf3 protocol, TCP and UDP) | `iperf` (iperf2 protocol) | `zperf` (iperf3 protocol, TCP only) | As FreeRTOS |
| BOOTSEL from the shell | `bootsel` | `bootsel` | `bootsel` | `bootsel` |
| `sensor get` | Returns at once; the result comes after the prompt | Waits up to 2 s for the result | As NuttX | As NuttX |
| Right after a Telnet connection | Sends nothing | Sends the prompt | Sends nothing | Sends nothing |
| Echo on Telnet | No | Yes | No | No |
| Error numbers (errno) | Zephyr's (`EILSEQ` 138) | NuttX's (`EILSEQ` 84) | newlib's (as Zephyr's; `EILSEQ` 138) | newlib's (as FreeRTOS) |
| Priorities | Zephyr | `118 − Zephyr` | `20 − Zephyr` | `15 − Zephyr` (ThreadX's number Zephyr + 16, 0 the most urgent) |
| Device interrupt priority | 0xC0 | 0x80 | 0x80 | 0x80 |
| Kernel clock | SysTick (late while sending) | Tickless on the TIMER | 1 kHz SysTick (catches up with the TIMER after interrupts were disabled); `k_uptime` from the TIMER | As FreeRTOS |
| Saving the divider | At switches, only when DIRTY (local patch) | At switches, only when DIRTY (patch 0009) | Four words at every switch (the Pico SDK port) | None (ThreadX's Cortex-M0 port) |
| The NINA's READY | Edge interrupt (GPIO10, read again every 10 ms) | Edge interrupt (GPIO10) | Edge interrupt (GPIO10, read again every 10 ms) | Edge interrupt (GPIO10, read again every 10 ms) |
| Sockets | Offload (the calling thread waits) | usrsock and `nina_event` | The Zephyr build's offload on the layer's socket layer (the calling task waits) | As FreeRTOS |
| RAM log | 4 KiB | 8 KiB | 4 KiB | 4 KiB |
| After a fatal error | Restart, report in `dmesg` | Restart, report in `dmesg` | Restart, record in `dmesg`, `fault` and the banner | As FreeRTOS |
| `ifconfig` netmask while disconnected | – | Kept | 0.0.0.0 | 0.0.0.0 |
