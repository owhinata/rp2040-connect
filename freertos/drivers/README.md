# Drivers and compatibility layer (FreeRTOS build)

English | [日本語](README_ja.md)

The internals of the FreeRTOS build (`freertos/`). How to use it and the list of commands are in
[`freertos/README.md`](../README.md), the summary of the design in §13 of the
[design document](../../design/imu-raw-mqtt-design.md). The details are in the comment at the top of each
file; this page shows where the files are and how they relate to the Zephyr and NuttX builds.

Most of the FreeRTOS build's sources are in `common/`, which the three builds share (acquisition,
transport, time synchronization, calibration, key-value store, the LSM6DSO, NINA and I2C0 drivers, the
shell's command handling). The shared sources are written on Zephyr's API, so the FreeRTOS build, like the
NuttX build, runs them on a compatibility layer that provides Zephyr's API under Zephyr's names. The
OS-independent part of the layer is `common/compat/`, the FreeRTOS part `freertos/drivers/compat/`.

## Directories

| Place | Contents |
|---|---|
| `app/usb-console/` | Start-up (`main.c`), the commands of this build only (`commands.c`), the kernel configuration (`FreeRTOSConfig.h`), the priority mapping (`priorities_os.h`), time (`os_clock.h`), the tick hook (`tick.c`), the CPU load (`idle.c`), fatal errors (`fault.[ch]`), the free interrupt stack (`irq_stack.c`) |
| `drivers/compat/` | The FreeRTOS part of the layer: the kernel (`zephyr_kernel.c`, `zephyr_mutex.c`, `zephyr_os*.h`), the choice among tasks of one priority (`zephyr_os_sched.h`), the network interface (`zephyr_net_if.c`), the wall clock (`zephyr_sys_clock.c`), the sensor clock, the prelude header (`common_prelude.h`), the self-test (`zephyr_selftest.[ch]`), the RP2040 part (`rp2040/`: GPIO bank 0, chip ID) |
| `drivers/shell/` | Line editing (`cli.c`), the consoles and each task's output (`nano_console.c`), the replacement of the Pico SDK's stdio driver (`nano_stdio.c`), the command table and execution (`nano_shell.c`), a copy of Zephyr's argument split (`shell_argv.c`), getopt per session (`getopt_state.c`) |
| `drivers/libc/` | Wrappers of newlib's FILE functions (`stdio_wrap.c`), the socket headers newlib lacks (`include/`, types and constants only), `inet_pton` and `inet_ntop` (`inet.c`) |
| `drivers/log/` | The RAM log (`log_ram.c`; the ring is `common/log/log_ring.c`) |
| `drivers/kv/` | The flash boundary of the key-value store (`nano_flash_kv.c`) and the linker script override (`ld/`, `kv.cmake`) |
| `drivers/nina/` | The NINA-W102's SPI1 and lines (`nano_nina.c`), the `nina_os.h` boundary (`nina_os.c`), sockets (`nina_socket.c`), the interface (`nina_netif.c`) |
| `drivers/i2c_dw/` | The FreeRTOS part of the I2C0 and DMA core (`common/drivers/i2c_dw/`), self-test |
| `drivers/lsm6dso/` | The LSM6DSOX device (`lsm6dso_instance.h`) and its initialization (`nano_lsm6dso.c`), self-test |
| `drivers/imu/` | Builds in the acquisition service, MQTT and time synchronization (`common/imu/`; `imu.cmake`), the prelude of `time_sync.c` |
| `drivers/wifi/` | Builds in Wi-Fi (`common/drivers/nina/nina_net.c`, credentials, reconnection) and `ifconfig` |
| `drivers/telnet/` | The Telnet shell (`nano_telnet.c`) |
| `drivers/zperf/` | Builds in `zperf` (the code is `common/compat/zperf_iperf3.c` and `zephyr_zperf_shell.c`) |
| `drivers/sensor_shell/` | The FreeRTOS part of the `sensor` command (`sensor_shell_os.[ch]`) |
| `drivers/include/` | The FreeRTOS part of `app_log.h` |
| `tests/` | Host tests ("Host tests" below) |

Shared files only the FreeRTOS build compiles: `common/compat/zephyr_wifi_shell.c` (`connect`,
`disconnect`, `scan` and `status` taken from Zephyr's `wifi_shell.c`), `zephyr_zperf_shell.c` and
`zperf_iperf3.[ch]` (`zperf`), `common/drivers/nina/nina_shell.c` (`nina version`),
`common/drivers/i2c_dw/i2c_dma_shell.c` (`i2c_dma`), `common/log/fault_record.[ch]` (the format of the
fatal error record).

## The FreeRTOS part of the Zephyr compatibility layer

What the layer provides is listed at the top of `common/compat/zephyr/kernel.h`; how it is built on
FreeRTOS is in the comments at the top of `drivers/compat/zephyr_os.h` and `zephyr_kernel.c`. These
assumptions of Zephyr are reproduced:

- **Mutexes** (`zephyr_mutex.c`): recursive, with priority inheritance. An all-zero `k_mutex` is
  unlocked and is created by its first lock (Zephyr's MQTT library and settings rely on it). The return
  values are Zephyr's.
- **Semaphores and `k_poll`**: Zephyr's rules (a give hands the count to the highest-priority waiter,
  `k_sem_reset()` ends the waits with `-EAGAIN`) on a count of the layer's own; waits use a task
  notification (index 1; 0 is left to the Pico SDK and the application). A give from an interrupt
  handler asks for a switch at the end of the handler, so the woken task runs right after it.
- **Static threads**: the tasks of `K_THREAD_DEFINE()` are started by the start-up task (`main`) in
  Zephyr's order.
- **Time**: `k_uptime_*()`, `k_cycle_*()` and `time_sync_mono_us()` come from the RP2040's TIMER (1 MHz),
  not from the tick (`os_clock.h`).

Left as approximations:

| Item | Zephyr | FreeRTOS build |
|---|---|---|
| Priorities | Lower runs first; cooperative -16..-1, preemptible 0..14 | `20 − Zephyr` (preemptible 0..14 become 20..6, cooperative -1 becomes 21). `Tmr Svc` 31, idle 0 |
| Cooperative threads | Not preempted | Approximated by a higher priority (a still higher task preempts; as on NuttX) |
| Priority inheritance | Follows a chain of holders, adjusted at each unlock | One level; dropped when the holder has released all its mutexes |
| Choice among one priority | A preempted thread goes on; 20 ms time slices | Zephyr's rule by trace macros in `zephyr_os_sched.h` (they name internals of kernel V11.3.3); 20 ms slices by the tick hook (`tick.c`). The order of tasks made ready is FreeRTOS's |
| Tick and sleeps | 100 µs; a relative wait is "rounded up to ticks, plus one tick" | 100 µs (`configTICK_RATE_HZ` 10000, 32 bits), the same rounding. A periodic SysTick (not tickless) |

- Built with `-DNANO_SCHED_ZEPHYR=OFF`, the choice among tasks of one priority is FreeRTOS's own (for
  comparison).
- The 32-bit tick wraps after about 4.97 days. While interrupts are disabled (a flash erase) ticks are
  lost: `vTaskDelay()`, delayed work and time slices are late by that much; `k_uptime_*()`, from the
  TIMER, is not.
- `portTICK_PERIOD_MS` (0 at this rate) and the Pico SDK's `sleep_ms()` (which spins the last 150 µs) are
  not used.

## Tasks

The priorities are in `common/compat/priorities.h` (the Zephyr build's values and `20 − Zephyr`) and
`app/usb-console/src/priorities_os.h` (the tasks of this build only). Stacks are in bytes. Interrupts
use a stack of their own (MSP, 2048 bytes; the `(interrupts)` row of `ps`).

| Task | Priority (Zephyr) | Stack | Role |
|---|---|---|---|
| `Tmr Svc` | 31 (–) | 1024 | The kernel's timer task; wakes the waiters of Pico SDK locks released in interrupt handlers |
| `net_mgmt`, `sysworkq` | 21 (-1) | 2048, 1024 | Network events; the system work queue (the work of the LSM6DSOX FIFO interrupt) |
| `main` | 20 (0) | 2048 | The second half of the start-up ("Start-up" below); ends when done |
| `sensor_shell_processing_tid`, `RTIO WQ 0` | 20 (0) | 3072, 1024 | Printing `sensor` results; RTIO work |
| `nina_scan_tid`, `nina_connect_tid`, `wifi_reconnect_tid` | 12 (8) | 1536, 1024, 2048 | Wi-Fi scan, connection, automatic reconnection |
| `nina_monitor_tid`, `time_sync_tid`, `imu_stream_tid` | 11 (9) | 1536, 2048, 2048 | Link monitor, time synchronization, acquisition |
| `imu_fifo_worker` | 10 (10) | 2048 | FIFO reads |
| `imu_raw_mqtt_tid` | 9 (11) | 2048 | MQTT transport |
| `console`, `shell_telnet`, `telnet_service`, `zperf_server_tid` | 6 (14) | 3072, 3072, 2048, 2048 | The USB and Telnet shells (commands run in these tasks), Telnet accepts, the server of `zperf tcp download` |
| `heartbeat` | 1 (–) | 1024 | Blinks the LED (GPIO6) |
| `IDLE` | 0 (–) | 1024 | Idle; measures the CPU load (`idle.c`) |

The heap is FreeRTOS's heap_4 with 32 KiB (`free`). Task stacks are allocated statically.

**Start-up** (`main.c`): `main()` sets up the RAM log, reads the fatal error record, sets up the console,
the layer, the NINA's SPI, DMA, I2C0 and the flash of the key-value store; then the task `main` starts the
LSM6DSOX, the NINA's network interface, the Wi-Fi credentials, Telnet, the `K_THREAD_DEFINE()` threads,
the key-value store, the calibration, the time synchronization and the automatic connection, in this
order (the Zephyr build's initialization order and the NuttX build's start-up order).

**CPU load** (`idle.c`): the idle hook disables interrupts, opens the window, waits with `WFI`, closes the
window and enables interrupts (the measurement of the Zephyr build's `CONFIG_CPU_LOAD` and the NuttX
build's `up_idle()`). The tick interrupt every 100 µs counts as load (the Zephyr and NuttX builds have no
such interrupt). The CPU column of `ps` is cumulative since the start, timed by the TIMER.

## Shell and consoles

- Commands are written with Zephyr's shell API (`SHELL_CMD_REGISTER()` and so on); matching, help,
  wildcards and the argument count check are done by the shared `common/shell/zshell.c`. The command table
  is `nano_shell.c`, the commands of this build only are in `app/usb-console/src/commands.c`.
- A console (`nano_console.c`) is a session's terminal (USB or Telnet); each task is bound to one console
  (a thread local storage pointer), and its `printf()` and `fprintf(stderr)` go there. Tasks bound to no
  console write to USB. The replacement of the Pico SDK's stdio driver (`nano_stdio.c`) and `--wrap` of
  newlib's FILE functions (`drivers/libc/stdio_wrap.c`) route every output path through it.
- USB (`main.c`): the prompt appears when the terminal opens the port. When the terminal stops reading,
  the Pico SDK's driver waits up to 500 ms for room in its 64-byte buffer and drops the rest.
- Telnet (`drivers/telnet/nano_telnet.c`): port 23, one client at a time. As Zephyr's `shell_telnet.c`,
  it sends nothing on connection, takes Telnet commands out without answering, and does not echo. Where it
  differs from Zephyr is listed at the top of the file (it looks once more whether the first client has
  closed before refusing a second, a new client starts with an empty line, and so on).
- The getopt state is per session (`getopt_state.c`): USB and Telnet can run commands at the same time.

## RAM log and fatal errors

- The RAM log (`drivers/log/log_ram.c`) is 8 KiB in RAM that the start-up code does not initialize
  (`__uninitialized_ram`). It survives the watchdog's restart, `reboot` and the RESET button, not a power
  cycle. `LOG_*` lines are formatted in the caller's context (interrupt handlers included), timed by the
  TIMER. `dmesg` prints it.
- Fatal errors (`app/usb-console/src/fault.[ch]`): `panic()`, HardFault, `configASSERT()` and `assert()`,
  stack overflow (`configCHECK_FOR_STACK_OVERFLOW` 2), heap exhaustion and `_exit()` end in one exit,
  which keeps a record (reason, PC, LR, SP, exception frame, task, time) in RAM and in the watchdog's
  scratch registers 0..3 and restarts the board with the watchdog. The next start writes the record to
  `dmesg`, and `fault` and the console's banner show it. The record's format is
  `common/log/fault_record.[ch]`.

## The key-value store's flash

As in the Zephyr and NuttX builds, the key-value store is in the last 8 KiB of the flash
(`0xffe000`..`0xffffff`, two 4 KiB sectors) in the format of Zephyr's NVS and settings (the copies are in
`common/compat/subsys/`, the format is described in "Key-value store: copies of Zephyr's NVS and settings"
of [`nuttx/drivers/README.md`](../../nuttx/drivers/README.md)). The FreeRTOS build implements the flash
boundary (the four functions of `common/compat/nano_flash_kv.h`, `drivers/kv/nano_flash_kv.c`).

- The linker script (`drivers/kv/ld/`, `kv.cmake`) sets the FLASH length to `0xffe000`, keeping the image
  out of the area, and checks `__kv_start`, `__kv_end` and the end of the image with `ASSERT`.
- A program or erase request is checked against the area three times: in `zephyr_flash_kv.c`, in the
  boundary's function, and right before the flash is touched (with interrupts disabled). Until the linker
  symbols are found equal to the constants, programs and erases are refused.
- `flash_range_program()` and `flash_range_erase()` are called through `flash_safe_execute()` with
  `PICO_FLASH_ASSUME_CORE1_SAFE=1`, which disables this core's interrupts only (core 1 is not used).
  Interrupts stay disabled during an erase, and ticks are lost.
- Data that is neither Zephyr's NVS nor erased is left alone and logged to `dmesg`; the store's calls
  return `-EILSEQ` (138 with newlib). `kv wipe confirm` erases it (`common/compat/kv_store_wipe.c`).

## NINA-W102, sockets, Wi-Fi

- The SPI transport, the socket table and commands, and name resolution are the shared core
  (`common/drivers/nina/nina_spi.c`, `nina_sock.c`, `nina_dns.c`). The FreeRTOS build implements the
  boundaries `nano_nina.h` (`drivers/nina/nano_nina.c`: SPI1 polled at 7.8125 MHz and the CS, READY, RESET
  and IRQ lines, set up in the order and with the values of Zephyr's drivers) and `nina_os.h`
  (`nina_os.c`: the recursive lock with priority inheritance, the name resolution lock, the "rounded up,
  plus one tick" sleep, the ms clock). The boundaries mean what the "NINA-W102" tables of
  [`nuttx/drivers/README.md`](../../nuttx/drivers/README.md) say.
- Sockets (`drivers/nina/nina_socket.c`): the layer's `zsock_*()` calls go to the shared core through a
  table of 10 descriptors. As the Zephyr build's socket offload, the calling task asks the NINA and waits
  (there is no event thread like the NuttX build's usrsock and `nina_event`). The descriptors are not
  newlib's; only the `zsock_*()` calls take them. The types and constants newlib lacks are in
  `drivers/libc/include/`, with Linux's values (`EAI_*` are Zephyr's negative values).
- Wi-Fi: the Zephyr build's `common/drivers/nina/nina_net.c`, the credentials (a copy of Zephyr's
  `wifi_credentials`) and the automatic reconnection (`common/imu/wifi_profile.c`) run on the layer's
  `net_if`, `net_mgmt` and `wifi_mgmt` (`zephyr_net_if.c`, `drivers/nina/nina_netif.c`). The `wifi` command
  is `common/compat/zephyr_wifi_shell.c`, keeping only the options this board can use; `wifi cred` is an
  unchanged copy of Zephyr's `wifi_credentials_shell.c`. `ifconfig` (`drivers/wifi/nano_ifconfig.c`)
  prints in the NuttX build's format.

## I2C0 and DMA, LSM6DSOX, GPIO interrupt

- I2C0 and DMA are the shared core (`common/drivers/i2c_dw/nano_i2c.c`, `nano_dma.c`); the FreeRTOS build
  implements the `i2c_dw_os.h` boundary in `drivers/i2c_dw/i2c_dw_os_impl.h` (semaphores, clock, interrupt
  masking) and `nano_i2c_instance.h`, `nano_dma_instance.h` (the layer's `i2c0` device with Zephyr's I2C
  API, the interrupt registration, resets, pins). The boundary is listed in the "I2C0" table of
  [`nuttx/drivers/README.md`](../../nuttx/drivers/README.md).
- Interrupt priorities stay at the Pico SDK's default 0x80, so all device interrupts are at one level (as
  on NuttX; the Zephyr build uses 0xC0).
- The LSM6DSOX runs on the shared driver (`common/drivers/lsm6dso/`), ST's `lsm6dso_reg.c` (fetched to
  `freertos/stmemsc/` by `setup.sh`) and the stmemsc glue (`common/drivers/stmemsc/`); one device is made
  with the board's values (`drivers/lsm6dso/lsm6dso_instance.h`).
- GPIO (`drivers/compat/rp2040/nano_gpio.c`): GPIO bank 0 under the layer's `gpio0`, driven with the same
  Pico SDK calls as Zephyr's `gpio_rpi_pico.c`. The interrupt handler is added with
  `gpio_add_raw_irq_handler_masked()`; only the LSM6DSOX's INT1 (GPIO24) can interrupt (other pins get
  `-ENOTSUP`).

## Acquisition, time synchronization, MQTT

The acquisition service (`imu_stream.c`), the time synchronization (`time_sync.c`, `time_offset.c`), the
transport (`imu_raw_mqtt.c`, `imu_raw_status.c`) and the calibration (`imu_cal.c`) are the Zephyr build's
files in `common/imu/`, built in by `drivers/imu/imu.cmake`. The MQTT client is `common/imu/mqtt_min.c` and
`mqtt_min_sock.c`, shared with the NuttX build.

- The wall clock (`drivers/compat/zephyr_sys_clock.c`) is the TIMER plus a base (built as in the NuttX
  build); newlib's `gettimeofday()` reads the same clock. `date` shows it in UTC.
- `time_sync.c` uses a name that the Pico SDK's `TIMER_BASE` also has; a prelude
  (`drivers/imu/time_sync_prelude.h`) keeps them apart.
- The chip ID (the first 8 bytes of the session ID) is `pico_get_unique_board_id()`, the same flash ID as
  the Zephyr build's `hwinfo_get_device_id()` (`drivers/compat/rp2040/zephyr_hwinfo.c`).

## Build options

Given as `freertos/script/build.sh -DNAME=VALUE` (kept until `--clean`).

| Option | Default | Meaning |
|---|---|---|
| `NANO_SELFTEST` | OFF | Builds in the `selftest` command (below) |
| `NANO_SCHED_ZEPHYR` | ON | OFF leaves the choice among tasks of one priority to FreeRTOS (`selftest`'s `timeslice` and `timeslice_preempt` are skipped) |

## Self-test (`selftest`)

A test command only in builds with `-DNANO_SELFTEST=ON`. Run it with the measurement stopped and the
board at rest. It prints, as `N of M tests failed`, the results for the layer
(`drivers/compat/zephyr_selftest.c`: mutexes, semaphores, sleep, clock, wake-ups from interrupts,
`k_poll`, work, spinlock, atomic, static threads, time slices; 22 items), I2C0
(`drivers/i2c_dw/i2c_dw_selftest.c`: 6 items) and the LSM6DSOX (`drivers/lsm6dso/lsm6dso_selftest.c`:
3 items).

## Host tests

`freertos/script/host_tests.sh [-v]` runs, after the tests of the shared sources
(`common/tests/run.sh`), these on FreeRTOS's POSIX port. No board is needed.

| Test | Subject |
|---|---|
| `compat` | The layer's self-test (the same `zephyr_selftest.c` as `selftest`) |
| `shell` | Two shell sessions, line editing, output routing |
| `kv` | The flash boundary and the key-value store (on fakes of the Pico SDK's flash functions) |
| `nina` | The NINA's SPI and line boundaries, kernel services |
| `i2c_dw` | I2C0's boundary and device, Zephyr's I2C API |
| `gpio` | GPIO bank 0's boundary, `gpio0`, the FIFO interrupt |
| `imu` | Address conversions, `hwinfo` |
| `mqtt` | The MQTT transport (fake broker); then a run against the host's `imu-host check` (skipped without `host/.venv`) |
| `time_sync` | Wall clock, TIMER read, NTP exchanges, `ntp status` |
| `socket` | `zsock_*()`, wait loops, options, poll, name resolution |
| `wifi` | Interface, events, credentials, reconnection, `wifi` and `ifconfig` |
| `telnet` | The Telnet protocol, one client, reconnections, `zperf` (with the host's python3 and iperf3; that part is skipped without iperf3) |
| `sensor_shell` | The `sensor` command, two sessions, calibration (skipped without ST's `lsm6dso_reg.c`) |

## Differences from the Zephyr and NuttX builds

The custom commands (`imu`, `kv`, `ntp`, `nina`, `dmesg`, `i2c_dma`, `sensor`, `wifi cred`) and `zperf` have
the Zephyr build's syntax and output.

| Item | Zephyr build | NuttX build | FreeRTOS build |
|---|---|---|---|
| Prompt | `uart:~$ ` on USB, `~$ ` on Telnet | `nsh$ ` | `freertos:~$ ` |
| Version, restart, task list, time since start | `kernel version`, `kernel reboot`, `kernel ps`, `kernel uptime` | `uname -a`, `reboot`, `ps`, `uptime` | `version`, `reboot`, `ps`, `uptime` |
| IP address, date | `net iface`, `date get` | `ifconfig`, `date` | `ifconfig`, `date` (UTC, show only) |
| `wifi` | All of Zephyr's | `wifi cred` only (`wapi` connects) | `connect` (`-s`, `-p`, `-k 0/1`), `disconnect`, `scan`, `status`, `cred` |
| Speed test | `zperf` (iperf3 protocol, TCP and UDP) | `iperf` (iperf2 protocol) | `zperf` (iperf3 protocol, TCP only) |
| BOOTSEL from the shell | No | No | `bootsel` |
| `sensor get` | Returns at once; the result comes after the prompt | Waits up to 2 s for the result | As NuttX |
| Right after a Telnet connection | Sends nothing | Sends the prompt | Sends nothing |
| Echo on Telnet | No | Yes | No |
| Error numbers (errno) | Zephyr's (`EILSEQ` 138) | NuttX's (`EILSEQ` 84) | newlib's (as Zephyr's; `EILSEQ` 138) |
| Priorities | Zephyr | `118 − Zephyr` | `20 − Zephyr` |
| Device interrupt priority | 0xC0 | 0x80 | 0x80 |
| Kernel clock | SysTick (late while sending) | Tickless on the TIMER | 10 kHz SysTick (late during erases); `k_uptime` from the TIMER |
| Sockets | Offload (the calling thread waits) | usrsock and `nina_event` | The calling task waits (no event thread) |
| RAM log | 4 KiB | 8 KiB | 8 KiB |
| After a fatal error | Restart, report in `dmesg` | Restart, report in `dmesg` | Restart, record in `dmesg`, `fault` and the banner |
| `ifconfig` netmask while disconnected | – | Kept | 0.0.0.0 |
