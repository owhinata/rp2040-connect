# Drivers and compatibility layer (FreeRTOS build)

English | [日本語](README_ja.md)

The internals of the FreeRTOS build (`freertos/`). How to use it and the list of commands are in
[`freertos/README.md`](../README.md), the summary of the design in §13 of the
[design document](../../design/imu-raw-mqtt-design.md). The details are in the comment at the top of each
file; this page shows where the files are and how they relate to the Zephyr and NuttX builds.

Most of the FreeRTOS build's sources are in `common/`, which the three builds share (acquisition,
transport, time synchronization, calibration, key-value store, the LSM6DSO, NINA and I2C drivers, the
shell's command handling). The shared sources are written on Zephyr's API, so the FreeRTOS build, like the
NuttX build, runs them on a compatibility layer that provides Zephyr's API under Zephyr's names. The
OS-independent part of the layer is `common/compat/`, the FreeRTOS part `freertos/drivers/compat/`.

## Directories

| Place | Contents |
|---|---|
| `app/usb-console/` | Start-up (`main.c`), the commands of this build only (`commands.c`), the kernel configuration (`FreeRTOSConfig.h`), the priority mapping (`priorities_os.h`), time (`os_clock.h`), the tick hook (`tick.c`; the catch-up's decision in `tick_catchup.h`), the CPU load (`idle.c`), fatal errors (`fault.[ch]`), the free interrupt stack (`irq_stack.c`) |
| `drivers/compat/` | The FreeRTOS part of the layer: the kernel (`zephyr_kernel.c`, `zephyr_mutex.c`, `zephyr_os*.h`, `k_heap` in `zephyr_heap.c`), the network interface and its start (`zephyr_net_if.c`, `zephyr_net_init.c`), Zephyr's socket layer (reduced copies of Zephyr's files in `subsys/net/lib/sockets/` and `lib/os/zvfs/`, their headers under `zephyr/`, the linker script's part `ld/`), the wall clock (`zephyr_sys_clock.c`), the prelude header (`common_prelude.h`), the self-test (`zephyr_selftest.[ch]`), the RP2040 part (`rp2040/`: GPIO bank 0, chip ID) |
| `drivers/shell/` | Line editing (`cli.c`), the consoles and each task's output (`nano_console.c`), the replacement of the Pico SDK's stdio driver (`nano_stdio.c`), the command table and execution (`nano_shell.c`), a copy of Zephyr's argument split (`shell_argv.c`), getopt per session (`getopt_state.c`) |
| `drivers/libc/` | Wrappers of newlib's FILE functions (`stdio_wrap.c`), the socket headers newlib lacks (`include/`, types and constants only), `inet_pton` and `inet_ntop` (`inet.c`) |
| `drivers/log/` | The RAM log (`log_ram.c`; the ring is `common/log/log_ring.c`) |
| `drivers/kv/` | The flash boundary of the key-value store (`nano_flash_kv.c`) and the linker script override (`ld/`, `kv.cmake`) |
| `drivers/nina/` | Only `nina.cmake`, which adds the NINA driver to the build. The driver is in `common/drivers/nina/`, and this build's parts are its FreeRTOS binding `common/drivers/nina/port/freertos/`: the `nina_os.h` boundary (`nina_os.c`), the passthrough's host side (`nina_passthrough_os.c`), SPI1 and the module's lines on the Pico SDK (`nina_bus_rp2040.c`), the passthrough's module side (`nina_passthrough_bus_rp2040.c`: UART1, the module's lines, the watchdog's scratch registers, the LED); the sockets and the interface are the Zephyr binding's (`common/drivers/nina/port/zephyr/nina_socket.c`, `nina_net.c`) |
| `drivers/i2c_dw/` | `i2c_dw.cmake`, which adds the I2C driver to the build (I2C0, and I2C1 as an option), and the self-test (`i2c_dw_selftest.[ch]`). The driver is in `common/drivers/i2c_dw/`, and this build's parts are its FreeRTOS binding `common/drivers/i2c_dw/port/freertos/`: the `i2c_dw_os.h` boundary on FreeRTOS and the TIMER (`i2c_dw_os_impl.h`), the devices with Zephyr's I2C API, their interrupts, resets and pins (`i2c_dw_instance.h`), their start (`i2c_dw_controllers_rp2040.h`), I2C0's DMA reads on the Pico SDK's DMA (`i2c_dw_dma_os_rp2040.c`) |
| `drivers/lsm6dso/` | The LSM6DSOX's build rules (`lsm6dso.cmake`), its initialization (`nano_lsm6dso.c`), self-test |
| `drivers/imu/` | Builds in the acquisition service, MQTT and time synchronization (`common/imu/`; `imu.cmake`), the prelude of `time_sync.c` |
| `drivers/wifi/` | Builds in Wi-Fi (the NINA core's `common/drivers/nina/nina_wifi.c` with its Zephyr binding `common/drivers/nina/port/zephyr/nina_net.c`, credentials, reconnection) and `ifconfig` |
| `drivers/telnet/` | The Telnet shell (`nano_telnet.c`) |
| `drivers/zperf/` | Builds in `zperf` (the code is `common/compat/zperf_iperf3.c` and `zephyr_zperf_shell.c`) |
| `drivers/sensor_shell/` | The FreeRTOS part of the `sensor` command (`sensor_shell_os.[ch]`) |
| `drivers/include/` | The FreeRTOS part of `app_log.h` |
| `tests/` | Host tests ("Host tests" below) |

Shared files only the FreeRTOS build compiles: `common/compat/zephyr_wifi_shell.c` (`connect`,
`disconnect`, `scan` and `status` taken from Zephyr's `wifi_shell.c`), `zephyr_zperf_shell.c` and
`zperf_iperf3.[ch]` (`zperf`), `common/log/fault_record.[ch]` (the format of the fatal error record).
`i2c_dma`'s `common/shell/i2c_dma_shell.c` and `nina`'s `common/shell/nina_shell.c` (`nina version`,
`nina passthrough`) are built by the Zephyr build too.

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
| Choice among one priority | A preempted thread goes on; 20 ms time slices, counted from the switch to the thread | FreeRTOS's: at every switch the next ready task of the priority in turn, so a preempted task goes behind the other ready tasks of its priority. 20 ms slices by the tick hook (`tick.c`) on a fixed grid of 20 ticks, not counted from the switch |
| Tick and sleeps | 100 µs; a relative wait is "rounded up to ticks, plus one tick" | 1 ms (`configTICK_RATE_HZ` 1000, 32 bits), the same rounding: a wait shorter than 1 ms lasts 1 to 2 ms. A periodic SysTick (not tickless) |

- The 32-bit tick wraps after about 49.7 days.
- While interrupts are disabled (a flash erase or program; about 40 to 50 ms per erase), SysTick keeps one
  interrupt pending and the other ticks are lost. The tick hook (`tick.c`, decision in `tick_catchup.h`)
  compares the count with the TIMER at every tick and, when it is a whole tick behind, pends SysTick
  again, so the port's usual handler counts the lost ticks one by one (wake-ups, switch requests, and the
  tick hook's timeouts of the layer and time slices run on each). The tick never gets ahead of the
  TIMER, so apart from the time interrupts are disabled it stays within one tick of it.
  `xTaskCatchUpTicks()` is not used (it can be called from tasks only, and the tick hook does not run for
  the ticks it adds). `main.c` checks that clk_sys is a whole number of ticks. `k_uptime_*()` is the TIMER.
- The Pico SDK's `sleep_ms()` (which spins the last 150 µs) is not used.

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
the layer, the NINA's SPI, I2C0 (first the DMA interrupt's handler for its reads) and the flash of the key-value store; then the task `main` starts the
LSM6DSOX, the NINA's network interface, the Wi-Fi credentials, Telnet, the `K_THREAD_DEFINE()` threads,
the key-value store, the calibration, the time synchronization and the automatic connection, in this
order (the Zephyr build's initialization order and the NuttX build's start-up order).

**CPU load** (`idle.c`): the idle hook disables interrupts, opens the window, waits with `WFI`, closes the
window and enables interrupts (the measurement of the Zephyr build's `CONFIG_CPU_LOAD` and the NuttX
build's `up_idle()`). The tick interrupt every 1 ms counts as load (the Zephyr and NuttX builds have no
such interrupt). The CPU column of `ps` is the share of the
interval since the previous `ps` (from USB or Telnet; since the start for the first) in all the time of
the interval, idle included, so the rows add up to about 100 % (type it at the start and at the end of
the interval of interest); timed by the TIMER.

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
  Interrupts stay disabled during an erase; the tick catches up with the TIMER afterwards (above).
- Data that is neither Zephyr's NVS nor erased is left alone and logged to `dmesg`; the store's calls
  return `-EILSEQ` (138 with newlib). `kv wipe confirm` erases it (`common/compat/kv_store_wipe.c`).

## NINA-W102, sockets, Wi-Fi

- The SPI transport, the socket table and commands, name resolution and the Wi-Fi operations are the
  shared core (`internal/nina_proto.c`, `nina_sock.c`, `nina_dns.c` and `nina_wifi.c` of `common/drivers/nina/`). The FreeRTOS build implements the
  boundaries `nina_bus.h` (`common/drivers/nina/port/freertos/nina_bus_rp2040.c`: SPI1 polled at 7.8125 MHz and the CS, READY, RESET
  and IRQ lines, set up in the order and with the values of Zephyr's drivers) and `nina_os.h`
  (the core's FreeRTOS binding `common/drivers/nina/port/freertos/nina_os.c`, written on FreeRTOS and the
  Pico SDK without the compatibility layer: FreeRTOS recursive mutexes with priority inheritance for the
  NINA lock, the name resolution lock and the Wi-Fi operations' lock, binary semaphores for the Wi-Fi
  threads' requests, the "rounded up, plus one tick" sleep, the TIMER in ms as the clock). Its objects are
  static and made at the first use of any of them.
  The boundaries mean what the "NINA-W102" tables of
  [`nuttx/drivers/README.md`](../../nuttx/drivers/README.md) say.
- The core's waits for READY sleep on a binary semaphore that READY's edge interrupt (GPIO10) gives, in
  waits of at most 10 ms, after which READY is read again. The binding has its own raw handler of
  `IO_IRQ_BANK0` for GPIO10 and GPIO2 (`gpio_add_raw_irq_handler_masked()`, beside the GPIO bank 0
  backend's for GPIO24; each handler acknowledges only its own pins). The Zephyr build does the same; the
  NuttX build waits without the 10 ms slices. Whether the interrupt is used is decided at run time: if
  `nina_os_ready_start()` fails after the first reset, READY is read every 100 µs instead (in all three
  builds; the FreeRTOS binding's does not fail).
- Sockets: the layer has Zephyr's socket layer down to its list of socket implementations (reduced
  copies of Zephyr's `sockets.c`, `getaddrinfo.c`, `socket_offload.c`, `zvfs_fdtable.c` and
  `zvfs_poll.c`; what each leaves out is in its header). An implementation registers itself with
  `NET_SOCKET_OFFLOAD_REGISTER()` or `NET_SOCKET_REGISTER()`; the linker script collects the
  registrations sorted by priority (`drivers/compat/ld/section_extra_post_text.incl`), and
  `zsock_socket()` takes the first whose family matches and whose `is_supported()` accepts. The other
  calls go through the descriptor table to the implementation's `struct socket_op_vtable` under the
  descriptor's mutex, `poll` through `ZFD_IOCTL_POLL_PREPARE` and `ZFD_IOCTL_POLL_OFFLOAD`, and
  `zsock_getaddrinfo()` to the registered resolver. The NINA's sockets are the Zephyr build's binding,
  compiled unchanged (`common/drivers/nina/port/zephyr/nina_socket.c`, priority 40): as in the Zephyr build,
  the calling task asks the NINA and waits (there is no event thread like the NuttX build's usrsock and
  `nina_event`). The table has 16 descriptors, 0 to 15 from the lowest free one (none kept for stdin,
  stdout and stderr, as in the Zephyr build); the NINA has ten sockets, so an eleventh gets `EMFILE`.
  The descriptors are not newlib's; only the `zsock_*()` calls take them. The types and constants newlib
  lacks are in `drivers/libc/include/`, with Linux's values (`EAI_*` are Zephyr's negative values); the
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
  (`common/drivers/nina/port/zephyr/nina_net.c`: Zephyr's Wi-Fi management and interface, the three threads),
  the credentials (a copy of Zephyr's `wifi_credentials`) and the automatic reconnection
  (`common/imu/wifi_profile.c`) run on the layer's `net_if`, `net_mgmt` and `wifi_mgmt` (`zephyr_net_if.c`;
  `zephyr_net_init.c` registers the interface at the start, as Zephyr's `net_init()`); this build has no
  network layer of its own, so it uses the Zephyr binding. The `wifi` command
  is `common/compat/zephyr_wifi_shell.c`, keeping only the options this board can use; `wifi cred` is an
  unchanged copy of Zephyr's `wifi_credentials_shell.c`. `ifconfig` (`drivers/wifi/nano_ifconfig.c`)
  prints in the NuttX build's format.

- The esptool passthrough (`nina passthrough confirm`): the NINA driver's core shared with the Zephyr and
  NuttX builds (`common/drivers/nina/nina_passthrough.c`), with its host side on TinyUSB and the Pico SDK
  (`common/drivers/nina/port/freertos/nina_passthrough_os.c`) and its module side (`common/drivers/nina/port/freertos/nina_passthrough_bus_rp2040.c`)
  (wiring and lines in "The esptool passthrough" of
  [`zephyr/app/usb-console/drivers/README.md`](../../zephyr/app/usb-console/drivers/README.md)). The command
  leaves a mark in the watchdog's scratch registers 0 to 3 and restarts; `main()` sees it before anything
  else and, with no task, scheduler, SPI1, NINA, I2C or KV store, relays between TinyUSB (`tud_task()`) and
  UART1 in one loop. From the module to the PC, UART1's receive interrupt moves the bytes into a 16 KiB
  buffer (from the FreeRTOS heap); that interrupt and the core's part of it run from RAM. The LED blinks at
  5 Hz. The core's host test is `nina_passthrough` of `common/tests/run.sh`.

## Divider, `memcpy` and `memset`, boot2

The Pico SDK's defaults. Integer division is `pico_divider` on the hardware divider, and the port
(`portUSE_DIVIDER_SAVE_RESTORE`) saves and restores the divider's four words at every switch (the Zephyr
and NuttX builds, with local patches, save it only in the middle of a division, DIRTY). A division by
zero gives the Pico SDK routines' results, the same in the three builds. `memcpy` and `memset` are
`pico_mem_ops`, the boot ROM's routines; boot2 is the Pico SDK board definition's, for the W25Q080
(divider 2, 62.5 MHz).

## I2C0 and DMA, LSM6DSOX, GPIO interrupt

- **The driver**: I2C0 and I2C1 are the I2C core shared with the NuttX build (`common/drivers/i2c_dw/`;
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
  (the core, as the Zephyr build's driver: the controller cannot transfer no bytes). The layer's I2C API
  has no `i2c_recover_bus()` (the board has no bus recovery).
- **The instances** are chosen as in the NuttX build's Kconfig: `drivers/i2c_dw/i2c_dw.cmake` defines
  `CONFIG_I2C_DW_I2C0` always (the LSM6DSOX is on it; `drivers/lsm6dso/lsm6dso.cmake` stops the
  configuration without it) and `CONFIG_I2C_DW_I2C1` with the CMake option `I2C_DW_I2C1` (off by
  default). Each instance has its own device, interrupt handler, speed class and start-up
  (`i2c_dw_i2c0_initialize()`, `i2c_dw_i2c1_initialize()` in `i2c_dw_controllers_rp2040.h`) under its symbol;
  the API, the pins' setup, the reset and the interrupt's connection are shared.
- **I2C1** (nothing is connected to it on the board) is the device `i2c1`
  (`DEVICE_DT_GET(DT_NODELABEL(i2c1))`, name `i2c@40048000`), with the same Zephyr I2C API as `i2c0`, on
  GPIO18 (SDA, Arduino D6) and GPIO19 (SCL, D7) with the internal pull-ups (`I2C1_SDA_GPIO`,
  `I2C1_SCL_GPIO` in `i2c_dw_instance.h`), always on the interrupts (the DMA reads and `i2c_dma` are
  I2C0's). It starts after I2C0 in `main()`; a failure is logged and leaves `i2c1` not ready. It is not
  in the layer's device list (`device_get_binding()`). With `-DNANO_SELFTEST=ON` the `selftest`
  command checks it (below). I2C1 has been built and host-tested but never used with a real device.
- **The DMA reads of I2C0** use the Pico SDK's own DMA (`hardware_dma`, `hardware_irq`) through the core's
  second boundary, `port/i2c_dw_dma_os.h`, implemented in `port/freertos/i2c_dw_dma_os_rp2040.c`. Per read, two
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
  (`common/drivers/lsm6dso/port/freertos/lsm6dso_instance.h`, before `port/zephyr/` on the include path).
- GPIO (`drivers/compat/rp2040/nano_gpio.c`): GPIO bank 0 under the layer's `gpio0`, driven with the same
  Pico SDK calls as Zephyr's `gpio_rpi_pico.c`. The interrupt handler is added with
  `gpio_add_raw_irq_handler_masked()`; only the LSM6DSOX's INT1 (GPIO24) can interrupt through `gpio0`
  (other pins get `-ENOTSUP`; the NINA's READY and IRQ line are on the NINA binding's handler).

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
| `I2C_DW_I2C1` | OFF | Builds in I2C1, the device `i2c1` (above) |

## Self-test (`selftest`)

A test command only in builds with `-DNANO_SELFTEST=ON`. Run it with the measurement stopped and the
board at rest. It prints, as `N of M tests failed`, the results for the layer
(`drivers/compat/zephyr_selftest.c`: mutexes, semaphores, sleep, clock, wake-ups from interrupts,
`k_poll`, work, spinlock, atomic, static threads, time slices, the tick's catch-up after interrupts are disabled for 50 ms; 22 items), I2C0
(`drivers/i2c_dw/i2c_dw_selftest.c`: 6 items; with I2C1 built, a 7th, `i2c1`: with nothing connected,
i2c1 is ready, its pins are I2C with the pull-ups, a read from 0x10 ends with `-EIO` in under 100 ms,
IC_CON has no DMA bit, a 40-byte read leaves the DMA counters alone, and i2c0 still answers) and the LSM6DSOX (`drivers/lsm6dso/lsm6dso_selftest.c`:
3 items).

## Host tests

`freertos/script/host_tests.sh [-v]` runs, after the tests of the shared sources
(`common/tests/run.sh`), these on FreeRTOS's POSIX port. No board is needed.

| Test | Subject |
|---|---|
| `compat` | The layer's self-test (the same `zephyr_selftest.c` as `selftest`) |
| `tick` | The tick's catch-up (`tick_catchup.h`): the step itself, and on a model of SysTick and the CPU (interrupt latencies, interrupts disabled for 0 to 60 ticks) that the tick never gets ahead of the TIMER and is at most one tick behind once caught up |
| `shell` | Two shell sessions, line editing, output routing |
| `kv` | The flash boundary and the key-value store (on fakes of the Pico SDK's flash functions) |
| `nina` | The NINA's SPI and line boundaries and the FreeRTOS binding of `nina_os.h` (no compatibility layer; built twice, with a 1 ms and a 4 ms tick): the wait for READY on its interrupt, the waits for the IRQ line's edges (two tasks woken by one edge, the timeout), the shared bank interrupt, locks, requests, sleeps' ticks, the clock across the tick's wrap |
| `i2c_dw` | I2C0's boundary and device, Zephyr's I2C API (built twice: i2c0 only, and with `CONFIG_I2C_DW_I2C1`, which adds i2c1's start, pins, transfers, an unanswered address, long reads that never take the DMA, i2c0 unaffected, and transfers on both buses from two tasks); the binding's semaphores, waits' ticks, sleeps and clock; the DMA part (`i2c_dw_dma_os_rp2040.c`) on fakes of the Pico SDK's DMA: claims, setup, start, the shared handler (end, bus error, other users' channels), stop, release; the core's DMA reads on it: the read, the 32-byte limit, fallbacks, bus errors on TX and RX, timeout, the reads off, a stop that does not end (quarantine), the counters; the core's DMA command words |
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

The custom commands (`imu`, `kv`, `ntp`, `nina`, `dmesg`, `i2c_dma`, `sensor`, `wifi cred`) and `zperf` have
the Zephyr build's syntax and output.

| Item | Zephyr build | NuttX build | FreeRTOS build |
|---|---|---|---|
| Prompt | `uart:~$ ` on USB, `~$ ` on Telnet | `nsh$ ` | `freertos:~$ ` |
| Version, restart, task list, time since start | `kernel version`, `kernel reboot`, `kernel ps`, `kernel uptime` | `uname -a`, `reboot`, `ps`, `uptime` | `version`, `reboot`, `ps`, `uptime` |
| IP address, date | `net iface`, `date get` | `ifconfig`, `date` | `ifconfig`, `date` (UTC, show only) |
| `wifi` | All of Zephyr's | `wifi cred` only (`wapi` connects) | `connect` (`-s`, `-p`, `-k 0/1`), `disconnect`, `scan`, `status`, `cred` |
| Speed test | `zperf` (iperf3 protocol, TCP and UDP) | `iperf` (iperf2 protocol) | `zperf` (iperf3 protocol, TCP only) |
| BOOTSEL from the shell | `bootsel` | `bootsel` | `bootsel` |
| `sensor get` | Returns at once; the result comes after the prompt | Waits up to 2 s for the result | As NuttX |
| Right after a Telnet connection | Sends nothing | Sends the prompt | Sends nothing |
| Echo on Telnet | No | Yes | No |
| Error numbers (errno) | Zephyr's (`EILSEQ` 138) | NuttX's (`EILSEQ` 84) | newlib's (as Zephyr's; `EILSEQ` 138) |
| Priorities | Zephyr | `118 − Zephyr` | `20 − Zephyr` |
| Device interrupt priority | 0xC0 | 0x80 | 0x80 |
| Kernel clock | SysTick (late while sending) | Tickless on the TIMER | 1 kHz SysTick (catches up with the TIMER after interrupts were disabled); `k_uptime` from the TIMER |
| Saving the divider | At switches, only when DIRTY (local patch) | At switches, only when DIRTY (patch 0009) | Four words at every switch (the Pico SDK port) |
| The NINA's READY | Edge interrupt (GPIO10, read again every 10 ms) | Edge interrupt (GPIO10) | Edge interrupt (GPIO10, read again every 10 ms) |
| Sockets | Offload (the calling thread waits) | usrsock and `nina_event` | The Zephyr build's offload on the layer's socket layer (the calling task waits) |
| RAM log | 4 KiB | 8 KiB | 8 KiB |
| After a fatal error | Restart, report in `dmesg` | Restart, report in `dmesg` | Restart, record in `dmesg`, `fault` and the banner |
| `ifconfig` netmask while disconnected | – | Kept | 0.0.0.0 |
