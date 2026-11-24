# The FreeRTOS port and the parts shared by the OSes on the Pico SDK

English | [日本語](freertos-port-ja.md)

The firmware of this repository is the same firmware built on four OSes, Zephyr, NuttX, Pico SDK + FreeRTOS
and Pico SDK + ThreadX, called the Zephyr, NuttX, FreeRTOS and ThreadX builds. This document describes the
parts shared by the builds on the Pico SDK (the FreeRTOS and ThreadX builds) and the inside of the FreeRTOS
build's port (`freertos/`). How to use it and the list of commands are in [`freertos/README.md`](../freertos/README.md),
the list of what each build writes in [`os-boundary.md`](os-boundary.md), the summary of the design in §9.4 of the
[design document](imu-raw-mqtt-design.md). The ThreadX build's part is [`threadx-port.md`](threadx-port.md).
The details are in the comment at the top of each file; this page shows where the files are and how they
relate to the other builds.

Most of the sources are in `common/`, which the four builds share (acquisition, transport, time
synchronization, calibration, key-value store, the LSM6DSO, NINA and I2C drivers, the shell's command
handling). The shared sources are written on Zephyr's API, so the FreeRTOS build, like the NuttX and ThreadX
builds, runs them on a layer that provides Zephyr's API under Zephyr's names (the "compatibility layer"; its
OS-independent part is `common/compat/`, the FreeRTOS part `common/compat/port/freertos/`). The layer and the
drivers themselves are described in their own READMEs.

| Part | Described in |
|---|---|
| The compatibility layer (the kernel part, the network part, the shell engine and the consoles, the RAM log) | [`common/compat/README.md`](../common/compat/README.md) |
| NINA-W102 (SPI, sockets, Wi-Fi, passthrough mode) | [`common/drivers/nina/README.md`](../common/drivers/nina/README.md) |
| I2C and its DMA reads | [`common/drivers/i2c_dw/README.md`](../common/drivers/i2c_dw/README.md) |
| LSM6DSOX, the `sensor` command | [`common/drivers/lsm6dso/README.md`](../common/drivers/lsm6dso/README.md) |

Each module of `common/` keeps the headers other modules read in `include/`, its own in `internal/`, the
boundary headers an OS implements in `port/`, each OS's implementation in `port/<os>/`, what the builds on
the Pico SDK share in `port/pico_sdk/` (the parts written on the Pico SDK and the compatibility layer, not on
FreeRTOS or ThreadX, with their cmake files and linker script parts), and the parts for an OS without the
model of its own in `port/generic/`.

## Shared by the OSes on the Pico SDK

This chapter is what the FreeRTOS and ThreadX builds share; it depends on neither FreeRTOS nor ThreadX. A new
OS on the Pico SDK uses this chapter's parts as they are, and writes what the next chapter (the FreeRTOS
build's part) and [`threadx-port.md`](threadx-port.md) describe for those two.

### Where

| Place | Contents |
|---|---|
| `common/imu/port/pico_sdk/` | The application's part: the build (`usb-console.cmake`: the drivers, the include directories, the SDK's options; `ram_code.cmake`: the code that runs from RAM; `imu.cmake`: the acquisition service, MQTT and time synchronization of `common/imu/`; `ld/`: the start-up thread's stack in SCRATCH_X), the shared steps of the start (`usb_console.[ch]`, called by the OS's `main()`), the commands `version` and `free` and the table of all commands (`commands.c`, on `commands_os.h`), the idle pass and the CPU load (`idle.c`; the LED's blink `idle_blink.h`), fatal errors (`fault.[ch]`, with the functions the OS's entries call and the ones the OS provides), the free interrupt stack (`irq_stack.c`), `app.h` |
| `common/chip/rp2040/port/pico_sdk/` | The key-value store's flash operations on the Pico SDK (`rp2040_flash_kv_os.c`, on the boundary `common/chip/rp2040/port/rp2040_flash_kv_os.h` under the RP2040's shared part `common/chip/rp2040/rp2040_flash_kv.c`) and the linker script override (`ld/`: `pico_flash_region.ld`, `section_extra_post_end.incl`); its build is `common/compat/port/pico_sdk/kv.cmake` |
| `common/compat/port/pico_sdk/` | The layer's part on the Pico SDK (the builds `compat.cmake`, `kv.cmake`, `wifi.cmake`, `telnet.cmake` and `zperf.cmake`, the linker script's part for the socket registrations, the wall clock, GPIO bank 0, the chip ID, the console transport, the stdio replacement, the RAM log's part); [`common/compat/README.md`](../common/compat/README.md) |
| `common/shell/port/pico_sdk/` | Builds in the `sensor` command (`sensor_shell.cmake`) and the `i2c` command (`i2c_shell.cmake`) |
| `common/drivers/nina/port/pico_sdk/` | The Pico SDK's part of the NINA's SPI1, DMA, lines and passthrough mode, and its build (`nina.cmake`); [`common/drivers/nina/README.md`](../common/drivers/nina/README.md) |
| `common/drivers/i2c_dw/port/pico_sdk/` | The I2C devices and their start, the Pico SDK's part of I2C0's DMA reads, and the build (`i2c_dw.cmake`); [`common/drivers/i2c_dw/README.md`](../common/drivers/i2c_dw/README.md) |
| `common/drivers/lsm6dso/port/pico_sdk/` | The LSM6DSOX's device and its start, the build (`lsm6dso.cmake`); [`common/drivers/lsm6dso/README.md`](../common/drivers/lsm6dso/README.md) |

Shared files only the FreeRTOS and ThreadX builds on the Pico SDK compile: `common/compat/zephyr_wifi_shell.c` (`connect`,
`disconnect`, `scan` and `status` taken from Zephyr's `wifi_shell.c`), `common/compat/zephyr_zperf_shell.c` and
`zperf_iperf3.c` with `internal/zperf_iperf3.h` (`zperf`), `common/compat/log/fault_record.c` and `common/compat/include/fault_record.h` (the format of the fatal error record),
`common/shell/dmesg_shell.c` (`dmesg`), `common/shell/kernel_shell.c` (`id`, `uptime`, `date`, `sleep`, `devmem`,
`fault`, `reboot`, `bootsel`, on the boundary `common/shell/port/kernel_shell_os.h`), `common/shell/ifconfig_shell.c`
(`ifconfig`). The RAM log `common/compat/log/log_ram.c` (on its ring `log_ring.c`) is shared with
the NuttX build too.
`i2c_dma`'s `common/shell/i2c_dma_shell.c` and `nina`'s `common/shell/nina_shell.c` (`nina version`,
`nina passthrough`) are built by the Zephyr build too. `i2c`'s `common/shell/i2c_shell.c` is a
byte-for-byte copy of Zephyr's `drivers/i2c/i2c_shell.c` with the local patch 0004 (`zephyr_i2c_shell_patched`
of `common/tests/run.sh` compares them), built by the NuttX build too (the Zephyr build builds the file in its tree).

### What the OS writes

A build on the Pico SDK is the shared parts above plus the following (the list, with the header that
declares each, is in [`os-boundary.md`](os-boundary.md)):

- `<os>/app/usb-console/`: the build (`CMakeLists.txt`, which gives the OS's settings and sources to
  `common/imu/port/pico_sdk/usb-console.cmake` and lists the OS's code for RAM in `RAM_CODE_OS_SOURCES`), the
  kernel configuration, and in `src/` the entry of the start (`main.c`), the tick (`tick.c`) and the OS parts
  of the shared application's boundaries (idle `idle_os.h`, fatal errors `fault_os.c`, `ps` and the kernel's
  version `commands_os.c`);
- `common/compat/port/<os>/`: the primitives of the layer's kernel part, the priorities, the port header
  ([`common/compat/README.md`](../common/compat/README.md));
- `common/shell/port/<os>/`: the OS part of the generic commands (`kernel_shell_os_impl.h`);
- `common/drivers/i2c_dw/port/<os>/` and `common/drivers/nina/port/<os>/`: the drivers' waits and locks;
- `<os>/script/` and `<os>/tests/`; and, if the OS's port has no interop of the SDK's locks with its threads,
  that too (the ThreadX build's `threadx/drivers/pico_sdk_interop/`).

### Start-up

The entry `main()` is in each OS's directory. It calls the shared steps
(`common/imu/port/pico_sdk/usb_console.h`) in this order, with the OS's part between them:

1. `usb_console_early_init()`: the interrupt stack's paint, the RAM log, the check of the NINA passthrough
   mode's request, the last run's fatal error record. In the passthrough mode it runs the mode here and does
   not return.
2. The kernel's set-up (the OS's part: whatever it needs before objects and threads can be made).
3. `usb_console_init()`: the USB stdio and the console "usb", the compatibility layer (its work queues'
   threads are made, not run), I2C0 (first the DMA interrupt's handler for its reads; I2C1 too when it is
   built) and the key-value area's bounds.
4. The thread "main" (the OS's part: static, priority `APP_PRIO_MAIN`, `APP_STACK_MAIN` bytes of stack), whose
   body is `usb_console_main_thread()`.
5. `usb_console_shell_start()`: the thread "console" (the shell on USB) and the LED the idle pass blinks.
6. The scheduler's start (the OS's part; no return).

The shared steps from 2 to 6 do only what needs no interrupt and no time to pass (the OS may keep interrupts
masked there). The prompt is the OS's name followed by `:~$ ` (`freertos:~$ `).

The thread "main" (`usb_console_main_thread()`) starts the LSM6DSOX and the NINA's network interface, then the
application's start shared by the four builds (`common/imu/app_main.h`, the Zephyr build's initialization
order) with the `wifi` command and Telnet of the builds on the Pico SDK in it: the Wi-Fi credentials, the
`wifi` command, Telnet, the `K_THREAD_DEFINE()` threads (`app_threads_start()`, `common/imu/app_init.c`), the
key-value store, the calibration, the time synchronization and the automatic connection, in this order. It
returns when done and the OS ends the thread. Its stack is in SCRATCH_X, which only core 1 (never started)
would use (`common/imu/port/pico_sdk/ld/`, the OS's `main.c`).

### Idle and the CPU load

`common/imu/port/pico_sdk/idle.c` has the idle wait and the CPU load measured from idle windows
(`cpu_load_get()`). It disables interrupts, opens the window, waits with `WFI`, closes the window and enables
interrupts (the measurement of the Zephyr build's `CONFIG_CPU_LOAD` and the NuttX build's `up_idle()`). The
handler of the interrupt that ended the wait runs after the window closes, so it counts as load. While it
runs it blinks the LED (GPIO6) every 500 ms.

The OS's part, `idle_os.h`, names how the OS's idle calls the functions of `idle.c`, in one of two shapes:

- `IDLE_OS_HOOK`: the whole pass (from masking interrupts to unmasking them), called from the OS's idle hook
  (the FreeRTOS build: `vApplicationIdleHook`);
- `IDLE_OS_ENTER` and `IDLE_OS_EXIT`: the two halves (open and close the window), for an OS whose own wait
  masks interrupts and runs `WFI` (the ThreadX build: the scheduler's wait, `tx_low_power_enter()` and
  `tx_low_power_exit()`).

### Commands

The commands only the OS and the Pico SDK answer are `common/imu/port/pico_sdk/commands.c` (`version`, `free`,
with the table of all commands, on `commands_os.h`: the kernel's name and version, `ps`) and the OS's
`app/usb-console/src/commands_os.c`, called by `commands.c`, which registers the commands. The generic
commands are `common/shell/kernel_shell.c`; what they need from the OS (the tick count, the two restarts, the
last fatal error's record) is declared in `common/shell/port/kernel_shell_os.h` and implemented in
`common/shell/port/<os>/kernel_shell_os_impl.h`. The shell engine, the consoles and Telnet are in
[`common/compat/README.md`](../common/compat/README.md).

The `(interrupts)` row of `ps` is the use of the interrupt stack (MSP, 2048 bytes; `irq_stack.c`, painted
first at the start).

### RAM log and fatal errors

- The RAM log (`common/compat/log/log_ram.c`, shared with the NuttX build too; the part on the Pico SDK is
  `common/compat/port/pico_sdk/log_ram_os_impl.h`; its size, `LOG_RAM_SIZE` in `common/compat/port/generic/app_log_os.h`) is
  4 KiB, as in the Zephyr build (a fatal error leaves a record of a few lines, written into the log at the
  next start), in RAM that the start-up code does not initialize (`__uninitialized_ram`). It survives the watchdog's restart, `reboot` and the RESET button, not a power
  cycle. `LOG_*` lines are formatted in the caller's context (interrupt handlers included), timed by the
  TIMER. `dmesg` (`common/shell/dmesg_shell.c`) prints it.
- Fatal errors (`common/imu/port/pico_sdk/fault.[ch]`): `panic()`, HardFault, the OS's assertion and `assert()`,
  the OS's own entries (the OS's `fault_os.c`: the FreeRTOS build's stack overflow hook, the ThreadX build's
  stack check handler) and `_exit()` end in one exit,
  which keeps a record (reason, PC, LR, SP, exception frame, thread, time) in RAM and in the watchdog's
  scratch registers 0..3 and restarts the board with the watchdog. The next start writes the record to
  `dmesg`, and `fault` and the console's banner show it. The record's format, the reading of the
  HardFault's exception frame (only where the frame lies in the RAM) and the choice at the next start
  between the record in RAM and the scratch registers' copy are `common/compat/log/fault_record.c` and `common/compat/include/fault_record.h`;
  `fault.c` keeps the entries, the handler's entry (`isr_hardfault`), the watchdog's registers and the
  restart. The OS writes the running thread's name (`fault_os_thread_name()`) and its own entries, which call
  `fault_begin()` and `fault_finish()` of `fault.h`.

### The key-value store's flash

As in the Zephyr and NuttX builds, the key-value store is in the last 8 KiB of the flash
(`0xffe000`..`0xffffff`, two 4 KiB sectors) in the format of Zephyr's NVS and settings (the copies are in
`common/compat/subsys/`, the format is described in "Key-value store: copies of Zephyr's NVS and settings" of
[`common/compat/README.md`](../common/compat/README.md)). The flash
boundary (the five functions of `common/compat/port/flash_kv_os.h`) is implemented by the RP2040's part shared with the NuttX build
(`common/chip/rp2040/rp2040_flash_kv.c`: the checks, the reads, the page copy) on the flash operations on the Pico SDK
(`common/chip/rp2040/port/rp2040_flash_kv_os.h`: `common/chip/rp2040/port/pico_sdk/rp2040_flash_kv_os.c`, the Pico SDK's
`flash_range_program()` and `flash_range_erase()` through `flash_safe_execute()`).

- The linker script (`common/chip/rp2040/port/pico_sdk/ld/`, `common/compat/port/pico_sdk/kv.cmake`) sets the FLASH length to `0xffe000`, keeping the image
  out of the area, and checks `__kv_start`, `__kv_end` and the end of the image with `ASSERT`.
- A program or erase request is checked against the area three times: in `common/compat/zephyr_flash_kv.c`, in the
  RP2040's shared part, and right before the flash is touched (with interrupts disabled). Until the linker
  symbols are found equal to the constants, programs and erases are refused.
- `flash_range_program()` and `flash_range_erase()` are called through `flash_safe_execute()` with
  `PICO_FLASH_ASSUME_CORE1_SAFE=1`, which disables this core's interrupts only (core 1 is not used).
  Interrupts stay disabled during an erase; the tick catches up with the TIMER afterwards ("The tick" below
  for the FreeRTOS build, "The tick" of [`threadx-port.md`](threadx-port.md) for the ThreadX build).
- Data that is neither Zephyr's NVS nor erased is left alone and logged to `dmesg`; the store's calls
  return `-EILSEQ` (138 with newlib). `kv wipe confirm` erases it (`common/compat/kv_store_wipe.c`).

### Divider, `memcpy` and `memset`, boot2

The Pico SDK's defaults. Integer division is `pico_divider` on the hardware divider. What happens to the
divider at a switch depends on the OS's port ("Differences from the other builds" below for the FreeRTOS
build, "Differences from the FreeRTOS build" of [`threadx-port.md`](threadx-port.md) for the ThreadX build).
A division by zero gives the Pico SDK routines' results, the same in the four builds. `memcpy` and `memset` are
`pico_mem_ops`, the boot ROM's routines; boot2 is the Pico SDK board definition's, for the W25Q080
(divider 2, 62.5 MHz).

### Code in RAM

The flash is executed through the RP2040's 16 KiB XIP cache, and the code on the 6667 Hz path of the
sensor and on the send path does not fit in it. The code (`.text*`; their read-only data stays in the flash)
of the objects listed in `common/imu/port/pico_sdk/ram_code.cmake` runs from RAM:

- the layer's kernel part, work queue, RTIO, memory blocks, sensor clock, wall clock and `gpio0`, with the
  Pico SDK's `hardware_gpio`;
- I2C0 (`i2c_dw.c`, also I2C1's when it is built) with its DMA part and the Pico SDK's `hardware_dma`;
- the LSM6DSOX driver and the stmemsc glue (not ST's `lsm6dso_reg.c`, which only configures);
- the IMU service's stream, batches, queue and time stamps (not the NTP exchange nor the calibration);
- MQTT, the layer's sockets, the NINA's sockets, protocol and waits with its SPI and DMA, and the Pico SDK's
  `hardware_spi`.

The OS's part is added by the OS's `CMakeLists.txt` in `RAM_CODE_OS_SOURCES` (in both the FreeRTOS and ThreadX
builds only the NINA core's binding, `common/drivers/nina/port/<os>/nina_os.c`). The kernel stays in the flash.

`ram_code.cmake` replaces the Pico SDK's `default_text_excludes.incl` with a copy in the build directory that
leaves these objects out of the flash's `.text`; the SDK's `.data` takes their code, and crt0 copies it to
RAM before anything else runs. To check a placement, clear the cache's counters while sending
(`devmem 0x1400000c 32 0`, `devmem 0x14000010 32 0`; they saturate) and read them about 15 s later (the hits
and the accesses).

In the FreeRTOS build the code costs about 49 KiB of RAM (`.data` is about 57 KiB instead of 8 KiB, with the long-branch stubs
between RAM and the flash). It is taken from what newlib's `malloc()` can grow into, between the end of
`.bss` and the stacks (`free`). About 89 kB is left above the Pico SDK's 2 KiB minimum. The
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
and libgcc. The fatal error path (`common/imu/port/pico_sdk/fault.c`, `common/compat/log/fault_record.c`) and the key-value
store's flash part stay in the flash; while XIP is off, only the Pico SDK's own RAM functions run, with
interrupts disabled.

### Interrupts

- Interrupt priorities stay at the Pico SDK's default 0x80, so all device interrupts are at one level (as
  on NuttX; the Zephyr build uses 0xC0).
- Shared interrupt handlers: the Pico SDK keeps them in a fixed number of slots for all lines
  (`PICO_MAX_SHARED_IRQ_HANDLERS`, 6 by default with stdio over USB), and adding one more than that ends in
  `hard_assert()`. Six are in use: USBCTRL_IRQ's two (TinyUSB, stdio_usb), IO_IRQ_BANK0's two raw handlers
  (`common/compat/port/pico_sdk/gpio_os_rp2040.c`, the NINA's `nina_os.c`) and DMA_IRQ_1's two (I2C0's reads, the NINA's SPI). The
  application sets 8 (`common/imu/port/pico_sdk/usb-console.cmake`; 12 bytes of RAM per slot).
- Interrupts use a stack of their own, not the threads' (MSP, 2048 bytes; the `(interrupts)` row of `ps`).

### Time, the wall clock, the chip ID

The acquisition service (`imu_stream.c`), the time synchronization (`time_sync.c`, `time_offset.c`), the
transport (`imu_raw_mqtt.c`, `imu_raw_status.c`) and the calibration (`imu_cal.c`) are the Zephyr build's
files in `common/imu/`, built in by `common/imu/port/pico_sdk/imu.cmake`. The MQTT client is `common/imu/mqtt_min.c` and
`mqtt_min_sock.c`, shared with the NuttX build too.

- The wall clock (`common/compat/port/pico_sdk/zephyr_sys_clock.c`) is the TIMER plus a base (built as in the NuttX
  build); newlib's `gettimeofday()` reads the same clock. `date` shows it in UTC.
- `time_sync.c` reads the TIMER with `rp2040_timer_us()` of `common/chip/rp2040`, as the layer's clocks do.
- The chip ID (the first 8 bytes of the session ID) is `pico_get_unique_board_id()`, the same flash ID as
  the Zephyr build's `hwinfo_get_device_id()` (`common/compat/port/pico_sdk/zephyr_hwinfo.c`).

## The FreeRTOS build's part

### Directories

The FreeRTOS build's own files are in `freertos/`; its FreeRTOS implementations are in
`common/compat/port/freertos/`, `common/shell/port/freertos/` and the drivers' `port/freertos/`. Below, `src/`
is `freertos/app/usb-console/src/`.

| Place | Contents |
|---|---|
| `freertos/app/usb-console/` | The build (`CMakeLists.txt`: the FreeRTOS settings of `common/imu/port/pico_sdk/usb-console.cmake`, the list of the sources, `RAM_CODE_OS_SOURCES`), the kernel configuration (`FreeRTOSConfig.h`, which also checks the one core), and `src/`: the entry of the start (`main.c`: `main()`, the task "main", the scheduler), the tick hook (`tick.c`; the catch-up's decision is `common/compat/include/tick_catchup.h`), and the FreeRTOS parts of the shared application's boundaries: the idle pass (`idle_os.h`: the idle hook), the fatal errors (`fault_os.c`: the running task's name, the stack overflow hook), the commands (`commands_os.c`: `ps` with the clock of its run times, the kernel's version) |
| `common/compat/port/freertos/` | The FreeRTOS part of the layer: the primitives of the generic kernel part (`zephyr_kernel_os_impl.h`), the priority mapping (`priorities_os.h`) and the port header (`rp2040/zephyr_os_port.h`) |
| `common/shell/port/freertos/` | The OS part of the generic commands (`kernel_shell_os_impl.h`, with the tick count of `uptime`) |
| `common/drivers/i2c_dw/port/freertos/` | The I2C core's waits, clock, sleeps and interrupt masking (`i2c_dw_os_impl.h`) |
| `common/drivers/nina/port/freertos/` | The NINA core's binding to FreeRTOS (`nina_os.c`) and the SPI DMA's wait (`nina_bus_dma_os_impl.h`) |
| `freertos/tests/` | Host tests ("Host tests" below) |
| `freertos/script/` | `setup.sh`, `build.sh`, `flash.sh`, `format.sh`, `host_tests.sh` |

### The compatibility layer and the drivers' bindings

The layer's kernel part is the generic implementation (`common/compat/kernel_generic/`) on primitives that
FreeRTOS provides (`common/compat/port/freertos/zephyr_kernel_os_impl.h`). The assumptions of Zephyr it
reproduces and what is left as approximations (priorities, cooperative threads, priority inheritance, the
choice among one priority) are in [`common/compat/README.md`](../common/compat/README.md). The I2C and NINA
bindings are written on FreeRTOS (task notifications) and the Pico SDK directly, not on the layer
([`common/drivers/i2c_dw/README.md`](../common/drivers/i2c_dw/README.md),
[`common/drivers/nina/README.md`](../common/drivers/nina/README.md)).

### Tasks

The priorities are in `common/compat/include/priorities.h` (the Zephyr build's values and `20 − Zephyr`) and
`common/compat/port/freertos/priorities_os.h` (the tasks of this build only, and the stacks of the layer's own
threads sized as measured on this build: peak use plus half, at least 256 bytes, a multiple of 64; the
shell sessions keep Zephyr's 3072). Stacks are in bytes. Interrupts
use a stack of their own (MSP, 2048 bytes; the `(interrupts)` row of `ps`).

| Task | Priority (Zephyr) | Stack | Role |
|---|---|---|---|
| `Tmr Svc` | 31 (–) | 384 | The kernel's timer task; wakes the waiters of Pico SDK locks released in interrupt handlers |
| `net_mgmt`, `sysworkq` | 21 (-1) | 2048, 384 | Network events; the system work queue (the work of the LSM6DSOX FIFO interrupt) |
| `main` | 20 (0) | 2048 | The second half of the start-up ("Start-up" above); ends when done. Its stack is in SCRATCH_X, which only core 1 (never started) would use (`common/imu/port/pico_sdk/ld/`, `src/main.c`) |
| `sensor_shell_tid`, `RTIO WQ 0` | 20 (0) | 2048, 1024 | Printing `sensor` results; RTIO work |
| `nina_wifi_tid`, `wifi_reconnect_tid` | 12 (8) | 2048, 2048 | Wi-Fi scan, connection and link monitor (one at a time); automatic reconnection |
| `time_sync_tid`, `imu_stream_tid` | 11 (9) | 2048, 2048 | Time synchronization, acquisition |
| `imu_fifo_worker` | 10 (10) | 2048 | FIFO reads |
| `imu_raw_mqtt_tid` | 9 (11) | 2048 | MQTT transport |
| `console`, `shell_telnet`, `telnet_service`, `zperf_server_tid` | 6 (14) | 3072, 3072, 2048, 2048 | The USB and Telnet shells (commands run in these tasks), Telnet accepts, the server of `zperf tcp download` |
| `IDLE` | 0 (–) | 256 | Idle; measures the CPU load and blinks the LED (GPIO6) every 500 ms while it runs (`common/imu/port/pico_sdk/idle.c`, the idle hook of `idle_os.h`) |

The kernel has no heap (`configSUPPORT_DYNAMIC_ALLOCATION` 0): every task, queue and semaphore is
static. Each thread of the layer keeps its control block (136 bytes) after its stack, in the array of
`K_THREAD_STACK_DEFINE()` (`Z_OS_THREAD_RESERVED`, `common/compat/port/freertos/zephyr_kernel_os_impl.h`); the sizes above are
the stacks without it.

**Start-up** (`src/main.c`): `main()` calls the shared steps ("Start-up" above) in order. The FreeRTOS build
needs no kernel set-up: after `usb_console_early_init()` and `usb_console_init()` it makes the task `main`
(`xTaskCreateStatic()`), `usb_console_shell_start()` makes the task `console`, and after checking that clk_sys
is a whole number of ticks it starts the scheduler (`vTaskStartScheduler()`).

**CPU load and `ps`**: the idle hook (`IDLE_OS_HOOK` of `idle_os.h`) runs the pass of "Idle and the CPU load"
above. The tick interrupt every 1 ms counts as load (the Zephyr and NuttX builds have no
such interrupt). The CPU column of `ps` is the share of the
interval since the previous `ps` (from USB or Telnet; since the start for the first) in all the time of
the interval, idle included, so the rows add up to about 100 % (type it at the start and at the end of
the interval of interest); timed by the TIMER.

### The tick

- The tick is a periodic SysTick of 1 ms (`configTICK_RATE_HZ` 1000, 32 bits; not tickless). The 32-bit tick
  wraps after about 49.7 days.
- While interrupts are disabled (a flash erase or program; about 40 to 50 ms per erase), SysTick keeps one
  interrupt pending and the other ticks are lost. The tick hook (`src/tick.c`, decision in `common/compat/include/tick_catchup.h`)
  compares the count with the TIMER at every tick and, when it is a whole tick behind, pends SysTick
  again, so the port's usual handler counts the lost ticks one by one (wake-ups, switch requests, and the
  tick hook's timeouts of the layer and time slices run on each). The tick never gets ahead of the
  TIMER, so apart from the time interrupts are disabled it stays within one tick of it.
  `xTaskCatchUpTicks()` is not used (it can be called from tasks only, and the tick hook does not run for
  the ticks it adds). `main.c` checks that clk_sys is a whole number of ticks. `k_uptime_*()` is the TIMER.
- The time slices are made by the tick hook (`tick.c`) on a fixed grid of 20 ticks, not counted from the
  switch.
- The Pico SDK's `sleep_ms()` (which spins the last 150 µs) is not used.

### The FreeRTOS part of fatal errors

`src/fault_os.c` has the running task's name and the stack overflow hook (`configCHECK_FOR_STACK_OVERFLOW` 2,
checked at every switch), which calls `fault_begin()` and `fault_finish()` of `fault.h`. `configASSERT()` is the
shared `app_assert_failed()`. The exit and the record are "RAM log and fatal errors" above.

### Build options

Given as `freertos/script/build.sh -DNAME=VALUE` (kept until `--clean`).

| Option | Default | Meaning |
|---|---|---|
| `I2C_DW_I2C1` | OFF | Builds in I2C1, the device `i2c1` ([`common/drivers/i2c_dw/README.md`](../common/drivers/i2c_dw/README.md)) |

### Host tests

`freertos/script/host_tests.sh [-v]` runs, after the tests of the shared sources
(`common/tests/run.sh`), these on FreeRTOS's POSIX port. No board is needed. The tests are in
`freertos/tests/<name>/`.

| Test | Subject |
|---|---|
| `compat` | The layer's kernel (`common/tests/zephyr_kernel/zephyr_kernel_test.c`, 22 items: mutexes, semaphores, sleep, clock, `k_poll`, work, spinlock, atomic, static threads, time slices; wake-ups from the tick interrupt; on this port the checks of real time, of a hardware interrupt and of the tick's catch-up are skipped) |
| `shell` | Two shell sessions, line editing, output routing, `help` and the generic commands with their output |
| `kv` | The flash boundary and the key-value store (on fakes of the Pico SDK's flash functions) |
| `nina` | The NINA's SPI and line boundaries (with the long transfers by DMA) and the binding of `nina_os.h` to FreeRTOS (built twice, with a 1 ms and a 4 ms tick). The items are in [the NINA-W102 driver](../common/drivers/nina/README.md#tests) |
| `i2c_dw` | I2C0's boundary and device, Zephyr's I2C API, the `i2c` command, the binding, the DMA part and the core's DMA reads (built twice: i2c0 only and with `CONFIG_I2C_DW_I2C1`). The items are in [the DesignWare I2C core](../common/drivers/i2c_dw/README.md#tests) |
| `gpio` | GPIO bank 0's boundary, `gpio0`, the FIFO interrupt |
| `imu` | Address conversions, `hwinfo` |
| `mqtt` | The MQTT transport (fake broker); then a run against the host's `imu-host check` (skipped without `host/.venv`) |
| `time_sync` | Wall clock, TIMER read, NTP exchanges, `ntp status` |
| `sockets` | The layer's socket part on fake implementations: the descriptor table, the registrations' order, the dispatch of every call, fcntl, poll (offloaded, other kinds, mixed), the resolver, `k_heap` |
| `socket` | The NINA's sockets on the layer: descriptors, wait loops, options, poll, the descriptor's lock, name resolution |
| `wifi` | Interface, events, credentials, reconnection, `wifi` and `ifconfig`, the scans by the firmware's bits (link kept, asynchronous, failures, one that never ends, the link lost during it, 3.0.2) |
| `telnet` | The Telnet protocol, one client, reconnections, `zperf` (with the host's python3 and iperf3; that part is skipped without iperf3) |
| `sensor_shell` | The `sensor` command, two sessions, calibration (skipped without ST's `lsm6dso_reg.c`) |

The OS-independent body of `compat` is `common/tests/zephyr_kernel/` (`zephyr_kernel_test.h` lists what an OS's
part and a platform's part provide; `kernel_generic/zephyr_kernel_test_os.h`, its items on the primitives);
this build's part is `freertos/tests/compat/`.

### Differences from the other builds

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
| Saving the divider | At switches, only when DIRTY (local patch) | At switches, only when DIRTY (patch 0009) | Four words at every switch (the Pico SDK port's `portUSE_DIVIDER_SAVE_RESTORE`) | None (ThreadX's Cortex-M0 port) |
| The NINA's READY | Edge interrupt (GPIO10, read again every 10 ms) | Edge interrupt (GPIO10) | Edge interrupt (GPIO10, read again every 10 ms) | Edge interrupt (GPIO10, read again every 10 ms) |
| Sockets | Offload (the calling thread waits) | usrsock and `nina_event` | The Zephyr build's offload on the layer's socket layer (the calling task waits) | As FreeRTOS |
| RAM log | 4 KiB | 8 KiB | 4 KiB | 4 KiB |
| After a fatal error | Restart, report in `dmesg` | Restart, report in `dmesg` | Restart, record in `dmesg`, `fault` and the banner | As FreeRTOS |
| `ifconfig` netmask while disconnected | – | Kept | 0.0.0.0 | 0.0.0.0 |

The finer differences between the ThreadX and FreeRTOS builds (the rows of `ps`, the choice among one
priority, the stack check and so on) are in "Differences from the FreeRTOS build" of
[`threadx-port.md`](threadx-port.md).
