# Pico SDK + FreeRTOS build

English | [日本語](README_ja.md)

A build environment for firmware on the Raspberry Pi Pico SDK and the FreeRTOS kernel, for the Arduino
Nano RP2040 Connect. Like `zephyr/`, `nuttx/` and `openmv/`, it fetches what it needs under `freertos/`.

For now it holds one application, `app/usb-console`: a command line (CLI) on USB, a blinking LED, and
the kernel settings, CPU load measurement and fatal error handling that the port of the Zephyr build will
build on. The IMU measurement of the Zephyr and NuttX builds has not been ported yet.

| Item | Content |
|---|---|
| Pico SDK | 2.3.1 (board `arduino_nano_rp2040_connect`) |
| FreeRTOS kernel | V11.3.3 (the RP2040 port in `portable/ThirdParty/GCC/RP2040`, used on one core) |
| Compiler | Arm GNU Toolchain 15.2.Rel1 (GCC 15.2) |

## Requirements

- A Linux PC (x86_64). No sudo is needed.
- `git`, `curl` or `wget`, `tar`, `xz`, `sha256sum`
- `python3` (with the `venv` module; `python3-venv` on Ubuntu), `cmake` (3.13 or later), `make`, `gcc`, `g++` (the Pico SDK builds `picotool` for the host)
- About 1.5 GB of free disk space (the toolchain takes 1.1 GB)

## Setup

Source it from the root of the repository. It fetches only what is missing, so it can be sourced again
at any time. Source it again in a new terminal.

```bash
source freertos/script/setup.sh
```

| Place | Content | Version |
|---|---|---|
| `freertos/pico-sdk` | The Raspberry Pi Pico SDK and, of its submodules, TinyUSB (for the USB console) | tag `2.3.1` (shallow clone) |
| `freertos/FreeRTOS-Kernel` | The FreeRTOS kernel | tag `V11.3.3` (shallow clone) |
| `freertos/toolchain/` | Arm GNU Toolchain (arm-none-eabi, for x86_64 Linux) | 15.2.Rel1 (SHA-256 checked) |
| `freertos/.venv` | A Python venv (`clang-format`, used for formatting) | clang-format 18.1.3 (PyPI) |

It also sets `PICO_SDK_PATH`, `FREERTOS_KERNEL_PATH`, `PICO_TOOLCHAIN_PATH` and `PATH` (the toolchain,
`freertos/.venv/bin` and `freertos/script`) in the current shell. The versions are pinned at the top of the script. What it
fetches is not under Git.

## Build

```bash
freertos/script/build.sh [--clean] [-j N] [-v] [APP]
```

It sources `setup.sh` itself and builds `freertos/app/APP` (default: `usb-console`) in
`freertos/build/APP/`. The result is `freertos/build/usb-console/usb-console.uf2`.

On the first build the Pico SDK fetches the source of `picotool` (the tool that makes the UF2) from
GitHub into `freertos/picotool/` and builds it for the host (this needs the network). After `--clean`,
`picotool` is built again as well.

## Flashing

```bash
freertos/script/flash.sh [--build] [--wait SEC] [UF2]
```

1. If there is an `RPI-RP2` drive (an RP2040 in BOOTSEL), the UF2 is copied to it.
2. Otherwise, if this firmware's USB console is there, the script sends it the `bootsel` command and
   waits for the drive. Nothing has to be done by hand.
3. Otherwise it explains how to enter BOOTSEL and waits for the drive.

After the copy it waits for the USB console (`/dev/serial/by-id/usb-*FreeRTOS_USB_Console*`) and prints
its path. A drive that is not mounted is mounted with `udisksctl`.

Entering BOOTSEL:

- With this firmware running: the CLI's `bootsel` command. Opening the serial port at 1200 bps and
  `picotool reboot -f -u` work too (standard features of the Pico SDK's USB console).
- With the Zephyr or NuttX build running: press the RESET button twice quickly. If that does not work,
  connect REC to GND and press RESET.
- With the OpenMV build running: run `import machine; machine.bootloader()` in the REPL.

**Before flashing**: flashing replaces the firmware of the Zephyr, NuttX or OpenMV build. This
application does not write to the end of the flash (the key-value store of the Zephyr and NuttX builds,
the file system of the OpenMV build).

## The application `app/usb-console`

The console is the USB CDC ACM port (UART0 is not used). Opening a terminal shows the prompt
`freertos:~$ `. Line editing is Backspace and Ctrl-C (drop the line) only.

| Command | Content |
|---|---|
| `help` | List the commands |
| `version` | Versions of the FreeRTOS kernel, the Pico SDK and the compiler |
| `id` | The board's unique ID (the ID of the flash, 16 hexadecimal digits) |
| `uptime` | Time since the start (the RP2040 TIMER) and the number of ticks |
| `sleep <ms>` | Sleep for ms milliseconds, rounded as in Zephyr (the Zephyr build's `kernel sleep`) |
| `ps` | The tasks: state, priority, the least free stack so far (bytes), share of the CPU time since the start; then the CPU load (`cpu_active`) since the previous `ps` |
| `irqstack` | The use of the interrupt stack since the start (the NuttX build's `irqstack`) |
| `fault` | The fatal error that ended the run before this start |
| `free` | Size of the FreeRTOS heap, free space and the least free space so far |
| `reboot` | Restart |
| `bootsel` | Restart into the USB bootloader (BOOTSEL, the `RPI-RP2` drive) |

| File | Role |
|---|---|
| `CMakeLists.txt` | Imports the Pico SDK and the FreeRTOS kernel and the shared sources of `common/`; the USB product name (`FreeRTOS USB Console`) |
| `FreeRTOSConfig.h` | Kernel configuration |
| `src/main.c` | Start-up, the console task, the LED task |
| `src/cli.[ch]` | Line editing and the table of commands. Depends on neither the kernel nor the SDK |
| `src/commands.c` | The commands above |
| `src/priorities_os.h` | The FreeRTOS part of the table of priorities and stacks (`common/compat/priorities.h`) |
| `src/os_clock.h` | The clock since the start (TIMER), the tick count, sleeps with Zephyr's rounding |
| `src/tick.c` | The tick hook: 20 ms time slices |
| `src/idle.c` | The idle hook: the CPU load (`cpu_load_get()`, `common/compat/cpu_load.h`) |
| `src/irq_stack.c` | The high-water mark of the interrupt stack |
| `src/fault.[ch]` | The common exit of fatal errors and the record of the last one (`common/log/fault_record.[ch]`) |

### Tasks, priorities and stacks

The priorities follow the table of the port (`common/compat/priorities.h`, shared with the NuttX build),
whose FreeRTOS part is `src/priorities_os.h`: FreeRTOS priority = 20 − Zephyr priority. Zephyr's
preemptible priorities 0 to 14 become 20 to 6, the cooperative −1 becomes 21 (FreeRTOS cannot keep a
cooperative thread from being preempted by a higher one; the NuttX build makes the same approximation).
Stacks are given in bytes, as in Zephyr, and are statically allocated.

| Task | Priority | Stack | Zephyr build |
|---|---|---|---|
| `Tmr Svc` (the kernel's timer task) | 31 | 1024 B | none (see below) |
| `console` (the USB console; the commands run in it) | 6 | 3072 B | the shell thread (14, 3072 B) |
| `heartbeat` (toggles the LED on GPIO6 every 500 ms) | 1 | 1024 B | none (the Zephyr build leaves the LED alone) |
| `IDLE` (measures the CPU load) | 0 | 1024 B | idle (15, 256 B) |

- `Tmr Svc` has the highest priority because the Pico SDK's locks and sleeps wake the waiting task through
  it when an interrupt handler releases the lock (the USB code's stdio mutex) or the sleep's alarm fires;
  in Zephyr the interrupt handler makes the thread ready itself.
- The interrupt stack (the main stack) is the Pico SDK's `PICO_STACK_SIZE`, 2048 bytes, the size of the
  Zephyr build's `CONFIG_ISR_STACK_SIZE`. `irqstack` shows its use in the format of the `IRQ 00` line of
  the Zephyr build's `kernel thread stacks`. The value is a high-water mark (the stack is filled with a
  pattern at the start of `main()`) and includes the frames of `main()`, which stay on that stack after
  the scheduler starts. Nothing guards the bottom of the interrupt stack.

### Kernel configuration

- One core (`configNUMBER_OF_CORES` 1), priorities 0 to 31 (a higher number is a higher priority),
  task names up to 31 characters (`configMAX_TASK_NAME_LEN` 32, Zephyr's `CONFIG_THREAD_MAX_NAME_LEN`),
  two notifications per task (`configTASK_NOTIFICATION_ARRAY_ENTRIES` 2), 32 KB of heap with `heap_4`.
  The Pico SDK's locks and sleeps block the calling task (`configSUPPORT_PICO_SYNC_INTEROP`,
  `configSUPPORT_PICO_TIME_INTEROP`).
- The tick is 100 µs (`configTICK_RATE_HZ` 10000), as in the Zephyr and NuttX builds, and 32 bits wide
  (the RP2040 port has no 64-bit tick), so it wraps after about 4.97 days. The time since the start
  comes from the RP2040 TIMER (64 bits, 1 µs), not from the tick. The tick is periodic (SysTick); the
  Zephyr and NuttX builds are tickless. `portTICK_PERIOD_MS` is 0 at this rate and is not used; the
  Pico SDK's `sleep_ms()` is not used either (it spins the last 150 µs).
- Sleeps are rounded as in Zephyr: a sleep of t lasts more than ⌈t / 100 µs⌉ ticks and at most one tick
  more (`vTaskDelay(⌈t⌉ + 1)`; 0 only yields).
- Tasks of the same priority get time slices of 20 ms, as in the Zephyr build (`CONFIG_TIMESLICE_SIZE=20`,
  `CONFIG_TIMESLICE_PRIORITY=0`): FreeRTOS's own slicing, which would switch at every tick, is off
  (`configUSE_TIME_SLICING` 0) and the tick hook asks for a switch after 200 ticks of the same task. The
  slice is counted from the first tick that finds the task running.

### CPU load

The idle task measures the CPU load the way the Zephyr build (`CONFIG_CPU_LOAD` with the idle hook
backend) and the NuttX build (`up_idle()`) do: with interrupts disabled it opens a window, waits with
`wfi`, closes the window and enables interrupts, so interrupt handlers count as load even when they
interrupt the idle task. The arithmetic is `common/compat/nano_cpuload.h`; `cpu_load_get()`
(`common/compat/cpu_load.h`) is what the IMU service's `cpu_active` (`imu status`) will use. Neither the
Zephyr nor the NuttX build has a command of its own for it; here the last line of `ps` shows the load
since the previous `ps`, without resetting the window of `cpu_load_get()`:

```
cpu_active: <load>% in the <seconds> s since the previous "ps"
```

The periodic 100 µs tick wakes the CPU 10000 times a second and its handler counts as load; the Zephyr
and NuttX builds have no such interrupt.

### Fatal errors

`panic()` (also the Pico SDK's callers, `hard_assert`, out of memory in `malloc()`, a task function that
returns), the HardFault exception, a failed `configASSERT()` or `assert()`, a stack overflow
(`configCHECK_FOR_STACK_OVERFLOW` 2), an exhausted heap and `exit()` all end in one exit
(`src/fault.c`). It disables interrupts, records the reason, PC, LR, SP (for a HardFault also r0–r3,
r12 and xPSR from the exception frame), the exception number, the running task, the time and a message
in RAM that the start-up code does not clear (`__uninitialized_ram`, with a CRC-32), keeps the reason, PC
and LR in the watchdog's scratch registers 0 to 3 as well, and restarts the board with the watchdog
(as `reboot` does). The Zephyr and NuttX builds also restart after a fatal error. Nothing is written to
the console at that moment (the USB code needs interrupts and tasks).

After the restart the first terminal connection shows the record before the prompt, and `fault` shows it
again, for example (the values are those of the host test, not of the board):

```
*** The last run ended with a fatal error ("fault" shows it again):
Fatal error: HardFault
In task console, 3723.000042 s after the start
pc 0x10001234 lr 0x10005679 sp 0x20010f00
r0 0x30000000 r1 0x00000001 r2 0x00000002 r3 0x00000003
r12 0x0000000c xpsr 0x61000000
```

For an error other than a HardFault, the PC is the address the exit was called from and the LR is 0
(not known). The record is read once at the start and then cleared, so a later `reboot` or a power cycle
shows `No fatal error before this start`. The format and its check are OS-independent
(`common/log/fault_record.[ch]`, tested by `common/tests/run.sh`).

### Other notes

- The console task is woken by the notification of received USB data (the Pico SDK's
  `stdio_set_chars_available_callback()`), and every 50 ms to notice a terminal connecting.
- The USB product name is `FreeRTOS USB Console`. The Pico SDK cuts USB strings to 19 characters by
  default, so the limit (`USBD_DESC_STR_MAX`) is set to 32.
- The CLI is this repository's own, not FreeRTOS-Plus-CLI.

## What has been checked

Checked on 2026-11-16, in the build and on the board (Arduino Nano RP2040 Connect).

- The build completes without warnings (GCC 15.2), and `freertos/script/format.sh --check` reports no
  difference.
- `flash.sh` copies the UF2 to the `RPI-RP2` drive and finds the USB console after the restart.
- With the firmware running, `flash.sh` sends `bootsel` and flashes without anything done by hand
  (twice). `bootsel` typed in a terminal enters BOOTSEL too (once).
- The output of `help`, `version`, `id`, `uptime`, `ps` and `free`, and the error for an unknown command.

Not checked: `reboot`, entering BOOTSEL by opening the port at 1200 bps or with `picotool`, and long runs.

The 100 µs tick, the time slices, the CPU load, `sleep`, `irqstack` and the fatal error record
(2026-11-17) have only been built (without warnings) and, for the record's format, tested on the host; none
of them has been checked on the board yet.
