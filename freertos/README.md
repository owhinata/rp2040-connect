# Pico SDK + FreeRTOS build

English | [日本語](README_ja.md)

A build environment for firmware on the Raspberry Pi Pico SDK and the FreeRTOS kernel, for the Arduino
Nano RP2040 Connect. Like `zephyr/`, `nuttx/` and `openmv/`, it fetches what it needs under `freertos/`.

For now it holds one application, `app/usb-console`: a command line (CLI) on USB and a blinking LED. The
IMU measurement of the Zephyr and NuttX builds has not been ported yet.

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
| `ps` | The tasks: state, priority, the least free stack so far (bytes), share of the CPU time since the start |
| `free` | Size of the FreeRTOS heap, free space and the least free space so far |
| `reboot` | Restart |
| `bootsel` | Restart into the USB bootloader (BOOTSEL, the `RPI-RP2` drive) |

| File | Role |
|---|---|
| `CMakeLists.txt` | Imports the Pico SDK and the FreeRTOS kernel; the USB product name (`FreeRTOS USB Console`) |
| `FreeRTOSConfig.h` | Kernel configuration |
| `src/main.c` | Start-up, the console task, the LED task, the kernel's hooks |
| `src/cli.[ch]` | Line editing and the table of commands. Depends on neither the kernel nor the SDK |
| `src/commands.c` | The commands above |

- The tasks are `console` (priority 2, 4 KB of stack) and `heartbeat` (priority 1, toggles the LED on
  GPIO6 every 500 ms); both have statically allocated stacks. The kernel adds `IDLE` and `Tmr Svc`.
- The console task is woken by the notification of received USB data (the Pico SDK's
  `stdio_set_chars_available_callback()`), and every 50 ms to notice a terminal connecting.
- Kernel configuration: one core (`configNUMBER_OF_CORES` 1), a 1 kHz tick, priorities 0 to 31 (a higher
  number is a higher priority), 32 KB of heap with `heap_4`. The Pico SDK's locks and sleeps block the
  calling task (`configSUPPORT_PICO_SYNC_INTEROP`, `configSUPPORT_PICO_TIME_INTEROP`). A failed assertion,
  a stack overflow (`configCHECK_FOR_STACK_OVERFLOW` 2) and an exhausted heap stop in `panic()`.
- The USB product name is `FreeRTOS USB Console`. The Pico SDK cuts USB strings to 19 characters by
  default, so the limit (`USBD_DESC_STR_MAX`) is set to 32.
- The tick differs from that of the Zephyr and NuttX builds (100 µs). It will be decided again when the
  measurement code is ported.
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
