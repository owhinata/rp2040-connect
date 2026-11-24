# Pico SDK + ThreadX build

English | [日本語](README_ja.md)

An environment that builds the firmware for the Arduino Nano RP2040 Connect on the Raspberry Pi Pico SDK and
the ThreadX kernel. As `zephyr/`, `nuttx/` and `freertos/` do, it fetches what it needs under `threadx/`.

Like the Zephyr, NuttX and FreeRTOS builds, it reads the LSM6DSOX's raw data at 6667 Hz and sends it to the PC
by MQTT over Wi-Fi. Most of the sources are in `common/`, shared by the four builds, and the part on the Pico
SDK (the start, the console, the KV, the board part of the drivers) is the FreeRTOS build's files.

| Item | Content |
|---|---|
| Pico SDK | 2.3.1 (board `arduino_nano_rp2040_connect`) |
| ThreadX | Eclipse ThreadX v6.5.2.202603b, the kernel only (port `ports/cortex_m0/gnu`; no USBX, NetX Duo or FileX) |
| Compiler | Arm GNU Toolchain 15.2.Rel1 (GCC 15.2) |

## Requirements

The same as the [FreeRTOS build](../freertos/README.md) (a Linux PC, `git`, `python3`, `cmake`, `make`, `gcc`
and so on; about 1.5 GB of disk).

## Setup

Source it at the root of the repository. It fetches only what is missing, so it can be sourced any number of
times.

```bash
source threadx/script/setup.sh
```

It fetches ThreadX (`threadx/threadx`), the Pico SDK, the Arm GNU Toolchain, `picotool`, `clang-format` and
ST's register definitions of the LSM6DSO under `threadx/`. The versions are fixed at the top of the script.
What it fetches is not tracked by Git.

## Build

```bash
threadx/script/build.sh [--clean] [-j N] [-v] [-DNAME=VALUE ...] [APP]
```

Builds `threadx/app/APP` (`usb-console` by default) in `threadx/build/APP/`. The result is
`threadx/build/usb-console/usb-console.uf2`.

| Option | Default | Content |
|---|---|---|
| `-DI2C_DW_I2C1=ON` | OFF | Makes I2C1 (SDA on GPIO18 = D6, SCL on GPIO19 = D7) usable with the driver of I2C0. Not checked on the board. |

## Host tests

```bash
threadx/script/host_tests.sh [-v]
```

Runs the tests of the shared sources and of the ThreadX part (the kernel part of the compatibility layer, the
waits of I2C and of the NINA) on ThreadX's Linux port (about 25 s). No board is needed. The items are listed
in [`drivers/README.md`](drivers/README.md).

## Flashing

```bash
threadx/script/flash.sh [--build] [--wait seconds] [--dry-run] [UF2]
```

This is `script/flash_uf2.sh`, shared by the four builds. With an `RPI-RP2` drive it copies the UF2; without
one it sends `bootsel` to the USB shell of the firmware that is running and waits for the drive (it does not
send when another program has the console open). After the copy it prints the path of the USB console
(`/dev/serial/by-id/usb-*ThreadX_USB_Console*`).

**Before flashing**: flashing replaces the firmware of the other builds. The last 8 KB of the flash (the KV,
shared by the four builds) are not rewritten.

## Console

The USB CDC ACM port is the console. A terminal shows the prompt `threadx:~$ `. The commands are those of the
[FreeRTOS build](../freertos/README.md) (the table under "Console"), with these differences:

- `version` prints ThreadX's version.
- The PRIO column of `ps` is ThreadX's number (0 is the most urgent; the Zephyr priority + 16). The time in
  which nothing runs is the row `(idle)`. Threads that have ended are not listed.

## Directories

| Place | Content |
|---|---|
| `app/usb-console/` | The ThreadX part of the application: the entry `main()` (`src/main.c`), the kernel configuration (`tx_user.h`), the tick, the priorities, the ThreadX parts of the fatal errors and `ps` |
| `drivers/compat/` | The ThreadX part of the Zephyr compatibility layer (runs the shared sources of `common/`) |
| `drivers/pico_sdk_interop/` | Ties the Pico SDK's locks and waits to ThreadX's threads |
| `../common/pico_sdk/` | What the builds on the Pico SDK share (the same as the FreeRTOS build) |
| `../common/drivers/<driver>/port/` | The drivers' glue: `pico_sdk/` (the same as the FreeRTOS build) and `threadx/` (waits and locks) |
| `tests/` | The host tests and the board test firmware `board_kernel` ([`drivers/README.md`](drivers/README.md)) |
| `script/` | `setup.sh`, `build.sh`, `flash.sh`, `format.sh`, `host_tests.sh` |
| `PORTING_PLAN.md`, `PORTING_STATUS.md` | The plan of the port and the record of progress and of what was checked |
