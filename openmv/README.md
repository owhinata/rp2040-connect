# Build environment for the OpenMV MicroPython firmware

English | [日本語](README_ja.md)

An environment to build the [OpenMV](https://github.com/openmv/openmv) firmware (OpenMV's
MicroPython) from source for the Arduino Nano RP2040 Connect. Like `zephyr/` and `nuttx/`, it
fetches what it needs under `openmv/`. It is a build environment only; there is no code of this
repository in it yet. It is separate from the IMU acquisition firmware (the Zephyr and NuttX
builds).

| Item | Content |
|---|---|
| OpenMV | v4.7.0 (MicroPython 1.25.0-preview). The target is `ARDUINO_NANO_RP2040_CONNECT` |
| Tools | OpenMV SDK 1.6.0 (Arm GNU Toolchain 14.3.Rel1, CMake) |

v4.7.0 is the last release for which OpenMV built and published firmware for this board. The
later releases could not be used on it.

| Release | Result |
|---|---|
| v4.7.0 | Builds and runs on the board. |
| v4.8.1 | Builds, but when `main.py` is interrupted with Ctrl-C on the board, it stops after the first line of the traceback and USB stops responding (a reset recovers it; the cause was not investigated). |
| v5.0.1 | The rp2 port does not build (it refers to `common/usbdbg.c`, which was removed). |

The OpenMV SDK was introduced with v5.0.0, and v4.7.0 is made to use the toolchain from Arm's site
directly (OpenMV's CI used 13.2.Rel1). Here the build uses the compiler (14.3.Rel1) and CMake of
SDK 1.6.0.

## Requirements

- A Linux PC (x86_64). No sudo.
- `git`, `curl` or `wget`, `tar`, `xz`, `sha256sum`
- `python3`, `make`, `gcc`, `g++` (to build `mpy-cross` and `picotool` for the host)
- About 7 GB of free disk space (4.6 GB for the SDK, 0.9 GB for OpenMV and its submodules, 0.9 GB
  for the temporary download)

## Setup

Source it from the repository root. It fetches only what is missing, so it can be sourced any
number of times. Source it again in a new terminal.

```bash
source openmv/script/setup.sh
```

What it does:

1. Fetches OpenMV v4.7.0 into `openmv/openmv/` (a shallow clone; the commit of the tag is checked).
2. Fetches only the submodules that the rp2 port uses (`lib/micropython` and `modules/ulab` of
   OpenMV; `lib/pico-sdk`, `lib/mbedtls`, `lib/tinyusb`, `lib/mynewt-nimble` and
   `lib/micropython-lib` of MicroPython).
3. Provides OpenMV SDK 1.6.0. One in `~/openmv-sdk-1.6.0` (where OpenMV's `make sdk` puts it) is
   used as it is; otherwise it is downloaded from `https://download.openmv.io/sdk/` (about 0.9 GB,
   the SHA-256 is checked) and extracted to `openmv/openmv-sdk-1.6.0/`.
4. Sets `OPENMV_SDK_DIR` and adds the SDK's `gcc/bin` and `cmake/bin`, and `openmv/script`, to `PATH`.

The versions and the SHA-256 are at the top of `setup.sh`. Everything it fetches is outside Git.

## Build

```bash
openmv/script/build.sh [--clean] [-j N] [-v]
```

It sources `setup.sh` itself, builds `mpy-cross` and then the firmware, and copies the result to
`openmv/build/` (`firmware.uf2`, `firmware.bin`, `firmware.elf`). `--clean` removes
`openmv/openmv/build/` first.

In the first build, pico-sdk fetches `picotool` from GitHub and builds it for the host (this needs
the network).

## Flashing

Copy `openmv/build/firmware.uf2` to the `RPI-RP2` drive in BOOTSEL mode. There is no script for it.

```bash
cp openmv/build/firmware.uf2 /media/$USER/RPI-RP2/ && sync
```

Entering BOOTSEL:

- While the Zephyr or NuttX build is running: press the RESET button twice quickly.
- While this firmware is running: run `import machine; machine.bootloader()` in the REPL.

**Before flashing**: MicroPython uses the last 14 MB of the flash for its file system (FAT,
`MICROPY_HW_FLASH_STORAGE_BYTES`). The KV area of the Zephyr and NuttX builds (calibration,
`imu.port`, Wi-Fi credentials) lies in that range, so save it first (`script/kv_transfer.py`).

## After flashing

The board appears as a USB serial port (`/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_*`)
and a 14 MB USB drive (which holds `main.py`). On start the default `main.py` keeps blinking the
blue LED, so send Ctrl-C to the serial port to use the REPL.

Checked on the board (2026-11-15, once each):

- Ctrl-C stops `main.py` and gives the REPL.
- `lsm6dsox` reads acceleration and angular rate (I2C0, SCL on GPIO13, SDA on GPIO12).
- `network.WLAN` reads the version of the NINA firmware (3.0.2). The NINA on the board runs the
  firmware patched in this repository (`firmware/nina-fw/`).

Not checked: Wi-Fi connection and traffic, Bluetooth, connecting from OpenMV IDE, the camera and
audio modules.
