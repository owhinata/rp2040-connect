# Build environment for the OpenMV MicroPython firmware

English | [日本語](README_ja.md)

An environment to build the [OpenMV](https://github.com/openmv/openmv) firmware (OpenMV's
MicroPython) from source for the Arduino Nano RP2040 Connect. Like `zephyr/` and `nuttx/`, it
fetches what it needs under `openmv/`. It is a build environment only; there is no code of this
repository in it yet. It is separate from the IMU acquisition firmware (the Zephyr and NuttX
builds).

| Item | Content |
|---|---|
| OpenMV | v5.0.1 (MicroPython 1.28.0) with the patches in `patches/openmv/`. The target is `ARDUINO_NANO_RP2040_CONNECT` |
| Tools | OpenMV SDK 1.6.0 (Arm GNU Toolchain 14.3.Rel1, CMake) |

v4.7.0 is the last release for which OpenMV published firmware for this board, and the rp2 port of
v5.0.1 does not build as it is. v5.0.0 changed the communication with OpenMV IDE to a new protocol
(`protocol/`), and the rp2 port was not moved to it. v4.7.0 speaks only the
old protocol. Here the rp2 port of v5.0.1 is moved to the new protocol with patches ("Patches").
The version is shown as `v5.0.1-dirty`.

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

1. Fetches OpenMV v5.0.1 into `openmv/openmv/` (a shallow clone; the commit of the tag is checked).
2. Fetches only the submodules that the rp2 port uses (`lib/micropython`, `modules/ulab` and
   `lib/apriltag` of OpenMV; `lib/pico-sdk`, `lib/mbedtls`, `lib/tinyusb`, `lib/mynewt-nimble` and
   `lib/micropython-lib` of MicroPython).
3. Applies `patches/openmv/*.patch` to the OpenMV clone. What was applied is recorded in the
   clone's `.git/openmv-patches/`; when a patch is added, removed or changed, the recorded ones
   are taken back and the current ones applied.
4. Provides OpenMV SDK 1.6.0. One in `~/openmv-sdk-1.6.0` (where OpenMV's `make sdk` puts it) is
   used as it is; otherwise it is downloaded from `https://download.openmv.io/sdk/` (about 0.9 GB,
   the SHA-256 is checked) and extracted to `openmv/openmv-sdk-1.6.0/`.
5. Sets `OPENMV_SDK_DIR` and adds the SDK's `gcc/bin` and `cmake/bin`, and `openmv/script`, to `PATH`.

The versions and the SHA-256 are at the top of `setup.sh`. Everything it fetches is outside Git.

## Build

```bash
openmv/script/build.sh [--clean] [-j N] [-v]
```

It sources `setup.sh` itself, builds `mpy-cross` and then the firmware, and copies the result to
`openmv/build/` (`firmware.uf2`, `firmware.bin`, `firmware.elf`). `--clean` removes
`openmv/openmv/build/ARDUINO_NANO_RP2040_CONNECT/` first.

In the first build, pico-sdk fetches `picotool` from GitHub and builds it for the host (this needs
the network).

## Patches

They are in `patches/openmv/`, and `setup.sh` applies them in name order.

| Patch | Content |
|---|---|
| `0001-rp2-port-to-openmv-v5.patch` | Brings the rp2 port up to v5. `main.c` moves to the new protocol (`omv_protocol`) and calls `soft_timer_init()` only once, as MicroPython's `main.c` does (calling it on every soft reboot makes pico-sdk stop on the alarm that is already claimed). The CMake source list, the build output directories and the linker script (the UMA memory block and its table) follow v5. The Cortex-M0+ has no DWT cycle counter, so `omv_cycles` uses the microsecond timer. |
| `0002-protocol-no-unaligned-access.patch` | Fixes three places where the protocol's receive path read 16-bit and 32-bit values straight out of the middle of a byte buffer. The Cortex-M0+ takes a HardFault on an unaligned access (the CPUs of the other ports do not). |

After changing a patch, source `openmv/script/setup.sh` again. After editing the clone directly,
write `git -C openmv/openmv diff` to the patches, then restore the clone with
`git -C openmv/openmv checkout -- .` and `rm -rf openmv/openmv/.git/openmv-patches`, and source
`setup.sh` again.

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
and a 14 MB USB drive (which holds `main.py`). The file system is mounted at `/flash`. On start the
default `main.py` keeps blinking the blue LED, so send Ctrl-C to the serial port to use the REPL.

OpenMV IDE opens the serial port at 921600 bps and speaks the new protocol. At any other speed the
port is the ordinary REPL.

Checked on the board (2026-11-16):

- Ctrl-C stops `main.py` and gives the REPL.
- `lsm6dsox` reads acceleration and angular rate (I2C0, SCL on GPIO13, SDA on GPIO12).
- `network.WLAN` reads the version of the NINA firmware (3.0.2). The NINA on the board runs the
  firmware patched in this repository (`firmware/nina-fw/`).
- The new protocol. With OpenMV's Python client (`openmv` 1.0.9 from PyPI): connecting, reading the
  version and the system information, listing the channels, running a script and reading its
  output, stopping a running script, enabling and disabling streaming. Running a script and the
  soft reboot after it were repeated more than 10 times.

- Connecting from OpenMV IDE and running its `lsm6dsox_mlc` example. The UCF file the example reads
  comes from `application_examples/lsm6dsox/` of ST's
  [STMems_Machine_Learning_Core](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core)
  and is copied to the USB drive.

Not checked: Wi-Fi connection and traffic, Bluetooth, the camera and audio modules, frame transfer
(there is no camera).

Notes:

- A file copied from the PC to the USB drive is not visible to MicroPython until the RESET button
  of the board is pressed (opening it gives `OSError: [Errno 2] ENOENT`).
- After even one line has been run in the REPL, running and stopping a script through the protocol
  have no effect (interrupts are disabled while the prompt waits; the other OpenMV ports are made
  the same way). A soft reboot with Ctrl-D, or a reset, brings it back.
- v4.8.1 builds without patches, but stops on the board when `main.py` is interrupted with Ctrl-C
  (the cause was not investigated). v4.7.0 builds and runs on the board, but speaks the old
  protocol.
