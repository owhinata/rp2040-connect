# Pico SDK + FreeRTOS build

English | [日本語](README_ja.md)

A build environment for firmware on the Raspberry Pi Pico SDK and the FreeRTOS kernel, for the Arduino
Nano RP2040 Connect. Like `zephyr/`, `nuttx/` and `openmv/`, it fetches what it needs under `freertos/`.

Like the Zephyr and NuttX builds, it reads the raw data of the LSM6DSOX at 6667 Hz and sends it to the PC
over MQTT on Wi-Fi. Most of the sources are in `common/`, shared by the three builds.

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
| `freertos/stmemsc/` | ST's register definitions of the LSM6DSO (`lsm6dso_reg.[ch]`) | The commit the NuttX build uses (SHA-256 checked) |

It also sets `PICO_SDK_PATH`, `FREERTOS_KERNEL_PATH`, `PICO_TOOLCHAIN_PATH` and `PATH` (the toolchain,
`freertos/.venv/bin` and `freertos/script`) in the current shell. The versions are pinned at the top of the script. What it
fetches is not under Git.

## Build

```bash
freertos/script/build.sh [--clean] [-j N] [-v] [-DNAME=VALUE ...] [APP]
```

Builds `freertos/app/APP` (default: `usb-console`) in `freertos/build/APP/`. The result is
`freertos/build/usb-console/usb-console.uf2`. The script sources `setup.sh` itself.

On the first build the Pico SDK fetches the source of `picotool` from GitHub and builds it for the host
(this needs the network).

`-DNAME=VALUE` is an option passed to CMake; it stays until `--clean`.

| Option | Default | Content |
|---|---|---|
| `-DI2C_DW_I2C1=ON` | OFF | Makes I2C1 available (SDA on GPIO18 = D6, SCL on GPIO19 = D7, internal pull-ups) with the same driver as I2C0. Not tried with a real device. |

## Host tests

```bash
freertos/script/host_tests.sh [-v]
```

Runs the tests of the shared sources and of the parts of the FreeRTOS build on the host (about 2 minutes).
No board is needed. It uses the kernel fetched by `setup.sh` and the host's `cc`; some tests also use
`python3`, `iperf3` and `host/.venv` and are skipped without them. The list is in
[`drivers/README.md`](drivers/README.md).

## Flashing

```bash
freertos/script/flash.sh [--build] [--wait SEC] [--dry-run] [UF2]
```

1. If there is an `RPI-RP2` drive (an RP2040 in BOOTSEL), the UF2 is copied to it.
2. Otherwise, if the USB console of one of the three builds (Zephyr, NuttX, FreeRTOS) is there, the script
   sends it the `bootsel` command and waits for the drive. Nothing has to be done by hand (nothing is sent
   while another program has the console open).
3. If neither is there, or the shell does not answer, it explains how to enter BOOTSEL and waits for the drive.

`--dry-run` only prints what would be done. The work is done by `script/flash_uf2.sh`, shared by the three builds.

After the copy it prints the path of the USB console (`/dev/serial/by-id/usb-*FreeRTOS_USB_Console*`).

Entering BOOTSEL:

- With this firmware running: the `bootsel` command.
- With the Zephyr or NuttX build running: the `bootsel` command of its shell (`flash.sh` sends it). With an image from before
  2026-11-18, press the RESET button twice quickly; if that does not work, connect REC to GND and press RESET.
- With the OpenMV build running: run `import machine; machine.bootloader()` in the REPL.

**Before flashing**: flashing replaces the firmware of the Zephyr, NuttX or OpenMV build. It does not
rewrite the last 8 KiB (the key-value store, shared with the Zephyr and NuttX builds).

## Console

The console is the USB CDC ACM port (UART0 is not used). Opening a terminal shows the prompt
`freertos:~$ `.

| Command | Content |
|---|---|
| `help` | List the commands |
| `version` | Versions of the FreeRTOS kernel, the Pico SDK and the compiler |
| `id` | The board's unique ID (16 hexadecimal digits) |
| `uptime` | Time since the start and the number of ticks |
| `sensor get <device> [channel...]`, `sensor attr_set`, `sensor attr_get`, `sensor stream` | Read and set the sensor's values and attributes, and print its stream (the syntax and output of the Zephyr build). The device is `lsm6dsox@6a` |
| `sleep <ms>` | Sleep for ms milliseconds |
| `date` | Date and time (UTC, in the format of the NuttX build; counts from 1970 until the time is synchronized) |
| `devmem <address> [<width> [<value>]]` | Read or write memory. Address and value in hexadecimal, width 8, 16 or 32 bits (default 32). A value writes |
| `ps` | The tasks (state, priority, least free stack, share of the CPU time since the previous `ps`), the free interrupt stack (the `(interrupts)` row) and the CPU load since the previous `ps`. Type it at the start and at the end of the interval of interest |
| `imu start`, `imu stop`, `imu status`, `imu cal [reload\|zero]` | Start and stop the IMU acquisition, show its counters, show and reload the calibration (as in the Zephyr build) |
| `wifi connect`, `wifi disconnect`, `wifi scan`, `wifi status` | Connect (`connect -s <SSID> -k 1 -p <passphrase>`; `-k 0` for an open network), disconnect, scan, and show the state of Wi-Fi: the part of the Zephyr build's `wifi` that works on this board |
| `wifi cred add`, `wifi cred delete`, `wifi cred list`, `wifi cred auto_connect` | Add, delete and list the stored credentials, and connect to a stored network. `list` shows the passphrases too |
| `zperf tcp upload <IP> [port] [seconds] [size]`, `zperf tcp download [port]`, `zperf tcp download stop` | TCP throughput test. On the PC: `iperf3 -s` / `iperf3 -c <board>`. No UDP |
| `ntp status` | The state of the time synchronization (the output of the Zephyr build) |
| `ifconfig` | MAC address, link state, IP address, gateway and mask (the format of the NuttX build) |
| `dmesg` | The RAM log (4 KB; kept across restarts) |
| `kv set <key> <value>`, `kv sethex <key> <hex>`, `kv get <key>`, `kv delete <key>` | Read and write the key-value store (settings and calibration). `get` prints hexadecimal |
| `kv wipe confirm` | Erase the whole key-value area (the Wi-Fi credentials too) |
| `nina version` | The firmware version of the NINA-W102 and its capabilities (second line; `in use` are those this firmware uses) |
| `nina passthrough confirm` | Restart into the NINA passthrough (USB serial <-> the NINA's UART, for reading and writing the NINA's firmware with `esptool`). Leave it by closing the port at 300 bps, or with RESET |
| `i2c_dma mode [on\|off]`, `i2c_dma stats` | Switch or show the DMA reads of I2C0; show their counters |
| `fault` | The record of the fatal error that ended the last run |
| `free` | Use of the FreeRTOS heap and of `malloc()` |
| `reboot` | Restart |
| `bootsel` | Restart into BOOTSEL (the `RPI-RP2` drive) |

- Line editing: Backspace, Ctrl-C (drop the line) and the up and down arrows (history). There is no Tab
  completion and no editing inside the line. Quotes and `\` work as in the Zephyr build's shell.
- A fatal error (HardFault, `panic()`, an assertion, a stack overflow, an exhausted heap) leaves a record
  and restarts the board by itself. The record is shown when a terminal connects next, by `fault` and in
  `dmesg`.
- The key-value store is in the last 8 KiB of the flash, in the format of the Zephyr and NuttX builds, so
  it survives a change of build. If the area holds other data, `kv` fails with `-138` and `dmesg` says
  what to do; `kv wipe confirm` erases it.
- `imu start` sends over MQTT to `imu.host` of the key-value store (the default gateway if not set), port
  `imu.port`. Topics and messages are those of the Zephyr and NuttX builds, so `imu-host` on the PC receives
  them as they are. Without a receiver it waits about 3 s, then only acquires and counts the batches as
  dropped.
- Once Wi-Fi is connected, the board asks the NTP server of the default gateway (the PC, UDP port 123) every
  2 s and keeps its time. Run chrony or similar on the PC.
- About 4 s after the start the board connects to a stored network by itself, and it reconnects when the
  connection is lost. After `wifi disconnect` it does not reconnect by itself until the next connection.
  The credentials are in the key-value store, shared with the Zephyr and NuttX builds.
- The same shell is on Telnet (port 23): one connection at a time, no authentication, no encryption.
  Nothing is shown on connecting (Enter gives the prompt). Echo and line editing are the client's. A second
  connection is closed at once.
- After the start, set the sampling frequency before `sensor get` (for example
  `sensor attr_set lsm6dsox@6a accel_xyz sampling_frequency 104 gyro_xyz sampling_frequency 104`). The stream
  is `sensor stream lsm6dsox@6a on fifo_wm incl` and `off`. During a measurement (`imu start`) the reads of
  `sensor` are refused (-16).
- Reading an address that does not exist with `devmem` ends in a fatal error (for example
  `devmem 30000000`).

- `wifi scan` while connected keeps the connection (when the NINA's firmware supports it; the second line of
  `nina version` tells). During the scan the radio leaves the AP's channel, so traffic slows down.
- Reading and writing the NINA's firmware: after `nina passthrough confirm`, use `esptool` on the PC with this
  board's serial port. Give `esptool` a configuration file with the two lines `[esptool]` and
  `custom_reset_sequence = R0|D1|W0.2|R1|W0.6` through `ESPTOOL_CFGFILE`, and pass
  `--before default-reset --after no-reset` (for example `read-flash 0 0x200000 <file>`,
  `write-flash 0 NINA_W102.bin`). The passthrough writes nothing to the flash (the key-value store included).
  A file read from the NINA may contain Wi-Fi credentials: keep it private.

## Directories

| Place | Content |
|---|---|
| `app/usb-console/` | The application: start-up, commands, kernel configuration (`FreeRTOSConfig.h`) |
| `drivers/compat/` | The FreeRTOS part of the Zephyr compatibility layer (runs the shared sources in `common/`) |
| `drivers/shell/`, `drivers/libc/` | The USB serial transport of the console and the connection of standard output (the console, shell, Telnet, RAM log and network layer are in `../common/`) |
| `drivers/kv/` | Reading and writing the flash of the key-value store |
| `drivers/nina/` | Adds the NINA-W102 driver to the build (the driver is `../common/drivers/nina/`) |
| `drivers/wifi/` | Brings in Wi-Fi (the code is in `common/`) |
| `drivers/telnet/`, `drivers/zperf/` | Bring in the Telnet shell and the throughput test (the code is in `../common/`) |
| `drivers/i2c_dw/` | I2C0 and DMA (the sensor's bus) |
| `drivers/lsm6dso/` | Build rules and the start-up check of the LSM6DSOX (the driver and its device are in `common/drivers/lsm6dso/`) |
| `drivers/imu/` | Brings in the acquisition, the transport and the time synchronization (in `common/imu/`) |
| `drivers/sensor_shell/` | Brings in the `sensor` command (in `common/drivers/sensor_shell/`) |
| `tests/` | Host tests |
| `script/` | `setup.sh`, `build.sh`, `flash.sh`, `format.sh`, `host_tests.sh` |
| `PORTING_PLAN.md`, `PORTING_STATUS.md` | The plan of the port, and the record of its progress and of what was checked (in Japanese) |

The internals (the compatibility layer, the tasks, the differences from the Zephyr and NuttX builds) are in
[`drivers/README.md`](drivers/README.md). The key-value store is saved and restored with `script/kv_transfer.py`
at the root (`backup --usb`, `restore --usb`).

Sources are formatted with `freertos/script/format.sh` (`--check` only checks).
