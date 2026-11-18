# rp2040-connect

English | [日本語](README_ja.md)

A Zephyr workspace for the Arduino Nano RP2040 Connect (RP2040, NINA-W102, LSM6DSOX).
The main application, `zephyr/app/usb-console`, reads accelerometer and gyroscope data at 6667 Hz as
raw values, adds the PC's time, and sends it over Wi-Fi with MQTT.

- The IMU FIFO is read over I2C (with DMA) and batched with numbers that make loss detectable.
- Acquisition continues when Wi-Fi drops; what cannot be sent is discarded and counted.
- The board synchronizes with an NTP server on the PC and puts the PC's time (Unix time in
  microseconds) on every batch.
- It is operated from a shell over USB or Telnet.

See the [IMU RAW and MQTT design](design/imu-raw-mqtt-design.md) for the details and
[zephyr/app/usb-console/drivers](zephyr/app/usb-console/drivers/README.md) for the drivers (LSM6DSO, I2C,
NINA). The same firmware also has a NuttX build (`nuttx/`) and a Pico SDK + FreeRTOS build (`freertos/`)
("The three builds and `common/`" below).

This is experimental. Measurements are started and stopped by hand from the shell, and the
only software on the PC side is a receiver script for verification. The design document lists
what has and has not been verified.

## Setup

These steps are for Ubuntu 24.04. Install the packages needed for the build first.

```bash
sudo apt update
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
  wget python3-dev python3-venv python3-tk xz-utils file make gcc \
  gcc-multilib g++-multilib libsdl2-dev libmagic1
```

Leave out `gcc-multilib` and `g++-multilib` on AArch64. For other systems see the
[Zephyr getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/).

The Zephyr workspace is under `zephyr/`; run the commands below there. First source the
setup script.

```bash
cd zephyr
source script/setup.sh
```

It prepares, under `zephyr/`, `.venv`, the west workspace (Zephyr v4.5.0 and modules), Python packages and the
Zephyr SDK as needed, applies this repository's local patches to Zephyr and its modules
(`zephyr/manifest/zephyr/patches.yml`), and sets `ZEPHYR_SDK_INSTALL_DIR` and a `west` alias in the
current shell. Source it again in every new terminal. "Local patches of Zephyr" in the
[driver notes](zephyr/app/usb-console/drivers/README.md) describes the patches and how to take them back.

## Build and flash

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
./script/flash.sh            # write build/usb-console/zephyr/zephyr.uf2 (--build: build first)
```

There is one configuration, `zephyr/app/usb-console/prj.conf`. `memcpy` and `memset` use the boot ROM's
routines (`CONFIG_RP2040_ROM_MEM_OPS`), integer division the hardware divider (`CONFIG_NANO_HW_DIVIDER`),
and `CONFIG_NANO_NINA_PASSTHROUGH` adds `nina passthrough confirm` (all on by default).

The UF2 goes in through the bootloader mode (BOOTSEL), in which a USB drive named `RPI-RP2` appears.
`flash.sh` (shared with the NuttX and FreeRTOS builds: `script/flash_uf2.sh`) copies the UF2 to that
drive. Without the drive, it sends `bootsel` to the USB shell of whichever of the three builds is
running (not when the console is open in another program), waits for the drive and its mount, and
after the copy prints the new USB console. When no shell answers, it shows how to enter BOOTSEL by
hand and waits: press RESET twice quickly, or, if that does not do it either, press RESET while `REC`
and `GND` are joined with a jumper wire, and remove the wire once the drive appears. `--dry-run`
only shows what it would do.

`zephyr/app/blink` is a minimal sample that toggles `led0` every 500 ms.

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

The NINA-W102 is expected to run Arduino NINA firmware 3.0.2 with this repository's patches (0001 to
0006). The driver also works, as before, with a firmware that lacks what 0003 to 0006 add (a scan that
keeps the connection, socket events, and so on). See [firmware/nina-fw](firmware/nina-fw/README.md) for
the build and cautions. The first line of `nina version` is the version, the second
(`NINA capabilities: 0x... (in use 0x...)`) the module's capability bits and those in use.

### Reading and writing the NINA firmware

`nina passthrough confirm` restarts the board with its USB serial port relayed to the NINA's UART (all
three builds; the RP2040's firmware and KV store stay). esptool on the PC then reads and writes the module (run the example from the repository root).
esptool 5.3.0 gets no answer with `--before no-reset`, so give it a configuration file that leaves RTS set:

```bash
printf '[esptool]\ncustom_reset_sequence = R0|D1|W0.2|R1|W0.6\n' > esptool.cfg
export ESPTOOL_CFGFILE=$PWD/esptool.cfg
esptool --port <port> --baud 921600 --before default-reset --after no-reset \
  read-flash 0 0x200000 firmware/nina-fw/images/backup.bin      # back up (about 26 s)
esptool --port <port> --baud 921600 --before default-reset --after no-reset \
  write-flash 0 firmware/nina-fw/source/NINA_W102.bin           # write (about 20 s)
stty -F <port> 300                                               # leave the passthrough
```

Setting the port to 300 bps and closing it, or RESET, starts the normal firmware again. A backup may hold
credentials; keep it private (`firmware/nina-fw/images/` is not tracked by Git). The Arduino
SerialNINAPassthrough sketch is not needed.

## Usage

### 1. Connect to the shell

Open a terminal on the USB serial port (CDC ACM) and press Enter for a prompt.

```bash
minicom -D /dev/ttyACM0
```

The device name varies; `/dev/serial/by-id/` shows it.

### 2. Connect to Wi-Fi

WPA2-PSK on 2.4 GHz is supported. With stored credentials the board connects at boot and after
an unexpected disconnection.

```text
wifi cred add -s "SSID" -k 1 -p "passphrase"
wifi cred auto_connect
wifi status
net iface
```

- The passphrase is echoed to the terminal, and `wifi cred list` prints it too.
- `wifi connect` only connects; it does not store the settings.
- After `wifi disconnect` the board stays disconnected until the next boot or manual connect.
- `wifi scan` keeps the connection when the NINA firmware has patch 0004. Without it, the scan
  interrupts the connection and reconnects with the stored settings afterwards.

`script/wifi_ap.sh` turns a Wi-Fi adapter on the PC into a temporary access point.

```bash
./script/wifi_ap.sh ap <USB-interface> --ssid rp2040-test --channel 6
./script/wifi_ap.sh station <USB-interface>
```

### 3. Run an NTP server on the PC

By default the board queries the default gateway, the PC providing the access point above,
with NTP every 2 seconds. Install chrony on the PC and allow queries from the access point's
subnet.

```bash
sudo apt install chrony
sudo tee /etc/chrony/conf.d/rp2040-ap.conf >/dev/null <<'CONF'
allow 10.42.0.0/24
local stratum 10
CONF
sudo systemctl restart chrony
```

Open UDP port 123 if a firewall such as ufw is active. `ntp status` shows the board's side.

### 4. Receive on the PC

The PC-side tools are in `host/`. See [`host/README.md`](host/README.md) for installation
and for recording (`record`) and calibration (`calib`). The desktop application that converts
the received data to physical units and shows the attitude is in
[`host/ImuStudio/`](host/ImuStudio/README.md).

`imu-host check` is a receiver for verification. It accepts one MQTT client directly, checks
the format, sequence numbers, sample numbers and times of the RAW batches, and prints a
summary as JSON when it ends. It is not a general MQTT broker and stores nothing.

```bash
host/.venv/bin/imu-host check --port 1883
```

Ctrl-C ends it and prints the result. `--drop-at`, `--stall-at` and `--rcvbuf` reproduce a
stopped or congested receiver.

### 5. Start a measurement

```text
imu start
imu status
imu stop
```

`imu start` takes no arguments. It sets the sensor to 6667 Hz, FIFO watermark 128 and I2C at
1 MHz, and streams until stopped.

- **No MQTT server running**: acquisition starts, and batches are discarded and counted until
  the connection is made. Delivery starts from that point.
- **No NTP synchronization**: batches are sent with their PC time marked invalid. They still
  carry the sensor tick.
- **No Wi-Fi**: acquisition starts alone, and sending begins once Wi-Fi and MQTT are up.

Settings come from the key-value store. All are optional and fall back to the defaults.

| Key | Meaning | Default |
|---|---|---|
| `imu.host` | IPv4 address of the MQTT server | the default gateway |
| `imu.port` | Port of the MQTT server | `1883` |
| `imu.device` | Name used in the topic `imu/<device>/raw` | `rp2040` |
| `imu.odr` | Output data rate of both sensors in Hz | `6667` |
| `imu.watermark` | FIFO watermark in words | `128` |

```text
kv set imu.port 18830
kv delete imu.port
```

A session ID is assigned at every start from the chip's unique ID and a stored counter.

## Shell commands

| Command | Purpose |
|---|---|
| `imu start` / `stop` / `status` | Start and stop a measurement, show counters and errors |
| `imu cal` / `imu cal reload` / `imu cal zero` | Show the conversion `sensor get` / `stream` use; read the KV store again; take the gyro bias at rest |
| `ntp status` | State of the synchronization with PC time |
| `wifi ...`, `net ...` | Wi-Fi and IP (Zephyr) |
| `nina version` | NINA firmware version and capability bits |
| `nina passthrough confirm` | Restart with the USB serial port relayed to the NINA's UART (for esptool) |
| `bootsel` | Restart into BOOTSEL (`RPI-RP2`) |
| `kv set` / `sethex` / `get` / `delete` | Key-value store |
| `sensor ...` | One-shot reads, streaming and attributes (Zephyr, extended) |
| `i2c_dma mode` / `stats` | DMA for I2C reads: switch and counters |
| `zperf ...` | Throughput test with iperf3 (Zephyr, see below) |
| `dmesg` | Show the log kept in RAM |
| `kernel ps`, `kernel reboot` | Thread CPU and stack use, reboot |
| `gpio ...`, `i2c ...`, `log ...` | Zephyr |

### `imu status`

One item per line, grouped by category. Every item under `errors` and `fifo`, and the items
from `unix_unavailable` to `tick_out_of_range` under `time`, stay at 0 when all is well.

| Category | What to look at |
|---|---|
| top | `state`, the session, `epoch` (advances by one per loss of unknown size), `cpu_active` |
| `acquired` | Samples and batches acquired, counted whether or not they could be sent |
| `errors` | Why the stream ended; a non-zero value means the measurement is over |
| `fifo` | FIFO overruns, I2C errors, number of automatic restarts |
| `time` | Whether PC time is available, batches left without time, deviations from the timestamp layout |
| `mqtt` | Server, connection state, batches sent and discarded, peak queue use |

`state` is `idle` (no measurement yet), `starting` (just started, before the first summary),
`running` or `stopped`. While it is `starting`, the items below still show the previous run.

### Applying a calibration

The physical values (m/s², rad/s) that `sensor get` and `sensor stream` print can use a
calibration kept in the KV store. Compute it on the PC with `imu-host calib` and write it with
`imu-host write` ([`host/README.md`](host/README.md)). **It is not applied to the RAW data
sent over MQTT.**

```text
imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
  gyro bias: 7.65 -4.25 -0.81 LSB (tracked at rest, 14 updates, 0 rejected)
```

A calibration is stored under a key per sensor and sensitivity, and is used only while a key
for the current sensitivity exists. Otherwise, for example after changing a range, the nominal
sensitivity is used (`nominal`). After changing the keys with `kv`, run `imu cal reload`; the
keys are also read at boot.

**The gyro bias** changes a little at every power-up, so it is taken again from data recorded
at rest.

- During an `imu start` measurement and during `sensor stream` it is updated automatically
  each time the board has rested for about a second. It is not updated while moving.
- While the sensor is not running, `imu cal zero` takes it on request. Put the board down and
  run it; the sensor runs for one to three seconds and is switched off again. It cannot be used
  during a measurement, which updates the bias itself.
- The value is kept in RAM only. A reboot, `imu cal reload` or a range change returns to the
  bias of the stored calibration.
- A value more than 1 dps from the stored one is not used and counted as `rejected`, because
  turning slowly about the gravity axis cannot be told from rest.

### Sensor attributes

The FIFO settings are available through `sensor attr_set` and `attr_get`. Both apply at the
next stream start and cannot be changed while a stream runs. `imu start` overrides the
watermark with the value from the key-value store (default 128).

```text
sensor attr_set lsm6dsox all fifo_watermark 128
sensor attr_set lsm6dsox all fifo_timestamp_decimation 8
```

`fifo_timestamp_decimation` is the interval between timestamps in the FIFO: 0 (none), 1, 8 or
32, with 8 as the default. With 0 the batches carry no time.

### Throughput test (zperf)

Zephyr's `zperf` is enabled with the iperf3 protocol; use `iperf3` on the PC.

Board to PC (sending): run `iperf3 -s -p 5201` on the PC first. The arguments are the
destination, port, seconds, size of one send and, for UDP, the rate.

```text
zperf tcp upload <PC IPv4 address> 5201 30 1024
zperf udp upload <PC IPv4 address> 5201 30 500 500K
```

PC to board (receiving): start the server on the board, then run
`iperf3 -c <board IPv4 address> -p 5201 -t 30` on the PC.

```text
zperf tcp download 5201
zperf tcp download stop
```

UDP sending at the requested 500 kbit/s had no loss (2026-11-12, 30 s, one run). The TCP rates are
in the last section, "Performance of the three builds". Both were taken with IMU streaming stopped and
depend on the radio environment.

### Shell over Wi-Fi

Once Wi-Fi is connected a Telnet shell is available on TCP port 23, for one client at a time.
It has no authentication or encryption and anyone who can reach it gets the full shell, so use
it on trusted networks only.

### Key-value store

Stored with Zephyr Settings on NVS. Keys are 1 to 32 characters of letters, digits, `_`, `-`
and `.`; values are 1 to 128 bytes. `set` stores text, `sethex` stores bytes given in hex, and
`get` prints hex. Keys starting with `sys.` are reserved. Data is not encrypted. Wi-Fi
credentials share the same 8 KiB area.

### Log

The log is kept in a 4 KiB ring buffer in RAM and shown with `dmesg`. It survives a normal
reset but not a power cycle.

On a fatal error (an exception, the stack sentinel and the like) the firmware logs the error report and
then reboots by itself (`CONFIG_RESET_ON_FATAL_ERROR=y`). The report can be read with `dmesg` after the
reboot.

## Tests

```bash
zephyr/script/host_tests.sh
```

This runs, on the PC and without a board, the tests for FIFO word parsing, batches and
payloads, the send queue, time assignment, the NTP estimate, the driver's FIFO setup and stop
handling, and NINA SPI framing. The tests of the sources shared with the NuttX and FreeRTOS builds
and of the OS-independent parts of the Zephyr API compatibility layer, the shell's subcommand handling,
the RAM log's ring and the key-value store (`common/tests/run.sh`) run first.

To check on hardware, follow the usage steps with the receiver running and look at its
summary (missing batches, valid and reversed times) and at the errors in `imu status`.

## Limitations

- No TLS. MQTT is unencrypted and unauthenticated.
- The build uses Zephyr's non-cryptographic test random generator.
- The board's estimate was verified to stay within a few milliseconds of PC time, but the
  delay inside the sensor has not been measured.
- Zephyr's kernel clock (`kernel uptime` and the like) loses about 480 ppm while streaming at
  6667 Hz. It is not used as a time reference.

## Repository layout

| Path | Content |
|---|---|
| `zephyr/app/usb-console/src/` | The parts of the application that are the Zephyr build's own: key-value store, RAM log, start-up. Those shared with the other builds (measurement, time sync, MQTT, Wi-Fi settings, calibration and others) are in `common/imu/` |
| `zephyr/app/usb-console/drivers/` | Local drivers for the LSM6DSO, I2C and the sensor shell, and the Zephyr side of the NINA (the socket offload, the core's boundaries) |
| `zephyr/app/usb-console/tests/` | Host tests (those of the Zephyr build only) |
| `zephyr/app/blink/` | LED blink sample |
| `zephyr/script/` | Setup, host tests |
| `nuttx/` | The NuttX build (board definition, drivers, applications, patches, scripts, host tests) |
| `common/` | Sources whose files the three builds share (`imu/`: the acquisition service, transport, time synchronization, calibration, the Wi-Fi profile, the key-value API and the `kv` command; `drivers/`: the LSM6DSO driver, the sensor shell, the OS-independent core of the NINA, the OS-independent core of I2C0 and DMA (used by the NuttX and FreeRTOS builds); `compat/`: the OS-independent part of the Zephyr API compatibility layer that the NuttX and FreeRTOS builds use, with copies of Zephyr's libraries; `shell/` and `log/`: the shell's command handling and the RAM log (NuttX and FreeRTOS builds)) and their host tests (`tests/`; `tests/run.sh` is called by each build's `host_tests.sh`). The layout is in section 12 of the [design](design/imu-raw-mqtt-design.md) |
| `openmv/` | Build environment for the OpenMV MicroPython firmware ([`openmv/README.md`](openmv/README.md)) |
| `freertos/` | The Pico SDK + FreeRTOS build (build environment, application, the FreeRTOS part of the compatibility layer, drivers, scripts, host tests; [`freertos/README.md`](freertos/README.md)) |
| `zephyr/manifest/west.yml` | Versions of Zephyr and its modules |
| `zephyr/manifest/zephyr/` | Local patches of Zephyr and its modules (`patches.yml`, `patches/`) |
| `design/` | Design and investigation notes |
| `firmware/nina-fw/` | Fixes and build for the NINA firmware |
| `host/` | PC-side tools (receive check, recording, calibration) and the display application `ImuStudio` |
| `script/` | Script that turns the PC into a temporary access point, the backup and restore of the KV store (`kv_transfer.py`, for flashing another firmware and the like), and the writing of a UF2 shared by the three builds' `flash.sh` (`flash_uf2.sh`) |

## The three builds and `common/`

The same firmware comes in three builds: Zephyr (`zephyr/`), NuttX (`nuttx/`) and Pico SDK + FreeRTOS
(`freertos/`). The sources of acquisition, transport, time synchronization, calibration, the key-value store
and the drivers are the same files in `common/` (the three builds use the same NINA core; only the Zephyr
build has its own I2C0 and DMA driver). The Zephyr build compiles them on Zephyr; the NuttX and
FreeRTOS builds compile them on a compatibility layer that reproduces Zephyr's API under Zephyr's names
(`common/compat/` and each build's `drivers/compat/`). The key-value store has the same place and format in
the three builds, so it stays when you switch builds. Section 12 of the [design](design/imu-raw-mqtt-design.md)
describes the layout of `common/` and the boundaries to each OS.

## NuttX build

A port of the same firmware to NuttX 13.1.0 (the NuttX build) is in [`nuttx/`](nuttx/README.md).
Acquisition, MQTT delivery, time synchronization, the KV store, calibration, Wi-Fi and the Telnet
shell behave as in the Zephyr build, and the commands of this project (`imu`, `kv`, `ntp`, `nina`,
`dmesg`, `i2c_dma`, `sensor`, `wifi cred`) have the same syntax and output. The other commands are
replaced by the standard NuttX ones (`wapi`, `ifconfig`, `iperf`, `ps`, `reboot` and so on).

- See [`nuttx/README.md`](nuttx/README.md) for setup, building, flashing and usage. It also has the
  table of Zephyr commands and their NuttX counterparts, the table of error numbers (errno), and
  the differences from the Zephyr build.
- The PC tools (`imu-host` and `ImuStudio` in `host/`) work with either firmware ("Board firmware"
  in [`host/README.md`](host/README.md)).
- If the key-value area holds other data (the NuttX build's earlier format, for example), the NuttX
  build does not use it; erase it with `kv wipe confirm` and store the values again ("Moving the values from the Zephyr build" in
  `nuttx/README.md`).
- Section 11 of the [design](design/imu-raw-mqtt-design.md) describes the design differences.

## FreeRTOS build

The same firmware built on the Raspberry Pi Pico SDK 2.3.1 and the FreeRTOS kernel V11.3.3 (the FreeRTOS
build) is in [`freertos/`](freertos/README.md). Acquisition, MQTT delivery, time synchronization, the KV
store, calibration, Wi-Fi and the Telnet shell behave as in the Zephyr build, and the commands of this
project (`imu`, `kv`, `ntp`, `nina`, `dmesg`, `i2c_dma`, `sensor`, `wifi cred`), `zperf` (TCP only) and
`wifi connect`, `disconnect`, `scan` and `status` have the same syntax and output. The others have short
names (`version`, `reboot`, `ps`, `uptime`, `ifconfig`, `date`); there is no `kernel ...` or `net ...`.

- See [`freertos/README.md`](freertos/README.md) for setup, building, flashing and the commands, and
  [`freertos/drivers/README.md`](freertos/drivers/README.md) for the internals and the table of differences
  from the Zephyr and NuttX builds.
- `imu-host` and `script/kv_transfer.py` on the PC recognize the FreeRTOS build (by the USB name and the
  prompt `freertos:~$ `; "Board firmware" in [`host/README.md`](host/README.md)).
- Section 13 of the [design](design/imu-raw-mqtt-design.md) describes the design differences.

## Performance of the three builds

Measured on the same board, PC, access point and receiver (`imu-host check`) on the same day (2026-11-18),
flashing one firmware after the other. The values while sending are from a 6667 Hz measurement sent over MQTT,
with `imu start` from the USB shell and `imu stop` 60 to 70 seconds later. No run had missing batches. The TCP
rates were taken with IMU streaming stopped, for about 10 seconds, the board sending 1024 bytes at a time.
The NINA ran the firmware with patches 0001 to 0006.

| Item | Zephyr build | NuttX build | FreeRTOS build |
|---|---|---|---|
| `cpu_active` while sending | 49.4% and 49.5% (2 runs) | 50.8% and 50.8% (2 runs) | 40.3% and 40.4% (2 runs) |
| `cpu_active` at rest | not measured | not measured | 0.7% (30 seconds, 3 runs) |
| TCP send (board to PC) | 3.36 and 3.47 Mbit/s (`zperf`) | 3.34 Mbit/s (`iperf`, 1 run) | 3.47 to 3.84 Mbit/s (`zperf`, 3 runs) |
| TCP receive (PC to board) | 1.89 and 1.99 Mbit/s (`zperf`) | 1.22 Mbit/s (`iperf`, 1 run) | 2.41 to 2.52 Mbit/s (`zperf`, 3 runs) |
| Image size (`.bin`) | 370,144 bytes | 284,900 bytes | 231,872 bytes |

- The TCP rates come from different commands (`zperf` in the Zephyr and FreeRTOS builds speaks the iperf 3
  protocol, `iperf` in the NuttX build the iperf 2 protocol), so they are a rough comparison.
- `cpu_active` moves by about one point when only the layout of the image changes, with the same functions.
- The FreeRTOS build's `cpu_active` includes the time of the tick interrupt every 1 ms (the Zephyr and NuttX
  builds have no such interrupt).
- With the images of 2026-11-14 (boot2 at clock divider 4, the C library's `memcpy`, `memset` and division,
  the NINA firmware with patches 0001 and 0002), the `cpu_active` while sending was 62.6 to 63.3% in the
  Zephyr build and 67.7 to 67.9% in the NuttX build.
- `sensor stream` stopped with `Read failed` at higher output data rates in both the Zephyr and the NuttX
  build: at 833 Hz and above in the Zephyr build, at 3333 Hz and above in the NuttX build (the images of
  2026-11-14, watermark 128, USB console, one run each).
