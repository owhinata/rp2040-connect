# rp2040-connect

English | [日本語](README_ja.md)

A Zephyr workspace for the Arduino Nano RP2040 Connect (RP2040, NINA-W102, LSM6DSOX).
The main application, `app/usb-console`, reads accelerometer and gyroscope data at 6667 Hz as
raw values, adds the PC's time, and sends it over Wi-Fi with MQTT.

- The IMU FIFO is read over I2C (with DMA) and batched with numbers that make loss detectable.
- Acquisition continues when Wi-Fi drops; what cannot be sent is discarded and counted.
- The board synchronizes with an NTP server on the PC and puts the PC's time (Unix time in
  microseconds) on every batch.
- It is operated from a shell over USB or Telnet.

See the [IMU RAW and MQTT design](design/imu-raw-mqtt-design.md) for the details and
[app/usb-console/drivers](app/usb-console/drivers/README.md) for the drivers (LSM6DSO, I2C,
NINA).

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

Source the setup script from the repository root.

```bash
source script/setup.sh
```

It prepares `.venv`, the west workspace (Zephyr v4.5.0 and modules), Python packages and the
Zephyr SDK as needed, and sets `ZEPHYR_SDK_INSTALL_DIR` and a `west` alias in the current shell.
Source it again in every new terminal.

## Build and flash

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
west flash -d build/usb-console -r uf2
```

There is one configuration, `app/usb-console/prj.conf`.

Before flashing, connect the board over USB and put it into bootloader mode, in which a USB
drive named `RPI-RP2` appears. If pressing RESET twice does not do it, press RESET while `REC`
and `GND` are joined with a jumper wire, and remove the wire once the drive appears.

`app/blink` is a minimal sample that toggles `led0` every 500 ms.

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

The NINA-W102 is expected to run Arduino NINA firmware 3.0.2 with two fixes applied. See
[firmware/nina-fw](firmware/nina-fw/README.md) for the build and cautions. `nina version` shows
the version on the board.

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
- `wifi scan` interrupts the connection and reconnects with the stored settings afterwards.

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
and for recording (`record`) and calibration (`calib`).

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
| `ntp status` | State of the synchronization with PC time |
| `wifi ...`, `net ...` | Wi-Fi and IP (Zephyr) |
| `nina version` | NINA firmware version |
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

Single 30 s runs on hardware gave 2.93 Mbit/s for TCP sending, 1.54 Mbit/s for TCP receiving
and no loss for UDP sending at the requested 500 kbit/s. These were taken with IMU streaming
stopped and depend on the radio environment.

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

## Tests

```bash
./script/host_tests.sh
```

This runs, on the PC and without a board, the tests for FIFO word parsing, batches and
payloads, the send queue, time assignment, the NTP estimate, the driver's FIFO setup and stop
handling, and NINA SPI framing.

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
| `app/usb-console/src/` | The application: measurement, MQTT, time sync, key-value store, Wi-Fi settings |
| `app/usb-console/drivers/` | Local drivers for the LSM6DSO, I2C, NINA and the sensor shell |
| `app/usb-console/tests/` | Host tests |
| `app/blink/` | LED blink sample |
| `design/` | Design and investigation notes |
| `firmware/nina-fw/` | Fixes and build for the NINA firmware |
| `host/` | PC-side tools (receive check, recording, calibration) |
| `script/` | Setup, access point, host tests |
| `manifest/west.yml` | Versions of Zephyr and its modules |
