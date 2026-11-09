# rp2040-connect

English | [日本語](README_ja.md)

A Zephyr workspace for the Arduino Nano RP2040 Connect. Multiple applications share the Zephyr version and modules specified in `manifest/west.yml`. Add modules as needed by your applications.

See the [IMU FIFO driver plan](design/imu-fifo-driver-plan.md) for the design and implementation goals.

## Setup

The following instructions are for Ubuntu 24.04. First, install the packages required for building:

```bash
sudo apt update
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
  wget python3-dev python3-venv python3-tk xz-utils file make gcc \
  gcc-multilib g++-multilib libsdl2-dev libmagic1
```

On AArch64, omit `gcc-multilib` and `g++-multilib`. For other operating systems, see the [official Zephyr instructions](https://docs.zephyrproject.org/latest/develop/getting_started/).

From the root of this repository, source the setup script:

```bash
source script/setup.sh
```

The script prepares `.venv`, the west workspace, modules, Python packages, the Zephyr SDK, and the Arm toolchain as needed. It also sets `ZEPHYR_SDK_INSTALL_DIR` and a `west` alias in the current shell. You do not need to activate the virtual environment. Run `source script/setup.sh` again in each new terminal.

The script checks for west projects, key Python packages, the SDK, and the Arm compiler to determine what is already installed. If a new application requires additional modules, update `manifest/west.yml` and run:

```bash
west update
west packages pip --install
```

## Building applications

`app/blink` is a sample that toggles the board's `led0` every 500 ms. Build it with:

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

Connect the board over USB, enter bootloader mode, and flash it using the UF2 runner.

If double-pressing RESET does not enter bootloader mode, connect `REC` to `GND` with a jumper while pressing RESET. Remove the jumper when the USB storage device appears.

```bash
west flash -d build/blink -r uf2
```

## USB serial console

`app/usb-console` provides a Zephyr shell over USB CDC ACM. Run `west update` the first time or after updating the manifest, then build and flash:

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
west flash -d build/usb-console -r uf2
```

After flashing, connect with a USB serial terminal. The device name depends on your system; for example, if it is `/dev/ttyACM0`:

```bash
minicom -D /dev/ttyACM0
```

Press Enter to display the shell. Use `help` to list commands and `hello` to display a greeting. Logs are stored in a 4 KiB RAM ring buffer and can be displayed with `dmesg`; old entries are overwritten when the buffer fills. The buffer is stored in RAM that is not initialized at startup so logs can survive a normal reset. They are lost on power failure or a reset that initializes RAM, and unprocessed logs immediately before a reset may not be saved. Timestamps are microseconds since boot and restart from zero after a reset.

Install minicom with `sudo apt install minicom`; exit it with `Ctrl-A` followed by `X`.

The shell enables GPIO, I²C, sensor, reboot, and logging commands. Use `gpio --help`, `i2c --help`, `sensor --help`, `kernel reboot --help`, and `log --help` for details. Use `log` to inspect or change log levels and `dmesg` to display logs stored in RAM.

Run `kernel ps` to display thread CPU and stack usage.

## Wi-Fi connection (under development)

An experimental SPI adapter connects the NINA-W102 running Arduino firmware to Zephyr's
Wi-Fi management API and IPv4 sockets. WPA2-PSK association and DHCP address
assignment with the standard shell have been verified on hardware with NINA
firmware `1.4.8`:

```text
wifi connect --ssid "SSID" --key-mgmt 1 --passphrase "passphrase"
wifi scan
wifi status
net iface
ntp status
wifi disconnect
```

The implementation supports open and WPA2-PSK on 2.4 GHz; open networks are untested. The passphrase is echoed;
Zephyr shell history is disabled but terminal-side recording may retain it.

`wifi scan` lists up to 10 nearby 2.4 GHz access points. NINA firmware stops Wi-Fi
during the scan, so an existing connection is interrupted and reconnection using the
saved profile is attempted afterward. Filtering by SSID or channel and other scan
options are unsupported. Hardware checks confirmed the scan results and automatic
reconnection with IP address and gateway restoration.

After Wi-Fi connects, Zephyr SNTP obtains time from `time.cloudflare.com` and sets the
system clock. `ntp status` shows the sync state and Unix time. Configure the server and
port with `CONFIG_NINA_NTP_SERVER` and `CONFIG_NINA_NTP_PORT`. Failed attempts are
retried no more often than once per 60 seconds; a successful sync is refreshed after
six hours. The application time-ready state expires after 24 hours without a sync.
NINA cannot use UDP while a TCP socket is active, so time synchronization waits for
a later retry in that case.

SNTP replies are unauthenticated; obtaining a time does not establish the server's
authenticity, and TLS security also depends on a trustworthy time source. Automatic
synchronization with the public server and `ntp status` were checked on hardware.

Key-value storage uses Zephyr Settings/NVS. Wi-Fi credentials use Zephyr's
credentials API with the same Settings/NVS backend:

```text
kv set greeting hello
kv get greeting
kv sethex bytes 0001ff
kv get bytes
kv delete greeting
```

`get` prints hexadecimal bytes. `set` stores text without a terminating NUL;
`sethex` stores hexadecimal byte pairs. Keys contain 1–32 ASCII letters,
digits, `_`, `-` or `.`, and values contain 1–128 bytes. Empty values are not
supported. Keys beginning with `sys.` are reserved for internal use and cannot
be accessed through the shell. Applications can call `kv_store_set`, `kv_store_get` and
`kv_store_delete` from `kv_store.h`; a small read buffer returns `-ENOBUFS`.
The 8 KiB storage area is shared with the Wi-Fi profile and can fill up. Flash
writes can affect acquisition. Values are unencrypted and readable through the
shell; a full flash erase or other firmware can erase them.

Manage Wi-Fi credentials using the standard shell commands. `wifi connect` only
connects; it does not automatically save the profile:

```text
wifi cred add -s "SSID" -k 1 -p "passphrase"
wifi cred list
wifi cred auto_connect
wifi cred delete "SSID"
```

`-k 1` selects WPA2-PSK. Adding the same SSID updates it. `wifi cred list` prints
passphrases, so take care with terminal output and logs. Stored credentials are
retried at boot and after an unexpected disconnect; explicit `wifi disconnect`
pauses retries until reboot or a manual connection. The current Zephyr stored
connection implementation tries the first accepted request, so failover across
multiple stored networks is not guaranteed. Old application profiles are deleted
only after they are saved as standard credentials; failed migration is retried at
the next boot.

`wifi status` retrieves the connection state, SSID, BSSID and RSSI. Security type,
channel and link mode are not yet available from this driver.

The passphrase is stored unencrypted and can be recovered by anyone with access to the
board's flash. The final 8 KiB of flash is reserved separately from this application's
code. Other firmware or a full erase may erase the profile. Flash writes can affect IMU
acquisition, so wait for Wi-Fi connection to complete before starting a measurement.

On hardware, a simple PC-side MQTT test receiver accepted CONNECT, sent CONNACK, and
received a QoS 0 payload. A full MQTT broker, long-running communication, error
handling and disconnection remain untested.
DNS lookups through Zephyr `getaddrinfo` are offloaded to NINA; numeric addresses and
`example.com` were checked on hardware. IPv4 TCP clients and UDP are supported; TLS is
unsupported. Up to four concurrent TCP sockets are supported; sending,
receiving, and polling multiple sockets were checked on hardware. Polling mixed socket
types is unsupported, and concurrent operations during disconnect remain untested.
This build uses Zephyr's test-only non-cryptographic random generator.

UDP is limited to one socket and cannot be used concurrently with TCP or another UDP
socket; attempts return `EOPNOTSUPP`. NINA firmware 1.4.8 can stall UDP reception when
another socket has unread data. UDP sends are limited to 512 bytes, and incoming
zero-length datagrams cannot be detected. UDP connect, send, receive, truncation and
poll were checked on hardware. One consecutive-packet test timed out initially; four
later attempts succeeded, and the cause remains unknown.
Bytes beyond NINA's 1,500-byte receive buffer are discarded without notification.
UDP bind supports the wildcard address only; `sendmsg`, multicast configuration and
socket timeouts are unsupported.

TLS for MQTTS and the ATECC608A secure element are not yet used. I2C0 defaults to
100 kHz. An earlier scan did not detect the ATECC608A at 1 MHz; coexistence with the
IMU needs testing, including the device wake sequence and bus speed.

Hardware checks confirmed address and gateway removal and reconnection after an
unexpected disconnect, and that explicit `wifi disconnect` does not trigger
reconnection. A 600-byte binary TCP echo and a refused connection returning
`ECONNREFUSED` were also checked.

## Shell over Wi-Fi

With the patched NINA 3.0.2 firmware, the Zephyr Telnet shell listens on TCP
port 23. Find the board's IPv4 address with `net iface` and connect with
`telnet <address> 23`. Press Enter if the initial prompt is not shown. One
Telnet client can use the shell at a time; the USB console remains available.

Telnet provides neither authentication nor encryption and exposes the full
shell, including commands that display Wi-Fi credentials. Use it only on a
trusted network. Basic commands, repeated connections, recovery after Wi-Fi
reconnection, a short IMU measurement, and startup SNTP synchronization were
tested on hardware. One connection timed out during testing and recovered
after reboot; subsequent reconnection tests did not reproduce it. Long-running
operation and concurrent MQTT are unverified.

## Measuring the IMU FIFO

`imu_measure` uses the Sensor Streaming API to collect FIFO batches and report counts and transfer times without printing each sample. Set the sampling frequencies of the sensors to use, then specify the measurement duration. For example, to measure both sensors at 208 Hz for 10 seconds:

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 208
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 208
imu_measure start 10
```

The results are printed after the measurement stops. Use `imu_measure stop` to stop early and `imu_measure result` to show the last result again. The duration must be between 1 and 3,600 seconds. To measure one sensor, set the other sensor's `sampling_frequency` to `0`. `sensor stream` and `imu_measure` cannot run simultaneously. Use `imu_measure stats` to inspect the current FIFO counters for either kind of stream.

Use `sensor attr_get lsm6dsox all fifo_watermark` to display the FIFO watermark, or `sensor attr_set lsm6dsox all fifo_watermark 128` to set it to 128 words before starting a stream. Values 1–256 are supported; the default is 32 and a reset restores it. A larger watermark generally reduces batch overhead but increases delivery latency and reduces FIFO headroom. DMA support is complete for the IMU FIFO use case and enabled for eligible I²C0 reads in the normal build; `i2c_dma stats` displays cumulative DMA counters. The 60-second validation is sufficient for this use case; competition with other DMA users will be assessed when needed. See the [driver notes](app/usb-console/drivers/README.md) for transfer limitations and measurements.

See the [driver TODOs](app/usb-console/drivers/README.md#todo) for sensor timestamp and internal temperature support.

At the end of the requested duration or an early stop, the driver stops adding new FIFO batches and accounts for the remaining FIFO words and in-flight RTIO completions before finalizing the result. `stop_requested_ms` is the elapsed time when stopping was requested; `elapsed_ms` includes the time to retrieve remaining data and finish cleanup. Batching is stopped for each sensor in turn, so the stop boundary is not exactly simultaneous.

The result includes received accelerometer and gyroscope sample counts, unsupported-tag word counts, configured ODRs, FIFO burst counts and total/maximum transfer times, I²C errors, buffer shortages, FIFO overrun observations, dropped RTIO completions, and stream errors. `cpu_active` is the fraction of the measurement interval spent outside CPU idle, measured using the idle hooks and including interrupts. It includes measurement overhead and has 0.1% output resolution. `cpu_busy` is the scheduler’s non-idle time, including the measurement and other threads. Interrupts that occur while the idle thread is running may be charged to idle and omitted, so it is not the total physical CPU busy time. `observed_hz` is an estimate obtained by dividing received samples by `elapsed_ms`. Setup and final drain time affect it, especially for short measurements. It does not measure the FIFO hardware timestamp or the sensor's exact sample generation time.

`accounting=match` means that the driver's read word and transfer counts match the application's counted words and batches, with no reported errors preventing reconciliation. `mismatch` means the counts differ; `incomplete` means an error, dropped completion, or another condition prevented reconciliation. A match does not prove that every sample generated by the sensor was received.

An overrun observation signals a discontinuity, but does not give the number of lost samples or distinct overruns. The stream stops and reports `stream_error` if it encounters a buffer shortage, I²C error, or overrun. If `cleanup_timeout=1`, cleanup did not complete and a reboot is required before another measurement. Dropped completions or unknown tags also prevent a guarantee of data continuity.
