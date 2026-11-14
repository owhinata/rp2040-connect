# NuttX build

English | [日本語](README_ja.md)

The IMU measurement firmware of `zephyr/app/usb-console` (the Zephyr build) ported to NuttX 13.1.0 (the
NuttX build). It runs on the same board, the Arduino Nano RP2040 Connect (RP2040, NINA-W102,
LSM6DSOX): it reads the LSM6DSOX's acceleration and angular rate as RAW samples at 6667 Hz, adds the
PC's time and sends them by MQTT over Wi-Fi.

- The application's own commands (`imu`, `kv`, `ntp`, `nina`, `dmesg`, `i2c_dma`, `sensor`, `wifi cred`)
  have the same syntax and output as in the Zephyr build. Zephyr's standard commands are replaced by
  NuttX's ("Commands of the Zephyr build and of the NuttX build").
- Error numbers (errno) are shown with NuttX's values ("Error numbers (errno)").
- The PC tools (`imu-host`, `ImuStudio` in `host/`) work unchanged ("Board firmware" in
  `host/README.md`).

Further reading:

- How each part works in detail: [`doc/internals.md`](doc/internals.md)
- The drivers (NINA, I2C0, LSM6DSOX) and the Zephyr API compatibility layer:
  [`drivers/README.md`](drivers/README.md)
- The design summary: [§11](../design/imu-raw-mqtt-design.md) of the design document ("11. The NuttX port")
- Performance compared with the Zephyr build: "Performance of the Zephyr and NuttX builds" in the
  top-level [`README.md`](../README.md)
- The plan, history and verified facts of the port (work logs, in Japanese):
  [`PORTING_PLAN.md`](PORTING_PLAN.md), [`PORTING_STATUS.md`](PORTING_STATUS.md),
  [`PORTING_NOTES.md`](PORTING_NOTES.md)

## Requirements

- An Arduino Nano RP2040 Connect (with the same NINA firmware as the Zephyr build;
  `firmware/nina-fw/README.md`)
- A Linux PC (x86_64). No sudo is needed.
- `git`, `curl` or `wget`, `tar`, `xz`, `sha256sum`
- `python3` with the `venv` module (`python3-venv` on Ubuntu)
- `make`, `gcc` (NuttX builds host tools)

## Setup

Source the script from the repository root. It fetches only what is missing, under `nuttx/`, so it
can be sourced any number of times. Source it again in each new terminal.

```bash
source nuttx/script/setup.sh
```

| Location | Contents | Version |
|---|---|---|
| `nuttx/nuttx` | NuttX | tag `nuttx-13.1.0` (shallow clone) |
| `nuttx/nuttx-apps` | NuttX applications | tag `nuttx-13.1.0` (shallow clone) |
| `nuttx/pico-sdk` | Only the parts of the Raspberry Pi Pico SDK that the boot2 build reads (about 4 MB). A full checkout already there is used as it is | tag `2.2.0` (partial clone and sparse checkout) |
| `nuttx/toolchain/` | Arm GNU Toolchain (arm-none-eabi, for x86_64 Linux) | 13.2.Rel1 (SHA-256 checked) |
| `nuttx/.venv` | Python venv (kconfiglib, used by NuttX's `menuconfig` and `olddefconfig`) | kconfiglib 14.1.0 |
| `nuttx/stmemsc/` | ST's LSM6DSO register driver (`lsm6dso_reg.c`, `lsm6dso_reg.h`) | lsm6dso-pid v3.2.2 (the same content as Zephyr's hal_st; SHA-256 checked) |
| `nuttx/nuttx-apps/system/argtable3/argtable3` | argtable3 (argument parsing of `iperf`) | tag `v3.2.0.7402e6e` (shallow clone) |

It also creates the symlinks `nuttx/nuttx/external` → `nuttx/drivers` and `nuttx/nuttx-apps/external`
→ `nuttx/apps`, applies the patches below, and sets `PICO_SDK_PATH` and `PATH` (the toolchain,
`nuttx/.venv/bin`, `nuttx/script`) in the current shell. The versions are fixed at the top of the
script. The fetched files are not tracked by Git. How the applied patches are recorded and what
happens when a clone has local changes is in "What `setup.sh` does" in
[`doc/internals.md`](doc/internals.md).

### Patches to NuttX

`nuttx/patches/nuttx/` patches NuttX and `nuttx/patches/apps/` patches nuttx-apps. Each file starts
with its purpose and the Zephyr code it matches.

| Patch | Contents |
|---|---|
| `nuttx/0001-cdcacm-no-at-protocol` | Sets the CDC ACM protocol to 0, so ModemManager does not treat the board as a modem (as in the Zephyr build) |
| `nuttx/0002-cdcacm-drop-output-without-dtr` | Drops output while no program on the PC has the port open (DTR low) |
| `nuttx/0003-serial-canonical-ctrl-c-eof` | Lets Ctrl-C and Ctrl-D end a command that reads standard input |
| `nuttx/0004-rp2040-make-defs-alarm-arch-idle-custom` | Leaves out the in-tree `rp2040_timerisr.c` and `rp2040_idle.c`, because the board provides the system clock and the idle loop |
| `nuttx/0005-usrsock-dontwait-to-daemon-and-daemon-send-timeout` | Makes non-waiting usrsock sends and receives, and the send timeout, behave as the Zephyr build's NINA sockets |
| `nuttx/0006-telnet-crlf-line-end` | Ends Telnet output lines with CR LF |
| `nuttx/0007-sched-critmon-cpuload-in-perf-counts` | Accumulates the CPU column of `ps` in 1 µs units and fixes the values going wrong when a task exits |
| `nuttx/0008-sched-critmon-irqcount-only-for-csection-busywait` | Makes `SCHED_CRITMONITOR` select `IRQCOUNT` only when it is used |
| `apps/0001-readline-echo-in-readline` | readline echoes the line being typed |
| `apps/0002-readline-cr-lf-line-end` | CR, LF, CR LF and LF CR each make one Enter |
| `apps/0003-readline-ctrl-c` | Ctrl-C drops the line being typed and shows a new prompt |
| `apps/0004-readline-full-line` | Drops further characters when the line is full |
| `apps/0005-iperf-free-argtable` | Fixes `iperf` leaking heap on every run |
| `apps/0006-iperf-len-option-udp-eagain` | Adds `-l` to `iperf` and retries a UDP send that fails with `EAGAIN` |

## Build

```bash
nuttx/script/build.sh
```

This builds the board `nuttx/boards/arduino-nano-rp2040-connect/` with its only configuration,
`configs/imu/defconfig`, into `nuttx/build/nuttx.uf2`. It sources `setup.sh` itself. When the
defconfig, a Kconfig, the patches and so on change, it configures again by itself, and it checks the
image (boot2, the flash range, no overlap with the key-value area) before making the UF2 ("What
`build.sh` does" in [`doc/internals.md`](doc/internals.md)).

| Option | Effect |
|---|---|
| `--clean` | Configure again from `make distclean` |
| `--menuconfig` | Configure when needed and run `make menuconfig` (no build) |
| `--savedefconfig` | Write the result of `make savedefconfig` back to `configs/imu/defconfig` (no build) |
| `-j N` | Number of parallel make jobs (default: the number of CPUs) |
| `-v` | Verbose make output (`V=1`) |

Changes made with `--menuconfig` are lost at the next configure unless saved with `--savedefconfig`.

## Flashing

The UF2 goes in through the RP2040 USB bootloader (BOOTSEL mode, in which a USB drive named
`RPI-RP2` appears). There is no shell command that enters BOOTSEL (as in the Zephyr build), so put
the board into BOOTSEL by hand every time:

1. Connect the board to USB and press the RESET button twice quickly. If that does not work,
   connect `REC` to `GND` with a jumper wire, press RESET, and remove the wire when the drive
   appears.
2. Write the image:

```bash
nuttx/script/flash.sh            # write nuttx/build/nuttx.uf2
nuttx/script/flash.sh --build    # build first, then write
```

`flash.sh` checks that the UF2 is for the RP2040 and looks for the `RPI-RP2` drive of the RP2040 boot
ROM. Without one, it explains how to enter BOOTSEL by hand and waits for the drive. If the drive is
not mounted it mounts it with `udisksctl` (no sudo), checks `INFO_UF2.TXT`, copies the UF2, waits for
the NuttX USB console and prints its path. It stops if there is more than one drive.

| Argument / option | Effect |
|---|---|
| `UF2` | The UF2 to write (default: `nuttx/build/nuttx.uf2`) |
| `--build` | Run `nuttx/script/build.sh` first |
| `--wait SEC` | How long to wait for the drive, and then for the console (default: 60) |
| `--dry-run` | Look for the drive and the console (`lsblk` and `/dev/serial/by-id/` only) and print what would be done |

## Moving the values from the Zephyr build

The key-value store's format is not compatible with the Zephyr build's, and the NuttX build erases the
Zephyr build's key-value area at its first boot (`<wrn> fs_nvs: Erasing the key-value area: not in
this build's format` in `dmesg`). Back the values up while the Zephyr build runs and write them back
after flashing the NuttX build:

```bash
# Back up while the Zephyr build runs (host/recordings/kv-backup.json, not in Git)
host/.venv/bin/python script/kv_transfer.py backup --usb
# Write them back after flashing the NuttX build, and register Wi-Fi (the password is typed at the
# terminal; the SSID defaults to rp2040-test, --ssid for another access point)
host/.venv/bin/python script/kv_transfer.py restore --usb
# Without registering Wi-Fi
host/.venv/bin/python script/kv_transfer.py restore --usb --no-wifi
```

`restore` writes with `kv sethex`, reads back with `kv get`, and sends `wifi cred add`,
`wifi cred auto_connect` and `imu cal reload`. It does not write `imu.session` back (the NuttX build
numbers sessions from 1). If two credentials are stored already, `wifi cred add` fails with
`err: -105`; remove one with `wifi cred delete` first.

## Usage

### Connecting to the shell

The USB port is `/dev/serial/by-id/usb-NuttX_NuttX_USB_Console_<chip ID>-if00` (the chip ID is the
Zephyr build's USB serial number).

```bash
picocom /dev/serial/by-id/usb-NuttX_NuttX_USB_Console_*-if00
```

- The shell is nsh, with the prompt `nsh$ `. Nothing is shown right after opening the port; press
  Enter (or Ctrl-C) for the prompt.
- `help` lists the commands and `help <command>` shows a command's usage. The up and down arrows
  recall the last 8 lines, Tab completes command names, and Ctrl-C drops the line being typed.
- nsh parses the arguments, so quote an argument that contains `$`, `;`, `#`, `>`, `&` or quotes.
- The application's commands write errors to standard error (the same terminal) and exit with 0 on
  success and 1 on failure (`echo $?`).
- `reboot` restarts the board (the Zephyr build's `kernel reboot`; it ignores its arguments).
- On a fatal error (an exception, `PANIC()`, a failed assertion, the stack sentinel) the board writes
  a report to the RAM log and restarts by itself (`CONFIG_BOARD_RESET_ON_ASSERT=2`). Read it with
  `dmesg` after the restart.

### Wi-Fi

With stored credentials the board connects at boot and after a disconnection (`wifi cred`, as in the
Zephyr build; up to 2 entries).

```text
nsh$ wifi cred add -s <SSID> -k 1 -p <passphrase>
nsh$ wifi cred auto_connect
nsh$ ifconfig wlan0
wlan0	Link encap:Ethernet HWaddr <MAC> at RUNNING mtu 1500
	inet addr:10.42.0.97 DRaddr:10.42.0.1 Mask:255.255.255.0
```

- `wifi cred add` prints nothing on success. `wifi cred list` lists the entries (passphrases
  included, as in the Zephyr build) and `wifi cred delete <SSID>` removes one. Only open (`-k 0`) and
  WPA2-PSK (`-k 1`) networks can be joined.
- Once connected, `ifconfig wlan0` shows `RUNNING` and the address from the NINA's DHCP.
  `wapi show wlan0` shows the ESSID, BSSID and signal strength (Sense).
- One-off connect, disconnect and scan use NuttX's `wapi` ("Commands of the Zephyr build and of the
  NuttX build"). `wapi` prints nothing on failure; check `echo $?`. After `wapi disconnect wlan0`
  the board does not reconnect by itself until the next successful connection (as in the Zephyr
  build).

### Telnet

While Wi-Fi is up, the same shell is available on TCP port 23, one client at a time. It has no
authentication or encryption; use it only on a trusted network.

```bash
telnet <board IPv4 address>
```

The prompt arrives right after connecting. A second connection is accepted and closed at once
(`<err> shell_telnet: Telnet client already connected.`).

### Measuring

The settings are read from the same keys as in the Zephyr build (`imu.host`, `imu.port`,
`imu.device`, `imu.odr`, `imu.watermark`). Without `imu.host` the data goes to the default gateway.

```text
nsh$ kv get imu.port
3138383330
nsh$ imu start
Streaming 6667 Hz to the default gateway, port 18830, as imu/rp2040/raw
nsh$ imu stop
nsh$ imu status
state: stopped
  ...
errors:
  stream:                 0
  ...
```

- `imu status` has the same items in the same order as in the Zephyr build (`errors`, `fifo`,
  `time`, `mqtt`). Prepare the receiver and the NTP server on the PC as in "Usage" of the top-level
  README (to check the reception: `host/.venv/bin/imu-host check --port <port>`).
- `kv set <key> <string>`, `kv sethex <key> <hex>`, `kv get <key>` and `kv delete <key>` have the same
  syntax and output as in the Zephyr build (`kv get` prints hexadecimal).
- `sensor` (`sensor get lsm6dsox accel_xyz` and so on) is the Zephyr build's sensor shell. It refuses
  with `-16` while a measurement runs.

### Time synchronization

```text
nsh$ ntp status
state: synchronized
  server:                     10.42.0.1:123 (default gateway)
  ...
```

NTP queries start when Wi-Fi connects (the default gateway by default, every 2 s). The output is the
same as in the Zephyr build. nsh's `date` shows the realtime clock (UTC).

### Calibration

```text
nsh$ imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
  gyro bias: 7.29 -3.24 -0.36 LSB (stored, 0 rejected)
```

`imu cal reload` (read the key-value store again) and `imu cal zero` (measure the gyro zero at rest
again, kept in RAM only) are as in the Zephyr build. The calibration applies to the physical values
of `sensor`, not to the RAW data sent by MQTT.

### Other commands

```text
nsh$ nina version
NINA firmware: 3.0.2
nsh$ i2c_dma stats
dma_transfers=... dma_bytes=... dma_errors=0 dma_fallbacks=0
nsh$ i2c get -a 6a -r 0f
READ Bus: 0 Addr: 6a Subaddr: 0f Value: 6c
nsh$ irqstack
0x20002f88 IRQ 00                           (real size 2048):	unused 1684	usage  364 / 2048 (17 %)
```

- `dmesg` prints the RAM log (8 KiB, kept across `reboot`). The lines have the Zephyr build's log
  format.
- `ps` shows each thread's priority, stack (STACK, USED, FILLED) and CPU share. The CPU column is a
  moving average weighted to the last 1 to 2 seconds, in 0.1 % steps (the Zephyr build's `kernel ps`
  shows the total since boot). The total run time since boot is the second value of
  `cat /proc/<tid>/critmon` (seconds).
- `irqstack` prints the peak use of the interrupt stack since boot (in the format of the `IRQ 00`
  line of the Zephyr build's `kernel thread stacks`).
- Throughput is measured with `iperf` (iperf 2, port 5001). Use `iperf` on the PC, not `iperf3`.

```bash
# From the board to the PC (iperf -s on the PC)
iperf -c <PC> -t 30 -l 1024
# From the PC to the board (on the board; iperf -c <board> -t 30 on the PC)
iperf -s -B 0.0.0.0
```

Give a server `-B 0.0.0.0` (the NINA does not accept a bind to a specific address). Keep UDP
datagrams at 512 bytes or less, as in `-u -l 500`.

## Commands of the Zephyr build and of the NuttX build

| Zephyr build | NuttX build | Differences |
|---|---|---|
| `imu start` / `stop` / `status` | Same | |
| `imu cal` / `imu cal reload` / `imu cal zero` | Same | |
| `ntp status` | Same | |
| `kv set` / `sethex` / `get` / `delete` | Same | The storage format is not compatible |
| `nina version` | Same | |
| `sensor get` / `attr_set` / `attr_get` / `stream` / `trig` | Same | Neither has `sensor info` |
| `i2c_dma mode` / `stats` | Same | |
| `dmesg` | Same | |
| `wifi cred add` / `delete` / `list` / `auto_connect` | Same | |
| `wifi connect -s <SSID> -p <passphrase> -k 1` | `wapi psk wlan0 <passphrase> 3`, then `wapi essid wlan0 <SSID> 1` | |
| `wifi connect -s <SSID> -k 0` | `wapi essid wlan0 <SSID> 1` | |
| `wifi disconnect` | `wapi disconnect wlan0` | |
| `wifi scan` | `wapi scan wlan0` | While connected, it counts as a disconnection and asks for a reconnection afterwards (as in the Zephyr build) |
| `wifi status` | `wapi show wlan0` and `ifconfig wlan0` | |
| `net iface` | `ifconfig` | |
| `zperf tcp upload <PC> 5201 30 1024` | `iperf -c <PC> -t 30 -l 1024` (`iperf -s` on the PC) | iperf 2, port 5001 |
| `zperf udp upload <PC> 5201 30 500 500K` | `iperf -c <PC> -u -t 30 -l 500` (`iperf -s -u` on the PC) | The rate cannot be set |
| `zperf tcp download 5201` / `download stop` | `iperf -s -B 0.0.0.0` | Ends after one connection |
| `kernel reboot` | `reboot` | Both ignore their arguments |
| `kernel ps` | `ps` | The CPU column means something else ("Other commands") |
| `kernel thread list` (`Total execution cycles`) | `cat /proc/<tid>/critmon` | The second value is the total run time since boot (seconds) |
| `kernel thread stacks` | `ps` and `irqstack` | |
| `kernel version` | `uname -a` | |
| `kernel uptime` | `uptime` | |
| `date get` | `date` | For example `Sat, Nov 14 03:23:03 2026` |
| `date set 2026-11-14 03:23:03` | `date -s "Nov 14 03:23:03 2026"` | |
| `i2c scan i2c@40044000` | `i2c dev 03 77` | |
| `i2c read_byte i2c@40044000 6a 0f` | `i2c get -a 6a -r 0f` | |
| `i2c read i2c@40044000 6a 10 10` | `i2c dump -a 6a -r 10 16` | The NuttX build's byte count is decimal |
| `i2c direct_read i2c@40044000 6a 1` | `i2c get -a 6a` | |
| `i2c write_byte i2c@40044000 6a 07 5a` | `i2c set -a 6a -r 07 5a` | |
| `i2c speed i2c@40044000 3` | `-f 1000000` | Only a setting of the tool in the NuttX build |
| `i2c recover i2c@40044000` | `i2c reset` | |
| `devmem <address>` (read) | `xd <hex address> <bytes>` | No write |
| `retval` | `echo $?` | |
| `rem <text>` | `# <text>` | |
| `help` | `help` | |
| – | `irqstack` | Use of the interrupt stack |

The Zephyr build's `i2c write`, `gpio`, `log`, `device`, `history`, `clear`, `resize`, `shell`, and the
subcommands of `wifi`, `net` and `kernel` not listed above do not exist in the NuttX build. The PC
tools (`imu-host`, `ImuStudio`, `script/kv_transfer.py`) only send commands from the "Same" rows.

## Error numbers (errno)

Error numbers in command output, the log, `imu status`, `ntp status` and the MQTT status are NuttX's
values (not converted to the Zephyr build's). The ones whose values differ from the Zephyr build's
(picolibc):

| Name | Zephyr build | NuttX build | Where it shows up in this firmware, for example |
|---|---|---|---|
| `ENOMSG` | 35 | 42 | |
| `ENOSYS` | 88 | 38 | `sensor trig` (`-38`) |
| `EBADMSG` | 77 | 74 | |
| `EOVERFLOW` | 139 | 75 | FIFO overrun |
| `ENOTSOCK` | 108 | 88 | |
| `EMSGSIZE` | 122 | 90 | A UDP send over 512 bytes (`err=90` from `iperf`) |
| `ENOPROTOOPT` | 109 | 92 | A socket option the NINA sockets do not take |
| `EPROTONOSUPPORT` | 123 | 93 | |
| `ENOTSUP` | 134 | 138 | An attribute `sensor` does not take, a cipher `wapi` does not take |
| `EAFNOSUPPORT` | 106 | 97 | |
| `EADDRINUSE` | 112 | 98 | |
| `EADDRNOTAVAIL` | 125 | 99 | `iperf -s` without `-B 0.0.0.0` (`error code: 99`) |
| `ENETDOWN` | 115 | 100 | |
| `ENETUNREACH` | 114 | 101 | |
| `ECONNABORTED` | 113 | 103 | |
| `EISCONN` | 127 | 106 | |
| `ENOTCONN` | 128 | 107 | |
| `ETIMEDOUT` | 116 | 110 | Wi-Fi connection (about 30 s), TCP connect (10 s), NINA replies, I2C transfers |
| `EHOSTUNREACH` | 118 | 113 | |
| `EALREADY` | 120 | 114 | A Wi-Fi connection request |
| `EINPROGRESS` | 119 | 115 | A non-blocking connect |
| `ECANCELED` | 140 | 125 | The end value of a stopped FIFO stream |

Main ones with the same value: `ENOENT` 2 (a missing key), `EIO` 5 (an I2C failure), `EAGAIN` 11,
`ENOMEM` 12, `EBUSY` 16 (`sensor` during a stream), `ENODEV` 19, `EINVAL` 22, `EMFILE` 24 (the 11th
NINA socket), `ENOSPC` 28 (the key-value store is full), `EPROTO` 71, `ENOBUFS` 105 (a third
credential).

## Differences from the Zephyr build

- **Shell**: nsh. The prompt is `nsh$ ` and an unknown command gives `nsh: <name>: command not
  found`. Tab completes command names only; the history has 8 lines. The prompt is not redrawn
  around the output of `sensor stream`.
- **USB console**: the product string is `NuttX USB Console`, the device class 02/00/00 without an
  IAD. Output is dropped while the port is closed, so the prompt only shows after an Enter.
- **Telnet**: the prompt is sent right after connecting and input is echoed. Ctrl-C does not end a
  command that reads standard input.
- **Commands**: Zephyr's standard commands become NuttX's (the table above). `wapi` does not print
  the error value. Throughput is measured with iperf 2.
- **Error numbers**: NuttX's values (the table above).
- **Key-value store and credentials**: the storage format is not compatible. `imu.session` is not
  carried over.
- **Log**: formatted in the calling thread (the Zephyr build defers it). The RAM ring is 8 KiB (4 KiB
  in the Zephyr build). No run-time level change.
- **Clocks**: the kernel clock, `k_uptime_get()`, `k_cycle_get_*()`, the sensor clock and the CPU
  load are timed by the RP2040 TIMER (1 MHz from the crystal). The Zephyr build uses the SysTick of
  the CPU clock, which falls behind while sending. Both use a 100 µs tickless tick.
- **Threads**: priority is `118 − Zephyr`. Zephyr's cooperative threads (`sysworkq`, `net_mgmt`) are
  ordinary threads at 119. `nina_event`, which turns the NINA socket states into events, exists only
  in the NuttX build. All interrupts have the same priority.
- **Sockets**: the NINA sockets are NuttX usrsock sockets. The MQTT and NTP sockets are polled by the
  calling thread; the others (Telnet, `iperf`) by `nina_event` every 20 ms.
- **Clocks and pins**: the PLL_USB VCO runs at 480 MHz (both outputs are 48 MHz), the vector table is
  in SRAM, GPIO4, 6 and 7 are not given to SPI0, the ROSC setup is left alone, the inputs of GPIO26
  to 29 are disabled, and the clock setup resets SYSCFG and USBCTRL.

## Threads and priorities

Priorities map as `NuttX = 118 − Zephyr` (`NANO_PRIO_*` and `NANO_STACK_*` in
`nuttx/drivers/include/priorities.h`).

| Thread | Zephyr | NuttX | Stack |
|---|---|---|---|
| `wifi_reconnect_tid`, `nina_scan_tid`, `nina_connect_tid` | 8 | 110 | 2048, 1536, 1024 |
| `time_sync_tid`, `imu_stream_tid`, `nina_monitor_tid` | 9 | 109 | 2048, 2048, 1536 |
| `imu_fifo_worker` | 10 | 108 | 2048 |
| `imu_raw_mqtt_tid` | 11 | 107 | 2048 |
| `nina_event` (NuttX build only) | – | 106 | 1536 |
| `sysworkq` | −1 (cooperative) | 119 | 1024 |
| `net_mgmt` | −1 (cooperative) | 119 | 2048 |
| `RTIO WQ 0` (`RTIO_WQ_0` in `ps`) | 0 | 118 | 1024 |
| `sensor_shell_processing_tid` | 0 | 118 | 3072 |
| `main` (`init` in NuttX; lowered to 104 after start-up) | 0 | 118 | 2048 |
| The shell (`nsh` on USB, `nsh_telnet` on Telnet) and its commands | 14 | 104 | 3072 |
| `telnetd` (`net_socket_service` in the Zephyr build) | 15 → 14 | 104 | 2048 |
| `iperf_traffic`, `iperf_report` | 15 → 14 | 100, 101 | 4096, 4096 |

The interrupt stack is 2048 bytes (`irqstack`). The STACK column of `ps` is smaller by the area that
NuttX puts at the start of a stack (2048 → 2008 and so on).

## Tests

```bash
nuttx/script/host_tests.sh
```

This builds and runs the 38 host tests of the OS-independent parts on the PC. It needs no board, only
the host's `cc` (C17). Without ST's `lsm6dso_reg.c` (fetched by `setup.sh`), the 3 tests that use it
(`lsm6dso_driver`, `imu_cal`, `sensor_shell`) print `skip`. It stops at the first failure. What each
test checks, and the list of files copied unchanged from the Zephyr build (to be replaced when the
Zephyr build changes), are in "Host test details" in [`doc/internals.md`](doc/internals.md).

## Directory layout

| Location | Contents |
|---|---|
| `boards/arduino-nano-rp2040-connect/` | Board definition (`configs/imu/defconfig`, `include/board.h`, linker script, and in `src/` the bring-up, clocks, I2C0, DMA, key-value flash and so on) |
| `drivers/` | Kernel-side code: NINA (`nina/`), LSM6DSOX (`lsm6dso/`), RAM log (`log/`), Zephyr API compatibility layer (`compat/`), shared headers (`include/`) |
| `apps/` | Applications: `init` (start-up and Telnet), `imu` (acquisition, MQTT, time synchronization, calibration, key-value store), `kv`, `nina`, `ntp`, `wifi`, `sensor`, `i2c_dma`, `dmesg`, `irqstack`, `subcmd` (shared subcommand handling) |
| `patches/` | Patches to NuttX and nuttx-apps |
| `script/` | `setup.sh`, `build.sh`, `flash.sh`, `host_tests.sh`, `bin2uf2.py`, `kconfig-tweak` |
| `tests/` | Host tests |
| `doc/` | Internals (`internals.md`, `internals_ja.md`) |
| `nuttx/`, `nuttx-apps/`, `pico-sdk/`, `toolchain/`, `stmemsc/`, `.venv/`, `build/` | What `setup.sh` fetches and the build products (not in Git) |
