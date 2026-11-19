# Patched NINA firmware

This directory contains patches for Arduino NINA firmware 3.0.2. The firmware
source and build output are kept in the ignored `source/` directory.

The source is pinned to the upstream 3.0.2 commit
`f9491329a818bc7f36cdda465b4bea53b03f4bda`. The upstream build requires Git,
Docker, network access, and the `espressif/idf:v4.4.8` image. Run from the
repository root:

```sh
firmware/nina-fw/build.sh
```

The build targets Arduino Nano RP2040 Connect and generates
`firmware/nina-fw/source/NINA_W102.bin` (to be written at 0x0; it ends before
the `storage` and `nvs` partitions). It does not flash either processor.
On first use the script clones the pinned source and applies the patches; a
checkout that has only the first patches gets the others. Any other change
in the checkout stops the build. Move the source directory aside for a fresh
build. Docker pulls `espressif/idf:v4.4.8` automatically if it is not present
locally; the build container itself has networking disabled. The image is not
byte-for-byte reproducible (the build embeds its time).

`tests/run.sh` builds and runs the host test of the ESP-IDF-free part of
0006 (`main/SocketEvents.c`) with the host's `cc`.

The RP2040 driver uses the firmware's BSD-style socket commands (0x70 to
0x7f), including the fixed socket poll, for every socket. It asks 0x6A once
at start-up and uses what 0003 to 0006 add; with an older firmware (3.0.2,
or 0001 and 0002 only) it works without those additions. Images built earlier and backups
can be kept in the ignored `images/` directory.

## Reading and writing the module's flash

The three RP2040 builds (Zephyr, NuttX, FreeRTOS) can relay their USB serial
port to the module's UART for esptool: `nina passthrough confirm` restarts
the board into that mode (RTS drives the module's RESETN, DTR its GPIO0, and
the UART follows the port's rate). Neither a separate passthrough image nor
the Arduino SerialNINAPassthrough sketch is needed, and the RP2040's firmware
and key-value store stay as they are. To leave the mode, set the port to
300 bps and close it (`stty -F <port> 300`), or press RESET; the board then
starts its normal firmware.

esptool 5.3.0 gets no answer with `--before no-reset`. Give it a
configuration file whose reset sequence leaves RTS set, and use
`--before default-reset --after no-reset`:

```sh
cat > esptool.cfg <<'EOF'
[esptool]
custom_reset_sequence = R0|D1|W0.2|R1|W0.6
EOF
export ESPTOOL_CFGFILE=$PWD/esptool.cfg
port=/dev/serial/by-id/usb-...   # the board's USB port
# Back up the whole 2 MB flash first (about 190 s at 115200 bps, 26 s at 921600 bps)
esptool --port "$port" --baud 921600 --before default-reset --after no-reset \
  read-flash 0 0x200000 firmware/nina-fw/images/backup.bin
# Write the image built here (about 20 s at 921600 bps)
esptool --port "$port" --baud 921600 --before default-reset --after no-reset \
  write-flash 0 firmware/nina-fw/source/NINA_W102.bin
stty -F "$port" 300              # leave the passthrough
```

The times were measured on 2026-11-18 (reading through each of the three
builds, writing once through the FreeRTOS build). Writing
`NINA_W102.bin` leaves the `storage` and `nvs` partitions as they were; to go
back, write the whole backup at 0. A backup may hold credentials: keep it
private (`images/` is not tracked by Git). Check that the driver supports a
firmware before writing it.

## Patches

They apply in order, and each can be left out together with the ones after
it.

| Patch | Change | 0x6A bit |
|---|---|---|
| 0001 | `socket_poll` clears the exception fd set it passes to select | |
| 0002 | `setPowerMode` reports the result of `esp_wifi_set_ps()` (the driver disables power saving on each connection) | |
| 0003 | Command 0x6A reports what the patches add | |
| 0004 | A scan while connected keeps the connection | 0 |
| 0005 | 0x36 starts a scan and returns; 0x6B reports it; 0x27 returns its results | 1 |
| 0006 | Socket events on the IRQ line (ESP32 GPIO0) | 2 |

## Host interface of 0003 to 0006

Commands use the usual framing: the host sends 0xE0, the command, the number
of parameters, each parameter with its length, 0xEE, padded to 4 bytes; the
reply is 0xE0, the command | 0x80, the number of parameters, each parameter
with its length (8-bit lengths for the commands below), 0xEE, padded to 4
bytes. A firmware without a command (upstream 3.0.2, or only 0001 and 0002)
replies 0xEF 0x00 0xEE instead (padded to 4 bytes): the first byte is 0xEF,
not 0xE0.

### 0x6A: capabilities

No parameters. Reply: one parameter of 4 bytes, a little-endian bit mask.

| Bit | Meaning |
|---|---|
| 0 | a scan while connected keeps the connection (0004) |
| 1 | asynchronous scan: 0x36 and 0x6B (0005) |
| 2 | socket events on the IRQ line (0006); cleared when its task could not start at boot |
| others | 0, reserved |

### Scans (0004, 0005)

A scan keeps the connection when the module is in station mode and
connected (0x20 reports 3, `WL_CONNECTED`): it scans in the background,
returning to the AP's channel for 30 ms between the channels it scans (100 ms
at least, 300 ms at most on each channel; about 1.7 to 4.3 s for 13
channels). The IP configuration and the sockets stay, and 0x20 keeps
reporting 3. Traffic slows down while the scan runs. If the connection is
lost during the scan, 0x20 reports what the disconnection set (6,
`WL_DISCONNECTED`, or 4 for reasons 201 and 202), the firmware closes the
sockets that are not listening, as on any disconnection, and the end of the
scan does not change 0x20.

When the module is not connected, or in AP mode, a scan restarts Wi-Fi in
station mode first and 0x20 reports 2 (`WL_SCAN_COMPLETED`) after it, as in
3.0.2 (an AP is ended, a connection attempt is abandoned).

- 0x36 (no parameters): starts a scan and returns at once. Reply: one
  parameter of 1 byte, 1 if a scan was started or is still running, 0 if it
  could not start. Without 0005, 0x36 does nothing and replies 1.
- 0x6B (no parameters): reply: one parameter of 1 byte, the state of the
  scan started by 0x36: 0 none (or its results were taken), 1 running,
  2 done, 3 could not start.
- 0x27 (unchanged reply: the number of networks as the number of
  parameters, each SSID a parameter): when a scan is running, waits for it;
  when one is done, returns its results at once; otherwise starts a scan and
  waits for it, as in 3.0.2. The state is 0 afterwards. 0x32, 0x33, 0x3C and
  0x3D then give each network's RSSI, encryption, BSSID and channel as
  before.

The host's sequence: 0x36, then 0x6B every 100 ms or so until it is 2 (the
module answers other commands meanwhile), then 0x27 and the per-network
commands. 3: the scan failed. A scan that never ends leaves 0x6B at 1; 0x27
would then not return. Connecting, disconnecting or starting an AP (setNet,
setPassPhrase, setKey, setEnt, setApNet, setApPassPhrase, disconnect) stops
a running scan and sets the state to 0.

### Socket events on the IRQ line (0006)

The line: ESP32 GPIO0, push-pull output, to RP2040 GPIO2. Active high.

- It rises when the firmware sees an armed event (below) and it is not
  already high.
- It falls when the module begins to handle the next command, of any kind
  (after that command's transfer, before it runs). It stays high at least
  5 us.
- It is also high while one of the older per-type sockets (0x28 to 0x46)
  has data, as in 3.0.2 (refreshed at most every 10 ms).

So the host gets one rising edge for the events seen since the last command
began, and it should count rising edges.

A command arms an event of its socket when it shows that the host waits for
it. An armed event is reported once: when it happens, it is disarmed and
the line rises, until a later command arms it again.

| Command | Arms |
|---|---|
| poll (0x7C) | readable if the reply lacks 0x01, writable if it lacks 0x02 (nothing if 0x80 is set) |
| recv (0x78), recvfrom (0x7A) | readable, when nothing came and errno is EAGAIN/EWOULDBLOCK |
| accept (0x75) | readable, when no connection was there (reply 255, EAGAIN) |
| send (0x77), sendto (0x79) | writable, when nothing was taken and errno is EAGAIN, EWOULDBLOCK or ENOMEM |
| connect (0x76) | writable, when it is in progress (EINPROGRESS, EALREADY) |

Readable means data, a connection to accept, the end of the stream, or an
error; writable means room to send, a connection made, or an error. Events
are forgotten when the socket is closed (0x71), when a new socket gets the
same number (0x70), and when Wi-Fi disconnects (the firmware closes the
sockets that are not listening).

The host's rule: note the edge count before the command that may arm an
event (the poll, recv, accept, send or connect), and if the reply says "not
ready", wait until the count changes or a timeout passes, then ask again.
An edge from before the command does not harm: the host asks once more.

Events can be missed, so the host still asks on a timeout: bit 2 of 0x6A is
0 (an older firmware, or the task did not start); the firmware's select()
failed (every event is then forgotten and the line raised, so the host
asks again and re-arms); an edge was lost on the host. A timeout of about
100 ms keeps the worst case short; without bit 2 the host polls at fixed intervals.

Inside the firmware a task waits for the armed events with the VFS
`select()` (an eventfd wakes it when events are armed). A socket is closed
only after its events are disarmed and, if the task's `select()` has it,
after that `select()` returned (at most 1 s).
