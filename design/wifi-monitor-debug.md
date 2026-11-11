# Tracing TCP ACKs with a Wi-Fi monitor

English | [日本語](wifi-monitor-debug-ja.md)

A procedure for following a TCP ACK sent by the PC from the access point (AP) to the NINA when
the NINA-W102 of an Arduino Nano RP2040 Connect sends to the PC over TCP or MQTT. A TCP capture
on the PC and an 802.11 capture from a separate Wi-Fi adapter are taken at the same time and
compared.

A capture on the PC alone shows that the PC sent an ACK, but not whether the AP transmitted it
to the NINA or whether the NINA received it. The radio capture misses frames too, so do not
conclude from an absent record alone that something was not sent or not received.

## What you need

- The RP2040 with its NINA-W102 and a PC on the same network, plus a way to reproduce the
  MQTT or TCP traffic and a listening application on the PC.
- A Wi-Fi adapter that supports monitor mode on Linux, **separate from the one the PC uses for
  its normal traffic**. It must cover the band and channel of the NINA and capture data frames
  and 802.11 control frames.
- `iw`, `ip`, `tcpdump`, and Wireshark / `tshark`. Installing them and switching the adapter's
  mode needs administrator rights.
- For decryption, the SSID and credentials of the network under investigation. For
  WPA2-Personal the 4-way handshake of the NINA's connection must be captured as well.

On Ubuntu:

```bash
sudo apt update
sudo apt install iw iproute2 tcpdump wireshark tshark
```

The UGREEN CM495 used here has USB ID `0bda:b832` and the driver `rtw89_8852bu`. In monitor
mode on channel 2 it captured the NINA's data in both directions and 802.11 ACKs. However,
**some of the NINA's data frames were not recorded by the monitor although they reached the
PC**. Supporting monitor mode is not the same as capturing every frame.

Capture files can contain information about other stations on the channel and about the
network. Limit where files containing credentials or decrypted traffic are kept and with whom
they are shared.

## Identifying devices and the channel

The examples use `wlo1` for the PC's normal traffic, `wlx020000000001` for the USB monitor
adapter, `192.168.1.31` for the NINA, `192.168.1.27` for the PC and `18830` for the listening
TCP port. Substitute your own values. Take care not to switch the interface used for normal
traffic into monitor mode.

```bash
lsusb
ip -br link
ip -4 addr show wlo1
sudo iw dev
```

Check under `Supported interface modes` of the adapter's `Wiphy` in `iw list` that monitor
mode exists. Even when `monitor` is listed, whether the frames you need are really captured is
checked later.

```bash
sudo iw list
nmcli -f CHAN,FREQ,SSID,BSSID device wifi list ifname wlo1
```

Choose the channel of the AP the NINA connects to from the SSID and BSSID. The AP here was on
2.4 GHz channel 2. Even if the PC is connected on 5 GHz, fix the monitor on the **2.4 GHz
channel between the NINA and the AP**.

On the RP2040's USB serial console run `wifi status` and `net iface` to see the connection
state, the AP's BSSID and the NINA's IP and MAC addresses. The NINA's MAC address in the
experiment was `02:00:00:00:00:02`.

## Setting up the monitor adapter

Place the USB adapter near the NINA. The following switches the adapter itself to monitor
mode. Replace the interface name and channel with the values you found.

```bash
MON=wlx020000000001
CHANNEL=2

sudo ip link set "$MON" down
sudo iw dev "$MON" set type monitor
sudo ip link set "$MON" up
sudo iw dev "$MON" set channel "$CHANNEL"
sudo iw dev "$MON" info
```

Confirm `type monitor` and the right channel. If NetworkManager reverts the setting, take the
USB adapter out of its management for the duration and restore it afterwards. Where `iw` was
not usable, switching the mode with `iwconfig` also worked with this Linux driver.

Return the adapter to its original mode after the test, and undo any change to
NetworkManager's management.

```bash
sudo ip link set "$MON" down
sudo iw dev "$MON" set type managed
sudo ip link set "$MON" up
```

## Capturing the handshake and the traffic together

1. Start the radio capture first. Do not use a capture filter such as a TCP port at this
   stage, so that the WPA2 handshake and control frames are not missed.

   ```bash
   sudo tcpdump -i "$MON" -s 0 -U -w wifi-radio.pcap
   ```

2. Once the capture is running, reconnect the NINA to Wi-Fi. If you reboot the RP2040, check
   beforehand that the firmware connects automatically with stored Wi-Fi settings. After the
   reconnection confirm the connection and address with `wifi status` and `net iface`.
3. Start the capture on the PC in another terminal. Here it can be limited to the TCP port
   under investigation, that is, the listening port of the MQTT broker or test receiver.

   ```bash
   sudo tcpdump -i wlo1 -s 0 -U -w wifi-pc.pcap 'tcp port 18830'
   ```

4. Start the receiving program on the PC, then send over TCP or MQTT from the RP2040. Note
   the send size, period, IMU data rate, duration and the power-save settings of the PC and
   the NINA, as well as the longest stall in sending and whether the FIFO overran.
5. After the test stop each `tcpdump` with `Ctrl-C` and check the number of captured packets
   and the kernel's drop count. **A kernel drop count of 0 does not mean nothing was missed by
   the radio or inside the adapter.**
6. Open `wifi-radio.pcap` in Wireshark and look for the EAPOL frames between the NINA and the
   AP with this display filter.

   ```text
   eapol && wlan.addr == 02:00:00:00:00:02
   ```

   Check that messages 1 to 4 of the 4-way handshake are present. If not, make sure the right
   channel is monitored and that the NINA reconnected after the capture started, then capture
   again.

## Decrypting WPA2-Personal

This is the WPA2-Personal case that worked here. The key material needed depends on the
authentication method; with WPA3-SAE or enterprise authentication in particular a passphrase
may not be enough.

1. Create and select a temporary personal configuration profile in Wireshark.
2. Under `Edit` → `Preferences` → `Protocols` → `IEEE 802.11`, enable `Enable decryption`.
3. Open `Decryption keys` on the same page and add `passphrase:SSID` with key type `wpa-pwd`.
   Do not put the passphrase on a shell command line or in its history. Use the SSID the NINA
   actually connected to.
4. Reopen `wifi-radio.pcap`, which contains the handshake, from the beginning. With the
   display filter `tcp.port == 18830` check that TCP between the NINA and the PC is decoded.
   If EAPOL is visible but TCP is not, check the SSID, the credentials, the handshake and
   which AP was joined.
5. Afterwards delete the key stored in the temporary Wireshark profile. The pcap still
   contains the handshake, so take care with sharing the file even after the key is gone.

`tshark` can use the same temporary profile. The value of `-C` below is the name of the
profile created in Wireshark. `tcp.ack_raw` is the absolute ACK number and does not depend on
Wireshark's relative sequence number setting.

```bash
tshark -C wifi-debug -r wifi-radio.pcap \
  -Y 'tcp.port == 18830' \
  -T fields -e frame.number -e frame.time_epoch \
  -e ip.src -e ip.dst -e tcp.seq_raw -e tcp.ack_raw -e tcp.len \
  -e wlan.seq -e wlan.qos.tid -e wlan.fc.retry
```

Printing the same TCP fields from `wifi-pc.pcap` lets you match the PC side against the radio
side. Compare TCP ACK numbers as **raw absolute numbers** on both sides. Column and filter
details may differ between Wireshark versions.

## The normal sequence

```text
NINA                    AP                       PC
  |--- TCP data -------->|----------------------->|
  |                      |<------ TCP ACK --------|
  |<-- 802.11 data frame carrying the ACK -------|
  |--- 802.11 ACK/Block ACK ---> AP              |
  |--- next TCP data --->|----------------------->|
```

- The `Acknowledgment Number` of a TCP ACK is the next TCP byte the receiver expects. For data
  `seq 1000:2436`, a receiver that got it all normally answers `ack 2436` or more. TCP ACKs
  are cumulative, so one ACK may cover several data segments.
- A TCP ACK travels **inside an encrypted 802.11 data frame**. An 802.11 ACK or Block ACK is a
  receipt on the radio link and something different.
- A Block ACK acknowledges several frames at once. Control frames can be short and the monitor
  can miss them, so do not match frames by time or destination alone.
- A capture on the PC may show several TCP segments merged by GRO and the like. Do not compare
  802.11 frames and PC capture lines one to one; match them by absolute TCP sequence and ACK
  numbers.

## Checking a TCP ACK

1. In `wifi-pc.pcap` display `tcp.port == 18830` and find where sending from the NINA to the
   PC stalled for long. Note the end sequence number of the data just before the stall, the
   `tcp.ack_raw` from the PC to the NINA, the NINA's later TCP retransmission and the PC's
   repeated ACK.
2. Search the decrypted `wifi-radio.pcap` for the ACK number you noted, for example
   `tcp.ack_raw == 2436`. Check `ip.src`, `ip.dst` and the TCP ports too, to be sure it is the
   same connection.
3. Look at `wlan.ra` / `wlan.ta`, `wlan.seq`, `wlan.qos.tid`, `wlan.fc.retry` and the time of
   that 802.11 data frame. Compare when the PC sent it with when it was seen on the radio, and
   tell the first ACK from a repeated ACK answering a TCP retransmission.
4. Check whether the NINA-to-PC data around it is recorded on the radio side as well. Data
   that reached the PC but is missing from the radio capture shows that the monitor missed
   frames in that stretch. USB adapters and drivers can also delay or shift recorded times, so
   do not match frames by time alone.
5. If the NINA side has diagnostic logs at its receive entry or in TCP processing, match them
   by the same absolute ACK number. That a frame could be decrypted from the radio capture
   does not prove it reached the NINA's driver or lwIP.

`wlan.seq` is **an 802.11 number unrelated to TCP sequence numbers**. Compare within the same
TID and sender. Consecutive numbers alone do not prove that the AP did not send a frame.

## Links

- [Linux Wireless: monitor mode and channel setting](https://wireless.docs.kernel.org/en/latest/en/users/documentation/iw/vif.html)
- [Wireshark: WLAN capture](https://wiki.wireshark.org/CaptureSetup/WLAN)
- [Wireshark: decrypting 802.11](https://wiki.wireshark.org/HowToDecrypt802.11)
