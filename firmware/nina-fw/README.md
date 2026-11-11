# Patched NINA firmware

This directory contains patches for Arduino NINA firmware 3.0.2.
They fix the uninitialized exception fd set in `socket_poll` and report the
result of setting Wi-Fi power saving, which the RP2040 driver disables on
each Wi-Fi connection. The firmware
source and build output are kept in the ignored `source/` directory.

The source is pinned to the upstream 3.0.2 commit
`f9491329a818bc7f36cdda465b4bea53b03f4bda`. The upstream build requires Git,
Docker, network access, and the `espressif/idf:v4.4.8` image. Run from the
repository root:

```sh
firmware/nina-fw/build.sh
```

The build targets Arduino Nano RP2040 Connect and generates
`firmware/nina-fw/source/NINA_W102.bin`. It does not flash either processor.
On first use the script clones the pinned source and applies the patch; later
builds reuse the checkout. Move the source directory aside for a fresh build.
Docker pulls `espressif/idf:v4.4.8` automatically if it is not present locally;
the build container itself has networking disabled.
The patched image was built with ESP-IDF 4.4.8 and installed on a Nano RP2040
Connect. The RP2040 driver uses the firmware's BSD-style socket commands
(0x70 to 0x7f), including the fixed socket poll, for every socket.
Updating the
NINA requires a separate passthrough image on the RP2040 and a compatible
flashing tool; consult the upstream README before attempting it. Preserve a
recovery image and check compatibility with the current Zephyr driver first.

Sources: [NINA 3.0.2](https://github.com/arduino/nina-fw/tree/3.0.2),
[upstream build instructions](https://github.com/arduino/nina-fw/blob/3.0.2/README.md).
