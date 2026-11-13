# Drivers (NuttX port)

English | [日本語](README_ja.md)

`nuttx/drivers/` holds the port's code that NuttX builds on the kernel side (through the symlink
`nuttx/nuttx/external`, into `libexternal.a`). The board's part is in
`nuttx/boards/arduino-nano-rp2040-connect/src/`. The Zephyr version's driver notes are
`zephyr/app/usb-console/drivers/README.md`. The port is in progress; this document covers only
the parts ported so far.

## NINA-W102 (`nina`)

### SPI transfer layer

| File | Role |
|---|---|
| `nina/nina_spi.c` | Framing of commands and answers (`nina_request()`, `nina_ip_config()`). A copy of the functions of the same names in the Zephyr version's `drivers/nina/nina_wifi.c`, with the branches, order, constants and error values matching one to one |
| `nina/nina_proto.h` | The public declarations, an unchanged copy of the Zephyr version's file |
| `nina/nina_os.[ch]` | The lock (a recursive mutex with priority inheritance), sleeps and the millisecond clock: Zephyr's `K_MUTEX`, `k_sleep()` and `k_uptime_get()` |
| `include/nano_nina.h`, the board's `src/nano_nina.c` | SPI1 and the four lines (CS, READY, RESET, IRQ): what Zephyr's devicetree and its SPI and GPIO drivers set up |

- A request is handled synchronously in the calling thread. The mutex serializes requests, and CS
  stays asserted from the start of a phase until its last byte, including the wait for READY.
- The NINA is reset once, on the first request (10 ms, 750 ms). At boot the board only sets up
  SPI1 and its pins and drives CS inactive.
- The SPI is NuttX's `rp2040_spi.c`, polled (no DMA). Each transfer takes `SPI_LOCK`; the first
  one sets mode 0, 8 bits and `CONFIG_NANO_NINA_SPI_FREQUENCY`. `SPI_SELECT` is not used.
- The time limits, the wiring and measurements are in the "NINA-W102" section of
  `nuttx/README.md`.
- `nina/nina_spi.c` is checked by a host test (`nuttx/tests/nina_spi.c`) with a fake board and
  clock.

Wi-Fi management and sockets (usrsock) are not ported yet.
