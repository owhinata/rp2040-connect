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

## I2C0

| File | Role |
|---|---|
| The board's `src/nano_i2c.c` | The I2C0 controller. A copy of the controller part of the Zephyr version's `drivers/i2c_dw/i2c_dw.c` (`i2c_dw_transfer()`, `i2c_dw_isr()`, `i2c_dw_data_ask()`, `_read()`, `_send()`, `i2c_dw_setup()`, `i2c_dw_configure()`, the initialization) with the same functions, branches, order, register values, time limit and error values, offered as a NuttX `struct i2c_master_s` (`I2C_TRANSFER()`, `I2C_RESET()`) |
| The board's `src/nano_i2c.h` | `nano_i2c0_initialize()` (called by `nano_bringup()` at boot, which registers `/dev/i2c0`) |
| The board's `src/nano_i2c_dw.h` | The OS-independent part: the definitions of the Zephyr version's `i2c_dw.h`, the SCL counts, the speed class of a frequency, the NuttX flags (`I2C_M_*`) turned into Zephyr's (`I2C_MSG_*`) and the words written to `IC_DATA_CMD`. Host test `nuttx/tests/nano_i2c_dw.c` |
| The board's `src/nano_i2c_dw_registers.h` | A copy of the Zephyr version's `i2c_dw_registers.h` (only the accessors are defined for NuttX) |

- The calling thread drives a transfer and the interrupt handler moves the bytes. The caller waits
  for the end on a semaphore (1 s; after that the transfer is aborted and returns `-ETIMEDOUT`). A
  bus semaphore serializes whole transfers (no priority inheritance, as in the Zephyr version).
- NuttX messages are handled as Zephyr messages: a STOP after the last message and after any
  message without `I2C_M_NOSTOP` whose next message has no `I2C_M_NOSTART`, a RESTART at the start
  of a message without `I2C_M_NOSTART` after one with `I2C_M_NOSTOP`. A change of direction (also
  from the last message of the previous transfer) adds a RESTART, as in the Zephyr version.
- The speed comes from the messages' frequency (100 kHz, 400 kHz, 1 MHz); when it changes, the
  counts are computed as Zephyr's `i2c_configure()` does. A speed that is not taken changes nothing
  and gives `-EINVAL` (the Zephyr version keeps the minimum counts then).
- Not ported: target (slave) mode, Intel LPSS DMA, `CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS`, the SDA/SCL
  timeouts (`CONFIG_I2C_DW_EXTENDED_SUPPORT`), power management, PCIe, code for other SoCs, the bus
  recovery and check callbacks (not registered on this board; recovery does nothing), i2c1.
- The RP2040 I2C DMA reads are not there yet. Use and measurements are in the "I2C0" section of
  `nuttx/README.md`.

Wi-Fi management and sockets (usrsock) are not ported yet.
