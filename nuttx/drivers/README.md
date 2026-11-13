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
| The board's `src/nano_i2c.c` | The I2C0 controller. A copy of the controller part of the Zephyr version's `drivers/i2c_dw/i2c_dw.c` (`i2c_dw_transfer()`, `i2c_dw_isr()`, `i2c_dw_data_ask()`, `_read()`, `_send()`, `i2c_dw_setup()`, `i2c_dw_configure()`, the initialization) and of the DMA reads (`i2c_dw_rp_dma_read()` and the rest) with the same functions, branches, order, register values, time limit and error values, offered as a NuttX `struct i2c_master_s` (`I2C_TRANSFER()`, `I2C_RESET()`) |
| The board's `src/nano_i2c.h` | `nano_i2c0_initialize()` (called by `nano_bringup()` at boot, which registers `/dev/i2c0`) |
| The board's `src/nano_i2c_dw.h` | The OS-independent part: the definitions of the Zephyr version's `i2c_dw.h`, the SCL counts, the speed class of a frequency, the NuttX flags (`I2C_M_*`) turned into Zephyr's (`I2C_MSG_*`) and the words written to `IC_DATA_CMD`. Host test `nuttx/tests/nano_i2c_dw.c` |
| The board's `src/nano_i2c_dw_registers.h` | A copy of the Zephyr version's `i2c_dw_registers.h` (only the accessors are defined for NuttX) |
| The board's `src/nano_dma.c`, `nano_dma.h` | The RP2040 DMA used by the DMA reads: Zephyr's `drivers/dma/dma_rpi_pico.c` (`dma_config()`, `dma_start()`, `dma_stop()`, the interrupt), the channel allocation of the DMA API (`dma_request_channel()`, `dma_release_channel()`) and the pico-sdk calls that `i2c_dw.c` makes itself (`dma_channel_is_busy()`, `dma_irqn_acknowledge_channel()`, `dma_hw->abort`), written on the registers |
| The board's `src/i2c_dw_rp2040_commands.[ch]` | Unchanged copies of the Zephyr version's files of that name (the command words sent by DMA). Host test `nuttx/tests/i2c_dma_commands.c` (also a copy) |
| `include/nano_i2c_dma.h` | The state that the `i2c_dma` command (`nuttx/apps/i2c_dma`) reads and sets: DMA on or off and the four counters |

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
- The DMA reads (`CONFIG_NANO_I2C0_DMA`, on by default; the Zephyr version's
  `CONFIG_I2C_DW_RP2040_DMA`) copy the DMA part of `i2c_dw.c` with the same branches, order and
  values:
  - Conditions: DMA on (`i2c_dma mode`), the DMA ready, 32 to 3584 bytes
    (`I2C_DW_RP2040_DMA_MAX_READ`), ending with STOP, the last message of the transfer, a 7-bit
    address. Otherwise `-EAGAIN` sends the read to the interrupt path. When a channel cannot be had
    or configured, `dma_fallbacks` is counted and the read also goes to the interrupt path (nothing
    has been started yet).
  - The TX channel writes the read command words (16 bits: READ on each, RESTART on the first only
    when one is needed, STOP on the last) to `IC_DATA_CMD`; the RX channel takes the data from
    `IC_DATA_CMD` 8 bits at a time. `IC_DMA_TDLR` is 8, `IC_DMA_RDLR` 0, the only interrupts are
    STOP_DET, TX_ABRT, TX_OVER, RX_OVER, RX_UNDER and SCL_STUCK_LOW. The RX channel starts first,
    then TX, then `IC_DMA_CR` gets 3.
  - The read ends when TX done, RX done and STOP have all come, or on an error (an I2C error
    interrupt, a DMA bus error). The wait uses the transfer time limit (1 s) as one deadline and
    waits for what is left of it. An error gives `-EIO`, the deadline `-ETIMEDOUT`.
  - Clean-up: `IC_DMA_CR` 0, interrupt mask 0; on failure abort the I2C and stop both DMA channels
    (interrupt off, CHAN_ABORT, wait in 1 ms steps for at most 100 ms until BUSY and CHAN_ABORT clear,
    then acknowledge late completions (RP2040-E13) on both DMA_IRQ_0 and DMA_IRQ_1). If they do not
    stop, the controller is quarantined (`<err> i2c_dw: DMA channels did not stop; I2C controller
    quarantined`, `-EIO`, the channels are not released) and every later transfer gives `-EIO` until
    a reboot. Otherwise the interrupts are cleared and, on failure, the abort is given at most 100 ms
    to finish (else quarantine and `<err> i2c_dw: I2C abort did not finish; controller quarantined`).
  - As in Zephyr's RP2040 DMA driver, channel 0 is on DMA_IRQ_0 and the others on DMA_IRQ_1, a
    channel request never waits and a stop does not wait (the I2C side above waits). NuttX's
    `rp2040_dmac.c` (`CONFIG_RP2040_DMAC`) uses DMA_IRQ_0 for every channel, waits for a free channel
    and waits without a limit in its stop, so it is off (these reads are the only DMA user; the
    NINA's SPI does not use DMA).
  - `rp_dma_events` is updated with interrupts off (Zephyr's `atomic_or()`); the four counters are
    written only by the transfer, which holds the bus semaphore (32 bits, wrapping).

Wi-Fi management and sockets (usrsock) are not ported yet.
