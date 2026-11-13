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

## LSM6DSOX (`lsm6dso`)

| File | Role |
|---|---|
| `lsm6dso/lsm6dso.c` | The driver. A copy of the Zephyr version's `drivers/lsm6dso/lsm6dso.c` with only the calls into the system replaced (the same functions, branches, order, register operations, conversions, error values and log lines). The devicetree instance macros at the end are one `DEVICE_DT_DEFINE()` with the board's values |
| `lsm6dso/lsm6dso.h` | A copy of the Zephyr version's file. Only the kernel object types (`k_mutex`, `k_spinlock`), the bus condition (`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` → `LSM6DSO_BUS_I2C`) and the stream condition (`CONFIG_SENSOR_ASYNC_API` → `CONFIG_NANO_LSM6DSO_STREAM`) are replaced; `timestamp_decimation` is moved out of the stream part (below) |
| `lsm6dso/lsm6dso_os.[ch]` | The kernel services: mutexes (recursive, priority inheritance), interrupts off (`k_spin_lock()`), `k_busy_wait()`, `k_msleep()`/`k_usleep()`, `k_cycle_get_32()` |
| `lsm6dso/stmemsc.h`, `stmemsc_i2c.c`, `stmemsc_mdelay.c` | A copy of the I2C part of Zephyr's `drivers/sensor/st/stmemsc/` (the glue between ST's `stmdev_ctx_t` and Zephyr's I2C API) |
| `lsm6dso/st_lsm6dso_reg.c` | A one-line wrapper that builds ST's `lsm6dso_reg.c` (`nuttx/stmemsc/`, not in Git) as it is |
| `lsm6dso/lsm6dso_fifo_hw.[ch]`, `lsm6dso_fifo_stats.h`, `lsm6dso_attributes.h` | Unchanged copies of the Zephyr version's files |
| the board's `src/nano_lsm6dso.[ch]` | Makes I2C0 the compatibility layer's bus device (`i2c0`, 100 kHz) and initializes `lsm6dsox` (from `nano_bringup()`) |

The replacements (the same table is at the top of `lsm6dso.c`):

| Zephyr | NuttX port |
|---|---|
| `<zephyr/logging/log.h>`, `CONFIG_SENSOR_LOG_LEVEL` (3) | `app_log.h` (the same lines in the RAM log), level 3 |
| `k_mutex_init/_lock/_unlock` | `lsm6dso_os_mutex_init/_lock/_unlock` (`rmutex_t`; a wait ended by a signal is repeated) |
| `k_spin_lock/_unlock` | `lsm6dso_os_spin_lock/_unlock` (`up_irq_save()`) |
| `k_busy_wait()` | `up_udelay()` |
| `k_msleep()`, `k_usleep()` | `nxsched_msleep()`, `nxsched_usleep()` (rounded up to ticks plus one tick: Zephyr's rule, the same 100 µs tick) |
| `k_cycle_get_32()`, `k_cyc_to_ns_floor64()` | the TIMER's low 32 bits (1 MHz) and ×1000 |
| `sys_get_le32()` | the same arithmetic, written in the file |
| the devicetree instance, `POST_KERNEL`, priority 90 | `DEVICE_DT_DEFINE(lsm6dsox, ...)`, `device_init()` in the board's bring-up |

- The board's `nano_bringup()` initializes the sensor right after I2C0 and before mounting the
  key-value store (NVS), the Zephyr version's order. A failure only leaves the device not ready
  (`device_is_ready()` is false, as with Zephyr's `do_device_init()`); the boot goes on.
- The FIFO functions (`lsm6dso_fifo_*_locked_driver()`, `lsm6dso_timestamp_read()`,
  `lsm6dso_fifo_get_stats()` and others) are built without the stream too, so
  `timestamp_decimation`, which the Zephyr version keeps in the stream part of the structure, is
  outside it here. It is 0 (no timestamp words) until the stream sets it. The stream
  (`lsm6dso_stream.c`, `lsm6dso_fifo_irq.c`, RTIO, the decoder, `lsm6dso_compat.c`) and its
  attributes (`fifo_watermark`, `fifo_timestamp_decimation`) come with
  `CONFIG_NANO_LSM6DSO_STREAM`.
- Not ported: SPI, the sensor hub (`lsm6dso_shub.c`), triggers (`lsm6dso_trigger.c`) and the die
  temperature. All are off in the Zephyr version's configuration (`CONFIG_LSM6DSO_SENSORHUB`,
  `_TRIGGER`, `_ENABLE_TEMP`) and not built there. Their `#ifdef` parts are kept in the copy.
- The FIFO read time (`transfer_time_ns` in the statistics) is measured with the TIMER (1 µs
  steps); the Zephyr version uses the CPU clock (SysTick, 8 ns steps).

## Zephyr API compatibility layer (`compat`)

`nuttx/drivers/compat/` provides, under Zephyr's names and no more than needed, the part of
Zephyr's API that the copied driver and its callers (the acquisition service, the calibration, the
sensor command) use. The include path is `$(TOPDIR)/external/compat`. Zephyr's sensor subsystem
as a whole is not ported.

| Header | Contents |
|---|---|
| `zephyr/device.h` | `struct device` (name, config, API, state, data, init function), `device_is_ready()`, `device_init()` (the meaning of Zephyr's `do_device_init()`), `DEVICE_DT_GET()`, `DEVICE_DT_DEFINE()`, `DEVICE_API()` |
| `zephyr/devicetree.h` | `DT_NODELABEL()`, `DT_BUS()`; the nodes are `i2c0` and `lsm6dsox` only |
| `zephyr/drivers/sensor.h` | `struct sensor_value`, the channel, attribute and trigger enumerations (Zephyr's numbers), `struct sensor_driver_api`, `sensor_attr_set()`/`_get()`, `sensor_sample_fetch()`/`_chan()`, `sensor_channel_get()`, `sensor_trigger_set()`, the unit conversions (`sensor_ms2_to_g()`, `sensor_rad_to_degrees()`, `sensor_value_to_micro()` and others) |
| `zephyr/drivers/i2c.h` | `struct i2c_dt_spec`, `i2c_burst_read_dt()`, `i2c_write_dt()`, `i2c_write_read_dt()`, `i2c_configure()`, the speed classes (`I2C_SPEED_*`, `I2C_SPEED_SET()`) |
| `zephyr/sys/util.h` | `BIT()`, `ARRAY_SIZE()`, `MIN()`/`MAX()`, `IS_ENABLED()`, `CONTAINER_OF()` and others |
| `zephyr/rtio/rtio.h` | A placeholder (only the two names `lsm6dso_fifo_stats.h` uses); RTIO comes with the stream |

- An I2C bus is a device holding a NuttX I2C master and a speed (the board binds them with
  `i2c_compat_bind()`). In Zephyr the bus driver keeps the speed and `i2c_configure()` changes it
  for every later transfer. A NuttX transfer carries a frequency in each message, so the layer's
  transfers (`i2c_burst_read_dt()` and so on) put the device's speed in their messages. Users of
  the NuttX master itself, such as the `i2c` tool, run at their own frequency.
- `i2c_configure()` takes only the classes the Zephyr version's RP2040 driver takes (100 kHz,
  400 kHz, 1 MHz); for anything else it changes nothing and returns `-EINVAL`. It ignores the mode
  bits (that driver forces controller mode too). While I2C0 is quarantined after a DMA failure,
  Zephyr's `i2c_configure()` returns `-EIO`; the layer records the speed and returns 0, and the
  next transfer gives `-EIO`.
- NuttX's `<nuttx/i2c/i2c_master.h>` has functions of its own named `i2c_write()` and `i2c_read()`
  and `I2C_SPEED_*` in Hz. In the layer, Zephyr's `i2c_write()` and `i2c_write_read()` on a bus
  device are named `i2c_compat_write()` and `i2c_compat_write_read()` (the `_dt` forms keep
  Zephyr's names). In a file that includes `<zephyr/drivers/i2c.h>`, `I2C_SPEED_*` are Zephyr's
  classes (include NuttX's header first if both are needed).

Wi-Fi management and sockets (usrsock) are not ported yet.
