# DesignWare I2C core (`i2c_dw`)

English | [日本語](README_ja.md)

The firmware of this repository is built on four OSes, Zephyr, NuttX, the Pico SDK with FreeRTOS and the Pico SDK
with ThreadX: the Zephyr, NuttX, FreeRTOS and ThreadX builds. The compatibility layer is the layer that provides
Zephyr's APIs under Zephyr's names on the other OSes ([`common/compat/README.md`](../../compat/README.md)).

`common/drivers/i2c_dw/` is the driver of the RP2040's I2C0 and I2C1. It is a port of the Zephyr build's driver
(Zephyr's `drivers/i2c/i2c_dw.c` with the local patch `zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`)
and is used by the NuttX, FreeRTOS and ThreadX builds. The Zephyr build does not run on this core; it uses the patched
upstream driver ("Zephyr" below). On I2C0, the reads that meet the conditions use DMA. The `i2c` and `i2c_dma` commands
are described here too. Where the core sits among the shared sources is in section 9.1 of the
[design](../../../design/imu-raw-mqtt-design.md), and where it sits on the paths that read the sensor is in
[section 3.4](../../../design/imu-raw-mqtt-design.md#34-the-three-paths-that-read-the-sensor).

## Directories

The same three layers as the NINA driver's: `i2c_dw.c` at the top, the header for the driver's users in `include/`,
the core's own headers in `internal/`, the two boundaries in `port/`, and each OS's binding in `port/<OS>/` and in
`port/pico_sdk/`, shared by the OSes on the Pico SDK.

| File | Role |
|---|---|
| `i2c_dw.c` | The core of the I2C0 and I2C1 controllers: one instance each (`i2c_0_runtime`, `i2c_1_runtime`, under `CONFIG_I2C_DW_I2C0` and `CONFIG_I2C_DW_I2C1`), as Zephyr's driver makes one for each enabled node; the values that differ between them come from `I2C_DW_INST()`. A copy of the controller part of the Zephyr version's `i2c_dw.c` (with patch 0003; the transfer's state machine, the body of the interrupt handler, `i2c_dw_data_ask()`, `_read()`, `_send()`, `i2c_dw_setup()`, `i2c_dw_configure()`, the register work of the initialization), of the DMA reads (`i2c_dw_rp2040_dma_read()` and the rest) and of the `i2c_dma` counters, with the same functions, branches, order, register values, time limit and error values. The intended differences are only those of the list at its top. It includes the boundary `port/i2c_dw_os.h` first and the binding's `i2c_dw_instance.h` at its end |
| `include/i2c_dw_rp2040_dma.h` | The state that the `i2c_dma` command reads and sets: DMA on or off and the four counters (`i2c_dw_rp2040_dma_get_stats()`, `_enabled()`, `_set_enabled()`). The declarations are those of the header that patch 0003 adds to Zephyr's tree (`i2c_dw_sync` checks). The only header of `include/`; it compiles with that directory alone (`i2c_dw_public_headers`) |
| `internal/i2c_dw.h` | Definitions and pure functions without OS headers: the definitions of the Zephyr version's `i2c_dw.h`, the SCL counts, the speed class of a frequency, the NuttX flags (`I2C_M_*`) turned into Zephyr's (`I2C_MSG_*`) and the words written to `IC_DATA_CMD` |
| `internal/i2c_dw_registers.h` | A copy of the Zephyr version's `i2c_dw_registers.h` (the accessors are defined on plain volatile 32-bit accesses, the addresses are `uintptr_t` in place of Zephyr's `mm_reg_t`) |
| `internal/i2c_dw_rp2040.h` | The RP2040's FIFO depth (16) and RX batch (4), as the patch's `drivers/i2c/i2c_dw_rp2040.h`; included once by `i2c_dw.c` |
| `internal/i2c_dw_rp2040_dma_priv.h` | The DMA reads and their command words (`i2c_dw_rp2040_read_commands()`), a copy of the patch's file of the same name with the same branches, order and values; included once by `i2c_dw.c` |
| `port/i2c_dw_os.h` | The boundary: what the core needs from the OS ("The boundaries the OS implements" below). It includes the binding's `i2c_dw_os_impl.h` first and declares the functions that it must define |
| `port/i2c_dw_dma_os.h` | The DMA boundary of the DMA reads: what they need from the OS's own DMA driver, one call for each call of Zephyr's DMA API in the patched driver. No OS name; the core touches no DMA register |
| `port/nuttx/` | The NuttX binding: `i2c_dw_os_impl.h`, `i2c_dw_instance.h`, `i2c_dw_dma_os_rp2040.c`, `i2c_dw_controllers_rp2040.h` ("NuttX" below) |
| `port/pico_sdk/` | The binding shared by the OSes on the Pico SDK: `i2c_dw.cmake`, `i2c_dw_instance.h`, `i2c_dw_controllers_rp2040.h`, `i2c_dw_dma_os_rp2040.c` ("Pico SDK" below) |
| `port/freertos/i2c_dw_os_impl.h` | The FreeRTOS implementation of the boundary ("FreeRTOS" below) |
| `port/threadx/i2c_dw_os_impl.h` | The ThreadX implementation of the boundary ("ThreadX" below) |

Related files outside the driver:

| File | Role |
|---|---|
| `common/shell/i2c_shell.c` | The `i2c` command: byte for byte Zephyr's `drivers/i2c/i2c_shell.c` with the local patch 0004 ("The `i2c` and `i2c_dma` commands" below) |
| `common/shell/i2c_dma_shell.c` | The `i2c_dma` command, shared by the four builds |
| `common/compat/include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` | The layer's `<zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>`, which includes `include/i2c_dw_rp2040_dma.h` |
| `common/tests/i2c_dw.c`, `i2c_dw_sync.py`, `i2c_dw_sync.expected` | Host tests ("Tests" below) |
| `nuttx/drivers/i2c_dw/Kconfig`, `Make.defs` | The NuttX build's options and build rules |

## How the core works

### On every controller

- A transfer with a message of no bytes is refused with `-ENOTSUP` before any of its messages is sent (the controller
  cannot transfer no bytes; upstream sends one byte instead). `i2c scan` then probes with a read of one byte ("The `i2c`
  command" below).
- A bus recovery (`i2c_recover_bus()`, the shell's `i2c recover`) returns `-ENOTSUP`: the board has no bus recovery.
  Before a transfer, the driver still clears a stuck-bus state and goes on to the transfer, as upstream.

### Changes for the RP2040's controllers

RX and TX FIFO depth 16 and an RX threshold of 4 bytes, lowered for short reads and at the end of a read. The TX_EMPTY
interrupt is masked while no more requests can be queued.

### Transfers

- The calling thread drives a transfer and the interrupt handler moves the bytes. The caller waits for the end on a
  semaphore (1 s; after that the transfer is aborted and returns `-ETIMEDOUT`). A bus semaphore serializes whole
  transfers (no priority inheritance, as in the Zephyr version).
- A change of direction (also from the last message of the previous transfer) adds a RESTART, as in the Zephyr version.
- When the speed changes, the counts are computed as Zephyr's `i2c_configure()` does.
- Each device has its own semaphores, so transfers on the two buses run side by side.
- Not ported: target (slave) mode, Intel LPSS DMA, `CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS`, the SDA/SCL timeouts
  (`CONFIG_I2C_DW_EXTENDED_SUPPORT`), power management, PCIe, code for other SoCs, the bus recovery and check callbacks
  (not registered on this board).

### I2C0 and I2C1

As the patched driver, the core has an instance for each controller built. I2C0 is always built, for the LSM6DSOX on
it (`CONFIG_I2C_DW_I2C0` in the NuttX build, on by default). I2C1 is built only with the NuttX build's
`CONFIG_I2C_DW_I2C1` or the FreeRTOS and ThreadX builds' `-DI2C_DW_I2C1=ON` (off by default in all).

- I2C1 (nothing is connected to it on the board) is on GPIO18 (SDA, Arduino D6) and GPIO19 (SCL, D7) with the internal
  pull-ups, and always transfers on the interrupts (the DMA reads and `i2c_dma` are I2C0's). It starts after I2C0; a
  failure is logged and changes nothing else. The `i2c` command does not reach it (nothing uses it yet).
- I2C1 has been built and host-tested but never used with a real device.

I2C0 runs at 1 MHz from the start in all four builds (the Zephyr build's `clock-frequency` in
`zephyr/app/usb-console/app.overlay`, the NuttX board's `BOARD_I2C0_FREQUENCY` in `include/board.h`, and `I2C0_SPEED` in
`port/pico_sdk/i2c_dw_instance.h` for the OSes on the Pico SDK; `i2c_dw_sync` compares the three).

### DMA reads (`CONFIG_I2C_DW_RP2040_DMA`)

- Conditions: I2C0, DMA on (`i2c_dma mode`), the DMA ready, 32 to 3584 bytes (`I2C_DW_RP2040_DMA_MAX_READ`), ending
  with STOP, the last message of the transfer, a 7-bit address. Otherwise `-EAGAIN` sends the read to the interrupt
  path; writes, short reads and anything else use interrupts. When a channel cannot be had or configured,
  `dma_fallbacks` is counted and the read also goes to the interrupt path (nothing has been started yet).
- The read uses two DMA channels: the TX channel writes the read command words (16 bits: READ on each, RESTART on the
  first only when one is needed, STOP on the last) to `IC_DATA_CMD`; the RX channel takes the data from `IC_DATA_CMD`
  8 bits at a time. `IC_DMA_TDLR` is 8, `IC_DMA_RDLR` 0, the only interrupts are STOP_DET, TX_ABRT, TX_OVER, RX_OVER,
  RX_UNDER and SCL_STUCK_LOW. The RX channel starts first, then TX, then `IC_DMA_CR` gets 3.
- The end: when TX done, RX done and STOP have all come, or on an error (an I2C error interrupt, a DMA bus error). The
  wait uses the transfer time limit (1 s) as one deadline and waits for what is left of it. An error gives `-EIO`, the
  deadline `-ETIMEDOUT`.
- Clean-up: `IC_DMA_CR` 0, interrupt mask 0; on failure abort the I2C and stop both DMA channels (interrupt off,
  CHAN_ABORT, wait in 1 ms steps for at most 100 ms until each channel's BUSY and its bit of CHAN_ABORT clear, then
  acknowledge late completions (RP2040-E13)). If they do not stop, the controller is quarantined (`<err> i2c_dw: DMA
  channels did not stop; I2C controller quarantined`, `-EIO`, the channels are not released) and every later transfer
  gives `-EIO` until a reboot. Otherwise the interrupts are cleared and, on failure, the abort is given at most 100 ms
  to finish (else quarantine and `<err> i2c_dw: I2C abort did not finish; controller quarantined`).
- `rp2040_dma_events` is updated with interrupts off (Zephyr's `atomic_or()`); the four counters are written only by
  the transfer, which holds the bus semaphore (32 bits, wrapping).
- Where the patch calls Zephyr's DMA API (and, in the stop, the Pico SDK's register functions), the core calls the DMA
  boundary `port/i2c_dw_dma_os.h`, one call for one. The only differences are the channel setup (two calls, for the TX
  and the RX side, in place of `dma_config()` and its structure) and the stop's wait (for each channel's own abort, not
  for the abort of all channels).

## The boundaries the OS implements

### `port/i2c_dw_os.h`

All are static inline functions or macros, and the OS's interrupt handlers and transfer entry call static inline
functions of the core, so the compiler makes one function of each (no extra calls on the transfer and interrupt
paths). Those marked "interrupt" are called from the interrupt handlers. Each OS's implementation is under "Each OS's
binding" below.

| Boundary | Meaning (Zephyr) |
|---|---|
| `i2c_dw_os_sem_t` | A counting semaphore (`k_sem`; `bus_sem` is used as a binary one without priority inheritance) |
| `i2c_dw_os_sem_give()` (interrupt) | `k_sem_give()` |
| `i2c_dw_os_sem_take_ms(sem, ms)` | `k_sem_take(sem, K_MSEC(ms))`: 0 when taken, nonzero when the time (rounded up to ticks) passes first; not ended by signals |
| `i2c_dw_os_timepoint_t`, `i2c_dw_os_timepoint_calc(ms)`, `i2c_dw_os_sem_take_until(sem, end)` | `sys_timepoint_calc()` and `k_sem_take(sem, sys_timepoint_timeout(end))` (after the deadline, one try without waiting) |
| `i2c_dw_os_sem_reset()` | `k_sem_reset()` |
| `i2c_dw_os_bus_lock()`, `_unlock()` | `k_sem_take(K_FOREVER)` and `k_sem_give()` on `bus_sem`; the errno of a failed lock is what the transfer returns |
| `i2c_dw_os_msleep(ms)` | `k_msleep()` |
| `i2c_dw_os_uptime_ms()` | `k_uptime_get()` (may be a macro) |
| `i2c_dw_os_irq_lock()`, `_unlock()` (interrupt) | `irq_lock()` (for `atomic_or()`); nests |
| `i2c_dw_os_msg_t`, `I2C_DW_OS_MSG_BUF()`, `_LEN()`, `_FLAGS()` | One message of a transfer (for Zephyr's `struct i2c_msg`: `buf`, `len`, `flags`, with STOP on the last message) |
| `I2C_DW_OS_DEV_HEAD` | The OS's device at the start of the controller's data (may be empty) |
| `I2C_DW_CLOCK_SPEED` | `CONFIG_I2C_DW_CLOCK_SPEED` (MHz, 125) |
| `i2c_dw_os_prepare(dw)` (`i2c_dw_instance.h`) | `reset_line_toggle_dt()` and `pinctrl_apply_state()`: reset I2C0 or I2C1, its pins as I2C (4 mA, slow slew, Schmitt trigger and input enabled; I2C0's GPIO12 and 13 without pulls, I2C1's with the internal pull-ups) |
| `i2c_dw_os_irq_enable(dw)` (same) | `IRQ_CONNECT()`, `irq_enable()`: a handler of I2C0_IRQ or I2C1_IRQ that calls `i2c_dw_isr_handler(dw)` |
| `app_log.h` | `LOG_MODULE_REGISTER()`, `LOG_ERR()`, `LOG_ERR_RATELIMIT()` (at the level 3 that the core sets) |
| `CONFIG_I2C_DW_I2C0`, `CONFIG_I2C_DW_I2C1` | The instances to build (Zephyr: the devicetree's enabled nodes) |
| `CONFIG_I2C_DW_RP2040_DMA` | Build the DMA reads of I2C0 (the same symbol as the Zephyr version's) |

### `port/i2c_dw_dma_os.h`

What the OS's DMA driver provides. These are ordinary functions (once per read, not per byte). Only the transfer that
holds the bus semaphore calls them, from its thread; the callback is called from the DMA interrupt. None waits for
another user of the DMA. The NuttX build and the OSes on the Pico SDK implement it on the OS's own DMA driver
(`i2c_dw_dma_os_rp2040.c` of `port/nuttx/` and `port/pico_sdk/`).

| Boundary | Meaning (Zephyr) |
|---|---|
| `i2c_dw_dma_os_initialize()` | The DMA device's start-up (Zephyr: before I2C0's); nothing is reset |
| `i2c_dw_dma_os_is_ready()` | `device_is_ready(dma)` |
| `i2c_dw_dma_os_request_channel()`, `_release_channel()` | `dma_request_channel(dma, NULL)`, `dma_release_channel()`; the request never waits (`-EBUSY`, `-ENODEV`) |
| `i2c_dw_dma_os_config_tx()`, `_config_rx()` | `dma_config()` of each side: 16-bit command words to `IC_DATA_CMD` paced by DREQ 32, bytes from it paced by DREQ 33, normal priority |
| `i2c_dw_dma_os_start()` | `dma_start()`: the channel's old flag cleared, its interrupt enabled |
| callback (interrupt) | The end of a channel (0) or its bus error (`-EIO`) |
| `i2c_dw_dma_os_stop()` | `dma_stop()`: interrupt off and abort requested, no wait |
| `i2c_dw_dma_os_is_busy()` | `dma_channel_is_busy()` and the channel's bit of CHAN_ABORT |
| `i2c_dw_dma_os_acknowledge()` | A late completion flag (RP2040-E13) cleared |

### Other code that uses DMA

Other code of the same program may use the OS's DMA as usual. It must not touch the two channels a read holds, reset
the DMA block, or write whole registers shared by the channels (interrupt enables and status, CHAN_ABORT: only its own
channels' bits). The rules that differ between the OSes, such as those of the interrupt lines, are in each OS's section
below.

## Each OS's binding

### Zephyr

The Zephyr build does not use this core. It uses Zephyr's DesignWare I2C driver (`drivers/i2c/i2c_dw.c`) with the
local patch `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` (the list of patches and how they are applied are in
[`design/zephyr-port.md`](../../../design/zephyr-port.md)). The patch holds everything the driver needs, in Zephyr's
tree: the application adds no source or include path to the driver's library. "How the core works" above is also how
the patched driver works. Other controllers behave as in Zephyr, but for the two changes of "On every controller".

- The patch puts the constants in a new file `drivers/i2c/i2c_dw_rp2040.h`, the DMA code with the read commands in
  `i2c_dw_rp2040_dma_priv.h` (included once by `i2c_dw.c`), and the declarations of the `i2c_dma` command's functions in
  `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`; 16 places go into the upstream files (`i2c_dw.c` 14, `i2c_dw.h` 1,
  `Kconfig.dw` 1).
- `CONFIG_I2C_DW_RP2040_DMA` is off by default in the patch and on in `prj.conf`. A read uses two channels of Zephyr's
  DMA driver (`dma_rpi_pico.c`). The command words and buffer (10,752 bytes together), semaphore, events and channels
  are static variables of the driver, for I2C0 alone. The per-instance data only holds whether a DMA read is in
  progress and whether the controller is quarantined.
- The DMA stop waits for all of CHAN_ABORT (the core: for each channel's own bit). A speed that is not taken keeps the
  minimum counts (the core changes nothing and gives `-EINVAL`).
- The `i2c_dma` command is the shared `common/shell/i2c_dma_shell.c`, built into the application; it calls the three
  functions that the patch declares. The `i2c` command is the original file in the tree, `drivers/i2c/i2c_shell.c` with
  patch 0004.
- After remaking 0003 for a newer Zephyr, run `zephyr/script/host_tests.sh`: bring the upstream changes that
  `i2c_dw_sync` shows into the shared core, or add the reason not to to the list at the top of `i2c_dw.c`, then rewrite
  the expected differences with `common/tests/i2c_dw_sync.py ... --update` ("Tests" below).

### NuttX

NuttX's `rp2040_i2c.c` (`CONFIG_RP2040_I2C`) is not used. `nuttx/drivers/i2c_dw/Make.defs` finds the sources of the core
and of `port/nuttx/` through `VPATH` and puts `include/`, `internal/`, `port/` and `port/nuttx/` on the include path (the
objects are made in `nuttx/drivers`). The options are `CONFIG_I2C_DW_I2C0` (on by default), `CONFIG_I2C_DW_I2C1` (off by
default) and `CONFIG_I2C_DW_RP2040_DMA` (on by default) of `nuttx/drivers/i2c_dw/Kconfig`.

| File | Role |
|---|---|
| `port/nuttx/i2c_dw_os_impl.h` | The NuttX implementation of the boundary: semaphores (`sem_t`: `nxsem_post()`, `nxsem_tickwait_uninterruptible()`, `nxsem_wait_uninterruptible()`, `nxsem_reset()`), time (the system ticks), `nxsched_msleep()`, `up_irq_save()`, the messages (`struct i2c_msg_s`, with Zephyr's flags made by `i2c_dw_msg_flags()`), `I2C_DW_CLOCK_SPEED` (`BOARD_PERI_FREQ`) |
| `port/nuttx/i2c_dw_instance.h` | The NuttX part of I2C0 and I2C1: the operations of `struct i2c_master_s` (`I2C_TRANSFER()` checks the request, takes the speed from the frequency and calls the core's transfer; `I2C_RESET()`), the interrupt handler attached with `irq_attach()`, the reset and the pins (`i2c_dw_pinctrl()`), shared by both, `i2c_dw_i2c0_initialize()` and `i2c_dw_i2c1_initialize()` |
| `port/nuttx/i2c_dw_dma_os_rp2040.c` | The NuttX implementation of the DMA boundary on NuttX's own DMA driver, `arch/arm/src/rp2040/rp2040_dmac.c` (`CONFIG_RP2040_DMAC`) |
| `port/nuttx/i2c_dw_controllers_rp2040.h` | `i2c_dw_i2c0_initialize()` and, with `CONFIG_I2C_DW_I2C1`, `i2c_dw_i2c1_initialize()` (called by `rp2040_bringup()` at boot, I2C1 after I2C0; nothing is registered as `/dev/i2c*`); the board's code finds it through the `port/nuttx` directory on its include path |

The NuttX implementation of the boundary:

| Boundary | NuttX build |
|---|---|
| `i2c_dw_os_sem_t` | `sem_t` |
| `i2c_dw_os_sem_give()` | `nxsem_post()` |
| `i2c_dw_os_sem_take_ms(sem, ms)` | `nxsem_tickwait_uninterruptible(sem, MSEC2TICK(ms))` |
| `i2c_dw_os_timepoint_t`, `i2c_dw_os_timepoint_calc(ms)`, `i2c_dw_os_sem_take_until(sem, end)` | The system ticks (`clock_systime_ticks()`) and `nxsem_tickwait_uninterruptible()` with what is left, at least 0 |
| `i2c_dw_os_sem_reset()` | `nxsem_reset(sem, 0)` |
| `i2c_dw_os_bus_lock()`, `_unlock()` | `nxsem_wait_uninterruptible()`, `nxsem_post()` |
| `i2c_dw_os_msleep(ms)` | `nxsched_msleep()` |
| `i2c_dw_os_uptime_ms()` | `TICK2MSEC(clock_systime_ticks())` (a macro) |
| `i2c_dw_os_irq_lock()`, `_unlock()` | `up_irq_save()`, `up_irq_restore()` |
| `i2c_dw_os_msg_t`, `I2C_DW_OS_MSG_BUF()`, `_LEN()`, `_FLAGS()` | `struct i2c_msg_s`; the flags are made from it and its neighbours by `i2c_dw_msg_flags()` |
| `I2C_DW_OS_DEV_HEAD` | `struct i2c_master_s dev;` |
| `I2C_DW_CLOCK_SPEED` | `BOARD_PERI_FREQ / 1000000` |
| `i2c_dw_os_prepare(dw)` | RESETS and `rp2040_gpio_*()` (`i2c_dw_pinctrl()`, the steps of Zephyr's `pinctrl_rpi_pico.c`) |
| `i2c_dw_os_irq_enable(dw)` | `irq_attach()`, `up_enable_irq()` |
| `app_log.h` | `common/compat/include/app_log.h` with its NuttX part `common/compat/port/nuttx/app_log_os.h` (the RAM log) |
| `CONFIG_I2C_DW_I2C0`, `CONFIG_I2C_DW_I2C1` | `nuttx/drivers/i2c_dw/Kconfig` (`I2C_DW_I2C0` on, `I2C_DW_I2C1` off by default) |
| `CONFIG_I2C_DW_RP2040_DMA` | Kconfig |

The NuttX implementation of the DMA boundary (ordinary functions of `port/nuttx/i2c_dw_dma_os_rp2040.c`):

| Boundary | NuttX build |
|---|---|
| `i2c_dw_dma_os_initialize()` | Called after I2C0's controller has started. `rp2040_dmachannel_trywait()` twice (the lowest free channels, without waiting), their interrupts off; the two channels stay the reads'. Without both, the one taken is freed again and it returns `-EBUSY` |
| `i2c_dw_dma_os_is_ready()` | The two channels were had |
| `i2c_dw_dma_os_request_channel()`, `_release_channel()` | One of the two that the read in progress has not taken; the release frees it for the next read |
| `i2c_dw_dma_os_config_tx()`, `_config_rx()` | `rp2040_txdmasetup()` (`RP2040_DMA_DREQ_I2C0_TX`, `RP2040_DMA_SIZE_HALFWORD`, read address incrementing), `rp2040_rxdmasetup()` (`RP2040_DMA_DREQ_I2C0_RX`, `RP2040_DMA_SIZE_BYTE`, write address incrementing) |
| `i2c_dw_dma_os_start()` | The channel's bit to INTS0 and INTE0, then `rp2040_dmastart()` with the callback |
| callback | From `rp2040_dmac.c`'s handler of DMA_IRQ_0 (its status `EIO` becomes `-EIO`; on an error the channel is aborted at once, see below) |
| `i2c_dw_dma_os_stop()` | The channel's bit off in INTE0, its bit to CHAN_ABORT (not `rp2040_dmastop()`, which waits without a limit) |
| `i2c_dw_dma_os_is_busy()` | BUSY of the channel's CTRL_TRIG (`rp2040_dma_register()`), CHAN_ABORT |
| `i2c_dw_dma_os_acknowledge()` | The channel's bit to INTS0 |

The writes of INTE0 (through the set and clear aliases), INTS0 and CHAN_ABORT carry only the channel's own bit, as
`rp2040_dmac.c` writes them; other channels are not touched.

**Devices and transfers**

- The OS's I2C0 device (`i2c_dw_instance.h`) sets `base` of `i2c_0_runtime` (0x40044000), initializes the semaphores
  (`device_sync_sem` and `rp2040_dma_sem` to 0, `bus_sem` to 1), calls `i2c_dw_turn_on()` once at boot and, when it
  succeeds, `i2c_dw_dma_os_initialize()`, and then sets `ready` (a controller that does not start takes no DMA channel).
  The I2C1 device does the same for `i2c_1_runtime` (0x40048000) without the DMA part and its semaphore.
- The transfer entry (one for both) refuses what the Zephyr API cannot express (no message array, more than 255
  messages, addresses or speeds that differ between messages, a 10-bit address, a missing buffer, a device that is not
  ready), returns 0 for no message, and otherwise returns `i2c_dw_transfer_msgs(dw, msgs, count, address, speed)`
  (`speed` is Zephyr's speed class).
- The I2C0 and DMA interrupts run at the same priority (`CONFIG_ARCH_IRQPRIO` off: every interrupt at the default), and
  the handlers and the transfers run on one core.
- NuttX messages are handled as Zephyr messages: a STOP after the last message and after any message without
  `I2C_M_NOSTOP` whose next message has no `I2C_M_NOSTART`, a RESTART at the start of a message without `I2C_M_NOSTART`
  after one with `I2C_M_NOSTOP`.
- The speed comes from the messages' frequency (100 kHz, 400 kHz, 1 MHz); when it changes, the counts are computed as
  Zephyr's `i2c_configure()` does. A speed that is not taken changes nothing and gives `-EINVAL`.
- No bus recovery is registered: the recovery before a transfer does nothing, and `I2C_RESET()` returns `-ENOTSUP`.
- I2C1's pins are `BOARD_I2C1_SDA_GPIO` and `BOARD_I2C1_SCL_GPIO` in the board's `include/board.h`. It has no device of
  the compatibility layer, so the `i2c` command does not reach it. To build it without changing the defconfig: after a
  build, `printf 'CONFIG_I2C_DW_I2C1=y\n' >> nuttx/nuttx/.config`, `make -C nuttx/nuttx olddefconfig`, then
  `nuttx/script/build.sh` (without `--clean`, which goes back to the defconfig).
- The compatibility layer's bus device `i2c0`, which the `i2c` command uses, is Zephyr's I2C calls on a NuttX I2C
  master (`common/compat/port/nuttx/zephyr_i2c.c`), bound to I2C0 by the board's `src/rp2040_lsm6dso.c`.

**On NuttX's DMA driver**

- The DMA is NuttX's own driver, `rp2040_dmac.c` (`CONFIG_RP2040_DMAC`, on; `arm_dma_initialize()` runs in
  `up_initialize()`, before the board's bring-up). It has one interrupt line, DMA_IRQ_0, for every channel. The local
  patch `nuttx/0011-rp2040-dmachannel-trywait` adds `rp2040_dmachannel_trywait()` to it, which never waits: it returns
  NULL at once when no channel is free or another task holds the allocator's lock, giving back the count it took, so no
  channel is lost (`rp2040_dmachannel()` waits while all twelve channels are taken).
- `i2c_dw_dma_os_rp2040.c` takes its two channels with `rp2040_dmachannel_trywait()` once at boot and keeps them for the
  reads. A read never waits for a channel, and the one read at a time always has its channels, so `dma_fallbacks` stays
  0 unless a setup is refused (a channel still busy). With fewer than two free channels at boot, it frees the one it took
  and I2C0 starts without the DMA: its reads take the interrupts (`dmesg`: `No DMA for the reads: -16`).
- `rp2040_dmastop()` waits for CHAN_ABORT without a limit, so `i2c_dw_dma_os_rp2040.c` stops a channel by writing
  CHAN_ABORT itself and the core waits with its limit (100 ms).
- NuttX's handler writes a failed channel's error flags back through CTRL_TRIG, a trigger register, which can start the
  channel again from where it stopped; the binding's callback then aborts it at once.
- The handler's status of an earlier channel of the same interrupt also carries over to later ones, so another user's
  bus error served in the same interrupt fails this read too (`-EIO`, one `dma_errors`).

**Other code that uses DMA**

- Other code of this build may use NuttX's DMA the usual way (`rp2040_dmachannel()` and the rest; the other ten
  channels).
- It must not: touch the two channels of the reads (they are allocated in `rp2040_dmac.c`'s table, so its allocation
  never hands them out); reset the DMA block; write whole registers shared by the channels (INTE0, INTS0, CHAN_ABORT:
  only its own bits, as `rp2040_dmac.c` does); take DMA_IRQ_0 from `rp2040_dmac.c` with `irq_attach()`; or route its
  channels' interrupts to DMA_IRQ_1 without its own handler there.
- The SPI driver (`CONFIG_RP2040_SPI_DMA`, on in the `imu` configuration) is such a user: it takes two more channels for
  SPI1 at the NINA's first request and keeps them ([NINA driver](../nina/README.md)).

### Pico SDK (shared by FreeRTOS and ThreadX)

The FreeRTOS and ThreadX builds share the Pico SDK part of the core's binding, and each has only its OS part
(`i2c_dw_os_impl.h` of `port/freertos/` and `port/threadx/`: semaphores, clock, sleeps, interrupt masking). The binding's
kernel services are the OS's own, not the compatibility layer's; the only things of Zephyr's left are the API it is
used with: the layer's devices and `<zephyr/drivers/i2c.h>`.

| File | Role |
|---|---|
| `port/pico_sdk/i2c_dw.cmake` | Adds the driver (I2C0 and, as an option, I2C1) and the `i2c_dma` command to the build; the OS part comes from `port/<OS>/` (`PICO_SDK_OS`) |
| `port/pico_sdk/i2c_dw_instance.h` | The layer's devices `i2c0` and `i2c1` with Zephyr's I2C API, their interrupts, resets and pins; included at the end of the core's `i2c_dw.c` |
| `port/pico_sdk/i2c_dw_controllers_rp2040.h` | The start-up functions `i2c_dw_i2c0_initialize()` and `i2c_dw_i2c1_initialize()` |
| `port/pico_sdk/i2c_dw_dma_os_rp2040.c` | I2C0's DMA reads on the Pico SDK's DMA (the implementation of the DMA boundary) |

- **The instances** are chosen as in the NuttX build's Kconfig: `i2c_dw.cmake` defines `CONFIG_I2C_DW_I2C0` always (the
  LSM6DSOX is on it; `common/drivers/lsm6dso/port/pico_sdk/lsm6dso.cmake` stops the configuration without it) and
  `CONFIG_I2C_DW_I2C1` with the CMake option `I2C_DW_I2C1` (off by default). Each instance has its own device, interrupt
  handler, speed class and start-up under its symbol; the API, the pins' setup, the reset and the interrupt's connection
  are shared.
- **I2C1** is the device `i2c1` (`DEVICE_DT_GET(DT_NODELABEL(i2c1))`, name `i2c@40048000`), with the same Zephyr I2C API
  as `i2c0`; its pins are `I2C1_SDA_GPIO` and `I2C1_SCL_GPIO` in `i2c_dw_instance.h`. It starts after I2C0 in
  `usb_console_init()` (`common/imu/port/pico_sdk/usb_console.c`); a failure is logged and leaves `i2c1` not ready. It is
  not in the layer's device list (`device_get_binding()`), so the `i2c` command does not reach it.
- **Refused requests**: as in the Zephyr build's driver there is no `get_config`: `i2c_get_config()` returns `-ENOSYS`,
  so `i2c speed` calls `i2c_configure()` with `I2C_MODE_CONTROLLER` and the new speed class (the later transfers run at
  it).
- **The DMA reads of I2C0** use the Pico SDK's own DMA (`hardware_dma`, `hardware_irq`) through the core's second
  boundary, `port/i2c_dw_dma_os.h`. Per read, two channels come from `dma_claim_unused_channel(false)` (none free: the
  read takes the interrupts and counts a fallback); they are set up with the SDK's channel configuration (TX: 16-bit
  command words to IC_DATA_CMD, DREQ 32; RX: bytes from it, DREQ 33), started with `dma_channel_start()` and given back
  with `dma_channel_unclaim()`. Nothing is reset at start-up.
- **Their interrupt and stop**: the channels' interrupts go to DMA_IRQ_1, on a handler added with
  `irq_add_shared_handler()` (the default order priority) that looks only at the channels the read holds. A stop writes
  the channel's bit to CHAN_ABORT and the core waits for it with a bound (100 ms, then the controller is quarantined);
  the SDK's `dma_channel_abort()` is not used, as it would wait without one.
- **Other code that uses DMA**: other code of this build may use the DMA the usual way: claim channels with the SDK's
  allocation, and share the interrupt lines with `irq_add_shared_handler()`. It must not: touch the two channels a read
  holds (claimed in the SDK's record during the read); install an exclusive handler on DMA_IRQ_1
  (`irq_set_exclusive_handler()` panics when a shared one is there) or acknowledge other channels' flags in its handler
  of a shared line; reset the DMA block or write the whole of INTE0/INTE1, INTS0/INTS1 or CHAN_ABORT (only its own
  channels' bits); claim all twelve channels for good if the reads are to keep the DMA (they then fall back to the
  interrupts).
- **The NINA's SPI by DMA** (`common/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c`) is another such user: it claims
  two channels with the SDK's allocation and keeps them, and shares DMA_IRQ_1 with `irq_add_shared_handler()` (see
  [the NINA driver](../nina/README.md#pico-sdk-shared-by-freertos-and-threadx)).
- **Interrupts**: the DMA_IRQ_1 handler of I2C0's reads takes one slot of the Pico SDK's shared interrupt handlers. The
  number of slots and the interrupt priorities are in [the FreeRTOS port's "Interrupts"](../../../design/freertos-port.md#interrupts).

### FreeRTOS

`port/freertos/i2c_dw_os_impl.h` writes the semaphores, the clock, the sleeps and the interrupt masking on FreeRTOS and
the Pico SDK directly.

- Each semaphore (`device_sync_sem`, `bus_sem`, the DMA reads' `rp2040_dma_sem`) is a count with its limit and a list of
  waiting tasks in the driver's own memory (no FreeRTOS queue, no heap), set up by the instance's start
  (`i2c_dw_i2c0_initialize()`); `bus_sem` has the limit 1 (no priority inheritance, as Zephyr's `k_sem`; its waiters by
  priority).
- A give hands the count to the first waiter, as Zephyr's, and wakes it with its task notification of the driver's own
  index 3 (`vTaskNotifyGiveIndexedFromISR()` and the switch at the end of an interrupt handler, `xTaskNotifyGiveIndexed()`
  from a task; `configTASK_NOTIFICATION_ARRAY_ENTRIES` is 4). A waiter decides by its done flag, not by the wake-up, so a
  give between its look at the count and its block is not lost and a notification left from before only makes it wait
  again for the rest of its time.
- A timed wait blocks for the time rounded up to ticks plus one tick, as Zephyr's `k_sem_take(K_MSEC())` (the
  transfer's 1000 ms are 1001 ticks of 1 ms, the abort's 1 ms 2), the sleeps the same with `vTaskDelay()`, and the clock
  is the TIMER (`time_us_64()`) in ms, not the tick.

### ThreadX

`port/threadx/i2c_dw_os_impl.h` is written on ThreadX and the Pico SDK directly, not on the layer. What it waits on are
`TX_SEMAPHORE`s the driver owns.

- Why the waits are `TX_SEMAPHORE`s and not a wake-up of the thread's own or a suspend and resume: see [the ThreadX port](../../../design/threadx-port.md#the-layers-kernel-part-and-the-drivers-waits).
- `device_sync_sem` and `rp2040_dma_sem` from 0 with the limit `UINT_MAX`, the bus lock `bus_sem` from 1 with the limit 1
  (no priority inheritance; at the release, with two or more waiters, `tx_semaphore_prioritize()` puts the most urgent
  first). A give is `tx_semaphore_ceiling_put()`, `k_sem_reset()` writes the count with interrupts masked.
- `i2c_dw_os_sem_init()` may be called twice before the kernel (when the controller did not start the first time); the
  second call keeps the object made and writes the count and the limit. Before the kernel and in an ISR ThreadX refuses
  a wait (`TX_WAIT_ERROR`); the take then tries once more without a wait.
- Timed waits and sleeps last the time rounded up to ticks plus one tick (Zephyr's rule; ThreadX's wait of n ends at the
  current tick + n). The clock is the TIMER in ms, not the tick.

## The `i2c` and `i2c_dma` commands

**The `i2c` command** (`common/shell/i2c_shell.c`) is byte for byte Zephyr's `drivers/i2c/i2c_shell.c` with the local
patch `zephyr/0004-i2c-shell-scan-read-probe.patch` (`zephyr_i2c_shell_patched` compares them). The Zephyr build builds
the original file in the tree; the NuttX build builds this file in `nuttx/apps/i2c`, the FreeRTOS and ThreadX builds
with `common/shell/port/pico_sdk/i2c_shell.cmake`. Its device is I2C0's `i2c@40044000`.

- `i2c scan` probes with writes of no bytes; when the first one returns `-ENOTSUP`, it probes that address and the rest
  with a read of one byte and prints `Probed with one-byte reads (no zero-length writes)` after the table (patch 0004,
  for any controller that refuses those writes). An address that acks is shown, any error (a NACK) is `--`. Probe one
  address with `i2c direct_read i2c@40044000 6a 1`.
- `i2c recover` gives `-ENOTSUP`.
- `i2c speed i2c@40044000 <1-3>` changes the bus's speed, so the sensor's transfers change speed too.
- The commands of `i2c target` are not built (`CONFIG_I2C_TARGET` is off in the Zephyr build too).

**The `i2c_dma` command** (`common/shell/i2c_dma_shell.c`) is shared by the four builds (the NuttX build builds it in
`nuttx/apps/i2c_dma`). The patched driver and the core define the same three functions (get and set the mode, get the
counters), and the command reads their declarations as `<zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>`: in the Zephyr build
the header that the patch adds to Zephyr's tree, in the NuttX, FreeRTOS and ThreadX builds the header of that name in
`common/compat`, which includes `include/i2c_dw_rp2040_dma.h`.

```text
i2c_dma mode          show the current mode
i2c_dma mode on|off   switch, only while no stream or transfer is active
i2c_dma stats         DMA transfers, bytes, errors and fallbacks (cumulative)
```

## Tests

The shared sources' tests (`common/tests/run.sh`, run first by the four builds' `host_tests.sh`):

| Test | Covers |
|---|---|
| `i2c_dw` | The core's definitions (`internal/i2c_dw.h`): SCL counts, speeds, the flags' conversion, the `IC_DATA_CMD` words (`common/tests/i2c_dw.c`) |
| `i2c_dw_public_headers` | The header for the core's users (`include/`) compiles with that directory alone: it reads nothing of the core's `internal/` or `port/` |
| `i2c_dw_sync` | The shared core against the patched driver (made from the Zephyr tree's files at HEAD and patch 0003), below. Only when the Zephyr tree is there |
| `zephyr_i2c_shell_patched` | `common/shell/i2c_shell.c` is byte for byte Zephyr's `drivers/i2c/i2c_shell.c` at HEAD with patch 0004. Only when the Zephyr tree is there |

**Checking for drift from upstream (`i2c_dw_sync`)**: the patched driver and the core are kept together this way.

- A change to either goes into the other too (or the reason not to goes into the list at the top of `i2c_dw.c`).
- `common/tests/i2c_dw_sync.py` compares the core with the tree's files at HEAD with the patch applied (whatever the
  tree's working files hold). It fails when any of these differ:
  - the definitions of both headers (state, interrupts, registers, the IC_DATA_CMD bits, FIFO depth 16 and batch 4, the
    DMA limit 3584), the SCL counts at 125 MHz, the DMA path's lower limit 32, thresholds 8/0, interrupt mask, IC_CON
    bit 9, stop waits of 100 ms and timeout of 1000 ms, the spike lengths. With the build, the values of its `.config`
    and `zephyr.dts` too (among them I2C0's speed at the start, compared between Zephyr's devicetree, `I2C0_SPEED` of
    `port/pico_sdk/i2c_dw_instance.h` and the NuttX board's `BOARD_I2C0_FREQUENCY`);
  - the pairs of functions: Zephyr's names, its DMA API's calls among them, turned into the core's (those of the DMA
    boundary `i2c_dw_dma_os.h`), compared statement by statement; the differences must be those of the saved
    `common/tests/i2c_dw_sync.expected`. Comments and line breaks are not compared;
  - the read commands' function `i2c_dw_rp2040_read_commands()`, static in `i2c_dw_rp2040_dma_priv.h` of the patch and
    of the core, statement by statement, with its command bits;
  - `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`, which the patch adds, against the core's
    `include/i2c_dw_rp2040_dma.h`, the same once the comments and include guards are taken out;
  - the patched driver reads no header from outside Zephyr's tree.
- A change to one side alone fails, so after an intended change look at the differences and rewrite them with
  `--update`.
- Drift in the functions not compared (Zephyr's initialization, PM, target mode, the OS parts) is not detected. Each
  OS's implementation of `i2c_dw_dma_os.h` is checked by the NuttX and FreeRTOS builds' host tests on fakes of the OS's
  DMA.

Each OS's tests:

| Test | Covers |
|---|---|
| NuttX: `i2c_dw_instance` | The NuttX binding (`i2c_dw_os_impl.h`, `i2c_dw_instance.h`) under the shared core, on fakes of the NuttX headers (`nuttx/tests/i2c_dw_instance_fakes`), the board's `include/board.h`, the controllers' registers and the DMA boundary: three times, I2C0 only, with I2C1 (`CONFIG_I2C_DW_I2C1`), and with a DMA boundary that fails to start (I2C0 starts all the same, on the interrupts) (`nuttx/tests/i2c_dw_instance.c`) |
| NuttX: `i2c_dw_dma_os` | The DMA part (`port/nuttx/i2c_dw_dma_os_rp2040.c`) on a fake of NuttX's `rp2040_dmac.c` and of the DMA's registers: the channels, their setup, start, interrupt, stop and release (`nuttx/tests/i2c_dw_dma_os.c`) |
| NuttX: `i2c_shell` | The `i2c` command on `zshell.c` and the layer's I2C bus device (`common/compat/port/nuttx/zephyr_i2c.c`) with a fake NuttX I2C master |
| NuttX: `i2c_dma_app` | The `i2c_dma` command on `zshell.c` with a fake driver side |
| FreeRTOS: `i2c_dw` | I2C0's boundary and device, Zephyr's I2C API, the `i2c` command on it (the devices of its arguments, `speed` and the speed of the transfers after it, `read_byte`, `write_byte`, `read` in hex, an unknown device, `scan` refused, `recover`) (built twice: i2c0 only, and with `CONFIG_I2C_DW_I2C1`, which adds i2c1's start, pins, transfers, an unanswered address, long reads that never take the DMA, i2c0 unaffected, and transfers on both buses from two tasks); the binding's semaphores, waits' ticks, sleeps and clock; the DMA part (`i2c_dw_dma_os_rp2040.c`) on fakes of the Pico SDK's DMA: claims, setup, start, the shared handler (end, bus error, other users' channels), stop, release; the core's DMA reads on it: the read, the 32-byte limit, fallbacks, bus errors on TX and RX, timeout, the reads off, a stop that does not end (quarantine), the counters; the core's DMA command words (`freertos/tests/i2c_dw/`) |
| ThreadX: `i2c_dw` | The FreeRTOS build's test body (`freertos/tests/i2c_dw/main.c`) with the ThreadX part (`threadx/tests/i2c_dw/i2c_dw_test_os.c`): the I2C0 and I2C1 boundaries and devices, the DMA reads, Zephyr's I2C API, the `i2c` command; and that a wait is not ended by the layer's wake flag or another semaphore, a give right before the wait is not lost, deadlines across the tick count's wrap, a second `i2c_dw_os_sem_init()`, calls before the kernel and in an ISR. An interrupt is one Linux thread inside ThreadX's context save and restore (`threadx/tests/i2c_dw/tx_test_isr.c`) |
