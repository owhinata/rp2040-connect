# The ThreadX port

English | [日本語](threadx-port-ja.md)

The firmware of this repository is the same firmware built on four OSes, Zephyr, NuttX, Pico SDK + FreeRTOS
and Pico SDK + ThreadX, called the Zephyr, NuttX, FreeRTOS and ThreadX builds. This document describes the
inside of the ThreadX build's port (`threadx/`). Its use is in [`threadx/README.md`](../threadx/README.md), the list of what
each build writes in [`os-boundary.md`](os-boundary.md), and the design summary in §9.5 of the
[design document](imu-raw-mqtt-design.md). The details are in the comment at the top of each file.

The ThreadX build uses only the ThreadX kernel (Eclipse ThreadX v6.5.2.202603b_rel, its Cortex-M0 port
`ports/cortex_m0/gnu`); the start, clocks, USB, flash and DMA are the Pico SDK 2.3.1's. USBX, NetX Duo and
FileX are not used. Most of the sources are in `common/`, shared by the four builds, and the parts on the
Pico SDK (the `port/pico_sdk/` directories of the modules of `common/`, and their `port/generic/`) are
shared with the FreeRTOS build. The shared sources are written on Zephyr's API, so the ThreadX build, as the FreeRTOS
build, runs them on a layer that provides that API under Zephyr's names (the "compatibility layer"). Its
kernel part is the generic implementation on an OS's primitives (`common/compat/kernel_generic/`); the
ThreadX build writes those primitives. The network is the layer's socket layer (`common/compat/net_offload/`),
the shell the shared shell engine (`common/compat/shell_engine/`). The parts shared by the OSes on the Pico SDK
(the steps of the start, the idle measurement, fatal errors, the key-value store's flash, the code in RAM) are
described in ["Shared by the OSes on the Pico SDK" of `freertos-port.md`](freertos-port.md#shared-by-the-oses-on-the-pico-sdk);
this document covers only ThreadX's.

## Directories

Below, `src/` is `threadx/app/usb-console/src/`.

| Where | Holds |
|---|---|
| `threadx/app/usb-console/` | the build (`CMakeLists.txt`: the settings and sources given to the shared `common/imu/port/pico_sdk/usb-console.cmake`, `RAM_CODE_OS_SOURCES`; `threadx_kernel.cmake`: ThreadX's kernel, port and execution profile kit as sources of the firmware, plus the board's part, the vectors and the Pico SDK's locks), the kernel configuration (`tx_user.h`), `src/`: the entry of the start (`main.c`: `main()`, the kernel's set-up, the thread "main", the scheduler), SysTick and the exception priorities (`tx_initialize_low_level.c`), the tick (`tick.c`), the ThreadX parts of the shared application's boundaries (idle `idle_os.h`, fatal errors `fault_os.c`, `ps` and the version `commands_os.c`), each thread's run time (`run_time.[ch]`) |
| `common/compat/port/threadx/` | the layer's ThreadX part: the primitives of the generic kernel part (`zephyr_kernel_os_impl.h`), the lock's variables (`zephyr_kernel_os.c`), priorities and time slices (`priorities_os.h`), the board's port header (`rp2040/zephyr_os_port.h`) |
| `common/shell/port/threadx/` | the OS part of the generic commands (`kernel_shell_os_impl.h`) |
| `threadx/drivers/pico_sdk_interop/` | the waits of the Pico SDK's mutexes, semaphores and sleeps on ThreadX's threads (`pico_sdk_interop.[ch]`, `pico_sdk_interop.cmake`; "The Pico SDK's locks" below) |
| `common/drivers/i2c_dw/port/threadx/` | the I2C core's waits, clock and sleeps (`i2c_dw_os_impl.h`) |
| `common/drivers/nina/port/threadx/` | the NINA core's binding to ThreadX (`nina_os.c`) and the SPI DMA's wait (`nina_bus_dma_os_impl.h`) |
| `threadx/tests/` | host tests (`compat/`, `i2c_dw/`, `nina/`) and the board test firmware (`board_kernel/`) |
| `threadx/script/` | `setup.sh` (fetches ThreadX, the Pico SDK, the toolchain, picotool, clang-format and ST's `lsm6dso_reg.c`), `build.sh`, `flash.sh`, `format.sh`, `host_tests.sh` |

`threadx/threadx`, `threadx/pico-sdk`, `threadx/toolchain`, `threadx/picotool` and `threadx/stmemsc` are
fetched by `setup.sh` and not in Git. ThreadX's sources are built unchanged.

## The layer's kernel part and the drivers' waits

The primitives of the layer's kernel part (the thread's lock, the wake flag and the storage after the stack,
the start and end of a thread, mutexes, priorities and time slices, clocks, assertion) are in
`common/compat/port/threadx/zephyr_kernel_os_impl.h`; how they are made is in
[`common/compat/README.md`](../common/compat/README.md). The I2C and NINA bindings are written on ThreadX and
the Pico SDK directly, not on the layer, and wait on `TX_SEMAPHORE`s the drivers own
([`common/drivers/i2c_dw/README.md`](../common/drivers/i2c_dw/README.md),
[`common/drivers/nina/README.md`](../common/drivers/nina/README.md)).

- ThreadX's semaphore is light (a count and a list of waiters) and an interrupt may put it (unlike FreeRTOS's
  semaphores made of queues).
- ThreadX has no wake-up of a thread's own that a driver may use (FreeRTOS's task notification indices). The
  per-thread semaphore is the layer's wake flag; sharing it would let either side end or eat the other's waits.
  Suspending and resuming the thread (`tx_thread_suspend()`, `tx_thread_resume()`) would lose a resume that comes
  before the suspension, so it is not used.

## The tick

- SysTick at 1 ms (`TX_TIMER_TICKS_PER_SECOND` 1000), set up by `src/tx_initialize_low_level.c`
  (`_tx_initialize_low_level()`) with a period of `clk_sys / 1000`. `main()` in `src/main.c` checks that clk_sys
  is a whole number of ticks before it starts the scheduler.
- Exception priorities: PendSV and SVCall 0xC0 (the lowest), SysTick 0x80 (the Pico SDK's default for the
  device interrupts). ThreadX's Cortex-M0 scheduler waits inside the PendSV handler while no thread is
  ready, and only an interrupt above PendSV ends that wait: with SysTick at PendSV's level, no tick would
  come while every thread sleeps, and nothing would wake. An interrupt at 0xC0 would not run during the idle
  wait, so every device interrupt is at 0x80.
- `isr_systick` (`src/tick.c`) reads the TIMER first, then runs ThreadX's tick (`_tx_timer_interrupt()`: the
  clock, the time slice, and with `TX_TIMER_PROCESS_IN_ISR` the sleeps and timeouts due), the catch-up of
  lost ticks and the layer's timeouts (`zephyr_compat_tick()`). There is no timer thread. The switch to a
  woken thread is PendSV after the handler.
- Catch-up: ticks lost while interrupts are masked (a flash erase) are found against the TIMER by the
  shared `common/compat/include/tick_catchup.h`, which pends SysTick again once per lost tick.
  SysTick is above PendSV, so the added ticks run one after another before the switch, each counted as a
  normal tick (the same test as the FreeRTOS build's). The tick never runs ahead of the TIMER.

## The kernel's start and the vectors

- The entry is `main()` in `src/main.c`. It calls the shared steps
  (["Start-up" of `freertos-port.md`](freertos-port.md#start-up)) in order, with ThreadX's part between them:
  1. `usb_console_early_init()`: the RAM log, the passthrough mode's check and its run (interrupts
     enabled), the last run's fatal error record;
  2. `_tx_initialize_kernel_setup()` (the low-level and ThreadX's initialization, without the scheduler)
     and the stack check's handler;
  3. `usb_console_init()`: the layer's objects and threads, I2C, the key-value area's bounds;
  4. `z_os_thread_start()`: the thread "main" (its body `usb_console_main_thread()`, then
     `z_os_thread_exit()`);
  5. `usb_console_shell_start()`: the thread "console";
  6. `tx_kernel_enter()` (which skips the initialization done and goes to the scheduler after an empty
     `tx_application_define()`).
- `_tx_initialize_low_level()` masks interrupts, so they stay masked from 2 until the scheduler takes its
  first thread. 3 to 5 do only what needs no interrupt and no time to pass (the USB stdio does not wait for
  the host, I2C makes no transfer, log lines go to the RAM log). The USB enumeration, the first ticks and
  the pending interrupts run once the scheduler starts.
- The start-up thread "main" has its stack in SCRATCH_X, not loaded, which the start-up code does not
  clear. After a watchdog restart the IDs of the last run's ThreadX objects are still there, so
  `main()` zeroes the storage after the stack before it starts the thread.
- Vectors: the Pico SDK's crt0 owns the vector table and the main stack. The SDK's weak `isr_pendsv` and
  `isr_svcall` are defined in the link (`--defsym`, `threadx_kernel.cmake`) as ThreadX's `__tx_PendSVHandler`
  and `__tx_SVCallHandler`; `isr_systick` is `src/tick.c`'s. Interrupts run on the main stack (MSP, 2048
  bytes; the `(interrupts)` row of `ps`), and so do the scheduler's wait and the idle hooks.
- The kernel allocates nothing: every object's memory is the caller's, and no byte or block pool is made.

## Idle and `cpu_active`

ThreadX has no idle thread. With no thread ready the scheduler loops inside PendSV: interrupts masked,
`tx_low_power_enter()`, `WFI`, `tx_low_power_exit()`, interrupts unmasked (`TX_LOW_POWER`, `TX_ENABLE_WFI` of
`tx_user.h`). `src/idle_os.h` names those two hooks as the two halves of the shared
`common/imu/port/pico_sdk/idle.c` (`IDLE_OS_ENTER`: open the window and blink the LED;
`IDLE_OS_EXIT`: close the window). The measurement is the other builds': the handler of the interrupt that
ended the wait runs after the window, so it counts as load. The 1 ms tick interrupt counts as load too.

## Run times (the CPU column of `ps`)

ThreadX's execution profile kit (`utility/execution_profile_kit`, `TX_EXECUTION_PROFILE_ENABLE`), unchanged.
Its clock is the TIMER's low 32 bits (1 µs); the totals are 64-bit. A thread's time runs from its switch in
to its switch out, so the handlers that interrupt a thread count in it (the Pico SDK's handlers do not call
the kit's `_tx_execution_isr_enter()`; as FreeRTOS's run time stats). The time the scheduler waited is the
kit's idle time, the `(idle)` row of `ps`. `cpu_active` is not that idle time but the windows above. Read by
`src/run_time.c`, shown by `src/commands_os.c`. The kit makes `TX_THREAD` 12 bytes larger.

## The Pico SDK's locks

By default the Pico SDK's `mutex_t`, `semaphore_t` and `sleep_ms()` take the core number for the owner and
wait spinning on WFE. With several threads on one core, a second thread takes a held mutex for its own
(a "deadlock"), and `stdio_usb` silently drops the output or the input. `threadx/drivers/pico_sdk_interop/` is the
SDK's RTOS adapter header (`PICO_CONFIG_RTOS_ADAPTER_HEADER`, read by every source) that overrides the hooks
of `pico/lock_core.h`, as FreeRTOS's RP2040 port does with `configSUPPORT_PICO_SYNC_INTEROP`.

- The owner ID is the calling thread's `TX_THREAD` (twice the exception number in an interrupt, 0 before
  the scheduler).
- A wait unlocks the spin lock, then gets a bit (the spin lock's number) of one `TX_EVENT_FLAGS_GROUP` with
  `TX_OR_CLEAR`. A notify sets the bit and wakes every thread waiting for it. The group is made by the first
  wait.
- Where a thread cannot wait as a thread (before the scheduler, in an interrupt, with interrupts masked by
  the caller, inside the layer's lock), the SDK's own WFE stays. `sleep_ms()` waits on the SDK's sleep
  notifier and spins the last 150 µs (`PICO_TIME_SLEEP_OVERHEAD_ADJUST_US`).
- `malloc()` is serialized by an SDK mutex too (`PICO_USE_MALLOC_MUTEX`). `flash_safe_execute()` only masks
  interrupts and needs nothing.

## Stack checking

`TX_ENABLE_STACK_CHECKING`: when a thread suspends itself (a wait, a sleep, a relinquish) and when it is
resumed, ThreadX checks the 0xEF fill of the lowest word of its stack and of the word above its top, and its
saved stack pointer, and on a failure calls `fault_os_stack_error()` (`src/fault_os.c`). The fault record
names that thread, which is not always the running one. The FreeRTOS build (`configCHECK_FOR_STACK_OVERFLOW`
2) checks at every switch; ThreadX does not check at a preemption by PendSV. ThreadX keeps the top word of
each stack for the pattern. STACK FREE of `ps` is the bytes at the bottom of the stack that still hold 0xEF.

## The code in RAM

The code on the hot paths (the drivers and the application on the paths of the sensor's reads and of the send)
runs from RAM instead of the flash (the shared list `common/imu/port/pico_sdk/ram_code.cmake`;
["Code in RAM" of `freertos-port.md`](freertos-port.md#code-in-ram)). The OS
part adds only the NINA core's binding, as in the FreeRTOS build (`RAM_CODE_OS_SOURCES` of
`threadx/app/usb-console/CMakeLists.txt`: `common/drivers/nina/port/threadx/nina_os.c`). ThreadX's kernel stays in the flash.

## ThreadX's internals in use

Only these places use what ThreadX's public API does not offer. When ThreadX is upgraded, check that these
names and their meanings have not changed. The layer's ThreadX part (`common/compat/port/threadx/`) and the
drivers' bindings are listed here too.

| What | Where | Why |
|---|---|---|
| `_tx_thread_preempt_disable` | `common/compat/port/threadx/zephyr_kernel_os_impl.h` (`z_os_task_lock()`), `threadx/drivers/pico_sdk_interop/pico_sdk_interop.c` (whether a thread may block) | no switch inside the lock (the thread's lock in [`common/compat/README.md`](../common/compat/README.md)) |
| `_tx_thread_system_preempt_check()` | `zephyr_kernel_os_impl.h` (`z_os_task_unlock()`) | the switch the services inside the lock left, at the unlock |
| `_tx_thread_current_ptr` | `zephyr_kernel_os_impl.h`, `src/fault_os.c`, `src/commands_os.c`, `pico_sdk_interop.c` | the calling thread (without a call) |
| `_tx_thread_system_state` | `zephyr_kernel_os_impl.h` (`z_os_pre_kernel()`) | before the kernel (without a call; whether in an ISR, the port header `common/compat/port/threadx/rp2040/zephyr_os_port.h` reads from IPSR) |
| members of `TX_THREAD` (`tx_thread_priority`, `tx_thread_state`, `tx_thread_name`, `tx_thread_stack_start`/`_end`, `tx_thread_timer`, the run times) | `zephyr_kernel_os_impl.h`, `src/commands_os.c`, `src/fault_os.c`, `src/run_time.c` | the priority, whether it ended, the name, the state and free stack of `ps` |
| `_tx_mutex_created_ptr`, `_tx_mutex_created_count` and relinking `tx_mutex_created_next`/`_previous` | `zephyr_kernel_os_impl.h` (`z_tx_mutex_create()`) | a mutex that may be zeroed is taken out of the list of created mutexes |
| members of `TX_MUTEX` and `_tx_mutex_priority_change()` | `zephyr_kernel_os_impl.h` (`z_tx_mutex_timed_out()`, `z_os_mutex_unlock()`) | the owner's priority dropped at a waiter's timeout; the owner's check |
| direct reads and writes of `tx_semaphore_count`, `tx_semaphore_suspended_count` | `common/drivers/i2c_dw/port/threadx/i2c_dw_os_impl.h` | `k_sem_reset()` and the second initialization before the kernel (ThreadX has no service that writes a count), the reordering with two or more waiters |
| `_tx_thread_created_ptr`, `_tx_thread_created_count` | `src/commands_os.c` (`ps`) | the list of threads (read with interrupts masked) |
| `_tx_initialize_kernel_setup()` | `src/main.c` | objects and threads made before the scheduler |
| `_tx_timer_interrupt()`, `_tx_initialize_unused_memory`, `_tx_thread_system_stack_ptr`, `__tx_PendSVHandler`/`__tx_SVCallHandler` | `src/tick.c`, `src/tx_initialize_low_level.c`, `threadx/app/usb-console/threadx_kernel.cmake` | the port's tick and handlers (as the port's example `tx_initialize_low_level.S` sets them) |
| `_tx_execution_idle_time_total`, `tx_low_power_enter()`/`_exit()` | `src/run_time.c`, `src/idle_os.h` | the kit's idle time, the hooks of the scheduler's wait (neither declared in a ThreadX header) |

## Threads

The priorities are ThreadX's numbers (the Zephyr priority + 16, 0 the most urgent;
`common/compat/port/threadx/priorities_os.h`). The kernel has no thread of its own (no idle thread, no timer
thread).

| Thread | Priority (ThreadX, Zephyr) | Stack | Role |
|---|---|---|---|
| `net_mgmt`, `sysworkq` | 15 (−1) | 2048, 384 | network events, the system work queue |
| `main` | 16 (0) | 2048 | the second half of the start; once ended, not shown by `ps`; stack in SCRATCH_X |
| `sensor_shell_tid`, `RTIO WQ 0` | 16 (0) | 2048, 1024 | `sensor` output, RTIO work |
| `nina_wifi_tid`, `wifi_reconnect_tid` | 24 (8) | 2048, 2048 | Wi-Fi scan, connect and watch; automatic reconnection |
| `time_sync_tid`, `imu_stream_tid` | 25 (9) | 2048, 2048 | time synchronization, acquisition |
| `imu_fifo_worker` | 26 (10) | 2048 | FIFO reads |
| `imu_raw_mqtt_tid` | 27 (11) | 2048 | MQTT transport |
| `console`, `shell_telnet`, `telnet_service`, `zperf_server_tid` | 30 (14) | 3072, 3072, 2048, 2048 | the USB and Telnet shells, the Telnet listener, the `zperf` server |

The stacks of the layer's threads are, for now, the values measured on the FreeRTOS build. The sizes do not
include the 272 bytes after each stack.

## Differences from the FreeRTOS build

The commands' syntax and the form of their output are the FreeRTOS build's. What differs (the table with the
Zephyr and NuttX builds is ["Differences from the other builds" of `freertos-port.md`](freertos-port.md#differences-from-the-other-builds)):

| Point | FreeRTOS build | ThreadX build |
|---|---|---|
| Prompt, USB product name | `freertos:~$ `, "FreeRTOS USB Console" | `threadx:~$ `, "ThreadX USB Console" |
| `version` | `FreeRTOS kernel <version>, …` | `ThreadX kernel 6.5.2.202603b, …` |
| PRIO of `ps` | 20 − Zephyr (larger more urgent) | Zephyr + 16 (0 most urgent) |
| Rows of `ps` | the tasks `Tmr Svc` and `IDLE` | no kernel thread; the scheduler's wait as `(idle)`; the ended "main" left out |
| Same-priority choice | round robin at every switch, time slices in the tick hook | first come, a preempted thread stays first, ThreadX's time slices |
| When an owner raised by priority inheritance drops back | when it has released all its mutexes | at each release, to the highest waiter of the mutexes it still holds |
| Stack check | at every switch | when a thread suspends itself and when it is resumed |
| A post to the Wi-Fi thread before its first wait | not kept | kept (as in the Zephyr build) |
| Divider | the port saves its 4 words at every switch | the port saves nothing; the Pico SDK's division routines save and restore a division in progress (DIRTY) themselves |

## Host tests

`threadx/script/host_tests.sh [-v]` runs the shared sources' tests (`common/tests/run.sh`), then the following
on ThreadX's Linux port (`ports/linux/gnu` in ThreadX's tree). No board is needed. An interrupt is one Linux
thread inside ThreadX's context save and restore (`threadx/tests/i2c_dw/tx_test_isr.c`).

| Test | Covers |
|---|---|
| `zephyr_kernel_os_impl` | the ThreadX part (`common/compat/port/threadx/zephyr_kernel_os_impl.h`) compiles alone, reading no header of the layer |
| `zephyr_kernel_test` | the layer's kernel (the 22 tests of `common/tests/zephyr_kernel/zephyr_kernel_test.c`, twice; the second run reuses the first's objects and stacks). The wake from a hardware interrupt, the real-time checks and the tick's catch-up skip. The test's tick is a one-tick ThreadX timer. The ThreadX part is `threadx/tests/compat/` |
| `i2c_dw` | the FreeRTOS build's test body (`freertos/tests/i2c_dw/main.c`) with the ThreadX part (`threadx/tests/i2c_dw/i2c_dw_test_os.c`). The items are in [the DesignWare I2C core](../common/drivers/i2c_dw/README.md#tests) |
| `nina` | the FreeRTOS build's test body (`freertos/tests/nina/main.c`) with `threadx/tests/nina/nina_test_os.c`. The items are in [the NINA-W102 driver](../common/drivers/nina/README.md#tests) |

## The board test firmware (`tests/board_kernel`)

A tool to check the ThreadX part on the board (`threadx/tests/board_kernel/`), not a product firmware. On the product's start and the
board's part (`tx_user.h`, the low-level initialization, the tick) it runs:

- the 22 tests of the layer's kernel with the board's platform part (`test_port.c`: a TIMER alarm's
  interrupt, real time, interrupts masked for a while); the tests the host skips run too;
- the board's direct checks (`lock_test.c`): the ticks counted in 1 s of the TIMER, that a wake inside the
  lock does not run until the unlock (nested too), and that an interrupt due inside the lock runs at the
  unlock and the thread it wakes runs before the unlock returns;
- idle and run times (`idle_test.c`): `cpu_load_get()` while idle and while spinning, the count of
  `tx_low_power_enter()`, the run times of two threads;
- the Pico SDK's locks (`sdk_test.c`): the SDK's waits before the kernel, a lower priority running while a
  thread waits on an SDK mutex, the timeouts, `sleep_ms()`, `printf()` from two threads at once.

Use:

```bash
threadx/script/build.sh board_kernel          # threadx/build/board_kernel/board_kernel.uf2
threadx/script/flash.sh threadx/build/board_kernel/board_kernel.uf2
```

While a product firmware runs, the flash script enters BOOTSEL with its `bootsel` command. The test
firmware has no `bootsel` command: to put a board that runs it into BOOTSEL, open its USB port at 1200 bps
(a feature of the Pico SDK's `stdio_usb`), then write once `RPI-RP2` appears. Its USB product name is
"ThreadX kernel test". Once the host opens the port (DTR) it prints the ThreadX version, the tick measured
against the TIMER, the lines of each check and `board_test: <n> passed, <m> failed`, and repeats the summary
every 5 s. Without USB the LED (GPIO6) shows the state: on (starting), 1 Hz (waiting for the host), 5 Hz
(running), one short flash every 2 s (all passed), three every 2 s (a failure), 1 s on and 1 s off (an
assertion or a HardFault).
