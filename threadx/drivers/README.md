# Drivers and the compatibility layer (ThreadX build)

English | [日本語](README_ja.md)

The firmware of this repository is the same firmware built on four OSes, Zephyr, NuttX, Pico SDK + FreeRTOS
and Pico SDK + ThreadX, called the Zephyr, NuttX, FreeRTOS and ThreadX builds. This document describes the
inside of the ThreadX build (`threadx/`). Its use is in [`threadx/README.md`](../README.md), the list of what
each build writes in [`design/os-boundary.md`](../../design/os-boundary.md), and the design summary in the
[design document](../../design/imu-raw-mqtt-design.md). The details are in the comment at the top of each file.

The ThreadX build uses only the ThreadX kernel (Eclipse ThreadX v6.5.2.202603b_rel, its Cortex-M0 port
`ports/cortex_m0/gnu`); the start, clocks, USB, flash and DMA are the Pico SDK 2.3.1's. USBX, NetX Duo and
FileX are not used. Most of the sources are in `common/`, shared by the four builds, and the parts on the
Pico SDK (`common/pico_sdk/`, each driver's `port/pico_sdk/`, `common/compat/os_generic/`) are shared with
the FreeRTOS build. The shared sources are written on Zephyr's API, so the ThreadX build, as the FreeRTOS
build, runs them on a layer that provides that API under Zephyr's names (the "compatibility layer"). Its
kernel part is the generic implementation on an OS's primitives (`common/compat/kernel_generic/`); the
ThreadX build writes those primitives. The network is the layer's socket layer (`common/compat/net_offload/`),
the shell the shared shell engine (`common/compat/shell_engine/`). The shared parts are described in
[`freertos/drivers/README.md`](../../freertos/drivers/README.md); this document covers only ThreadX's.

## Directories

| Where | Holds |
|---|---|
| `app/usb-console/` | the build (`CMakeLists.txt`: the settings and sources given to the shared `common/pico_sdk/app/usb-console/usb-console.cmake`; `threadx_kernel.cmake`: ThreadX's kernel, port and execution profile kit as sources of the firmware, plus the board's part, the vectors and the Pico SDK's locks), the kernel configuration (`tx_user.h`), `src/`: SysTick and the exception priorities (`tx_initialize_low_level.c`), the tick (`tick.c`), priorities and time slices (`priorities_os.h`), the ThreadX parts of the shared application's boundaries (start `main_os_impl.h`, idle `idle_os.h`, fatal errors `fault_os_impl.h`, `ps` and the version `commands_os_impl.h`, generic commands `kernel_shell_os_impl.h`), each thread's run time (`run_time.[ch]`) |
| `drivers/compat/` | the layer's ThreadX part: the primitives of the generic kernel part (`zephyr_kernel_os_impl.h`), the lock's variables (`zephyr_kernel_os.c`), the board's port header (`rp2040/zephyr_os_port.h`) |
| `drivers/pico_sdk_interop/` | the waits of the Pico SDK's mutexes, semaphores and sleeps on ThreadX's threads ("The Pico SDK's locks" below) |
| `common/drivers/i2c_dw/port/threadx/` | the I2C core's waits, clock and sleeps (`i2c_dw_os_impl.h`) |
| `common/drivers/nina/port/threadx/` | the NINA core's binding to ThreadX (`nina_os.c`) and the SPI DMA's wait (`nina_bus_dma_os_impl.h`) |
| `tests/` | host tests (`compat/`, `i2c_dw/`, `nina/`) and the board test firmware (`board_kernel/`) |
| `script/` | `setup.sh` (fetches ThreadX, the Pico SDK, the toolchain, picotool, clang-format and ST's `lsm6dso_reg.c`), `build.sh`, `flash.sh`, `format.sh`, `host_tests.sh` |

`threadx/threadx`, `threadx/pico-sdk`, `threadx/toolchain`, `threadx/picotool` and `threadx/stmemsc` are
fetched by `setup.sh` and not in Git. ThreadX's sources are built unchanged.

## The layer's kernel part

The primitives and what each must do are in `common/compat/kernel_generic/zephyr_kernel_os.h` and sections
2.5 and 2.6 of `design/os-boundary.md`. The ThreadX build makes them as follows.

- **The thread's lock** (`z_os_task_lock()`): interrupts masked (PRIMASK), and at the outermost level
  ThreadX's `_tx_thread_preempt_disable` raised. The unlock lowers it with interrupts still masked,
  restores the mask, then makes the switch the services inside the lock left with
  `_tx_thread_system_preempt_check()`. Masking alone is not enough: when a service makes a more urgent
  thread ready, ThreadX's Cortex-M port pends PendSV and unmasks interrupts for an instant to switch,
  which would switch in the middle of the lock. Every service checks `_tx_thread_preempt_disable` before it
  switches (ThreadX uses it around its own sequences), so "no switch until unlock" holds. Nothing waits
  inside the lock, so one depth and one saved mask serve all threads. An ISR's lock is PRIMASK alone.
- **The wake flag and the storage after the stack**: `K_THREAD_STACK_DEFINE()` keeps 272 bytes after each
  stack (`Z_OS_THREAD_RESERVED`) for a `struct z_tx_thread`: the `TX_THREAD` (200 bytes with the execution
  profile kit's), the wake flag's `TX_SEMAPHORE` (28), the thread's function and argument (8) and the name
  array (32). The wake flag is a binary semaphore with the ceiling 1 (`tx_semaphore_ceiling_put(…, 1)`), so a
  wake before the wait is kept. The flag is in that storage, so every thread that waits through the layer
  is started by `z_os_thread_start()` (the start-up thread "main" too); a thread made with ThreadX's API
  directly cannot call `k_sem_take()` or `k_poll()`.
- **Start of a thread**: the wake flag is made, `tx_thread_create()` is called unstarted (`TX_DONT_START`),
  `*tid` is set, then `tx_thread_resume()`. The thread cannot run before `*tid` is set (at the resume at once
  if it is more urgent than the caller). Also before the scheduler.
- **End of a thread and reuse of its stack**: `z_os_thread_exit()` `tx_thread_terminate()`s the caller.
  ThreadX keeps an ended thread in its list of created threads, `TX_TERMINATED` (`TX_COMPLETED` when its
  function returned), with the mutexes it held released. The next start on the same stack makes
  `z_os_thread_start()` delete that record and the wake flag first; on the stack of a thread that has not
  ended it returns `-EBUSY`.
- **Mutexes** (`k_mutex`): one `TX_MUTEX` (52 bytes) with priority inheritance (`TX_INHERIT`), recursive for
  its owner. All-zero is unlocked; the first lock creates it (inside the lock, so two threads cannot both
  create it). Right after the creation it is taken out of ThreadX's list of created mutexes
  (`_tx_mutex_created_ptr`, `_tx_mutex_created_count`) and it is never deleted: the application zeroes
  mutexes after use (`mqtt_client_init()` zeroes the client at every connection), which would break the
  list. ThreadX then refers to such a mutex only while a thread owns it (the owner's list of owned
  mutexes) or waits on it, as Zephyr does. ThreadX does not lower the owner when a waiter times out until
  the release, so on a timeout `z_os_mutex_lock()` computes the owner's priority as ThreadX's release does
  and sets it with `_tx_mutex_priority_change()` (Zephyr's rule of `kernel/mutex.c`; the acceptance test's
  `mutex_inherit` checks it). Inheritance is one level, and at each release the owner returns to the
  highest waiter of the mutexes it still holds, or its own priority.
- **Priorities** (`app/usb-console/src/priorities_os.h`): the layer's value is 15 − the Zephyr priority
  (larger more urgent), ThreadX's priority 31 − that value, that is the Zephyr priority + 16. Zephyr's 32
  levels (−16 to 15) map one to one on ThreadX's 32 (0 to 31). Zephyr's idle 15 (ThreadX 31) is not given to
  a thread of the layer. The kernel has no thread of its own (no idle thread, no timer thread).
- **Time slices**: threads of Zephyr priority 0 and below in urgency (layer value 15 and lower) get
  ThreadX's `time_slice` of 20 ticks from `z_os_thread_start()` (the Zephyr build's
  `CONFIG_TIMESLICE_SIZE=20`, `CONFIG_TIMESLICE_PRIORITY=0`); the cooperative priorities get none. ThreadX
  keeps the rest of a preempted thread's slice and starts a new one when the slice is used up (Zephyr starts
  a new one at every switch).
- **Clocks**: `k_cycle_*()` and `k_uptime_*()` are the RP2040 TIMER (`rp2040_timer_us()`), not the tick.
- **Assertion**: `Z_OS_ASSERT()` is `app_assert_failed()` (fault record and restart). The error checking of
  ThreadX's API stays on; a bad call returns an error code and is not a fatal error.

## The tick

- SysTick at 1 ms (`TX_TIMER_TICKS_PER_SECOND` 1000), set up by `src/tx_initialize_low_level.c`
  (`_tx_initialize_low_level()`) with a period of `clk_sys / 1000`. `main_os_kernel_start()` checks that clk_sys
  is a whole number of ticks.
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
  shared `common/compat/kernel_generic/tick_catchup.h`, which pends SysTick again once per lost tick.
  SysTick is above PendSV, so the added ticks run one after another before the switch, each counted as a
  normal tick (the same test as the FreeRTOS build's). The tick never runs ahead of the TIMER.

## The kernel's start and the vectors

- The start is the shared `common/pico_sdk/app/usb-console/src/main.c`; ThreadX's part is
  `src/main_os_impl.h`. `main_os_kernel_init()` calls `_tx_initialize_kernel_setup()` (the low-level and
  ThreadX's initialization, without the scheduler) and registers the stack check's handler. `main()` then
  makes the layer's objects and threads, and `main_os_kernel_start()` calls `tx_kernel_enter()` (which skips
  the initialization done and goes to the scheduler after an empty `tx_application_define()`).
- `_tx_initialize_low_level()` masks interrupts, so they stay masked from `main_os_kernel_init()` until the
  scheduler takes its first thread. `main()` meanwhile does only what needs no interrupt and no time to pass
  (the USB stdio does not wait for the host, I2C makes no transfer, log lines go to the RAM log). The USB
  enumeration, the first ticks and the pending interrupts run once the scheduler starts. The passthrough
  mode's check and its run come before `main_os_kernel_init()`, with interrupts enabled.
- The start-up thread "main" has its stack in SCRATCH_X, not loaded, which the start-up code does not
  clear. After a watchdog restart the IDs of the last run's ThreadX objects are still there, so
  `main_os_thread_start()` zeroes the storage after the stack before it starts the thread.
- Vectors: the Pico SDK's crt0 owns the vector table and the main stack. The SDK's weak `isr_pendsv` and
  `isr_svcall` are defined in the link (`--defsym`, `threadx_kernel.cmake`) as ThreadX's `__tx_PendSVHandler`
  and `__tx_SVCallHandler`; `isr_systick` is `src/tick.c`'s. Interrupts run on the main stack (MSP, 2048
  bytes; the `(interrupts)` row of `ps`), and so do the scheduler's wait and the idle hooks.
- The kernel allocates nothing: every object's memory is the caller's, and no byte or block pool is made.

## Idle and `cpu_active`

ThreadX has no idle thread. With no thread ready the scheduler loops inside PendSV: interrupts masked,
`tx_low_power_enter()`, `WFI`, `tx_low_power_exit()`, interrupts unmasked (`TX_LOW_POWER`, `TX_ENABLE_WFI` of
`tx_user.h`). `src/idle_os.h` names those two hooks as the two halves of the shared
`common/pico_sdk/app/usb-console/src/idle.c` (`IDLE_OS_ENTER`: open the window and blink the LED;
`IDLE_OS_EXIT`: close the window). The measurement is the other builds': the handler of the interrupt that
ended the wait runs after the window, so it counts as load. The 1 ms tick interrupt counts as load too.

## Run times (the CPU column of `ps`)

ThreadX's execution profile kit (`utility/execution_profile_kit`, `TX_EXECUTION_PROFILE_ENABLE`), unchanged.
Its clock is the TIMER's low 32 bits (1 µs); the totals are 64-bit. A thread's time runs from its switch in
to its switch out, so the handlers that interrupt a thread count in it (the Pico SDK's handlers do not call
the kit's `_tx_execution_isr_enter()`; as FreeRTOS's run time stats). The time the scheduler waited is the
kit's idle time, the `(idle)` row of `ps`. `cpu_active` is not that idle time but the windows above. Read by
`src/run_time.c`, shown by `src/commands_os_impl.h`. The kit makes `TX_THREAD` 12 bytes larger.

## The Pico SDK's locks

By default the Pico SDK's `mutex_t`, `semaphore_t` and `sleep_ms()` take the core number for the owner and
wait spinning on WFE. With several threads on one core, a second thread takes a held mutex for its own
(a "deadlock"), and `stdio_usb` silently drops the output or the input. `drivers/pico_sdk_interop/` is the
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
saved stack pointer, and on a failure calls `fault_os_stack_error()` (`src/fault_os_impl.h`). The fault record
names that thread, which is not always the running one. The FreeRTOS build (`configCHECK_FOR_STACK_OVERFLOW`
2) checks at every switch; ThreadX does not check at a preemption by PendSV. ThreadX keeps the top word of
each stack for the pattern. STACK FREE of `ps` is the bytes at the bottom of the stack that still hold 0xEF.

## The drivers' waits

The I2C and NINA bindings are written on ThreadX and the Pico SDK directly, not on the layer. What they
wait on are `TX_SEMAPHORE`s the drivers own.

- ThreadX's semaphore is light (a count and a list of waiters) and an interrupt may put it (unlike
  FreeRTOS's semaphores made of queues).
- ThreadX has no wake-up of a thread's own that a driver may use (FreeRTOS's task notification indices).
  The per-thread semaphore is the layer's wake flag; sharing it would let either side end or eat the other's
  waits. Suspending and resuming the thread (`tx_thread_suspend()`, `tx_thread_resume()`) would lose a resume
  that comes before the suspension, so it is not used.
- I2C (`i2c_dw_os_impl.h`): `device_sync_sem` and `rp2040_dma_sem` from 0 with the limit `UINT_MAX`, the bus
  lock `bus_sem` from 1 with the limit 1 (no priority inheritance; at the release, with two or more waiters,
  `tx_semaphore_prioritize()` puts the most urgent first). A give is `tx_semaphore_ceiling_put()`,
  `k_sem_reset()` writes the count with interrupts masked. `i2c_dw_os_sem_init()` may be called twice before
  the kernel (when the controller did not start the first time); the second call keeps the object made and
  writes the count and the limit. Before the kernel and in an ISR ThreadX refuses a wait (`TX_WAIT_ERROR`);
  the take then tries once more without a wait.
- NINA (`nina_os.c`): the three locks are `TX_MUTEX`es with priority inheritance, the READY wait and the
  Wi-Fi thread's wake-up `TX_SEMAPHORE`s. A socket event wait makes a semaphore on its stack for the call and
  deletes it at the end (the event interrupt walks the list of waiters and wakes them all). The SPI DMA's
  wait (`nina_bus_dma_os_impl.h`) is one semaphore of the transfers (one at a time, inside
  `nina_os_lock()`). The objects are made at the first use and never deleted (they stay in the lists of
  created objects).
- Timed waits and sleeps last the time rounded up to ticks plus one tick (Zephyr's rule; ThreadX's wait of
  n ends at the current tick + n). The clock is the TIMER in ms, not the tick.

## The code in RAM

The code on the hot paths runs from RAM instead of the flash (the shared list
`common/pico_sdk/app/usb-console/ram_code.cmake` and `RAM_CODE_OS_SOURCES` of
`app/usb-console/CMakeLists.txt`). ThreadX's part is the scheduler (PendSV, the switch and the wait), the
services that the layer's waits, the drivers' waits and the Pico SDK's locks call (semaphores, mutexes, event
flags, sleeps, the suspend and resume of threads), the tick (the timer interrupt, the timeouts and the time
slices), the execution profile kit, the idle hooks, the tie of the Pico SDK's locks and the NINA core's
binding (about 7 KiB). The FreeRTOS build puts only the NINA core's binding in RAM as its OS part.

## ThreadX's internals in use

Only these places use what ThreadX's public API does not offer. When ThreadX is upgraded, check that these
names and their meanings have not changed.

| What | Where | Why |
|---|---|---|
| `_tx_thread_preempt_disable` | `drivers/compat/zephyr_kernel_os_impl.h` (`z_os_task_lock()`), `drivers/pico_sdk_interop/pico_sdk_interop.c` (whether a thread may block) | no switch inside the lock ("The thread's lock" above) |
| `_tx_thread_system_preempt_check()` | `zephyr_kernel_os_impl.h` (`z_os_task_unlock()`) | the switch the services inside the lock left, at the unlock |
| `_tx_thread_current_ptr`, `_tx_thread_system_state` | `zephyr_kernel_os_impl.h`, the port header, `fault_os_impl.h`, `commands_os_impl.h`, `pico_sdk_interop.c` | the calling thread, before the kernel, in an ISR (without a call) |
| members of `TX_THREAD` (`tx_thread_priority`, `tx_thread_state`, `tx_thread_name`, `tx_thread_stack_start`/`_end`, `tx_thread_timer`, the run times) | `zephyr_kernel_os_impl.h`, `commands_os_impl.h`, `fault_os_impl.h`, `run_time.c` | the priority, whether it ended, the name, the state and free stack of `ps` |
| `_tx_mutex_created_ptr`, `_tx_mutex_created_count` and relinking `tx_mutex_created_next`/`_previous` | `zephyr_kernel_os_impl.h` (`z_tx_mutex_create()`) | a mutex that may be zeroed is taken out of the list of created mutexes |
| members of `TX_MUTEX` and `_tx_mutex_priority_change()` | `zephyr_kernel_os_impl.h` (`z_tx_mutex_timed_out()`, `z_os_mutex_unlock()`) | the owner's priority dropped at a waiter's timeout; the owner's check |
| direct reads and writes of `tx_semaphore_count`, `tx_semaphore_suspended_count` | `common/drivers/i2c_dw/port/threadx/i2c_dw_os_impl.h` | `k_sem_reset()` and the second initialization before the kernel (ThreadX has no service that writes a count), the reordering with two or more waiters |
| `_tx_thread_created_ptr`, `_tx_thread_created_count` | `commands_os_impl.h` (`ps`) | the list of threads (read with interrupts masked) |
| `_tx_initialize_kernel_setup()` | `src/main_os_impl.h` | objects and threads made before the scheduler |
| `_tx_timer_interrupt()`, `_tx_initialize_unused_memory`, `_tx_thread_system_stack_ptr`, `__tx_PendSVHandler`/`__tx_SVCallHandler` | `src/tick.c`, `src/tx_initialize_low_level.c`, `threadx_kernel.cmake` | the port's tick and handlers (as the port's example `tx_initialize_low_level.S` sets them) |
| `_tx_execution_idle_time_total`, `tx_low_power_enter()`/`_exit()` | `src/run_time.c`, `src/idle_os.h` | the kit's idle time, the hooks of the scheduler's wait (neither declared in a ThreadX header) |

## Threads

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

The commands' syntax and the form of their output are the FreeRTOS build's. What differs:

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
thread inside ThreadX's context save and restore (`tests/i2c_dw/tx_test_isr.c`).

| Test | Covers |
|---|---|
| `zephyr_kernel_os_impl` | the ThreadX part (`drivers/compat/zephyr_kernel_os_impl.h`) compiles alone, reading no header of the layer |
| `zephyr_kernel_test` | the layer's kernel (the 22 tests of `common/tests/zephyr_kernel/zephyr_kernel_test.c`, twice; the second run reuses the first's objects and stacks). The wake from a hardware interrupt, the real-time checks and the tick's catch-up skip. The test's tick is a one-tick ThreadX timer |
| `i2c_dw` | the FreeRTOS build's test body (`freertos/tests/i2c_dw/main.c`) with the ThreadX part (`tests/i2c_dw/i2c_dw_test_os.c`): the I2C0 and I2C1 boundaries and devices, the DMA reads, Zephyr's I2C API, the `i2c` command; and that a wait is not ended by the layer's wake flag or another semaphore, a give right before the wait is not lost, deadlines across the tick count's wrap, a second `i2c_dw_os_sem_init()`, calls before the kernel and in an ISR |
| `nina` | the FreeRTOS build's test body (`freertos/tests/nina/main.c`) with `tests/nina/nina_test_os.c`: the SPI and line boundary with its DMA, the READY and event interrupts, locks, sleeps, clock; and the calls before the kernel, a Wi-Fi post before the first wait, priority inheritance across two mutexes, the windows right before a wait |

## The board test firmware (`tests/board_kernel`)

A tool to check the ThreadX part on the board, not a product firmware. On the product's start and the
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
