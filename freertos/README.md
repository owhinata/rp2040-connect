# Pico SDK + FreeRTOS build

English | [日本語](README_ja.md)

A build environment for firmware on the Raspberry Pi Pico SDK and the FreeRTOS kernel, for the Arduino
Nano RP2040 Connect. Like `zephyr/`, `nuttx/` and `openmv/`, it fetches what it needs under `freertos/`.

For now it holds one application, `app/usb-console`: a command line (CLI) on USB, a blinking LED, and
the kernel settings, CPU load measurement and fatal error handling that the port of the Zephyr build will
build on, and the FreeRTOS implementation of the Zephyr compatibility layer (`drivers/compat`), on which
the sources shared with the Zephyr and NuttX builds (`common/`) are to run. The IMU measurement of the
Zephyr and NuttX builds has not been ported yet.

| Item | Content |
|---|---|
| Pico SDK | 2.3.1 (board `arduino_nano_rp2040_connect`) |
| FreeRTOS kernel | V11.3.3 (the RP2040 port in `portable/ThirdParty/GCC/RP2040`, used on one core) |
| Compiler | Arm GNU Toolchain 15.2.Rel1 (GCC 15.2) |

## Requirements

- A Linux PC (x86_64). No sudo is needed.
- `git`, `curl` or `wget`, `tar`, `xz`, `sha256sum`
- `python3` (with the `venv` module; `python3-venv` on Ubuntu), `cmake` (3.13 or later), `make`, `gcc`, `g++` (the Pico SDK builds `picotool` for the host)
- About 1.5 GB of free disk space (the toolchain takes 1.1 GB)

## Setup

Source it from the root of the repository. It fetches only what is missing, so it can be sourced again
at any time. Source it again in a new terminal.

```bash
source freertos/script/setup.sh
```

| Place | Content | Version |
|---|---|---|
| `freertos/pico-sdk` | The Raspberry Pi Pico SDK and, of its submodules, TinyUSB (for the USB console) | tag `2.3.1` (shallow clone) |
| `freertos/FreeRTOS-Kernel` | The FreeRTOS kernel | tag `V11.3.3` (shallow clone) |
| `freertos/toolchain/` | Arm GNU Toolchain (arm-none-eabi, for x86_64 Linux) | 15.2.Rel1 (SHA-256 checked) |
| `freertos/.venv` | A Python venv (`clang-format`, used for formatting) | clang-format 18.1.3 (PyPI) |

It also sets `PICO_SDK_PATH`, `FREERTOS_KERNEL_PATH`, `PICO_TOOLCHAIN_PATH` and `PATH` (the toolchain,
`freertos/.venv/bin` and `freertos/script`) in the current shell. The versions are pinned at the top of the script. What it
fetches is not under Git.

## Build

```bash
freertos/script/build.sh [--clean] [-j N] [-v] [-DNAME=VALUE ...] [APP]
```

It sources `setup.sh` itself and builds `freertos/app/APP` (default: `usb-console`) in
`freertos/build/APP/`. The result is `freertos/build/usb-console/usb-console.uf2`. `-DNAME=VALUE` is passed
on to CMake, which keeps it in the build directory until `--clean`. The one option of `usb-console` is
`-DNANO_SELFTEST=ON` (off by default): the `selftest` command, a test feature (see "The Zephyr
compatibility layer" below).

## Host tests

```bash
freertos/script/host_tests.sh [-v]
```

Runs the tests of the shared sources (`common/tests/run.sh`), then builds the self-test of the Zephyr
compatibility layer (the same source as the `selftest` command) on the FreeRTOS POSIX port, which is in the
kernel's tree, and runs it twice (`freertos/tests/compat`). It needs `freertos/FreeRTOS-Kernel`
(`setup.sh`) and the host's `cc` with POSIX threads, no board. `-v` shows the self-test's output.

On the first build the Pico SDK fetches the source of `picotool` (the tool that makes the UF2) from
GitHub into `freertos/picotool/` and builds it for the host (this needs the network). After `--clean`,
`picotool` is built again as well.

## Flashing

```bash
freertos/script/flash.sh [--build] [--wait SEC] [UF2]
```

1. If there is an `RPI-RP2` drive (an RP2040 in BOOTSEL), the UF2 is copied to it.
2. Otherwise, if this firmware's USB console is there, the script sends it the `bootsel` command and
   waits for the drive. Nothing has to be done by hand.
3. Otherwise it explains how to enter BOOTSEL and waits for the drive.

After the copy it waits for the USB console (`/dev/serial/by-id/usb-*FreeRTOS_USB_Console*`) and prints
its path. A drive that is not mounted is mounted with `udisksctl`.

Entering BOOTSEL:

- With this firmware running: the CLI's `bootsel` command. Opening the serial port at 1200 bps and
  `picotool reboot -f -u` work too (standard features of the Pico SDK's USB console).
- With the Zephyr or NuttX build running: press the RESET button twice quickly. If that does not work,
  connect REC to GND and press RESET.
- With the OpenMV build running: run `import machine; machine.bootloader()` in the REPL.

**Before flashing**: flashing replaces the firmware of the Zephyr, NuttX or OpenMV build. This
application does not write to the end of the flash (the key-value store of the Zephyr and NuttX builds,
the file system of the OpenMV build).

## The application `app/usb-console`

The console is the USB CDC ACM port (UART0 is not used). Opening a terminal shows the prompt
`freertos:~$ `. Line editing is Backspace and Ctrl-C (drop the line) only.

| Command | Content |
|---|---|
| `help` | List the commands |
| `version` | Versions of the FreeRTOS kernel, the Pico SDK and the compiler |
| `id` | The board's unique ID (the ID of the flash, 16 hexadecimal digits) |
| `uptime` | Time since the start (the RP2040 TIMER) and the number of ticks |
| `sleep <ms>` | Sleep for ms milliseconds, rounded as in Zephyr (the Zephyr build's `kernel sleep`) |
| `devmem <addr> [<bits> [<val>]]` | Read or write memory (the syntax of the Zephyr build's `devmem`: address and value in hexadecimal, width 8, 16 or 32 bits, a value writes). An address that does not exist ends in a fatal error |
| `ps` | The tasks: state, priority, the least free stack so far (bytes), share of the CPU time since the start; then the CPU load (`cpu_active`) since the previous `ps` |
| `irqstack` | The use of the interrupt stack since the start (the NuttX build's `irqstack`) |
| `fault` | The fatal error that ended the run before this start |
| `free` | Size of the FreeRTOS heap, free space and the least free space so far |
| `reboot` | Restart |
| `bootsel` | Restart into the USB bootloader (BOOTSEL, the `RPI-RP2` drive) |
| `selftest` | Only with `-DNANO_SELFTEST=ON`: the self-test of the Zephyr compatibility layer (below) |

| File | Role |
|---|---|
| `CMakeLists.txt` | Imports the Pico SDK and the FreeRTOS kernel, the Zephyr compatibility layer (`drivers/compat/compat.cmake`) and the shared sources of `common/`; the USB product name (`FreeRTOS USB Console`); the option `NANO_SELFTEST` |
| `FreeRTOSConfig.h` | Kernel configuration |
| `src/main.c` | Start-up (also `zephyr_compat_initialize()`), the console task, the LED task |
| `src/cli.[ch]` | Line editing and the table of commands. Depends on neither the kernel nor the SDK |
| `src/commands.c` | The commands above |
| `src/priorities_os.h` | The FreeRTOS part of the table of priorities and stacks (`common/compat/priorities.h`) |
| `src/os_clock.h` | The clock since the start (TIMER), the tick count, sleeps with Zephyr's rounding |
| `src/tick.c` | The tick hook: the compatibility layer's timeouts (`zephyr_compat_tick()`) and 20 ms time slices |
| `src/idle.c` | The idle hook: the CPU load (`cpu_load_get()`, `common/compat/cpu_load.h`) |
| `src/irq_stack.c` | The high-water mark of the interrupt stack |
| `src/fault.[ch]` | The common exit of fatal errors and the record of the last one (`common/log/fault_record.[ch]`) |

### Tasks, priorities and stacks

The priorities follow the table of the port (`common/compat/priorities.h`, shared with the NuttX build),
whose FreeRTOS part is `src/priorities_os.h`: FreeRTOS priority = 20 − Zephyr priority. Zephyr's
preemptible priorities 0 to 14 become 20 to 6, the cooperative −1 becomes 21 (FreeRTOS cannot keep a
cooperative thread from being preempted by a higher one; the NuttX build makes the same approximation).
Stacks are given in bytes, as in Zephyr, and are statically allocated.

| Task | Priority | Stack | Zephyr build |
|---|---|---|---|
| `Tmr Svc` (the kernel's timer task) | 31 | 1024 B | none (see below) |
| `console` (the USB console; the commands run in it) | 6 | 3072 B | the shell thread (14, 3072 B) |
| `heartbeat` (toggles the LED on GPIO6 every 500 ms) | 1 | 1024 B | none (the Zephyr build leaves the LED alone) |
| `IDLE` (measures the CPU load) | 0 | 1024 B | idle (15, 256 B) |
| `sysworkq` (the compatibility layer's system work queue) | 21 | 1024 B | sysworkq (−1, 1024 B) |
| `RTIO WQ 0` (the compatibility layer's RTIO work queue) | 20 | 1024 B | RTIO WQ 0 (0, 1024 B) |

With `-DNANO_SELFTEST=ON`, the first `selftest` also starts `st_static_hi_tid` (7) and
`st_static_eq_tid` (6) and creates `st_workq` (6), which then wait for ever, and makes short-lived helper
tasks (`st_*`).

- `Tmr Svc` has the highest priority because the Pico SDK's locks and sleeps wake the waiting task through
  it when an interrupt handler releases the lock (the USB code's stdio mutex) or the sleep's alarm fires;
  in Zephyr the interrupt handler makes the thread ready itself.
- The interrupt stack (the main stack) is the Pico SDK's `PICO_STACK_SIZE`, 2048 bytes, the size of the
  Zephyr build's `CONFIG_ISR_STACK_SIZE`. `irqstack` shows its use in the format of the `IRQ 00` line of
  the Zephyr build's `kernel thread stacks`. The value is a high-water mark (the stack is filled with a
  pattern at the start of `main()`) and includes the frames of `main()`, which stay on that stack after
  the scheduler starts. Nothing guards the bottom of the interrupt stack.

### Kernel configuration

- One core (`configNUMBER_OF_CORES` 1), priorities 0 to 31 (a higher number is a higher priority),
  task names up to 31 characters (`configMAX_TASK_NAME_LEN` 32, Zephyr's `CONFIG_THREAD_MAX_NAME_LEN`),
  two notifications per task (`configTASK_NOTIFICATION_ARRAY_ENTRIES` 2), 32 KB of heap with `heap_4`.
  The Pico SDK's locks and sleeps block the calling task (`configSUPPORT_PICO_SYNC_INTEROP`,
  `configSUPPORT_PICO_TIME_INTEROP`).
- The tick is 100 µs (`configTICK_RATE_HZ` 10000), as in the Zephyr and NuttX builds, and 32 bits wide
  (the RP2040 port has no 64-bit tick), so it wraps after about 4.97 days. The time since the start
  comes from the RP2040 TIMER (64 bits, 1 µs), not from the tick. The tick is periodic (SysTick); the
  Zephyr and NuttX builds are tickless. `portTICK_PERIOD_MS` is 0 at this rate and is not used; the
  Pico SDK's `sleep_ms()` is not used either (it spins the last 150 µs).
- Sleeps are rounded as in Zephyr: a sleep of t lasts more than ⌈t / 100 µs⌉ ticks and at most one tick
  more (`vTaskDelay(⌈t⌉ + 1)`; 0 only yields).
- Tasks of the same priority get time slices of 20 ms, as in the Zephyr build (`CONFIG_TIMESLICE_SIZE=20`,
  `CONFIG_TIMESLICE_PRIORITY=0`): FreeRTOS's own slicing, which would switch at every tick, is off
  (`configUSE_TIME_SLICING` 0) and the tick hook asks for a switch after 200 ticks of the same task. As in
  Zephyr the slice starts again whenever another task is switched in, and a task that a higher one
  preempts keeps its turn and goes on when that one blocks (see "The Zephyr compatibility layer"). The
  slice is counted in whole ticks: a task switched in between two ticks gets (199, 200] ticks (Zephyr:
  (200, 201]).

### CPU load

The idle task measures the CPU load the way the Zephyr build (`CONFIG_CPU_LOAD` with the idle hook
backend) and the NuttX build (`up_idle()`) do: with interrupts disabled it opens a window, waits with
`wfi`, closes the window and enables interrupts, so interrupt handlers count as load even when they
interrupt the idle task. The arithmetic is `common/compat/nano_cpuload.h`; `cpu_load_get()`
(`common/compat/cpu_load.h`) is what the IMU service's `cpu_active` (`imu status`) will use. Neither the
Zephyr nor the NuttX build has a command of its own for it; here the last line of `ps` shows the load
since the previous `ps`, without resetting the window of `cpu_load_get()`:

```
cpu_active: <load>% in the <seconds> s since the previous "ps"
```

The periodic 100 µs tick wakes the CPU 10000 times a second and its handler counts as load; the Zephyr
and NuttX builds have no such interrupt.

### Fatal errors

`panic()` (also the Pico SDK's callers, `hard_assert`, out of memory in `malloc()`, a task function that
returns), the HardFault exception, a failed `configASSERT()` or `assert()`, a stack overflow
(`configCHECK_FOR_STACK_OVERFLOW` 2), an exhausted heap and `exit()` all end in one exit
(`src/fault.c`). It disables interrupts, records the reason, PC, LR, SP (for a HardFault also r0–r3,
r12 and xPSR from the exception frame), the exception number, the running task, the time and a message
in RAM that the start-up code does not clear (`__uninitialized_ram`, with a CRC-32), keeps the reason, PC
and LR in the watchdog's scratch registers 0 to 3 as well, and restarts the board with the watchdog
(as `reboot` does). The Zephyr and NuttX builds also restart after a fatal error. Nothing is written to
the console at that moment (the USB code needs interrupts and tasks).

After the restart the first terminal connection shows the record before the prompt, and `fault` shows it
again, for example (the values are those of the host test, not of the board):

```
*** The last run ended with a fatal error ("fault" shows it again):
Fatal error: HardFault
In task console, 3723.000042 s after the start
pc 0x10001234 lr 0x10005679 sp 0x20010f00
r0 0x30000000 r1 0x00000001 r2 0x00000002 r3 0x00000003
r12 0x0000000c xpsr 0x61000000
```

For an error other than a HardFault, the PC is the address the exit was called from and the LR is 0
(not known). The record is read once at the start and then cleared, so a later `reboot` or a power cycle
shows `No fatal error before this start`. The format and its check are OS-independent
(`common/log/fault_record.[ch]`, tested by `common/tests/run.sh`).

### The Zephyr compatibility layer (`freertos/drivers/compat`)

The sources shared with the Zephyr and NuttX builds use Zephyr's kernel API. On NuttX it is the
compatibility layer: its OS-independent part (`common/compat`: the headers, `zephyr_work.c`, RTIO, ...) and
the NuttX part (`nuttx/drivers/compat`). `freertos/drivers/compat` is the FreeRTOS part, the counterpart of
the NuttX one, written to the list at the top of `common/compat/zephyr/kernel.h`. The application does not
use any shared driver yet; the layer's own OS-independent sources that link at this stage
(`zephyr_work.c`, `zephyr_rtio.c`, `zephyr_rtio_workq.c`, `zephyr_mem_blocks.c`) and
`common/log/log_ring.c` are built in.

| File | Role |
|---|---|
| `zephyr_os.h` | The OS part of `<zephyr/kernel.h>`: the tick (`Z_USEC_PER_TICK`), `struct k_mutex`, `struct k_sem`, the timer of a delayable work item, `k_tid_t` (the task handle), `K_PRIO_PREEMPT()`, `k_current_get()`, `k_cycle_get_32/64()` (the TIMER) |
| `zephyr_os_spinlock.h`, `zephyr_os_atomic.h` | `k_spin_lock()`; nothing for the atomics (no FreeRTOS or Pico SDK header uses the names) |
| `zephyr_kernel.c` | Semaphores, `k_poll()`, threads (`K_THREAD_DEFINE()`, work queue threads, `k_thread_name_set()`), timeouts, time and sleeps, `zephyr_compat_initialize()`, `zephyr_compat_tick()` |
| `zephyr_mutex.c` | `k_mutex` |
| `zephyr_rtio_workq_os.h` | The thread `RTIO WQ 0` |
| `zephyr/sys/__assert.h` | `__ASSERT()`: nothing, as in the Zephyr build (`CONFIG_ASSERT` off) |
| `common_prelude.h` | Included before each source of `common/` (`-include`); empty so far (the Kconfig values come with the sources that need them) |
| `zephyr_compat_os.h` | `zephyr_compat_tick()`, for the application's tick hook |
| `zephyr_os_internal.h` | What the files of the layer share |
| `zephyr_os_sched.h` | Included at the end of `FreeRTOSConfig.h`: Zephyr's choice among the tasks of one priority (trace macros of the kernel's switch) |
| `rp2040/zephyr_os_port.h` | The board's few dependencies (in-interrupt test, the switch at the end of an interrupt, the task lock, busy wait, the clock from `os_clock.h`); the host test has its own |
| `compat.cmake` | `freertos_zephyr_compat(<target> [SELFTEST])`: the sources, include paths and prelude |
| `zephyr_selftest.[ch]`, `rp2040/zephyr_selftest_port.c` | The self-test (`selftest`, host test) |
| `../include/app_log_os.h`, `../log/app_log_min.c` | The smallest receiver of `LOG_*()` (`common/log/app_log.h`): lines in Zephyr's format in a ring in ordinary RAM that nothing reads yet; the RAM log and `dmesg` replace it |

What it provides: `k_mutex` (`K_MUTEX_DEFINE()`), `k_sem` (`K_SEM_DEFINE()`), `k_poll()` on semaphores
(`K_POLL_MODE_NOTIFY_ONLY`), the timeouts (`K_NO_WAIT`, `K_FOREVER`, `K_TICKS()`, `K_MSEC()`, `K_SECONDS()`),
`k_sleep()`, `k_msleep()`, `k_usleep()`, `k_busy_wait()`, `k_uptime_get()`, `k_cycle_get_32/64()`,
`K_THREAD_DEFINE()` (started with `K_THREAD_DEFINE_START()`), `k_thread_name_set()`, `k_current_get()`,
work items and work queues (`zephyr_work.c`) with the system work queue, `k_spin_lock()` and the atomics.
These are what the shared sources use. Not provided, because no shared source uses them:
`k_thread_create()`, `k_yield()`, `k_wakeup()`, `k_timer`, `k_msgq`, `k_queue`, `k_fifo`, `k_condvar`,
`k_event`, `k_poll_signal`, `sys_timepoint_*()`, `k_uptime_ticks()`, `irq_lock()`, `k_is_in_isr()`.

How Zephyr's assumptions are kept:

- A `k_mutex` left all zero is unlocked (Zephyr's MQTT library relies on it). FreeRTOS has no static
  initializer for a mutex, so a FreeRTOS recursive mutex is created in the `k_mutex`'s own memory by its
  first lock or unlock (`K_MUTEX_DEFINE()` works the same way). The results are Zephyr's: `K_NO_WAIT` on
  a held mutex `-EBUSY`, a timeout `-EAGAIN`, unlocking a free mutex `-EINVAL`, another thread's `-EPERM`.
- `k_sem` and `k_poll()` follow Zephyr's `kernel/sem.c` and `kernel/poll.c` on a count of the layer (not a
  FreeRTOS semaphore), so `K_SEM_DEFINE()` is a static initializer: a give hands the count to the waiter
  of the highest priority (first come first served within one), or adds one up to the limit and signals
  the first `k_poll()` event (by the priority of its thread); `k_sem_reset()` ends the waits with
  `-EAGAIN`. A waiting task sleeps on its task notification 1 (index 0 is left to the Pico SDK).
- `k_sem_give()` (and so `k_work_submit()`) works in interrupt handlers: the woken task runs right after
  the handler (PendSV), without going through the timer task `Tmr Svc`.
- `k_spin_lock()`: no other task runs until the unlock, as in Zephyr. In a task it is the kernel's
  critical section (`taskENTER_CRITICAL()`; with PRIMASK only, the FreeRTOS calls made inside would enable
  interrupts again), in an interrupt handler `taskENTER_CRITICAL_FROM_ISR()`. A task must not call it
  with interrupts disabled by other means.
- Timeouts: a relative wait of n ticks (`k_sem_take()`, `k_mutex_lock()`, `k_poll()`, the sleeps) is a
  FreeRTOS block time of n + 1, so it ends after more than n and at most n + 1 ticks, as in Zephyr;
  `K_NO_WAIT` does not wait (a sleep yields), `K_FOREVER` waits for ever. A wait is at most
  `portMAX_DELAY − 1` ticks (about 4.97 days); longer sleeps are made of several.
- The timers of delayable work items are a list of the layer that the tick hook runs in the tick
  interrupt (`zephyr_compat_tick()`), as Zephyr runs its timeouts in the system clock interrupt; not
  FreeRTOS's software timers, whose start and stop are messages to the timer task (`zephyr_work.c` relies
  on a timer that cannot run while interrupts are masked). They count tick interrupts, so they fall
  behind, as the tick does, when interrupts stay masked for longer than a tick.
- `K_THREAD_DEFINE()` threads are tasks on their static stacks, started by the application with
  `K_THREAD_DEFINE_START()`; the task handle is stored in the `k_tid_t` variable before the task can run
  (the scheduler is suspended around the creation). The task's control block comes from the FreeRTOS heap
  and stays allocated if the entry function returns (the task is deleted).
- `k_uptime_get()` and the cycle counter come from the TIMER, not from the tick.
- Among the tasks of one priority Zephyr's rule holds (kernel/sched.c `next_up()`): a task that a higher
  one preempts stays at its place and goes on when the higher one blocks; only a yield
  (`k_sleep(K_NO_WAIT)`, the work queue's yield, the `sleep 0` command) or the end of its time slice
  gives the turn to the next one, and the slice starts again when another task is switched in
  (`z_time_slice_reset()`). FreeRTOS by itself hands the CPU to the next task of the priority at every
  switch (`listGET_OWNER_OF_NEXT_ENTRY()`), so a task preempted for a moment (the timer task, which every
  USB interrupt wakes through the Pico SDK's stdio mutex, a work queue) would lose its turn.
  `zephyr_os_sched.h` gives `FreeRTOSConfig.h` two trace macros that run inside the kernel's switch: at a
  switch-out without a yield, a task that is still ready has its priority's list index set back one
  entry, so that it is picked again; at a switch-in of another task, the slice starts again. They name the
  kernel's internal variables (V11.3.3, one core); the kernel's source is unchanged. What is left as
  FreeRTOS does it: a task made ready is put behind the list index, which may put it before tasks that
  waited longer (it takes three or more ready tasks of one priority), a task whose priority drops at the
  end of priority inheritance keeps its turn (Zephyr queues it at the end), and a plain FreeRTOS
  `taskYIELD()` counts as a preemption.

Where FreeRTOS differs (approximations):

- Priority inheritance is FreeRTOS's: one level (Zephyr follows a chain of holders), and the holder drops
  back only when it has released every mutex it holds; a waiter that times out lowers it again.
- Cooperative threads (negative Zephyr priorities) get higher priorities (21 and up) and can be preempted
  by higher ones; Zephyr does not preempt them at all (the NuttX build makes the same approximation).
- `k_tid_t` is the FreeRTOS task handle, a variable that `K_THREAD_DEFINE()` defines and the start sets,
  as on NuttX, not Zephyr's constant (so `common/imu/wifi_profile.c` needs a `wifi_profile_os.h` without
  `const`, as the NuttX build's).
- The waiters of a semaphore are ordered by their priority when they start waiting (Zephyr reorders a
  waiter whose priority changes).

The self-test (`zephyr_selftest.c`) checks these meanings: the all-zero mutex, recursion and the results,
the timeout, priority inheritance (also after a timeout), the semaphore's limit, timeouts, the order of
the waiters and reset, the sleeps and their rounding, the clock, `k_sem_give()` and `k_work_submit()` from
the tick interrupt and (on the board) from a TIMER alarm interrupt, `k_poll()` (one event, timeouts, the
order of the pollers), delayable work items, the spin lock, the atomics, the start of `K_THREAD_DEFINE()`
threads (once per start of the board), thread names, the 20 ms time slices, and that a task preempted by a
higher one keeps its turn (`timeslice_preempt`: two spinning tasks at one priority and a higher one that
wakes every 4 ms; the first spinner has the CPU throughout). On the board it also checks the measured
times with the TIMER; on the host only the tick counts (the POSIX port's tick is a thread's `usleep()`).
The `selftest` command prints the lines of each test after it, then waits until the USB console has sent
them (the USB interrupts and the timer task that the sending wakes would otherwise land in the next
measurement), ends with `selftest: N of 22 tests failed` and returns an error if N is not 0. It runs for
about a second; twice for 140 ms two tasks at priority 7 spin and every lower task waits.

### Other notes

- The console task is woken by the notification of received USB data (the Pico SDK's
  `stdio_set_chars_available_callback()`), and every 50 ms to notice a terminal connecting.
- The USB product name is `FreeRTOS USB Console`. The Pico SDK cuts USB strings to 19 characters by
  default, so the limit (`USBD_DESC_STR_MAX`) is set to 32.
- The CLI is this repository's own, not FreeRTOS-Plus-CLI.

## What has been checked

Checked on 2026-11-16, in the build and on the board (Arduino Nano RP2040 Connect).

- The build completes without warnings (GCC 15.2), and `freertos/script/format.sh --check` reports no
  difference.
- `flash.sh` copies the UF2 to the `RPI-RP2` drive and finds the USB console after the restart.
- With the firmware running, `flash.sh` sends `bootsel` and flashes without anything done by hand
  (twice). `bootsel` typed in a terminal enters BOOTSEL too (once).
- The output of `help`, `version`, `id`, `uptime`, `ps` and `free`, and the error for an unknown command.

Not checked: `reboot`, entering BOOTSEL by opening the port at 1200 bps or with `picotool`, and long runs.

Checked on the board on 2026-11-17:

- The tick rate: 231543 ticks in 23.15 s (10000.0/s).
- The round trip of `sleep` (including about 0.9 ms of USB round trip, the median of 9): `sleep 1` 1.99 ms,
  `sleep 10` 11.10 ms, `sleep 100` 101.16 ms, `sleep 1000` 1001.21 ms.
- `cpu_active` when idle: 2.0% over about 20 s (before the compatibility layer), 3.0% (the default build
  with it).
- `irqstack`: 468 / 2048 bytes.
- The fatal error record: `devmem 30000000` ends in a HardFault, the board restarts by itself, and the
  banner and `fault` show the task (`console`) and the PC (the load instruction of `devmem`). After the next
  `reboot` there is no record. Reading and writing with `devmem`.
- The self-test of the Zephyr compatibility layer (`selftest` in a build with `-DNANO_SELFTEST=ON`): all 22
  tests pass, four times in a row. `ps` shows `sysworkq` (21) and `RTIO WQ 0` (20). The free heap does not
  keep shrinking over repeated runs.
- The default build has no `selftest` command.

The Zephyr compatibility layer builds without warnings with and without `-DNANO_SELFTEST=ON`, and the
self-test also passes on the FreeRTOS POSIX port (`host_tests.sh`).

Not checked: the fatal error record from a stack overflow, an assertion or an exhausted heap; long runs.
