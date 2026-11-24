# SPDX-License-Identifier: MIT
# The USB console on the Raspberry Pi Pico SDK and the Eclipse ThreadX
# kernel, for the Arduino Nano RP2040 Connect.  Built by
# threadx/script/build.sh, which sets PICO_SDK_PATH, THREADX_PATH and
# PICO_TOOLCHAIN_PATH (threadx/script/setup.sh).  What the builds on the
# Pico SDK share is common/pico_sdk/app/usb-console/usb-console.cmake; this
# file gives it the ThreadX parts.

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD arduino_nano_rp2040_connect)

# Before project(): it sets up the toolchain.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(usb_console C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# ThreadX's kernel and the board's part of it: tx_user.h, the low-level
# initialization, the tick, the vectors and the Pico SDK's locks and sleeps
# on ThreadX (threadx_kernel.cmake).
include(${CMAKE_CURRENT_LIST_DIR}/threadx_kernel.cmake)

# The sources shared with the Zephyr and NuttX builds (common/ at the root of
# the repository), with the parts shared by the builds on the Pico SDK
# (common/pico_sdk), and the ThreadX part of the Zephyr compatibility layer
# that runs them (threadx/drivers/compat: zephyr_kernel_os_impl.h, the
# primitives of the generic kernel part, and rp2040/zephyr_os_port.h).
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../common ABSOLUTE)
set(PICO_SDK_OS threadx)
get_filename_component(THREADX_COMPAT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../drivers/compat ABSOLUTE)
set(PICO_SDK_OS_COMPAT_DIRS ${THREADX_COMPAT_DIR}/rp2040 ${THREADX_COMPAT_DIR})
get_filename_component(PICO_SDK_STMEMSC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../stmemsc ABSOLUTE)
# priorities_os.h and the ThreadX parts of the application's boundaries
# (src/); tx_user.h comes with the kernel (threadx_kernel()).
set(PICO_SDK_OS_APP_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
set(PICO_SDK_OS_LIBRARIES)
# The name under /dev/serial/by-id (threadx/script/flash.sh looks for it).
set(PICO_SDK_USB_PRODUCT "ThreadX USB Console")
# ThreadX's code on the sensor's and the send's paths runs from RAM with the
# shared code (ram_code.cmake): the scheduler (PendSV, the switch and the
# wait), the services the layer's waits, the drivers' waits and the Pico
# SDK's locks call (semaphores, mutexes, event flags, sleeps, the suspend
# and resume of threads, the preemption check), the tick (the timer
# interrupt, the timeouts and the time slices), the execution profile kit
# (at every switch); the layer's tick and the idle hooks; the Pico SDK's
# locks on ThreadX; the NINA core's binding (its locks, waits and READY's
# interrupt).  About 7 KiB of RAM.
set(THREADX_SRC $ENV{THREADX_PATH}/common/src)
set(THREADX_PORT_SRC $ENV{THREADX_PATH}/ports/cortex_m0/gnu/src)
set(RAM_CODE_OS_SOURCES
  ${THREADX_PORT_SRC}/tx_thread_schedule.S
  ${THREADX_PORT_SRC}/tx_timer_interrupt.S
  ${THREADX_SRC}/tx_thread_system_resume.c
  ${THREADX_SRC}/tx_thread_system_suspend.c
  ${THREADX_SRC}/tx_thread_system_preempt_check.c
  ${THREADX_SRC}/tx_thread_timeout.c
  ${THREADX_SRC}/tx_thread_time_slice.c
  ${THREADX_SRC}/tx_thread_sleep.c
  ${THREADX_SRC}/tx_thread_relinquish.c
  ${THREADX_SRC}/tx_thread_identify.c
  ${THREADX_SRC}/tx_timer_expiration_process.c
  ${THREADX_SRC}/tx_timer_system_activate.c
  ${THREADX_SRC}/tx_timer_system_deactivate.c
  ${THREADX_SRC}/tx_time_get.c
  ${THREADX_SRC}/tx_semaphore_get.c
  ${THREADX_SRC}/txe_semaphore_get.c
  ${THREADX_SRC}/tx_semaphore_ceiling_put.c
  ${THREADX_SRC}/txe_semaphore_ceiling_put.c
  ${THREADX_SRC}/tx_semaphore_cleanup.c
  ${THREADX_SRC}/tx_mutex_get.c
  ${THREADX_SRC}/txe_mutex_get.c
  ${THREADX_SRC}/tx_mutex_put.c
  ${THREADX_SRC}/txe_mutex_put.c
  ${THREADX_SRC}/tx_event_flags_get.c
  ${THREADX_SRC}/txe_event_flags_get.c
  ${THREADX_SRC}/tx_event_flags_set.c
  ${THREADX_SRC}/txe_event_flags_set.c
  $ENV{THREADX_PATH}/utility/execution_profile_kit/tx_execution_profile.c
  threadx/app/usb-console/src/tick.c
  common/pico_sdk/app/usb-console/src/idle.c
  threadx/drivers/pico_sdk_interop/pico_sdk_interop.c
  common/drivers/nina/port/threadx/nina_os.c
)
include(${COMMON_DIR}/pico_sdk/app/usb-console/usb-console.cmake)

# The shared start (src/main.c of PICO_SDK_APP_DIR) and the application's
# start shared by the three builds (common/imu/app_main.h: app_main.c,
# app_init.c), which it calls; the commands (src/commands.c, with "ps" and
# "version" of src/commands_os_impl.h) and the generic commands
# (common/shell/kernel_shell.c, on src/kernel_shell_os_impl.h); the fatal
# errors, the idle wait (its halves, src/idle_os.h) and the interrupt
# stack; the run times of "ps" (src/run_time.c); the variables of the
# layer's ThreadX part; the records of fatal errors.  threadx_kernel() adds
# ThreadX and its tick.
add_executable(usb-console
  ${PICO_SDK_APP_DIR}/src/main.c
  ${COMMON_DIR}/imu/app_main.c
  ${COMMON_DIR}/imu/app_init.c
  ${PICO_SDK_APP_DIR}/src/commands.c
  ${COMMON_DIR}/shell/kernel_shell.c
  ${PICO_SDK_APP_DIR}/src/fault.c
  ${PICO_SDK_APP_DIR}/src/idle.c
  ${PICO_SDK_APP_DIR}/src/irq_stack.c
  src/run_time.c
  ${THREADX_COMPAT_DIR}/zephyr_kernel_os.c
  ${COMMON_DIR}/compat/log/fault_record.c
)

threadx_kernel(usb-console)
pico_sdk_usb_console(usb-console)
