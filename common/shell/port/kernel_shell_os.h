/* SPDX-License-Identifier: MIT */
/* What the generic shell commands of kernel_shell.c need from the OS.
 * kernel_shell.c includes this header; it includes the OS's part,
 * "kernel_shell_os_impl.h", then declares the functions that part must
 * define, so a different one does not compile.  The OS's part defines them
 * as static (inline) functions.  Everything else the commands use is the
 * compatibility layer's: k_cycle_get_64() (a 1 MHz counter), k_sleep(),
 * Z_USEC_PER_TICK, sys_clock_gettime() and hwinfo_get_device_id().
 *
 * The FreeRTOS build's part is freertos/app/usb-console/src/
 * kernel_shell_os_impl.h.  The NuttX build does not compile these commands
 * (nsh has its own).
 */

#ifndef COMMON_SHELL_PORT_KERNEL_SHELL_OS_H
#define COMMON_SHELL_PORT_KERNEL_SHELL_OS_H

#include <stdint.h>

#include "fault_record.h"
#include "kernel_shell_os_impl.h"

/* The OS's tick count since the scheduler started, as the OS counts it
 * (it may wrap); "uptime" shows it with the tick rate 1000000 /
 * Z_USEC_PER_TICK.  From a thread.
 */
static inline uint32_t kernel_shell_os_tick_count(void);

/* "reboot": restarts the board delay_ms from now and does not return; the
 * calling thread may sleep until then, so that the command's last message
 * reaches the terminal.
 */
static inline void kernel_shell_os_reboot(uint32_t delay_ms);

/* "bootsel": after delay_ms, restarts into the boot ROM's USB loader (the
 * RPI-RP2 drive) and does not return.
 */
static inline void kernel_shell_os_bootsel(uint32_t delay_ms);

/* "fault": the record of the fatal error that ended the run before this
 * start (common/compat/include/fault_record.h), or NULL when there is none or
 * the OS keeps no such record.
 */
static inline const struct fault_record *kernel_shell_os_fault_last(void);

#endif /* COMMON_SHELL_PORT_KERNEL_SHELL_OS_H */
