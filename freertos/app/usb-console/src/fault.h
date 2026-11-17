/* SPDX-License-Identifier: MIT */
/* Fatal errors: the common exit and the record of the last one (fault.c).
 *
 * panic() (the Pico SDK's callers and the port's), the HardFault handler,
 * configASSERT() and assert(), the kernel's stack overflow check and heap
 * exhaustion, and _exit() all end in one exit, which disables interrupts,
 * fills a record (common/log/fault_record.h: reason, PC, LR, SP, the
 * exception frame of a HardFault, the running task, the time and a
 * message), keeps it in RAM that the start-up code leaves alone
 * (__uninitialized_ram) and the reason, PC and LR in the watchdog's scratch
 * registers 0..3, and restarts the board with the watchdog, as "reboot"
 * does.  It writes nothing to the console: the USB code needs interrupts
 * and tasks.  The Zephyr build (CONFIG_RESET_ON_FATAL_ERROR) and the NuttX
 * build (CONFIG_BOARD_RESET_ON_ASSERT=2) also restart after a fatal error.
 */

#ifndef FAULT_H
#define FAULT_H

#include "fault_record.h"

/* At the start, before anything else can fail: takes the record the last
 * run left (in RAM, or, if that fails its check, the scratch registers),
 * keeps it for fault_last(), writes its lines into the RAM log ("dmesg";
 * call log_ram_initialize() first), and clears both so that the next start
 * does not see it again.
 */
void fault_boot(void);

/* The fatal error that ended the run before this start, or NULL. */
const struct fault_record *fault_last(void);

#endif /* FAULT_H */
