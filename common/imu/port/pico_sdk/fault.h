/* SPDX-License-Identifier: MIT */
/* Fatal errors: the common exit and the record of the last one (fault.c).
 *
 * panic() (the Pico SDK's callers and the OS port's), the HardFault handler,
 * the OS's assertion and assert(), the OS's own entries (the OS's
 * fault_os.c: FreeRTOS's stack overflow hook, ThreadX's stack check's
 * handler) and _exit() all end in one exit, which disables interrupts,
 * fills a record (common/compat/include/fault_record.h: reason, PC, LR, SP, the
 * exception frame of a HardFault, the running thread, the time and a
 * message), keeps it in RAM that the start-up code leaves alone
 * (__uninitialized_ram) and the reason, PC and LR in the watchdog's scratch
 * registers 0..3, and restarts the board with the watchdog, as "reboot"
 * does.  It writes nothing to the console: the USB code needs interrupts
 * and threads.  The Zephyr build (CONFIG_RESET_ON_FATAL_ERROR) and the NuttX
 * build (CONFIG_BOARD_RESET_ON_ASSERT=2) also restart after a fatal error.
 *
 * fault.c's own entries are panic(), the HardFault handler, assert()
 * (__assert_func()), _exit() and the OS's assertion, app_assert_failed(),
 * which the OS's assert macro calls (FreeRTOS's configASSERT(), the
 * compatibility layer's Z_TX_ASSERT() on ThreadX).  The OS's part,
 * src/fault_os.c of the OS's application directory, defines
 * fault_os_thread_name() and the OS's own entries, which use
 * fault_begin() and fault_finish().
 */

#ifndef FAULT_H
#define FAULT_H

#include <stddef.h>
#include <stdint.h>

#include "fault_record.h"

/* The RP2040's SRAM, where stacks and the kernel's objects are: a fault may
 * come from a corrupted kernel, so the exit reads the kernel's memory only
 * inside it (fault_record_in_range()).
 */
#define FAULT_RAM_START 0x20000000u
#define FAULT_RAM_END   0x20042000u

/* At the start, before anything else can fail: takes the record the last
 * run left (in RAM, or, if that fails its check, the scratch registers),
 * keeps it for fault_last(), writes its lines into the RAM log ("dmesg";
 * call log_ram_initialize() first), and clears both so that the next start
 * does not see it again.
 */
void fault_boot(void);

/* The fatal error that ended the run before this start, or NULL. */
const struct fault_record *fault_last(void);

/* The OS's entries ---------------------------------------------------------*/

/* An entry of the OS (the OS's fault_os.c) is
 *
 *   struct fault_record *r = fault_begin(FAULT_REASON_..., pc);
 *
 *   if (r != NULL) {
 *           fill r (r->task with at most sizeof(r->task) - 1 bytes, r->text)
 *   }
 *   fault_finish(r);
 *
 * Both may be called from any context: a thread, an interrupt or exception
 * handler (a HardFault), before the scheduler starts, after a stack
 * overflow, with the kernel's objects corrupt.  They do not block, allocate
 * or call the OS.
 */

/* Disables interrupts (for good) and starts a cleared record with REASON
 * (FAULT_REASON_..., fault_record.h), the exception number, PC (a return
 * address, its Thumb bit is cleared; 0 when there is none) and the stack
 * pointer.  Returns NULL when a fatal error happens inside the exit itself
 * (the first one is being recorded); the caller then fills nothing and
 * calls fault_finish(NULL), which only restarts.
 */
struct fault_record *fault_begin(uint32_t reason, uint32_t pc);

/* Ends the record R of fault_begin(): the running thread's name when the
 * entry has left r->task empty (fault_os_thread_name()), the time, the
 * seal and the scratch registers, then restarts the board with the
 * watchdog.  With NULL it only restarts.  Does not return.
 */
void __attribute__((noreturn)) fault_finish(struct fault_record *r);

/* The OS's part ----------------------------------------------------------*/

/* Defined by the OS's fault_os.c: copies the name of the running thread
 * into NAME (SIZE bytes, ends with a NUL that is already there: it copies
 * at most SIZE - 1 bytes), or leaves NAME as it is when no thread runs
 * (before the scheduler starts, or ThreadX's scheduler waiting with no
 * thread ready) or when the thread's control block (or its name) is not
 * inside FAULT_RAM_START..FAULT_RAM_END.  Called by fault_finish() with
 * interrupts disabled, from any context (also a HardFault handler); it
 * must not block, allocate or call the OS's services that check their
 * context.
 */
void fault_os_thread_name(char *name, size_t size);

#endif /* FAULT_H */
