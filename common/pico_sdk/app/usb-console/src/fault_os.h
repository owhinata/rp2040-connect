/* SPDX-License-Identifier: MIT */
/* What the fatal error exit (fault.c) needs from the OS.  fault.c includes
 * this header first; the OS's part, "fault_os_impl.h" (in the OS's
 * application directory), is included by fault.c among its entries (after
 * the common exit, so that the OS's entry can use it), and defines:
 *
 *   fault_os_thread_name(name, size)
 *       Copies the name of the running thread into NAME (SIZE bytes, ends
 *       with a NUL that is already there: it copies at most SIZE - 1 bytes),
 *       or leaves NAME as it is when no thread runs yet (before the
 *       scheduler starts) or when the thread's control block is not in the
 *       RAM (FAULT_RAM_START..FAULT_RAM_END: a fault may come from a
 *       corrupted kernel, so only memory that can be read is read).  Called
 *       by the common exit with interrupts disabled, from any context (also
 *       a HardFault handler); it must not block, allocate or call the OS's
 *       services that check their context.  Static inline (always inline).
 *
 *   The OS's own fatal error entries, as ordinary functions with the OS's
 *   names (FreeRTOS: vApplicationStackOverflowHook(), ThreadX:
 *   fault_os_stack_error()).  Each starts with struct fault_record *r =
 *   fault_begin(FAULT_REASON_..., pc) (interrupts off; NULL inside the exit
 *   itself), fills the record when r is not NULL (the thread's name for a
 *   stack overflow, at most sizeof(r->task) - 1 bytes) and ends with
 *   fault_finish(r), which does not return (both static in fault.c).  None
 *   when the OS has no such check.
 *
 * The other entries (the Pico SDK's panic(), assert(), _exit(), the
 * HardFault handler) and the OS's assertion (app_assert_failed(), which the
 * OS's assert macro calls: FreeRTOS's configASSERT(), the compatibility
 * layer's Z_OS_ASSERT()) are fault.c's own.
 */

#ifndef FAULT_OS_H
#define FAULT_OS_H

#include <stddef.h>

static inline void fault_os_thread_name(char *name, size_t size);

#endif /* FAULT_OS_H */
