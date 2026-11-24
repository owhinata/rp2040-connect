/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the fatal error exit (fault.h of
 * common/pico_sdk/app/usb-console/src): the running thread's name from its
 * control block, and ThreadX's stack check as an entry.
 *
 * ThreadX's other fatal paths end in fault.c's own entries: the layer's
 * Z_TX_ASSERT() (threadx/drivers/compat/rp2040/zephyr_os_port.h) calls
 * app_assert_failed(); ThreadX's services report a bad call with an error
 * code (TX_DISABLE_ERROR_CHECKING is not set), which is not a fatal error;
 * a thread whose function returns is left TX_COMPLETED by ThreadX (the
 * layer's threads do not return).
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "tx_api.h"
#include "tx_thread.h"

#include "fault.h"

#ifndef TX_ENABLE_STACK_CHECKING
#error "fault_os_stack_error() is the handler of TX_ENABLE_STACK_CHECKING (tx_user.h)"
#endif

/* The name of THREAD into NAME (SIZE bytes, its NUL already there), when
 * the control block and the name are inside the RAM: a fault may come from
 * a corrupted kernel.  The layer's threads keep their names in the storage
 * after their stacks (zephyr_kernel_os_impl.h of threadx/drivers/compat).
 */
static void fault_os_copy_name(const TX_THREAD *thread, char *name, size_t size)
{
	if (thread == NULL || !fault_record_in_range((uintptr_t)thread, sizeof(TX_THREAD),
						     FAULT_RAM_START, FAULT_RAM_END)) {
		return;
	}
	if (thread->tx_thread_name != NULL &&
	    fault_record_in_range((uintptr_t)thread->tx_thread_name, size - 1, FAULT_RAM_START,
				  FAULT_RAM_END)) {
		strncpy(name, thread->tx_thread_name, size - 1);
	}
}

/* _tx_thread_current_ptr is NULL until the scheduler takes its first
 * thread, and while it waits with no thread ready (a fault in an interrupt
 * handler then has no thread).
 */
void fault_os_thread_name(char *name, size_t size)
{
	fault_os_copy_name(_tx_thread_current_ptr, name, size);
}

/* The handler of ThreadX's stack check (tx_user.h: TX_ENABLE_STACK_CHECKING),
 * registered by main() (main.c) with tx_thread_stack_error_notify():
 * ThreadX calls it when the fill pattern at the bottom or the top of
 * THREAD's stack has been overwritten, or its saved stack pointer is below
 * the stack, as it suspends THREAD (a wait, a sleep) or resumes it; from
 * the thread itself or from an interrupt handler that resumed it.  The
 * record names THREAD, which is not always the running one.
 */
void fault_os_stack_error(TX_THREAD *thread)
{
	struct fault_record *r =
		fault_begin(FAULT_REASON_STACK_OVERFLOW, (uint32_t)__builtin_return_address(0));

	if (r != NULL) {
		fault_os_copy_name(thread, r->task, sizeof(r->task));
	}
	fault_finish(r);
}
