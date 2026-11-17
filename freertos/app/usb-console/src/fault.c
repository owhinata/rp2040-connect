/* SPDX-License-Identifier: MIT */
/* Fatal errors: the entries, the common exit and the record (fault.h). */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "hardware/structs/watchdog.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "fault.h"
#include "os_clock.h"

/* The RP2040's SRAM, where stacks and the kernel's objects are. */
#define RAM_START 0x20000000u
#define RAM_END   0x20042000u

/* The exception numbers (IPSR). */
#define EXCEPTION_MASK     0x3fu
#define EXCEPTION_PENDSV   14u
#define EXCEPTION_FRAME_SZ 32u /* r0..r3, r12, lr, pc, xpsr */

/* xPSR bit 9 in the frame: the processor added 4 bytes to align the stack. */
#define XPSR_ALIGNED_BIT (1u << 9)

/* EXC_RETURN bit 2: the frame is on the process stack (PSP, a task). */
#define EXC_RETURN_PSP (1u << 2)

/* Left alone by the start-up code (crt0 only copies .data and zeroes .bss),
 * so it survives the watchdog's restart.  Power-on leaves random contents,
 * which fail the record's check.
 */
static struct fault_record __uninitialized_ram(fault_ram);

/* This start's copy of the record the last run left. */
static struct fault_record last;
static bool last_valid;

/* Set by the first entry; a fatal error inside the exit only restarts. */
static volatile bool exiting;

static bool in_ram(uintptr_t address, size_t size)
{
	return address >= RAM_START && address <= RAM_END - size;
}

static uint32_t current_exception(void)
{
	uint32_t ipsr;

	__asm volatile("mrs %0, ipsr" : "=r"(ipsr));
	return ipsr & EXCEPTION_MASK;
}

static uint32_t current_sp(void)
{
	uint32_t sp;

	__asm volatile("mov %0, sp" : "=r"(sp));
	return sp;
}

static const char *base_name(const char *path)
{
	const char *slash = path != NULL ? strrchr(path, '/') : NULL;

	if (path == NULL) {
		return "?";
	}
	return slash != NULL ? slash + 1 : path;
}

static void __attribute__((noreturn)) restart(void)
{
	/* Immediately (no delay): the watchdog resets everything but the
	 * oscillators (watchdog.c), as "reboot" does after its delay.
	 */
	watchdog_reboot(0, 0, 0);
	for (;;) {
		__asm volatile("wfi");
	}
}

/* Disables interrupts and starts a record.  Returns NULL when a fatal error
 * happens inside the exit; the caller then only restarts.
 */
static struct fault_record *begin(uint32_t reason, uint32_t pc)
{
	struct fault_record *r = &fault_ram;

	(void)save_and_disable_interrupts();
	if (exiting) {
		return NULL;
	}
	exiting = true;

	memset(r, 0, sizeof(*r));
	r->reason = reason;
	r->exception = current_exception();
	r->pc = pc & ~1u; /* a return address has the Thumb bit */
	r->sp = current_sp();
	return r;
}

/* The common exit: the running task, the time, the seal, the scratch
 * registers and the restart.
 */
static void __attribute__((noreturn)) finish(struct fault_record *r)
{
	uint32_t s[FAULT_SCRATCH_WORDS];

	if (r == NULL) {
		restart();
	}
	if (r->task[0] == '\0' && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
		TaskHandle_t task = xTaskGetCurrentTaskHandle();

		/* The handle is the task's control block; a fault may come from
		 * a corrupted kernel, so only read it inside the RAM.
		 */
		if (task != NULL && in_ram((uintptr_t)task, sizeof(StaticTask_t))) {
			strncpy(r->task, pcTaskGetName(task), sizeof(r->task) - 1);
		}
	}
	r->uptime_us = nano_mono_us();
	fault_record_seal(r);

	fault_record_to_scratch(r, s);
	for (int i = 0; i < FAULT_SCRATCH_WORDS; i++) {
		watchdog_hw->scratch[i] = s[i];
	}
	restart();
}

/* Entries ----------------------------------------------------------------*/

/* The Pico SDK's panic() (pico/platform/panic.h).  PICO_PANIC_FUNCTION is
 * defined empty (CMakeLists.txt), which leaves the SDK's own panic() out
 * (pico_platform_panic/panic.c) and its weak stand-in
 * (custom_panic_function.S) for this one.  panic_unsupported(),
 * hard_assertion_failure(), the SDK's malloc() when out of memory and a task
 * function that returns (the port's prvTaskExitError()) call it.
 */
void __attribute__((noreturn)) panic(const char *fmt, ...)
{
	struct fault_record *r = begin(FAULT_REASON_PANIC, (uint32_t)__builtin_return_address(0));

	if (r != NULL && fmt != NULL) {
		va_list args;

		va_start(args, fmt);
		vsnprintf(r->text, sizeof(r->text), fmt, args);
		va_end(args);
	}
	finish(r);
}

/* configASSERT() (FreeRTOSConfig.h). */
void app_assert_failed(const char *file, int line)
{
	struct fault_record *r = begin(FAULT_REASON_ASSERT, (uint32_t)__builtin_return_address(0));

	if (r != NULL) {
		snprintf(r->text, sizeof(r->text), "%s:%d", base_name(file), line);
	}
	finish(r);
}

/* newlib's assert() (the SDK's weak one prints and calls _exit()).  Release
 * builds (NDEBUG) leave assert() out; the SDK's code may still call this.
 */
void __assert_func(const char *file, int line, const char *func, const char *expr)
{
	struct fault_record *r = begin(FAULT_REASON_ASSERT, (uint32_t)__builtin_return_address(0));

	(void)func;
	if (r != NULL) {
		snprintf(r->text, sizeof(r->text), "%s:%d: %s", base_name(file), line,
			 expr != NULL ? expr : "");
	}
	finish(r);
}

/* exit() and abort() (the SDK's weak one waits at a breakpoint). */
void __attribute__((noreturn)) _exit(int status)
{
	struct fault_record *r = begin(FAULT_REASON_EXIT, (uint32_t)__builtin_return_address(0));

	if (r != NULL) {
		snprintf(r->text, sizeof(r->text), "status %d", status);
	}
	finish(r);
}

/* configCHECK_FOR_STACK_OVERFLOW 2: the kernel checks the stack of the task
 * it switches out (in PendSV, exception 14) for its limit and the pattern at
 * its end.
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
	struct fault_record *r =
		begin(FAULT_REASON_STACK_OVERFLOW, (uint32_t)__builtin_return_address(0));

	(void)task;
	if (r != NULL && name != NULL) {
		strncpy(r->task, name, sizeof(r->task) - 1);
	}
	finish(r);
}

/* configUSE_MALLOC_FAILED_HOOK: pvPortMalloc() of heap_4 found no room. */
void vApplicationMallocFailedHook(void)
{
	finish(begin(FAULT_REASON_HEAP, (uint32_t)__builtin_return_address(0)));
}

/* Called by isr_hardfault() with the stack that holds the exception frame
 * and EXC_RETURN.
 */
static void __attribute__((used, noreturn)) hardfault(const uint32_t *frame, uint32_t exc_return)
{
	struct fault_record *r = begin(FAULT_REASON_HARDFAULT, 0);

	if (r == NULL) {
		finish(r);
	}
	/* A bad stack pointer may be the fault itself: read the frame only
	 * inside the RAM.
	 */
	if (in_ram((uintptr_t)frame, EXCEPTION_FRAME_SZ) && ((uintptr_t)frame & 3u) == 0) {
		r->flags = FAULT_RECORD_FRAME;
		r->r0 = frame[0];
		r->r1 = frame[1];
		r->r2 = frame[2];
		r->r3 = frame[3];
		r->r12 = frame[4];
		r->lr = frame[5];
		r->pc = frame[6];
		r->xpsr = frame[7];
		/* Where the interrupted code was: thread mode (0) or the
		 * exception it was handling.
		 */
		r->exception = r->xpsr & EXCEPTION_MASK;
		r->sp = (uint32_t)frame + EXCEPTION_FRAME_SZ +
			((r->xpsr & XPSR_ALIGNED_BIT) ? 4u : 0u);
	} else {
		r->sp = (uint32_t)frame;
		snprintf(r->text, sizeof(r->text), "stack pointer outside the RAM (%s)",
			 (exc_return & EXC_RETURN_PSP) ? "PSP" : "MSP");
	}
	finish(r);
}

/* The HardFault exception (crt0's weak isr_hardfault waits at a
 * breakpoint): picks the stack the processor saved the frame on, from bit 2
 * of EXC_RETURN in LR, and continues in hardfault() on the main stack.
 */
void __attribute__((naked, noreturn)) isr_hardfault(void)
{
	__asm volatile("movs r0, #4\n\t"
		       "mov r1, lr\n\t"
		       "tst r0, r1\n\t"
		       "beq 1f\n\t"
		       "mrs r0, psp\n\t"
		       "b 2f\n"
		       "1:\n\t"
		       "mrs r0, msp\n"
		       "2:\n\t"
		       "ldr r2, =hardfault\n\t"
		       "bx r2\n\t"
		       ".ltorg");
}

/* The record ----------------------------------------------------------------*/

void fault_boot(void)
{
	uint32_t s[FAULT_SCRATCH_WORDS];

	for (int i = 0; i < FAULT_SCRATCH_WORDS; i++) {
		s[i] = watchdog_hw->scratch[i];
	}
	if (fault_record_valid(&fault_ram)) {
		last = fault_ram;
		last_valid = true;
	} else {
		last_valid = fault_record_from_scratch(s, &last);
	}
	memset(&fault_ram, 0, sizeof(fault_ram));
	for (int i = 0; i < FAULT_SCRATCH_WORDS; i++) {
		watchdog_hw->scratch[i] = 0;
	}
}

const struct fault_record *fault_last(void)
{
	return last_valid ? &last : NULL;
}
