/* SPDX-License-Identifier: MIT */
/* Fatal errors: the common exit, its own entries and the record (fault.h).
 * The OS's entries and the running thread's name are the OS's fault_os.c.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "hardware/structs/watchdog.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "app.h"
#include "app_log.h"
#include "fault.h"
#include "rp2040_timer_us.h"

LOG_MODULE_REGISTER(fault);

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

static uint32_t current_exception(void)
{
	uint32_t ipsr;

	__asm volatile("mrs %0, ipsr" : "=r"(ipsr));
	return ipsr & FAULT_RECORD_EXCEPTION_MASK;
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

/* The common exit (fault.h) ---------------------------------------------*/

struct fault_record *fault_begin(uint32_t reason, uint32_t pc)
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

void fault_finish(struct fault_record *r)
{
	uint32_t s[FAULT_SCRATCH_WORDS];

	if (r == NULL) {
		restart();
	}
	if (r->task[0] == '\0') {
		fault_os_thread_name(r->task, sizeof(r->task));
	}
	r->uptime_us = rp2040_timer_us();
	fault_record_seal(r);

	fault_record_to_scratch(r, s);
	for (int i = 0; i < FAULT_SCRATCH_WORDS; i++) {
		watchdog_hw->scratch[i] = s[i];
	}
	restart();
}

/* Its own entries --------------------------------------------------------*/

/* The Pico SDK's panic() (pico/platform/panic.h).  PICO_PANIC_FUNCTION is
 * defined empty (usb-console.cmake), which leaves the SDK's own panic() out
 * (pico_platform_panic/panic.c) and its weak stand-in
 * (custom_panic_function.S) for this one.  panic_unsupported(),
 * hard_assertion_failure() and, on FreeRTOS, a task function that returns
 * (the port's prvTaskExitError()) call it; the SDK's malloc() does not when
 * out of memory (PICO_MALLOC_PANIC=0, usb-console.cmake).
 */
void __attribute__((noreturn)) panic(const char *fmt, ...)
{
	struct fault_record *r =
		fault_begin(FAULT_REASON_PANIC, (uint32_t)__builtin_return_address(0));

	if (r != NULL && fmt != NULL) {
		va_list args;

		va_start(args, fmt);
		vsnprintf(r->text, sizeof(r->text), fmt, args);
		va_end(args);
	}
	fault_finish(r);
}

/* The OS's assertion (FreeRTOS: configASSERT(); ThreadX: Z_TX_ASSERT()). */
void app_assert_failed(const char *file, int line)
{
	struct fault_record *r =
		fault_begin(FAULT_REASON_ASSERT, (uint32_t)__builtin_return_address(0));

	if (r != NULL) {
		snprintf(r->text, sizeof(r->text), "%s:%d", base_name(file), line);
	}
	fault_finish(r);
}

/* newlib's assert() (the SDK's weak one prints and calls _exit()).  Release
 * builds (NDEBUG) leave assert() out; the SDK's code may still call this.
 */
void __assert_func(const char *file, int line, const char *func, const char *expr)
{
	struct fault_record *r =
		fault_begin(FAULT_REASON_ASSERT, (uint32_t)__builtin_return_address(0));

	(void)func;
	if (r != NULL) {
		snprintf(r->text, sizeof(r->text), "%s:%d: %s", base_name(file), line,
			 expr != NULL ? expr : "");
	}
	fault_finish(r);
}

/* exit() and abort() (the SDK's weak one waits at a breakpoint). */
void __attribute__((noreturn)) _exit(int status)
{
	struct fault_record *r =
		fault_begin(FAULT_REASON_EXIT, (uint32_t)__builtin_return_address(0));

	if (r != NULL) {
		snprintf(r->text, sizeof(r->text), "status %d", status);
	}
	fault_finish(r);
}

/* Called by isr_hardfault() with the stack that holds the exception frame
 * and EXC_RETURN.
 */
static void __attribute__((used, noreturn)) hardfault(const uint32_t *frame, uint32_t exc_return)
{
	struct fault_record *r = fault_begin(FAULT_REASON_HARDFAULT, 0);

	/* The frame is read only inside the RAM (fault_record.h). */
	if (r != NULL) {
		fault_record_set_frame(r, (uintptr_t)frame, exc_return, FAULT_RAM_START,
				       FAULT_RAM_END);
	}
	fault_finish(r);
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

/* The record of the last run into the RAM log, once, at the start (the
 * Zephyr and NuttX builds write their report there when the error happens;
 * this build cannot, see fault.h): <err> lines of the module "fault" with
 * this start's time, the record's lines as "fault" prints them.
 */
static void fault_log(void)
{
	char line[128];

	if (!last_valid) {
		return;
	}
	LOG_ERR("The last run ended with a fatal error:");
	for (unsigned int i = 0; fault_record_line(&last, i, line, sizeof(line)) > 0; i++) {
		LOG_ERR("%s", line);
	}
}

void fault_boot(void)
{
	uint32_t s[FAULT_SCRATCH_WORDS];

	for (int i = 0; i < FAULT_SCRATCH_WORDS; i++) {
		s[i] = watchdog_hw->scratch[i];
	}
	last_valid = fault_record_take(&fault_ram, s, &last);
	memset(&fault_ram, 0, sizeof(fault_ram));
	for (int i = 0; i < FAULT_SCRATCH_WORDS; i++) {
		watchdog_hw->scratch[i] = 0;
	}
	fault_log();
}

const struct fault_record *fault_last(void)
{
	return last_valid ? &last : NULL;
}
