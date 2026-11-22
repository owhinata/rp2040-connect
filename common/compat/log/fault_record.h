/* SPDX-License-Identifier: MIT */
/* The record of a fatal error that the firmware keeps across the restart it
 * makes after the error, and its text: the OS-independent part, kept apart
 * so the host tests (common/tests/fault_record.c) can run it.  Used by the
 * FreeRTOS build (freertos/app/usb-console/src/fault.c), which fills the
 * record at the common exit of panic(), the HardFault handler, failed
 * assertions, stack overflows and heap exhaustion, keeps it in RAM that the
 * start-up code leaves alone (__uninitialized_ram) with a copy of the reason,
 * PC and LR in the watchdog's scratch registers 0..3, restarts, and shows it
 * after the restart ("fault" and the banner).  The record's lines are
 * written one at a time (fault_record_line()) so the RAM log can take them
 * as well (common/compat/log/log_ring.c).
 *
 * The Zephyr build (CONFIG_RESET_ON_FATAL_ERROR) and the NuttX build
 * (CONFIG_BOARD_RESET_ON_ASSERT) write their report into the RAM log
 * instead and have no such record.
 */

#ifndef COMMON_LOG_FAULT_RECORD_H
#define COMMON_LOG_FAULT_RECORD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* "FLT1" in memory.  The size of the record is checked as well, so a record
 * of another layout left in RAM by another build fails the check.
 */
#define FAULT_RECORD_MAGIC 0x31544c46u

/* The high half of watchdog scratch register 0 ("FA"); the low half is the
 * reason.
 */
#define FAULT_SCRATCH_MAGIC 0x46410000u
#define FAULT_SCRATCH_WORDS 4

#define FAULT_RECORD_TASK_LEN 32 /* FreeRTOS configMAX_TASK_NAME_LEN */
#define FAULT_RECORD_TEXT_LEN 96

/* Why the firmware stopped. */
enum fault_reason {
	FAULT_REASON_NONE = 0,
	FAULT_REASON_HARDFAULT = 1,      /* the processor's HardFault exception */
	FAULT_REASON_PANIC = 2,          /* panic() (the Pico SDK's and ours) */
	FAULT_REASON_ASSERT = 3,         /* configASSERT(), assert() */
	FAULT_REASON_STACK_OVERFLOW = 4, /* a task's stack (the kernel's check) */
	FAULT_REASON_HEAP = 5,           /* the kernel's heap is exhausted */
	FAULT_REASON_EXIT = 6,           /* _exit() (exit(), abort()) */
};

/* The record's flags. */
#define FAULT_RECORD_FRAME   (1u << 0) /* r0..r3, r12 and xpsr are the exception frame */
#define FAULT_RECORD_SCRATCH (1u << 1) /* rebuilt from the watchdog's scratch registers */

/* Every field before check is covered by check (CRC-32 of the bytes). */
struct fault_record {
	uint32_t magic;
	uint32_t size; /* sizeof(struct fault_record) */
	uint32_t reason;
	uint32_t flags;
	/* The exception number the error happened in (IPSR: 0 in a task, 3
	 * HardFault, 14 PendSV, 15 SysTick, 16 + n interrupt n).
	 */
	uint32_t exception;
	/* HardFault: from the exception frame.  Otherwise pc is the address
	 * the common exit was called from (in its caller), lr is 0 (not
	 * known) and the others are 0.
	 */
	uint32_t pc;
	uint32_t lr;
	uint32_t sp;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t xpsr;
	uint64_t uptime_us;               /* the TIMER at the error */
	char task[FAULT_RECORD_TASK_LEN]; /* the running task, "" if none */
	char text[FAULT_RECORD_TEXT_LEN]; /* the message, "" if none */
	uint32_t check;
};

/* CRC-32 (IEEE 802.3, reflected, as zlib's crc32()) of len bytes. */
uint32_t fault_record_crc32(const void *data, size_t len);

/* Sets magic, size and check after the other fields are filled. */
void fault_record_seal(struct fault_record *r);

/* True if magic, size and check are those of a sealed record. */
bool fault_record_valid(const struct fault_record *r);

/* A short name of a reason ("HardFault", "panic", ...), "unknown" for a
 * value outside enum fault_reason.
 */
const char *fault_reason_name(uint32_t reason);

/* The words for the watchdog's scratch registers 0..3: FAULT_SCRATCH_MAGIC
 * with the reason, pc, lr and the CRC-32 of the first three words.
 */
void fault_record_to_scratch(const struct fault_record *r, uint32_t s[FAULT_SCRATCH_WORDS]);

/* If s holds what fault_record_to_scratch() wrote, fills r with the reason,
 * pc and lr, sets FAULT_RECORD_SCRATCH, seals it and returns true;
 * otherwise returns false and leaves r alone.
 */
bool fault_record_from_scratch(const uint32_t s[FAULT_SCRATCH_WORDS], struct fault_record *r);

/* Line index of the record's text, without a line end, into buf
 * (truncated to len - 1 characters and terminated).  Returns the length of
 * the line, or 0 when index is past the last line.  The lines:
 *
 *   Fatal error: <reason>[: <text>]
 *   In task <task>[, exception <n>], <s>.<us> s after the start
 *   pc 0x........ lr 0x........ sp 0x........
 *   r0 0x........ r1 0x........ r2 0x........ r3 0x........      (frame only)
 *   r12 0x........ xpsr 0x........                               (frame only)
 *   (from the watchdog scratch registers only)                   (scratch only)
 *
 * "In task" says "Before the scheduler started" when task is "".
 */
size_t fault_record_line(const struct fault_record *r, unsigned int index, char *buf, size_t len);

/* Writes the record's lines to stdout, each followed by "\n" ("fault" and
 * the FreeRTOS build's banner after the restart).
 */
void fault_record_print(const struct fault_record *r);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_LOG_FAULT_RECORD_H */
