/* SPDX-License-Identifier: MIT */
/* The record of a fatal error (fault_record.h). */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "fault_record.h"

uint32_t fault_record_crc32(const void *data, size_t len)
{
	const uint8_t *p = data;
	uint32_t crc = 0xffffffffu;

	while (len-- > 0) {
		crc ^= *p++;
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
		}
	}
	return ~crc;
}

static uint32_t record_check(const struct fault_record *r)
{
	return fault_record_crc32(r, offsetof(struct fault_record, check));
}

void fault_record_seal(struct fault_record *r)
{
	r->magic = FAULT_RECORD_MAGIC;
	r->size = sizeof(*r);
	/* The strings are always terminated, whatever the filler did. */
	r->task[sizeof(r->task) - 1] = '\0';
	r->text[sizeof(r->text) - 1] = '\0';
	r->check = record_check(r);
}

bool fault_record_valid(const struct fault_record *r)
{
	return r->magic == FAULT_RECORD_MAGIC && r->size == sizeof(*r) &&
	       r->check == record_check(r);
}

const char *fault_reason_name(uint32_t reason)
{
	switch (reason) {
	case FAULT_REASON_HARDFAULT:
		return "HardFault";
	case FAULT_REASON_PANIC:
		return "panic";
	case FAULT_REASON_ASSERT:
		return "assertion failed";
	case FAULT_REASON_STACK_OVERFLOW:
		return "stack overflow";
	case FAULT_REASON_HEAP:
		return "heap exhausted";
	case FAULT_REASON_EXIT:
		return "exit";
	default:
		return "unknown";
	}
}

void fault_record_to_scratch(const struct fault_record *r, uint32_t s[FAULT_SCRATCH_WORDS])
{
	s[0] = FAULT_SCRATCH_MAGIC | (r->reason & 0xffffu);
	s[1] = r->pc;
	s[2] = r->lr;
	s[3] = fault_record_crc32(s, 3 * sizeof(s[0]));
}

bool fault_record_from_scratch(const uint32_t s[FAULT_SCRATCH_WORDS], struct fault_record *r)
{
	if ((s[0] & 0xffff0000u) != FAULT_SCRATCH_MAGIC ||
	    s[3] != fault_record_crc32(s, 3 * sizeof(s[0]))) {
		return false;
	}
	memset(r, 0, sizeof(*r));
	r->reason = s[0] & 0xffffu;
	r->flags = FAULT_RECORD_SCRATCH;
	r->pc = s[1];
	r->lr = s[2];
	fault_record_seal(r);
	return true;
}

bool fault_record_in_range(uintptr_t address, size_t size, uintptr_t start, uintptr_t end)
{
	return start <= end && size <= end - start && address >= start &&
	       address - start <= (end - start) - size;
}

void fault_record_set_frame(struct fault_record *r, uintptr_t frame, uint32_t exc_return,
			    uintptr_t ram_start, uintptr_t ram_end)
{
	const uint32_t *f = (const uint32_t *)frame;

	if (!fault_record_in_range(frame, FAULT_RECORD_FRAME_SIZE, ram_start, ram_end) ||
	    (frame & 3u) != 0) {
		r->sp = (uint32_t)frame;
		snprintf(r->text, sizeof(r->text), "stack pointer outside the RAM (%s)",
			 (exc_return & FAULT_RECORD_EXC_RETURN_PSP) ? "PSP" : "MSP");
		return;
	}
	r->flags = FAULT_RECORD_FRAME;
	r->r0 = f[0];
	r->r1 = f[1];
	r->r2 = f[2];
	r->r3 = f[3];
	r->r12 = f[4];
	r->lr = f[5];
	r->pc = f[6];
	r->xpsr = f[7];
	/* Where the interrupted code was: thread mode (0) or the exception it
	 * was handling.
	 */
	r->exception = r->xpsr & FAULT_RECORD_EXCEPTION_MASK;
	r->sp = (uint32_t)frame + FAULT_RECORD_FRAME_SIZE +
		((r->xpsr & FAULT_RECORD_XPSR_ALIGNED) ? 4u : 0u);
}

bool fault_record_take(const struct fault_record *retained, const uint32_t s[FAULT_SCRATCH_WORDS],
		       struct fault_record *last)
{
	if (fault_record_valid(retained)) {
		*last = *retained;
		return true;
	}
	return fault_record_from_scratch(s, last);
}

/* The kinds of line, in their order. */
enum line_kind {
	LINE_REASON,
	LINE_WHERE,
	LINE_PC,
	LINE_FRAME_R0,
	LINE_FRAME_R12,
	LINE_SCRATCH,
};

static int line_kinds(const struct fault_record *r, enum line_kind kinds[6])
{
	int n = 0;

	kinds[n++] = LINE_REASON;
	if (!(r->flags & FAULT_RECORD_SCRATCH)) {
		kinds[n++] = LINE_WHERE;
	}
	kinds[n++] = LINE_PC;
	if (r->flags & FAULT_RECORD_FRAME) {
		kinds[n++] = LINE_FRAME_R0;
		kinds[n++] = LINE_FRAME_R12;
	}
	if (r->flags & FAULT_RECORD_SCRATCH) {
		kinds[n++] = LINE_SCRATCH;
	}
	return n;
}

size_t fault_record_line(const struct fault_record *r, unsigned int index, char *buf, size_t len)
{
	enum line_kind kinds[6];
	char exception[24] = "";
	int n = line_kinds(r, kinds);
	int ret;

	if (index >= (unsigned int)n || len == 0) {
		return 0;
	}
	switch (kinds[index]) {
	case LINE_REASON:
		ret = snprintf(buf, len, "Fatal error: %s%s%.*s", fault_reason_name(r->reason),
			       r->text[0] != '\0' ? ": " : "", (int)sizeof(r->text), r->text);
		break;
	case LINE_WHERE:
		if (r->exception != 0) {
			snprintf(exception, sizeof(exception), ", exception %" PRIu32,
				 r->exception);
		}
		if (r->task[0] != '\0') {
			ret = snprintf(buf, len,
				       "In task %.*s%s, %" PRIu64 ".%06" PRIu64
				       " s after the start",
				       (int)sizeof(r->task), r->task, exception,
				       r->uptime_us / 1000000u, r->uptime_us % 1000000u);
		} else {
			ret = snprintf(buf, len,
				       "Before the scheduler started%s, %" PRIu64 ".%06" PRIu64
				       " s after the start",
				       exception, r->uptime_us / 1000000u, r->uptime_us % 1000000u);
		}
		break;
	case LINE_PC:
		ret = snprintf(buf, len, "pc 0x%08" PRIx32 " lr 0x%08" PRIx32 " sp 0x%08" PRIx32,
			       r->pc, r->lr, r->sp);
		break;
	case LINE_FRAME_R0:
		ret = snprintf(buf, len,
			       "r0 0x%08" PRIx32 " r1 0x%08" PRIx32 " r2 0x%08" PRIx32
			       " r3 0x%08" PRIx32,
			       r->r0, r->r1, r->r2, r->r3);
		break;
	case LINE_FRAME_R12:
		ret = snprintf(buf, len, "r12 0x%08" PRIx32 " xpsr 0x%08" PRIx32, r->r12, r->xpsr);
		break;
	case LINE_SCRATCH:
	default:
		ret = snprintf(buf, len, "(from the watchdog scratch registers only)");
		break;
	}
	if (ret < 0) {
		buf[0] = '\0';
		return 0;
	}
	return (size_t)ret < len ? (size_t)ret : len - 1;
}

void fault_record_print(const struct fault_record *r)
{
	char line[128];

	for (unsigned int i = 0; fault_record_line(r, i, line, sizeof(line)) > 0; i++) {
		printf("%s\n", line);
	}
}
