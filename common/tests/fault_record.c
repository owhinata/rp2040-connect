/* SPDX-License-Identifier: MIT */
/* common/log/fault_record.c: the check of the record, the copy in the
 * watchdog's scratch registers and the lines of its text.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "fault_record.h"

static struct fault_record sample(void)
{
	struct fault_record r;

	memset(&r, 0, sizeof(r));
	r.reason = FAULT_REASON_HARDFAULT;
	r.flags = FAULT_RECORD_FRAME;
	r.exception = 0;
	r.pc = 0x10001234u;
	r.lr = 0x10005679u;
	r.sp = 0x20010f00u;
	r.r0 = 0x30000000u;
	r.r1 = 1;
	r.r2 = 2;
	r.r3 = 3;
	r.r12 = 12;
	r.xpsr = 0x61000000u;
	r.uptime_us = 3723000042ull;
	strcpy(r.task, "console");
	return r;
}

static void expect_lines(const struct fault_record *r, const char *const *lines)
{
	char buf[160];
	unsigned int i;

	for (i = 0; lines[i] != NULL; i++) {
		size_t n = fault_record_line(r, i, buf, sizeof(buf));

		if (n != strlen(lines[i]) || strcmp(buf, lines[i]) != 0) {
			fprintf(stderr, "line %u: \"%s\", expected \"%s\"\n", i, buf, lines[i]);
			assert(0);
		}
	}
	assert(fault_record_line(r, i, buf, sizeof(buf)) == 0);
}

int main(void)
{
	struct fault_record r = sample();
	struct fault_record copy;
	uint32_t s[FAULT_SCRATCH_WORDS];
	char buf[16];

	/* The CRC is zlib's crc32(): the check value of "123456789". */
	assert(fault_record_crc32("123456789", 9) == 0xcbf43926u);
	assert(fault_record_crc32("", 0) == 0);

	/* Zeroed or random RAM is not a record. */
	assert(!fault_record_valid(&r));
	fault_record_seal(&r);
	assert(r.magic == FAULT_RECORD_MAGIC);
	assert(r.size == sizeof(r));
	assert(fault_record_valid(&r));

	/* Every covered byte counts, and so do magic and size. */
	for (size_t i = 0; i < offsetof(struct fault_record, check); i++) {
		copy = r;
		((unsigned char *)&copy)[i] ^= 0x01;
		assert(!fault_record_valid(&copy));
	}
	copy = r;
	copy.check ^= 0x80000000u;
	assert(!fault_record_valid(&copy));

	/* Sealing terminates the strings. */
	copy = r;
	memset(copy.task, 'x', sizeof(copy.task));
	memset(copy.text, 'y', sizeof(copy.text));
	fault_record_seal(&copy);
	assert(fault_record_valid(&copy));
	assert(strlen(copy.task) == sizeof(copy.task) - 1);
	assert(strlen(copy.text) == sizeof(copy.text) - 1);

	/* A HardFault in a task, with its frame. */
	expect_lines(&r, (const char *const[]){
				 "Fatal error: HardFault",
				 "In task console, 3723.000042 s after the start",
				 "pc 0x10001234 lr 0x10005679 sp 0x20010f00",
				 "r0 0x30000000 r1 0x00000001 r2 0x00000002 r3 0x00000003",
				 "r12 0x0000000c xpsr 0x61000000",
				 NULL,
			 });

	/* An assertion in an interrupt handler, before the scheduler. */
	r = sample();
	r.reason = FAULT_REASON_ASSERT;
	r.flags = 0;
	r.exception = 16 + 5;
	r.lr = 0;
	r.task[0] = '\0';
	r.uptime_us = 999999;
	strcpy(r.text, "tasks.c:1234");
	fault_record_seal(&r);
	expect_lines(&r, (const char *const[]){
				 "Fatal error: assertion failed: tasks.c:1234",
				 "Before the scheduler started, exception 21, 0.999999 s after the "
				 "start",
				 "pc 0x10001234 lr 0x00000000 sp 0x20010f00",
				 NULL,
			 });
	strcpy(r.task, "imu_raw_mqtt_tid");
	expect_lines(&r,
		     (const char *const[]){
			     "Fatal error: assertion failed: tasks.c:1234",
			     "In task imu_raw_mqtt_tid, exception 21, 0.999999 s after the start",
			     "pc 0x10001234 lr 0x00000000 sp 0x20010f00",
			     NULL,
		     });

	/* Truncation: len - 1 characters and the terminator. */
	assert(fault_record_line(&r, 0, buf, sizeof(buf)) == sizeof(buf) - 1);
	assert(strcmp(buf, "Fatal error: as") == 0);
	assert(fault_record_line(&r, 0, buf, 0) == 0);

	/* The reasons. */
	assert(strcmp(fault_reason_name(FAULT_REASON_PANIC), "panic") == 0);
	assert(strcmp(fault_reason_name(FAULT_REASON_STACK_OVERFLOW), "stack overflow") == 0);
	assert(strcmp(fault_reason_name(FAULT_REASON_HEAP), "heap exhausted") == 0);
	assert(strcmp(fault_reason_name(FAULT_REASON_EXIT), "exit") == 0);
	assert(strcmp(fault_reason_name(FAULT_REASON_NONE), "unknown") == 0);
	assert(strcmp(fault_reason_name(77), "unknown") == 0);

	/* The scratch registers: reason, pc, lr, and a check of the three. */
	r = sample();
	r.reason = FAULT_REASON_STACK_OVERFLOW;
	fault_record_seal(&r);
	fault_record_to_scratch(&r, s);
	assert(s[0] == (FAULT_SCRATCH_MAGIC | FAULT_REASON_STACK_OVERFLOW));
	assert(s[1] == r.pc && s[2] == r.lr);
	assert(s[3] == fault_record_crc32(s, 12));
	memset(&copy, 0xa5, sizeof(copy));
	assert(fault_record_from_scratch(s, &copy));
	assert(fault_record_valid(&copy));
	assert(copy.reason == FAULT_REASON_STACK_OVERFLOW && copy.pc == r.pc && copy.lr == r.lr);
	assert(copy.flags == FAULT_RECORD_SCRATCH && copy.task[0] == '\0' && copy.text[0] == '\0');
	expect_lines(&copy, (const char *const[]){
				    "Fatal error: stack overflow",
				    "pc 0x10001234 lr 0x10005679 sp 0x00000000",
				    "(from the watchdog scratch registers only)",
				    NULL,
			    });
	/* Anything else in the registers (zero after power-on, a changed word)
	 * is not taken, and the record is left alone.
	 */
	for (int w = 0; w < FAULT_SCRATCH_WORDS; w++) {
		uint32_t t[FAULT_SCRATCH_WORDS];

		memcpy(t, s, sizeof(t));
		t[w] ^= 0x00010000u;
		copy.reason = 1234;
		assert(!fault_record_from_scratch(t, &copy));
		assert(copy.reason == 1234);
	}
	memset(s, 0, sizeof(s));
	assert(!fault_record_from_scratch(s, &copy));

	printf("fault_record: ok\n");
	return 0;
}
