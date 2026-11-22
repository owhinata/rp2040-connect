/* SPDX-License-Identifier: MIT */
/* A fake of the OS boundary of the key-value area (common/compat/
 * flash_kv_os.h) for host tests: 8 KiB of RAM standing for flash offsets
 * 0xffe000-0xffffff, with the flash's rules: a page program clears bits
 * only (AND), an erase sets a whole 4 KiB sector to 0xff, and a request
 * that is not inside the area or not aligned aborts the test (the layer
 * must never send one).  The RAM is shared (mmap MAP_SHARED), so that a
 * test can run each "boot" in a child process with fresh static state and
 * keep the flash across boots and power losses.
 */
#ifndef TEST_KV_FLASH_FAKE_H
#define TEST_KV_FLASH_FAKE_H

#include <stdbool.h>
#include <stdint.h>

#include "flash_kv_os.h"

struct kv_fake {
	uint8_t flash[FLASH_KV_SIZE];
	bool ready;           /* flash_kv_os_ready() */
	unsigned long reads, programs, erases;
	unsigned long ops;    /* programs + erases */
	int fail_next;        /* the next N programs and erases fail ... */
	int fail_errno;       /* ... with this (negative) value */
	unsigned long cut_at; /* power loss at this op (1-based; 0: never) */
};

/* The shared state, all erased and ready after kv_fake_init(). */
extern struct kv_fake *kv_fake;

void kv_fake_init(void);

/* The process exit status of a child that lost power (kv_fake->cut_at). */
#define KV_FAKE_CUT_EXIT 77

#endif
