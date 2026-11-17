/* SPDX-License-Identifier: MIT */
/* See kv_flash_fake.h. */
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "kv_flash_fake.h"

struct kv_fake *kv_fake;

void kv_fake_init(void)
{
	if (!kv_fake) {
		kv_fake = mmap(NULL, sizeof(*kv_fake), PROT_READ | PROT_WRITE,
			       MAP_SHARED | MAP_ANONYMOUS, -1, 0);
		assert(kv_fake != MAP_FAILED);
	}
	memset(kv_fake, 0, sizeof(*kv_fake));
	memset(kv_fake->flash, 0xff, sizeof(kv_fake->flash));
	kv_fake->ready = true;
	kv_fake->fail_errno = -EIO;
}

bool nano_flash_kv_os_ready(void)
{
	return kv_fake->ready;
}

int nano_flash_kv_os_read(uint32_t offset, void *buf, uint32_t len)
{
	assert(kv_fake->ready);
	assert(nano_kv_range_ok(offset, len, NANO_KV_OFFSET, NANO_KV_END, 1));
	kv_fake->reads++;
	memcpy(buf, &kv_fake->flash[offset - NANO_KV_OFFSET], len);
	return 0;
}

/* Count the op; at the cut, do half of it and lose power. */
static bool op_cut(void)
{
	kv_fake->ops++;
	return kv_fake->cut_at && kv_fake->ops == kv_fake->cut_at;
}

int nano_flash_kv_os_program(uint32_t page_offset, const uint8_t page[NANO_KV_PAGE_SIZE])
{
	uint8_t *p = &kv_fake->flash[page_offset - NANO_KV_OFFSET];
	bool cut;

	assert(kv_fake->ready);
	assert(nano_kv_range_ok(page_offset, NANO_KV_PAGE_SIZE, NANO_KV_OFFSET, NANO_KV_END,
				NANO_KV_PAGE_SIZE));
	if (kv_fake->fail_next > 0) {
		kv_fake->fail_next--;
		return kv_fake->fail_errno;
	}
	cut = op_cut();
	kv_fake->programs++;
	for (uint32_t i = 0; i < (cut ? NANO_KV_PAGE_SIZE / 2 : NANO_KV_PAGE_SIZE); i++) {
		p[i] &= page[i];
	}
	if (cut) {
		_exit(KV_FAKE_CUT_EXIT);
	}
	return 0;
}

int nano_flash_kv_os_erase(uint32_t sector_offset)
{
	uint8_t *p = &kv_fake->flash[sector_offset - NANO_KV_OFFSET];
	bool cut;

	assert(kv_fake->ready);
	assert(nano_kv_range_ok(sector_offset, NANO_KV_SECTOR_SIZE, NANO_KV_OFFSET, NANO_KV_END,
				NANO_KV_SECTOR_SIZE));
	if (kv_fake->fail_next > 0) {
		kv_fake->fail_next--;
		return kv_fake->fail_errno;
	}
	cut = op_cut();
	kv_fake->erases++;
	/* A cut erase leaves the sector's second half as it was. */
	memset(p, 0xff, cut ? NANO_KV_SECTOR_SIZE / 2 : NANO_KV_SECTOR_SIZE);
	if (cut) {
		_exit(KV_FAKE_CUT_EXIT);
	}
	return 0;
}
