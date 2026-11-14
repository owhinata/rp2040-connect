/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX 13.1.0's recursive mutex (<nuttx/mutex.h>, libs/libc/misc/
 * lib_mutex.c) as far as zephyr_mutex.c uses it, for its host test.  The
 * holder is a thread ID or NXSEM_NO_MHOLDER, as in the real one's
 * mholder: NXRMUTEX_INITIALIZER and nxrmutex_init() leave it without a
 * holder, while an all-zero mutex is held by thread 0 (the idle task), so
 * that any other thread's nxrmutex_lock() waits for ever.  Here such a
 * wait is counted in fake_waits_forever and fails the call with -EDEADLK
 * (the test's thread is the only one).  FAKE_TID is the calling thread. */
#ifndef TEST_ZMUTEX_NUTTX_MUTEX_H
#define TEST_ZMUTEX_NUTTX_MUTEX_H
#include <errno.h>
#include <stdint.h>

#define FAKE_NO_MHOLDER ((uint32_t)0x7ffffffe) /* NXSEM_NO_MHOLDER */

typedef struct {
	uint32_t mholder;
	unsigned int count;
} rmutex_t;

#define NXRMUTEX_INITIALIZER { FAKE_NO_MHOLDER, 0 }

extern uint32_t fake_tid;
extern int fake_waits_forever, fake_tick_waits;

static inline int nxrmutex_init(rmutex_t *m)
{
	m->mholder = FAKE_NO_MHOLDER;
	m->count = 0;
	return 0;
}

/* nxmutex_trylock() after nxrmutex_is_hold(): 0 or -EAGAIN. */
static inline int fake_take(rmutex_t *m)
{
	if (m->mholder == fake_tid) {
		m->count++;
		return 0;
	}
	if (m->mholder != FAKE_NO_MHOLDER) {
		return -EAGAIN;
	}
	m->mholder = fake_tid;
	m->count = 1;
	return 0;
}

static inline int nxrmutex_lock(rmutex_t *m)
{
	int ret = fake_take(m);

	if (ret) {
		fake_waits_forever++;
		return -EDEADLK;
	}
	return 0;
}

static inline int nxrmutex_trylock(rmutex_t *m)
{
	return fake_take(m);
}

static inline int nxrmutex_ticklock(rmutex_t *m, uint32_t delay)
{
	int ret = fake_take(m);

	if (ret) {
		fake_tick_waits++;
		(void)delay;
		return -ETIMEDOUT;
	}
	return 0;
}

static inline int nxrmutex_unlock(rmutex_t *m)
{
	if (m->mholder != fake_tid || m->count == 0) {
		return -EPERM;
	}
	if (--m->count == 0) {
		m->mholder = FAKE_NO_MHOLDER;
	}
	return 0;
}
#endif
