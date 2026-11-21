/* SPDX-License-Identifier: MIT */
/* <nuttx/mutex.h> for the host test of rp2040_dmac.c: a lock that the test
 * can hold as another task would (fake_mutex_held), so that a try fails.
 * A wait on a held lock is a fault of the code under test.
 */
#ifndef TEST_RP2040_DMAC_NUTTX_MUTEX_H
#define TEST_RP2040_DMAC_NUTTX_MUTEX_H
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
typedef struct {
	bool locked;
} mutex_t;
#define NXMUTEX_INITIALIZER {false}
extern bool fake_mutex_held;
static inline int nxmutex_trylock(mutex_t *m)
{
	if (fake_mutex_held || m->locked) {
		return -EAGAIN;
	}
	m->locked = true;
	return 0;
}
static inline int nxmutex_lock(mutex_t *m)
{
	assert(!fake_mutex_held && !m->locked);
	m->locked = true;
	return 0;
}
static inline int nxmutex_unlock(mutex_t *m)
{
	assert(m->locked);
	m->locked = false;
	return 0;
}
#endif
