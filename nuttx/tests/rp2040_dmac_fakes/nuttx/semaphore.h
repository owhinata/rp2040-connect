/* SPDX-License-Identifier: MIT */
/* <nuttx/semaphore.h> for the host test of rp2040_dmac.c: a count.  A wait
 * at count 0 would block for ever here and is a fault of the test.
 */
#ifndef TEST_RP2040_DMAC_NUTTX_SEMAPHORE_H
#define TEST_RP2040_DMAC_NUTTX_SEMAPHORE_H
#include <assert.h>
#include <errno.h>
typedef struct {
	int count;
} sem_t;
#define SEM_INITIALIZER(n) {(n)}
static inline int nxsem_trywait(sem_t *s)
{
	if (s->count == 0) {
		return -EAGAIN;
	}
	s->count--;
	return 0;
}
static inline int nxsem_wait_uninterruptible(sem_t *s)
{
	assert(s->count > 0);
	s->count--;
	return 0;
}
static inline int nxsem_post(sem_t *s)
{
	s->count++;
	return 0;
}
#endif
