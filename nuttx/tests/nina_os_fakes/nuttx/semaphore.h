/* SPDX-License-Identifier: MIT */
/* NuttX's semaphores for tests/nina_os.c: a count and one waiter (the
 * test), whose timed wait runs the test's clock until the count is
 * positive or the timeout has passed.
 */

#ifndef TEST_NINA_OS_NUTTX_SEMAPHORE_H
#define TEST_NINA_OS_NUTTX_SEMAPHORE_H

#include <stdint.h>

typedef struct
{
  int count;
} sem_t;

#define SEM_INITIALIZER(c) {(c)}

int nxsem_post(sem_t *sem);
int nxsem_get_value(sem_t *sem, int *sval);
int nxsem_reset(sem_t *sem, int16_t count);
int nxsem_tickwait_uninterruptible(sem_t *sem, uint32_t delay);
int nxsem_wait_uninterruptible(sem_t *sem);

#endif /* TEST_NINA_OS_NUTTX_SEMAPHORE_H */
