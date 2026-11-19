/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX's sem_t (<nuttx/semaphore.h>), defined by tests/nina_netdev.c. */

#ifndef TEST_NINA_NETDEV_NUTTX_SEMAPHORE_H
#define TEST_NINA_NETDEV_NUTTX_SEMAPHORE_H

typedef struct
{
  int count;
} sem_t;

#define SEM_INITIALIZER(c) {(c)}

int nxsem_post(sem_t *sem);
int nxsem_wait_uninterruptible(sem_t *sem);

#endif /* TEST_NINA_NETDEV_NUTTX_SEMAPHORE_H */
